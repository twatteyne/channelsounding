#!/usr/bin/env python3
"""Live Channel Sounding dashboard, listening directly to an nRF over USB serial.

Listens continuously on the nRF whose USB serial number ends in --serial (default 151),
parses the CSV export printed by the ras_initiator firmware ("CSVP," one row per
procedure, "CSV," one row per tone) and serves a real-time web dashboard.

    scripts/dashboard.py                          # auto-detect nRF (serial ..151), http://<host>:8950/
    scripts/dashboard.py --serial 151 --baud 115200
    scripts/dashboard.py --port /dev/ttyACM3 --rafraichir 300

Requires pyserial: pip install pyserial
"""
import argparse
import glob
import json
import os
import re
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import serial
except ImportError:
    serial = None

# Column order of the CSV output (must match csv_export.c / cs_csv_logger.py).
PROC = ["timestamp_ms", "ranging_counter", "ap", "tone_quality", "rtt_count",
        "rtt_mean_ns", "tof_ns", "rssi_dbm", "dist_ifft_m", "dist_phase_slope_m",
        "dist_rtt_m"]
TONE = ["timestamp_ms", "ranging_counter", "ap", "channel", "freq_mhz",
        "h_re", "h_im", "amplitude", "phase_deg", "n_iqs"]
FLOAT = {PROC[5], PROC[6], PROC[7], PROC[8], PROC[9], PROC[10],
         TONE[5], TONE[6], TONE[7], TONE[8]}


def num(v, default=None):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


# --------------------------------------------------------------------------- device discovery

def list_ports():
    return sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))


def serial_of(dev):
    head = os.path.basename(dev)
    try:
        node = os.path.realpath(f"/sys/class/tty/{head}")
    except OSError:
        node = ""
    d = node
    for _ in range(14):
        for attr in ("serial", "serial_number", "serial_id"):
            p = os.path.join(d, attr)
            if os.path.isfile(p):
                try:
                    with open(p) as f:
                        return f.read().strip()
                except OSError:
                    pass
        parent = os.path.dirname(d)
        if parent == d or not parent:
            break
        d = parent
    try:
        out = subprocess.run(["udevadm", "info", "-q", "property", "-n", dev],
                             capture_output=True, text=True, timeout=3).stdout
        for ln in out.splitlines():
            if ln.startswith("ID_SERIAL_SHORT="):
                return ln.split("=", 1)[1].strip()
    except Exception:
        pass
    return None


def sniff_csv(dev, baud, window=0.8):
    """Returns an estimate of CSV output rate on this port in `window` seconds."""
    if serial is None:
        return 0
    try:
        s = serial.Serial(dev, baud, timeout=0.5)
        s.reset_input_buffer()
        buf = b""
        end = time.time() + window
        while time.time() < end:
            d = s.read(2048)
            if d:
                buf += d
        s.close()
        return buf.count(b"CSVP") * 50 + buf.count(b"CSV,")
    except Exception:
        return 0


def discover(suffix, baud):
    """Auto-select the port belonging to a device whose serial ends with `suffix`."""
    if serial is None:
        return None, "pyserial n'est pas installé (pip install pyserial)"
    matched = [d for d in list_ports() if (serial_of(d) or "").endswith(suffix)]
    if not matched:
        return None, f"aucun port avec un numéro de série finissant par '{suffix}' détecté"
    best, bestscore = None, 0
    for d in matched:
        score = sniff_csv(d, baud)
        if score > bestscore:
            best, bestscore = d, score
    if best is None:
        best = matched[0]
    return best, None


# --------------------------------------------------------------------------- live capture

class Live:
    """Shared state filled continuously by the serial reader thread."""

    def __init__(self):
        self.lock = threading.Lock()
        self.procs = []          # [(seen_ts, row_dict)]
        self.tone_ts = deque()   # recent tone timestamps, for the rate
        self.total_procs = 0
        self.total_tones = 0
        self.alive = True
        self.dev, self.serial, self.baud = None, None, None
        self.started = time.time()
        self.last_active = time.time()
        self.error = None
        self.noncsv = 0

    def set_port(self, dev, serial, baud):
        self.dev, self.serial, self.baud = dev, serial, baud

    def touch(self):
        self.last_active = time.time()

    def add_proc(self, row):
        now = time.time()
        self.procs.append((now, row))
        self.total_procs += 1
        if len(self.procs) > 20000:
            self.procs = self.procs[-20000:]

    def add_tone(self, row):
        now = time.time()
        self.total_tones += 1
        self.tone_ts.append(now)
        if len(self.tone_ts) > 20000:
            self.tone_ts.popleft()


def fmt_row(fields, header):
    r = dict(zip(header, fields))
    for k in FLOAT.intersection(r):
        r[k] = num(r[k])
    return r


def handle_line(live, line):
    line = line.strip()
    idx = line.find("CSV")
    if idx < 0:
        live.noncsv += 1
        return
    part = line[idx:]
    tag, _, rest = part.partition(",")
    if tag in ("CSVH", "CSVPH"):
        return
    fields = rest.split(",")
    if tag == "CSVP":
        if len(fields) >= len(PROC):
            live.add_proc(fmt_row(fields[: len(PROC)], PROC))
    elif tag == "CSV":
        if len(fields) >= len(TONE):
            live.add_tone(fmt_row(fields[: len(TONE)], TONE))


def read_loop(live, dev, baud):
    try:
        s = serial.Serial(dev, baud, timeout=1)
    except Exception as e:
        live.error = f"ouverture du port impossible : {e}"
        return
    live.set_port(dev, serial_of(dev), baud)
    buf = b""
    try:
        while live.alive:
            try:
                d = s.read(8192)
            except serial.SerialException as e:
                live.error = f"erreur de lecture : {e}"
                break
            if not d:
                live.touch()
                continue
            live.touch()
            buf += d
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                handle_line(live, line.decode(errors="replace"))
    finally:
        s.close()


# --------------------------------------------------------------------------- json snapshot

def snapshot(live, window=300):
    now = time.time()
    recent = [r for r in live.procs if r[0] > now - window]
    last = live.procs[-1][1] if live.procs else {}
    step = max(1, len(recent) // 700)
    dist = [{"t": round(r[0] - now, 1),
             "ifft": r[1].get("dist_ifft_m"), "slope": r[1].get("dist_phase_slope_m"),
             "rtt": r[1].get("dist_rtt_m"), "rssi": r[1].get("rssi_dbm"),
             "q": (r[1].get("tone_quality") or "").strip().upper()}
            for r in recent[::step]]
    n10 = sum(1 for r in live.procs if r[0] > now - 10)
    t10 = sum(1 for t in live.tone_ts if t > now - 10)
    ok = [r[1] for r in recent if (r[1].get("tone_quality") or "").strip().upper() == "OK"]
    running = now - live.last_active < 3 and live.error is None
    return {
        "connected": live.dev is not None and not live.error,
        "dev": live.dev, "serial": live.serial, "baud": live.baud,
        "running": running, "error": live.error, "uptime": round(now - live.started),
        "procs_rate": n10 / 10, "tones_rate": t10 / 10,
        "total_procs": live.total_procs, "total_tones": live.total_tones,
        "noncsv": live.noncsv, "now": now,
        "quality": (last.get("tone_quality") or "").strip(),
        "rc": last.get("ranging_counter"), "ap": last.get("ap"),
        "ifft": last.get("dist_ifft_m"), "slope": last.get("dist_phase_slope_m"),
        "rtt": last.get("dist_rtt_m"), "rssi": last.get("rssi_dbm"),
        "rtt_count": last.get("rtt_count"), "rtt_mean_ns": last.get("rtt_mean_ns"),
        "tof_ns": last.get("tof_ns"),
        "okpct": 100 * len(ok) / len(recent) if recent else None,
        "dist": dist,
        "table": [{"t": round(r[0] - now, 0),
                   "time": time.strftime("%H:%M:%S", time.localtime(r[0])),
                   "rc": r[1].get("ranging_counter"), "ap": r[1].get("ap"),
                   "q": (r[1].get("tone_quality") or "").strip().upper(),
                   "cnt": r[1].get("rtt_count"), "rssi": r[1].get("rssi_dbm"),
                   "ifft": r[1].get("dist_ifft_m"), "slope": r[1].get("dist_phase_slope_m"),
                   "rtt": r[1].get("dist_rtt_m")}
                  for r in recent[-30:][::-1]],
    }


# --------------------------------------------------------------------------- web page

PAGE = r"""<!doctype html><html lang="fr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Dashboard CS — temps réel</title><style>
:root{--bg:#f6f7f4;--card:#fff;--fg:#1d2420;--mut:#66706a;--line:#dde2dc;--ok:#1f8a4c;--bad:#c0392b;--warn:#b7791f;--a:#2563a8;--b:#8e44ad;--c:#c26a12}
@media (prefers-color-scheme:dark){:root{--bg:#141816;--card:#1d2320;--fg:#e6ebe7;--mut:#9aa59e;--line:#2e3632;--a:#6aa5e8;--b:#c08ae0;--c:#e8a15a}}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.4 system-ui,sans-serif;padding:16px}
h1{font-size:18px;margin:0 0 12px}.g{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(170px,1fr))}
.c{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:10px 12px;min-width:0}
.l{font-size:11px;letter-spacing:.06em;text-transform:uppercase;color:var(--mut)}.v{font-size:24px;font-variant-numeric:tabular-nums}
.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}
.w{grid-column:1/-1}svg{width:100%;height:220px;display:block}
table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums;font-size:13px}td,th{padding:2px 8px;text-align:right;border-bottom:1px solid var(--line)}th{position:sticky;top:0;background:var(--card)}
.lg span{margin-right:14px}.sw{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:4px;vertical-align:-1px}
.pill{display:inline-block;padding:1px 8px;border-radius:10px;font-size:11px;text-transform:uppercase;letter-spacing:.05em}
</style></head><body><h1>Mesures CS — temps réel <span id="st" class="l"></span></h1>
<div class="g">
<div class="c w"><div class="l">Connexion nRF</div><div class="v" id="conn">–</div><div id="conninfo" class="l"></div></div>
<div class="c"><div class="l">Débit procédures</div><div class="v" id="prate">–</div><div class="l">procédures / s</div></div>
<div class="c"><div class="l">Débit tonalités</div><div class="v" id="trate">–</div><div class="l">tonalités / s</div></div>
<div class="c"><div class="l">Total</div><div class="v" id="tot">–</div><div id="tot2" class="l"></div></div>
<div class="c"><div class="l">Qualité</div><div class="v" id="qual">–</div><div id="okpct" class="l"></div></div>
<div class="c"><div class="l">RSSI</div><div class="v" id="rssi">–</div><div class="l">dBm</div></div>
<div class="c"><div class="l">Distance IFFT</div><div class="v" id="ifft">–</div><div class="l">m</div></div>
<div class="c"><div class="l">Pente de phase</div><div class="v" id="slope">–</div><div class="l">m</div></div>
<div class="c"><div class="l">Distance RTT</div><div class="v" id="rtt">–</div><div id="rttinfo" class="l"></div></div>
<div class="c w"><div class="l">Distances (m) — dernière procédure par timestamp</div>
<div class="lg l"><span><i class="sw" style="background:var(--a)"></i>IFFT</span><span><i class="sw" style="background:var(--b)"></i>pente de phase</span><span><i class="sw" style="background:var(--c)"></i>RTT</span></div><svg id="ts"></svg></div>
<div class="c w"><div class="l">RSSI (dBm)</div><svg id="rss"></svg></div>
<div class="c w"><div class="l">Dernières procédures</div><div style="overflow:auto;max-height:360px"><table id="tab"></table></div></div>
</div><script>
const $=id=>document.getElementById(id),f=(x,n=1)=>x==null?'–':Number(x).toFixed(n).replace('.',',');
function plot(svg,series,xl,yl){const W=svg.clientWidth||600,H=220,m={l:44,r:10,t:8,b:24};let xs=[],ys=[];
series.forEach(s=>s.pts.forEach(p=>{if(p[1]==null)return;xs.push(p[0]);ys.push(p[1])}));if(!xs.length){svg.innerHTML='';return}
let x0=Math.min(...xs),x1=Math.max(...xs),y0=Math.min(...ys),y1=Math.max(...ys);if(x1==x0)x1=x0+1;if(y1==y0)y1=y0+1;
const X=x=>m.l+(x-x0)/(x1-x0)*(W-m.l-m.r),Y=y=>H-m.b-(y-y0)/(y1-y0)*(H-m.t-m.b);let h='';
for(let i=0;i<=4;i++){const yv=y0+(y1-y0)*i/4;h+=`<line x1="${m.l}" x2="${W-m.r}" y1="${Y(yv)}" y2="${Y(yv)}" stroke="var(--line)"/><text x="${m.l-4}" y="${Y(yv)+4}" text-anchor="end" font-size="10" fill="var(--mut)">${yl(yv)}</text>`}
for(let i=0;i<=4;i++){const xv=x0+(x1-x0)*i/4;h+=`<text x="${X(xv)}" y="${H-6}" text-anchor="middle" font-size="10" fill="var(--mut)">${xl(xv)}</text>`}
series.forEach(s=>{if(s.line){h+=`<polyline fill="none" stroke="${s.col}" stroke-width="2" points="${s.pts.filter(p=>p[1]!=null).map(p=>X(p[0])+','+Y(p[1])).join(' ')}"/>`}
s.pts.forEach(p=>{if(p[1]!=null)h+=`<circle cx="${X(p[0])}" cy="${Y(p[1])}" r="${s.r||1.6}" fill="${s.col}"/>`})});svg.innerHTML=h}
async function tick(){let s;try{s=await (await fetch('api')).json()}catch(e){$('st').textContent='serveur injoignable';return}
$('st').textContent=s.error?('⚠ '+s.error):(s.running?'acquisition en cours':(s.connected?'connecté mais silencieux':'hors connexion'));
$('conn').textContent=s.dev?s.dev:'–';
$('conninfo').textContent=(s.serial?('série '+s.serial+' · '):'')+(s.baud?s.baud+' baud':'')+(s.uptime!=null?' · '+f(s.uptime,0)+' s de session':'');
$('prate').textContent=f(s.procs_rate,1);$('trate').textContent=f(s.tones_rate,0);
$('tot').textContent=s.total_procs;$('tot2').textContent=s.total_tones+' tonalités';
$('qual').textContent=s.quality?s.quality:'–';$('qual').className='v '+(s.quality=='OK'?'ok':'bad');
$('okpct').textContent=s.okpct==null?'':f(s.okpct,0)+' % OK sur 5 min';
$('rssi').textContent=f(s.rssi,1);$('ifft').textContent=f(s.ifft,3);
$('slope').textContent=f(s.slope,3);$('rtt').textContent=f(s.rtt,3);
$('rttinfo').textContent=(s.rtt_count!=null?'rtt_count '+s.rtt_count:'')+(s.tof_ns!=null?' · tof '+f(s.tof_ns,1)+' ns':'');
$('rc').textContent='';
const D=s.dist.filter(d=>d.ifft!=null||d.slope!=null||d.rtt!=null);
plot($('ts'),[{col:'var(--a)',pts:D.map(d=>[d.t,d.ifft])},{col:'var(--b)',pts:D.map(d=>[d.t,d.slope])},{col:'var(--c)',pts:D.map(d=>[d.t,d.rtt])}],x=>f(x,0)+' s',v=>f(v,3));
plot($('rss'),[{col:'var(--a)',line:1,r:2,pts:s.dist.map(d=>[d.t,d.rssi])}],x=>f(x,0)+' s',v=>f(v,1));
$('tab').innerHTML='<tr><th>il y a (s)</th><th>heure</th><th>rc</th><th>ap</th><th>qualité</th><th>rtt_count</th><th>RSSI (dBm)</th><th>IFFT (m)</th><th>pente (m)</th><th>RTT (m)</th></tr>'+
s.table.map(r=>`<tr><td>${f(r.t,0)}</td><td>${r.time}</td><td>${r.rc}</td><td>${r.ap}</td><td class="${r.q=='OK'?'ok':'bad'}">${r.q||''}</td><td>${f(r.cnt,0)}</td><td>${f(r.rssi,1)}</td><td>${f(r.ifft,3)}</td><td>${f(r.slope,3)}</td><td>${f(r.rtt,3)}</td></tr>`).join('');}
tick();setInterval(tick,__REFRESH__);
</script></body></html>"""


# --------------------------------------------------------------------------- server

def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=None,
                   help="port série explicite (ex. /dev/ttyACM3) ; sinon autodétection")
    p.add_argument("--serial", default="151", help="suffixe du numéro de série nRF (défaut 151)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--http", type=int, default=8950, help="port du serveur web")
    p.add_argument("--hote", default="0.0.0.0")
    p.add_argument("--rafraichir", type=int, default=500,
                   help="période de rafraîchissement de la page en ms (défaut 500)")
    a = p.parse_args()

    live = Live()
    if a.port:
        dev, err = a.port, None
    else:
        dev, err = discover(a.serial, a.baud)
    if err:
        print("impossible de détecter le nRF :", err, flush=True)

    reader = threading.Thread(target=read_loop, args=(live, dev, a.baud),
                              daemon=True, name="serial-reader")
    reader.start()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path.startswith("/api"):
                body, ctype = json.dumps(snapshot(live)).encode(), "application/json"
            else:
                body, ctype = PAGE.replace("__REFRESH__", str(a.rafraichir)).encode(), \
                    "text/html; charset=utf-8"
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

    print("dashboard temps réel sur http://%s:%d/ (port %s, %d baud, rafraîchissement %d ms)" % (
        a.hote, a.http, dev or "?", a.baud, a.rafraichir), flush=True)
    ThreadingHTTPServer((a.hote, a.http), H).serve_forever()


if __name__ == "__main__":
    main()
