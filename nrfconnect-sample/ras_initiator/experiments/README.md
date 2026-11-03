# Channel map experiments

Run from `ras_initiator/`, then `west flash`. Only the initiator needs to be reflashed.

## 1. Vary the number of channels (span fixed, ch 2-76)

```sh
# 72 channels
west build -b nrf54l15dk/nrf54l15/cpuapp -p -- -DCONFIG_SAMPLE_RAS_INITIATOR_CH_STEP=1
# 37 channels
west build -b nrf54l15dk/nrf54l15/cpuapp -p -- -DCONFIG_SAMPLE_RAS_INITIATOR_CH_STEP=2
# 24 channels
west build -b nrf54l15dk/nrf54l15/cpuapp -p -- -DCONFIG_SAMPLE_RAS_INITIATOR_CH_STEP=3
# 19 channels
west build -b nrf54l15dk/nrf54l15/cpuapp -p -- -DCONFIG_SAMPLE_RAS_INITIATOR_CH_STEP=4
# 15 channels
west build -b nrf54l15dk/nrf54l15/cpuapp -p -- -DCONFIG_SAMPLE_RAS_INITIATOR_CH_STEP=5
```

## 2. Vary the frequency span (15 channels fixed)

```sh
# span 14 MHz
west build -b nrf54l15dk/nrf54l15/cpuapp -p -- -DCONFIG_SAMPLE_RAS_INITIATOR_CH_MIN=62 -DCONFIG_SAMPLE_RAS_INITIATOR_CH_MAX=76 -DCONFIG_SAMPLE_RAS_INITIATOR_CH_STEP=1
# span 28 MHz
west build -b nrf54l15dk/nrf54l15/cpuapp -p -- -DCONFIG_SAMPLE_RAS_INITIATOR_CH_MIN=48 -DCONFIG_SAMPLE_RAS_INITIATOR_CH_MAX=76 -DCONFIG_SAMPLE_RAS_INITIATOR_CH_STEP=2
# span 42 MHz
west build -b nrf54l15dk/nrf54l15/cpuapp -p -- -DCONFIG_SAMPLE_RAS_INITIATOR_CH_MIN=34 -DCONFIG_SAMPLE_RAS_INITIATOR_CH_MAX=76 -DCONFIG_SAMPLE_RAS_INITIATOR_CH_STEP=3
# span 56 MHz
west build -b nrf54l15dk/nrf54l15/cpuapp -p -- -DCONFIG_SAMPLE_RAS_INITIATOR_CH_MIN=18 -DCONFIG_SAMPLE_RAS_INITIATOR_CH_MAX=74 -DCONFIG_SAMPLE_RAS_INITIATOR_CH_STEP=4
# span 70 MHz
west build -b nrf54l15dk/nrf54l15/cpuapp -p -- -DCONFIG_SAMPLE_RAS_INITIATOR_CH_MIN=6 -DCONFIG_SAMPLE_RAS_INITIATOR_CH_MAX=76 -DCONFIG_SAMPLE_RAS_INITIATOR_CH_STEP=5
```

The channel map actually used is printed at boot:

```
Channel map: 37 channels, ch 2-76 (2404-2478 MHz), span 74 MHz, step 2 MHz
```
