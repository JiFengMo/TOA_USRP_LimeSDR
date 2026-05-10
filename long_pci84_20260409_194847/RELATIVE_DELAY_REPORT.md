# PCI 84 Long Run Relative Delay Report

Config:
- center frequency: `2114550000 Hz`
- ARFCN: `422910`
- PCI: `84`
- sample rate: `30.72 Msps`
- RX gain: `64 dB`

Files:
- log: `run.log`
- delay csv: `ssb_relative_delay.csv`

Observed lock:
- PBCH/MIB success observed during this run
- one stable relative-delay cluster was logged for the successful lock window

Relative delay summary:
- rows: `34`
- mean: `10766459.760 ns`
- median: `10763340.416 ns`
- stddev: `3756.259 ns`
- min: `10762630.208 ns`
- max: `10770715.129 ns`
- max adjacent jump: `7916.128 ns`

Sample-domain summary:
- mean: `330745.643819 samples`
- stddev: `115.392262 samples`
- min: `330628.000000 samples`
- max: `330876.368773 samples`

Interpretation:
- this is a stable SSB-based relative arrival delay observation, not an absolute propagation delay
- the logged values sit around `10.766 ms`
- short-term spread in this run is on the order of `3.76 us`
