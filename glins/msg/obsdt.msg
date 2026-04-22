float64 time       # receiver sampling time (GPST)
uint8 sat
uint8 rcv     # satellite/receiver number
uint16[3] SNR # signal strength (0.001 dBHz)
uint8[3]  LLI # loss of lock indicator
uint8[3] code # code indicator (CODE_XXX)
float64[3] L # observation data carrier-phase (cycle)
float64[3] P # observation data pseudorange (m)
float32[3] D # observation data doppler frequency (Hz)

int32 timevalid          # time is valid (Valid GNSS fix) for time mark
float64 eventime         # time of event (GPST)
uint8[3] Lstd # stdev of carrier phase (0.004 cycles)
uint8[3] Pstd # stdev of pseudorange (0.01*2^(n+5) meters)
uint8 freq               # GLONASS frequency channel (0-13)