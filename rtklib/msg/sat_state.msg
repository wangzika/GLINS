uint8 sys        # navigation system
uint8 vs         # valid satellite flag single
float64[2] azel     # azimuth/elevation angles {az,el} (rad)
float64[2] azel_b     # azimuth/elevation angles {az,el} (rad)
float64[3] resp # residuals of pseudorange (m)
float64[3] resc # residuals of carrier-phase (m)
float64[3] resd # residuals of doppler (m/s)
float64[3] icbias  # glonass IC bias (cycles)
uint8[3] vsat # valid satellite flag
uint8[3] vsatL
uint8[3] vsatP
uint8[3] vsatD
uint16[3] snr_rover  # rover signal strength (0.25 dBHz)
uint16[3] snr_base   # base signal strength (0.25 dBHz)
uint8[3] fix  # ambiguity fix flag (1:fix,2:float,3:hold)
uint8[3] slip # cycle-slip flag
uint8[3] half # half-cycle valid flag
int32[3] lock    # lock counter of phase
uint32[3] outc  # obs outage counter of phase
uint32[3] slipc # cycle-slip counter
uint32[3] rejc  # reject counter
float64[2] gf # geometry-free phase (m)
float64[2] mw # MW-LC (m)
float64 phw         # phase windup (cycle)
float64[6] pt # previous carrier-phase time
float64[6] ph # previous carrier-phase observable (cycle)

float64[3] dion
float64[3] dtrp

float64[3] P_COR
float64[3] tgd

int32[3] init
int32[9] outlier_obs
float64[3] lam
int32[3] amb_id
float64 ephvar
int32 svh