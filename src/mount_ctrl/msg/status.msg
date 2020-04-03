int8 status # status of the mount, 0=disconnected, 1=connected, 2=scanning, 3=moving, 4=goto running, 5=tracking, 6=homing, 7=stop
float64 hr # hour angle
float64 dec  # declination
int16 signal # Signal strength (from ADS1115)
float32 scanProgress # scan progress if scan is running, 0 by default