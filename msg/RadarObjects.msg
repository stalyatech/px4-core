# Object list of one radar measurement cycle (Nanoradar MR72 over CAN)

uint64 timestamp            # time since system start (microseconds)
uint64 timestamp_sample     # reception of the cycle header (microseconds)

uint32 device_id            # unique device ID for the sensor that does not change between power cycles

uint16 meas_count           # radar cycle counter
uint8 reported              # objects the radar announced for this cycle
uint8 count                 # valid objects stored below, nearest first as sent

uint8 MAX_OBJECTS = 16

uint8[16] id                # track id, stable while the object is tracked
float32[16] dist_long       # [m] along the boresight
float32[16] dist_lat        # [m] lateral, radar frame
float32[16] vrel_long       # [m/s] radial velocity
float32[16] vrel_lat        # [m/s] lateral velocity
float32[16] rcs             # [dBm2] radar cross section
uint8[16] sector            # 1...3, 0 when not reported
