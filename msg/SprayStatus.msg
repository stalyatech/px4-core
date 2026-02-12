uint64  timestamp		# time since system update (microseconds)
float32 flyspeed		# Flying speed (m/s)
float32 flyheight		# Flying height (m)
float32 flowrate		# Spraying liquid flow rate (liter/minute)
uint8   mode			# Spraying mode
uint8   action			# Spraying action
uint8   status			# Spraying status

uint8 MODE_MANUEL      = 0	# Manuel spraying mode
uint8 MODE_AUTO        = 1	# Automatic spraying mode

uint8 STATUS_DISABLE   = 0	# Spraying status disabled
uint8 STATUS_ENABLE    = 1	# Spraying status enabled
uint8 STATUS_DONE      = 2	# Spraying status done

uint8 ACTION_NONE      = 0	# no action on tank empty violation
uint8 ACTION_WARN      = 1	# critical mavlink message
uint8 ACTION_LOITER    = 2	# switch to AUTO|LOITER
uint8 ACTION_RTL       = 3	# switch to AUTO|RTL
uint8 ACTION_TERMINATE = 4	# flight termination
uint8 ACTION_LAND      = 5	# switch to AUTO|LAND
