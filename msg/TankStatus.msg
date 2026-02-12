uint64  timestamp		# time since system update (microseconds)
float32 maxlevel		# maximum liquid level (liter)
float32 remlevel		# remaining liquid level (liter)
float32 consumed		# consumed liquid level (liter)
float32 flowrate		# consumed liquid flow rate (liter/minute)
uint8   empty_action  	# action to take when the tank is empty

uint8 TS_ACTION_NONE = 0                # no action on tank empty violation
uint8 TS_ACTION_WARN = 1                # critical mavlink message
uint8 TS_ACTION_LOITER = 2              # switch to AUTO|LOITER
uint8 TS_ACTION_RTL = 3                 # switch to AUTO|RTL
uint8 TS_ACTION_TERMINATE = 4           # flight termination
uint8 TS_ACTION_LAND = 5                # switch to AUTO|LAND
