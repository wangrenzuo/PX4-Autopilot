# this file is only used in the position_setpoint triple as a dependency

uint64 timestamp		# time since system start (microseconds)

uint8 SETPOINT_TYPE_IDLE     = 0 # do nothing, switch off motors or keep at idle speed (MC)
uint8 SETPOINT_TYPE_POSITION = 1 # position setpoint
uint8 SETPOINT_TYPE_LOITER   = 2 # loiter setpoint
uint8 SETPOINT_TYPE_TAKEOFF  = 3 # takeoff setpoint
uint8 SETPOINT_TYPE_LAND     = 4 # land setpoint, altitude must be ignored, descend until landing

bool valid			# true if setpoint is valid
uint8 type			# setpoint type to adjust behavior of position controller

float32 x			# local position setpoint in m in NED
float32 y			# local position setpoint in m in NED
float32 z			# local position setpoint in m in NED
bool position_valid		# true if local position setpoint valid

float64 lat			# latitude, in deg
float64 lon			# longitude, in deg
float32 alt			# altitude AMSL, in m

float32 yaw			# yaw (only for multirotors), in rad [-PI..PI), NaN = hold current yaw
bool yaw_valid			# true if yaw setpoint valid

float32 yawspeed		# yawspeed (only for multirotors, in rad/s)
bool yawspeed_valid		# true if yawspeed setpoint valid

int8 landing_gear		# landing gear: see definition of the states in landing_gear.msg

float32 loiter_radius		# loiter radius (only for fixed wing), in m
int8 loiter_direction		# loiter direction: 1 = CW, -1 = CCW
float32 pitch_min		# minimal pitch angle for fixed wing takeoff waypoints

float32 acceptance_radius   # navigation acceptance_radius if we're doing waypoint navigation

float32 cruising_speed		# the generally desired cruising speed (not a hard constraint)
float32 cruising_throttle	# the generally desired cruising throttle (not a hard constraint)

bool disable_weather_vane   # VTOL: disable (in auto mode) the weather vane feature that turns the nose into the wind
