#
# Build RPG sensor module
#

MODULE_COMMAND	 = rpg_sensors
MODULE_PRIORITY	= "SCHED_PRIORITY_MAX-5"

SRCS		= rpg_sensors.cpp \
		  rpg_sensors_params.c