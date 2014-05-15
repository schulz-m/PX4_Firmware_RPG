		  #
# Build RPG emergency state estimator
#
# Maximilian Schulz - 2014

MODULE_COMMAND	 = rpg_emergency_estimator

SRCS		=  rpg_emergency_estimator_main.cpp\
		   	   rpg_emergency_estimator_params.cpp
		  
#Let's see if I need this		  
#INCLUDE_DIRS	 += $(MAVLINK_SRC)/include/mavlink