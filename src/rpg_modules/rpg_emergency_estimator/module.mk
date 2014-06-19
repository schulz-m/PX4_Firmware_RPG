		  #
# Build RPG emergency state estimator
#
# Maximilian Schulz - 2014

MODULE_COMMAND	 = rpg_emergency_estimator

SRCS		=  rpg_emergency_estimator_main.cpp\
		   	   rpg_emergency_estimator_params.c\
		   	   EKFFunction.cpp \
		   	   codegen/composeCPPPrediction_terminate.c \
		   	   codegen/composeCPPPrediction_initialize.c \
		   	   codegen/composeCPPPrediction.c \
		   	   codegen/composeUMatrix.c \
		   	   codegen/rt_nonfinite.c \
		   	   codegen/rtGetInf.c \
		   	   codegen/rtGetNaN.c
		  
#Let's see if I need this		  
#INCLUDE_DIRS	 += $(MAVLINK_SRC)/include/mavlink