//
//  Config.hpp
//  Auto_Pilot
//
//  Created by Louis Faury on 29/12/2015.
//  Copyright © 2015 Louis Faury. All rights reserved.
//

#ifndef Config_h
#define Config_h


//*--- For Autopilot Block ---*//

// Target
#define MAX_ALTI_TARGET 2000 //cm
#define INI_ALTI_TARGET 50 //cm
// Time
#define MIN_TIME_RATE 5 //ms
#define INTERP_TIME 5 //s

// Transfer function boundaries
#define EXTREMUM_INTEGRAL_ERROR 50

//
// Command boundaries : to be set, depending on the boundaries of motor speed
//
// U1 Altitude limited by maximum acceleration on the drone being 0.5g. Minimum is zero (all motors shutdown)
#define MAX_COMMAND_ALTI 1000.
#define MIN_COMMAND_ALTI 0.
// U2 Pitch limited by abs(angle acceleration) < 1
#define MAX_COMMAND_PITCH 0.0081
#define MIN_COMMAND_PITCH -0.0081
// U3 Roll limited by abs(angle acceleration) < 1
#define MAX_COMMAND_ROLL 0.0081
#define MIN_COMMAND_ROLL -0.0081
// U4 Yaw limited by abs(angle acceleration) < 1
#define MAX_COMMAND_YAW 0.0142
#define MIN_COMMAND_YAW -0.0142

// PID constant values
#define ALTI_KP -0.8
#define ALTI_TI 1.6
#define ALTI_TD 3.75

#define PITCH_KP -3.
#define PITCH_TI 6.
#define PITCH_TD 18.

#define ROLL_KP -3.
#define ROLL_TI 6.
#define ROLL_TD 18.

#define YAW_KP -3.
#define YAW_TI 6.
#define YAW_TD 18.

#define K_ARW -1.
#define N_FILTER 20.

// Mechanical constants
#define thrust_factor 0.0000542
#define drag_factor 0.0000011
#define center_to_motor_distance 0.24


// Motors Speed and acceleration bounded
// TO DO : compute real values for these physical limitations (see doc motors or use 3/2g bounding?)
// #Motors1
#define MAX_MOTOR_SPEED 1.
#define MAX_MOTOR_ACCELERATION 1.










#endif
