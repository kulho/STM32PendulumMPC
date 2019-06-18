/*
 * userDefines.h
 *
 * Copyright (C) 2019 Jakub Kulhanek
 *
 * MIT Licence
 */

/* Modify this file to adjust system constants and choose type of control */

#ifndef INC_USERDEFINES_H_
#define INC_USERDEFINES_H_

/* Print levels, do not modify this!! */
#define _PRINT_NONE              0			// no printout
#define _PRINT_LOW               1			// prints states, input and reference
#define _PRINT_MEDIUM            2			// prints states, input, QP status, iter and cputime
#define _PRINT_HIGH              3			// prints states, input, QP status, iter, cputime and qpOASES output
#define _PRINT_MATLAB			 4			// prints for MATLAB plotting
/* When using the LQR control, only the first two printing options are available.
 * If other is selected, there will be no printout.
 */

/* select print level */
#define _PRINTOUT _PRINT_LOW		// define console print level

/* physical constants */
#define DEF_m2 				0.0785    		// pendulum mass [kg]
#define DEF_l 				0.5            	// pendulum length [m]
#define DEF_lg 				0.5           	// distance from the pivot to the CoG [m]
#define DEF_b 				0.069		 	// friction coefficient []
#define DEF_g 				9.81       		// gravitational acceleration [ms-2]
#define DEF_Eup 			0.1925     		// potential energy at upright position
#define DEF_r 				0.035		  	// radius of the pulley
#define DEF_pi 				3.14159265359 	// value of pi
#define DEF_p2m 			0.000032187  	// pulse to meters ratio

/* user constants */
#define DEF_nx 				4				// number of states
#define DEF_ny 				2				// number of observables
#define DEF_Treg			0.01			// sample time [s]
#define DEF_KalmanInit 		700   			// No of samples before swingup activates
#define DEF_swing2stab 		DEF_pi/6		// angle [rad] when swingup changes to stabilization algorithm
#define DEF_L 				0.4            	// cart position constraint |q|<=L [m] if cart starts in the middle of the track
#define DEF_vmaxSU 			1.5         	// maximal cart velocity during swingup
#define DEF_k1	 			2.0        		// ksu - swingup constat
#define DEF_k2 				3.0        		// kcw - swingup constat
#define DEF_k3 				4.0        		// kvw - swingup constat
#define DEF_k4 				0.0           	// kem - swingup constat
#define DEF_k5 				1.05          	// n - swingup constat
#define DEF_vmaxPWM 		2.5				// maximal cart velocity that can be set to PWM
#define DEF_pqref 			0.3				// lever switch positive reference [m]
#define DEF_nqref 			-DEF_pqref		// lever switch negative reference [m]
#define DEF_potL 			0.3				// potentiometer maximum reference [m]
#define DEF_SDAngle 		DEF_pi/4.0		// lower angle bounds when LQ regulator for swing down starts

/* choose type of control algorithm - comment out the unwanted */
//#define LQR_control
#define MPC_control

/* select type of QProblem to be solved (comment out both if only LQR is used)*/
#define MPC_qproblem						// bounded both input and states
//#define MPC_qproblemB						// bounded only input

#ifdef LQR_control
#define DEF_klq1 -2.949				// LQR constants
#define DEF_klq2 -3.695				// LQR constants
#define DEF_klq3 -29.369				// LQR constants
#define DEF_klq4 -4.5846				// LQR constants
#endif

#endif /* INC_USERDEFINES_H_ */
