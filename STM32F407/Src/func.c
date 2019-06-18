/*
 * func: file with user functions
 *
 * Copyright (C) 2019 Jakub Kulhanek
 *
 * MIT License
 */

#include "func.h"
#include "tinyekf_config.h"
#include "tiny_ekf.h"
#include <stdio.h>

#ifdef MPC_control
#include "MPC_setup.h"
#endif

/** system variables */
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

/** global variables definition */
long unsigned int period_count = 0;
double xhat_o[DEF_nx] = {0.0, 0.0, 0.0, 0.0};
volatile bool period_elapsed = false;
volatile bool userStart = false;
bool swingup_start = false;
bool stabilization_start = false;
bool swing_down = false;
bool swing_down_start = false;

/** local variables */
static double Ek;						// Kinetic energy of the system
static double Ep;						// Potential energy of the system
static double u_act = 0.0;				// Control input
static double u_prior = 0.0;			// Previous control input
static double v_prior = 0.0;			// Previous velocity of the cart
static double yObs[DEF_ny];				// Vector of observed parameters
static double xhat_k[DEF_nx];			// state estimate vector as calculated by EKF
static double K_sdlqr[4] = { 66.7523, 24.8604, 35.5829, -0.757 };// Swing-Down LQR gain
static double theta_prev = 0.0;			// angular position of the pendulum at previous sampling instant
static volatile double xref[DEF_nx] = { 0.0, 0.0, 0.0, 0.0 };// state reference vector
static volatile double deltaX[DEF_nx];	// current state estimate minus the reference

static volatile int encoder1, encoder2;	// variables to store the timer register values
static int count = 0;					// used to count periods during EKF calibration
static long unsigned int startSD = 0;	// tracks beginning of swing-down phase
static int middlePot = 0;				// reference value from the first ADC reading
static volatile int potRef = 0;			// stores ADC reading for reference tracking
static int potRefPrev = 0;				// stores previous ADC reading

static ekf_t ekf;						// initialization of EKF structure
static bool leverRef = false;			// flag handling reference tracking logic
static bool kalman_ok = false;			// flag handling EKF calibration
static bool MPC_init_done = false;		// flag handling MPC initialization
static bool SDCount = false;			// flag handling stabilization before swing down
static bool reached_bot = false;		// flag handling LQR activation during swing down
static bool firstMeasure = true;		// flag handling ADC initialization

#ifdef LQR_control /* LQR stabilization controller gain */
static double K_lqr[DEF_nx] = {DEF_klq1, DEF_klq2, DEF_klq3, DEF_klq4};
#endif

/* function prototypes */
static void init_kal(ekf_t * ekf);
static void model(ekf_t * ekf, double xhat[DEF_nx],
		volatile double u_kal);
double swingup(double q[DEF_nx]);
void setPWM(volatile double u_in);
static double sign(double num);


/**
  * @brief  Handles external interrupt callbacks.
  * @param  GPIO_Pin uint16_t, defines which pin triggers the interrupt.
  *
  * @note   Pin No. 15 corresponds to stop button. It is only allowed during
  * 		stabilization phase. When activated, it resets reference and LED
  * 		signals and sets swing_down flag.
  *
  * @note   Pin No. 10 corresponds to re/start button. It is only allowed once
  * 		before the system resets by the swing-down logic. It modifies
  * 		16 bit TIM3 and TIM4 registers responsible for rotary encoders.
  * 		There is a theoretical possibility of overflowing these values,
  * 		however physically the cart cannot move outside of its track and
  * 		pendulum will not rotate multiple times during the operation.
  *
  * @note   Pin No. 6 and 9 corresponds to two sides of a lever switch. They are
  * 		only allowed during stabilization phase and they change the reference
  * 		of the cart's position respectively. Furthermore, flag leverRef is
  * 		set true to prevent ADC readings for continuous change in reference.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	/* Stop button */
	if (GPIO_Pin == GPIO_PIN_15 && stabilization_start) {
		for (int i = 0; i < 4; i++) {
			xref[i] = 0.0;
		}
		swing_down = true;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	}

	/* Re/start button */
	if (GPIO_Pin == GPIO_PIN_10 && !userStart) {
		TIM3->CNT = 32765;
		TIM4->CNT = 32765;
		userStart = true;
	}

	/* Lever switch */
	if (GPIO_Pin == GPIO_PIN_6 && stabilization_start && !swing_down) {
		if (xref[0] != DEF_pqref) {
			xref[0] = DEF_pqref;
			leverRef = true;
		} else {
			xref[0] = 0.0;
			leverRef = false;
		}
	}

	if (GPIO_Pin == GPIO_PIN_9 && stabilization_start && !swing_down) {
		if (xref[0] != DEF_nqref) {
			xref[0] = DEF_nqref;
			leverRef = true;
		} else {
			xref[0] = 0.0;
			leverRef = false;
		};
	}
}


/**
  * @brief  Initialization of peripherals and software functions.
  * @param  None.
  *
  * @note   If defined, qpOASES options and problem structures are initialized.
  * 		There are predefined printout options that can be adjusted. Also,
  * 		the problem is initialized in an "MPC" setup, which can be altered.
  * 		For more information see qpOASES user manual.
  *
  * @note	Structures for extended Kalman filter are initialized.
  *
  * @note   Finally, necessary timers and GPIO pins are initialized. They handle
  * 		motor control, encoders and internal interrupt generation.
  * @retval None
  */
void systemInit() {
	/* qpOASES initialization */
#ifdef MPC_control
	qpOASES_Options_init(&options, 2);
	if (_PRINTOUT == _PRINT_HIGH) {
		options.printLevel = PL_MEDIUM;
	} else {
		options.printLevel = PL_NONE;
	}

#ifdef MPC_qproblem
	QProblem_setup(DEF_nV, DEF_nC, HST_UNKNOWN);
#endif

#ifdef MPC_qproblemB
	QProblemB_setup( DEF_nV,HST_UNKNOWN );
#endif

#endif

	/* Kalman initialization */
	ekf_init(&ekf, DEF_nx, DEF_ny);
	init_kal(&ekf);

	/* set lowest PWM frequency (around 3 pulses per minute - almost stopped) */
	TIM2->CNT = 0;
	TIM2->ARR = 2147483647;
	TIM2->CCR1 = (int) (TIM2->ARR / 2);
	/* Peripherals setup */
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);					// enable motor control
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);	// set initial direction of motor

	TIM5->CNT = 0;											// zero out interrupt timer

	HAL_TIM_Base_Start_IT(&htim5);							// start internal interrupt timer
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);				// start PWM generation for motor control
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);// position of the cart
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);// angular position of the pendulum
}


/**
  * @brief  Get positions of the cart and pendulum and user set reference.
  * @param  None.
  *
  * @note   Readings from encoders are obtained. They are converted into
  * 		meters and radians for cart's position and pendulum's angle
  * 		respectively.
  *
  * @note	First reading serves as calibration of the potentiometer, which
  * 		is assumed to be in the zero position. Following readings are then
  * 		converted into reference position of the cart. As the readings are
  * 		quite noisy, the difference between consecutive readings have to be
  * 		larger than "70" (experimentally found) in order to affect
  * 		the reference value.
  * @retval None
  */
void getObs() {
	/* Encoders readings and conversion */
	encoder1 = TIM3->CNT - 32765;
	encoder2 = TIM4->CNT - 32765;
	yObs[0] = (double) encoder1 * DEF_p2m;
	yObs[1] = DEF_pi + (double) encoder2 * (2.0 * DEF_pi / 8000.0);

	/* ADC reading and reference setting */
	if (!leverRef && !swing_down && stabilization_start) {
		if (!firstMeasure) {
			HAL_ADC_Start(&hadc1);
			if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
				potRef = HAL_ADC_GetValue(&hadc1);
				if (fabs(potRef - potRefPrev) > 70) {
					xref[0] = ((double) middlePot - (double) potRef) * DEF_potL
							/ (double) middlePot;
					if (xref[0] > DEF_pqref)
						xref[0] = DEF_pqref;
					if (xref[0] < DEF_nqref)
						xref[0] = DEF_nqref;
					potRefPrev = potRef;
				}
			}
		} else {
			firstMeasure = false;
			HAL_ADC_Start(&hadc1);
			if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
				middlePot = HAL_ADC_GetValue(&hadc1);
			}
			potRefPrev = middlePot;
		}
	}
}


/**
  * @brief  Get state estimates from EKF.
  * @param  None.
  *
  * @note   For more information on tiny_ekf see
  * 		https://github.com/simondlevy/TinyEKF
  *
  * @note	EKF works with state vector xhat_k, where angular position
  * 		of the pendulum is not bounded. The control algorithms require
  * 		that the third state is bounded between [-pi pi], which is stored
  * 		in a new state vector xhat_o.
  *
  * @note 	Reference tracking is included in deltaX.
  * @retval None
  */
void kalmanStep() {
	/* EKF estimation */
	model(&ekf, xhat_k, u_act);
	ekf_step(&ekf, yObs);

	xhat_k[0] = ekf.x[0];
	xhat_k[1] = ekf.x[1];
	xhat_k[2] = ekf.x[2];
	xhat_k[3] = ekf.x[3];

	/* bounding third state */
	xhat_o[0] = xhat_k[0];
	xhat_o[1] = xhat_k[1];
	xhat_o[2] = pmod((xhat_k[2] - DEF_pi), (2 * DEF_pi)) - DEF_pi;
	xhat_o[3] = xhat_k[3];

	/* reference tracking adjustment */
	for (int i = 0; i < DEF_nx; i++) {
		deltaX[i] = xref[i] - xhat_o[i];
	}
}


/**
  * @brief  EKF initialization.
  * @param  None.
  *
  * @note   Turns on the LED on control panel.
  *
  * @note	A small bump is introduced to the pendulum by moving
  * 		the cart back and forth. This logic counts the sampling
  * 		periods to change the acceleration of the cart. The whole
  * 		process is setup to last 2.5s.
  *
  * @note 	After a user-defined time (should be greater than 2.5s)
  * 		the pendulum has to cross the origin to enable the
  * 		swingup_start flag and panel LEDs are switched. This
  * 		provides ideal conditions for beginning of swing up.
  * @retval None
  */
void calibrateKalman() {
	/* LED signalization */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);

	/* period count */
	if (!kalman_ok && count < DEF_KalmanInit) {
		count++;
	} else {
		kalman_ok = true;
	}

	/* bump logic */
	if (!kalman_ok && count > 100 && count == 101) {
		u_act = 0.2;
	}
	if (!kalman_ok && count > 100 && count == 125) {
		u_act = -0.2;
	}
	if (!kalman_ok && count > 100 && count == 150) {
		u_act = -0.4;
	}
	if (!kalman_ok && count > 100 && count == 175) {
		u_act = 0.4;
	}
	if (!kalman_ok && count > 100 && count == 200) {
		u_act = 0.2;
	}
	if (!kalman_ok && count > 100 && count == 225) {
		u_act = -0.2;
	}

	/* aplying the control input */
	if (!kalman_ok && count > 100 && count < 250) {
		setPWM(u_act);
	}

	/* stopping the motion of the cart */
	if (!kalman_ok && count == 251) {
		TIM2->CNT = 0;
		TIM2->ARR = 2147483647;
		TIM2->CCR1 = (int) (TIM2->ARR / 2);
	}

	/* switching the LEDs and enabling next phase */
	if (kalman_ok && theta_prev > 0.0 && xhat_o[2] < 0.0
			&& !stabilization_start) {
		swingup_start = true;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
	} else {
		theta_prev = xhat_o[2];
	}
}


/**
  * @brief  Applying the swing-up control input.
  * @param  None.
  *
  * @note   Gets control input value from swingup algorithm and
  * 		applies the control input.
  * @retval None
  */
void swingupStep() {
	u_act = swingup(xhat_o);
	setPWM(u_act);
}


/**
  * @brief  Sets up the MPC problem and calculates first optimal
  * 		control input.
  * @param  None.
  *
  * @note   First call of the qpOASES solver is done using the
  * 		QProblem_init. Subsequent calls use an optimized
  * 		hotstart function.
  * @note	Control panel LED signalization is switched.
  * @retval None
  */
void controlInit() {
	/* enable stabilization flag and switch LED signals */
	stabilization_start = true;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	xref[0] = 0.0;

	/* first call of the qpOASES solver */
#ifdef MPC_control
	/* calculation of new gradient matrix */
	for (int i = 0; i < DEF_nV; i++) {
		for (int j = 0; j < DEF_nx; j++) {
			g[i] -= G[i][j] * deltaX[j];
		}
	}
	/* QProblem with both the state and the control input constraints */
#ifdef MPC_qproblem
	/* Calculation of constraint matrices with respect to reference tracking*/
	for (int i = 0; i < DEF_nC; i++) {
		for (int j = 0; j < DEF_nx; j++) {
			ubA[i] -= uB0[i][j] * deltaX[j];
			lbA[i] -= lB0[i][j] * deltaX[j];
		}
		ubA[i] += ub0[i] - xref[(int) fmod(i, DEF_nx)];
		lbA[i] += lb0[i] - xref[(int) fmod(i, DEF_nx)];
	}
	/* calling the solver */
	QProblem_init(H, g, A, lb, ub, lbA, ubA, (int_t* const ) &nWSR,
			(real_t* const ) &cputime, &options, uOpt, yOpt, &obj,
			(int_t* const ) &status);
#endif
	/* QProblem with control input constraint */
#ifdef MPC_qproblemB
	QProblemB_init( H,g,lb,ub,
			(int_t* const)&nWSR,(real_t* const)&cputime,&options,
			uOpt,yOpt,&obj,(int_t* const)&status
	);
#endif
	u_act = uOpt[0];
#endif
}


/**
  * @brief  Calculates and applies the control input.
  * @param  None.
  *
  * @note	The first call of this function skips the MPC hotstart call
  * 		as the optimal solution was already found by the init function.
  * @retval None
  */
void controlStep() {
	/* Includes possibility of LQR control */
#ifdef LQR_control
	u_act = 0.0;
	for (int i=0;i<DEF_nx;i++) {
		u_act -= K_lqr[i]*deltaX[i];
	}
	if (u_act>6.0) {u_act=6.0;}
	if (u_act<-6.0) {u_act=-6.0;}
#endif
	/* MPC control input calculation */
#ifdef MPC_control
	/* First control input is already calculated */
	if (MPC_init_done) {
		/* calculation of new gradient matrix */
		for (int i = 0; i < DEF_nV; i++) {
			g[i] = 0.0;
			for (int j = 0; j < DEF_nx; j++) {
				g[i] -= G[i][j] * deltaX[j];
			}
		}

		/* These variables serves for constraining the solver. After
		 * each call, the solver rewrites their values based on its
		 * last operation. For more information, please refer to
		 * qpOASES manual.
		 */
		nWSR = DEF_nWSR;
		cputime = DEF_cputime;

		/* QProblem with both the state and the control input constraints */
#ifdef MPC_qproblem
		/* Calculation of constraint matrices with respect to reference tracking */
		for (int i = 0; i < DEF_nC; i++) {
			ubA[i] = 0.0;
			lbA[i] = 0.0;
			for (int j = 0; j < DEF_nx; j++) {
				ubA[i] -= uB0[i][j] * deltaX[j];
				lbA[i] -= lB0[i][j] * deltaX[j];
			}
			ubA[i] += ub0[i] - xref[(int) fmod(i, DEF_nx)];
			lbA[i] += lb0[i] - xref[(int) fmod(i, DEF_nx)];
		}
		/* Calling the solver */
		QProblem_hotstart(g, lb, ub, lbA, ubA, (int_t* const ) &nWSR,
				(real_t* const ) &cputime, uOpt, yOpt, &obj,
				(int_t* const ) &status);
#endif

		/* QProblem with control input constraint */
#ifdef MPC_qproblemB
		QProblemB_hotstart( g,lb,ub,
				(int_t* const)&nWSR,(real_t* const)&cputime,
				uOpt,yOpt,&obj,(int_t* const)&status
		);
#endif
		/* Getting the optimal control input */
		u_act = uOpt[0];
	} else {
		MPC_init_done = true;
	}
#endif
	/* Applying the optimal control input */
	setPWM(u_act);
}


/**
  * @brief  Handles the console printing options
  * @param  None.
  *
  * @note	userDefines.h provides more information on the printing options
  * 		and selection.
  * @retval None
  */
void printout() {
	switch (_PRINTOUT) {
	case 0:
		break;

	case 1:
		printf("%11.4e  %11.4e  %11.4e  %11.4e  |  %11.4e  %f\n", xhat_o[0],
				xhat_o[1], xhat_o[2], xhat_o[3], u_act, xref[0]);
		break;

	case 2:
#ifdef MPC_control
		printf("%11.4e  %11.4e  %11.4e  %11.4e  |  %11.4e  | %2d  %d  |  %5d\n",
				xhat_o[0], xhat_o[1], xhat_o[2], xhat_o[3], u_act, status, nWSR,
				TIM5->CNT);
#endif
		break;

	case 3:
#ifdef MPC_control
		printf("%11.4e  %11.4e  %11.4e  %11.4e  |  %11.4e  | %2d  %d  |  %5d\n",
				xhat_o[0], xhat_o[1], xhat_o[2], xhat_o[3], u_act, status, nWSR,
				TIM5->CNT);
#endif
		break;

	case 4:
#ifdef MPC_control
		printf("%f, %f, %f, %f, %f, %d, %f, %f, %f\n", xhat_o[0], xhat_o[1],
				xhat_o[2], xhat_o[3], xref[0], nWSR, cputime, u_act, 0.01*period_count);
#endif
		break;

	default:
		break;
	}
}


/**
  * @brief  Contains swing-down logic.
  * @param  None.
  *
  * @note	Once the Stop button is pressed, reference tracking is disabled and
  * 		reference position of the cart is set to zero. After the stabilization
  * 		algorithm brings the pendulum within 3 cm of the zero reference,
  * 		a countdown of 2.5sec is started. This is enough time for the system
  * 		to stabilize.
  * 		After the countdown is finished the stabilization is disabled and
  * 		motor is set to zero minimum velocity (practically zero). When the falling
  * 		pendulum crosses an angle of pi/18 the modified swing-up logic is applied
  * 		to drain the energy from the system.
  * 		When the total energy of the system is less than experimentally found value
  * 		of -0.32, an LQR control input is applied to bring the system to its original
  * 		resting position. This is determined by comparing the quadratic norm of the
  * 		state vector to an experimentally found value of 0.00006. Finally the motor
  * 		is stopped and an internal reset of the system is called.
  * @note	Internal reset of the system reverts values of all registers to their initial
  * 		values except for the timer registers.
  * @retval None
  */
void swingDown() {
	/* checking the position of the cart and enabling the countdown */
	if (fabs(yObs[0]) < 0.03 && !SDCount) {
		startSD = period_count;
		SDCount = true;
	}
	/* After the countdown motor is stopped, control input integration parameters are
	 * set to zero and swing_down_start is enabled to proceed to the next phase
	 */
	if (SDCount && (period_count - startSD) > 250 && !swing_down_start) {
		TIM2->CNT = 0;
		TIM2->ARR = 2147483647;
		TIM2->CCR1 = (int) (TIM2->ARR / 2);
		swing_down_start = true;

		v_prior = 0.0;
		u_prior = 0.0;
	}
	/* Checks if the pendulum started falling and then applies modified swing-up control input */
	if (swing_down_start && fabs(xhat_o[2]) > DEF_pi / 18 && !reached_bot) {
		double u1, u2, u3, log_l, log_vmax;
		/* calculation of arguments used in logarithms */
		log_l = 1 - fabs(xhat_o[0]) / DEF_L;
		log_vmax = 1 - fabs(xhat_o[1]) / DEF_vmaxSU;
		/* if agruments are negative motor is stopped and system goes into infinite loop */
		if (log_l < 0 || log_vmax < 0) {
			TIM2->CNT = 0;
			TIM2->ARR = 2147483647;
			TIM2->CCR1 = (int) (TIM2->ARR / 2);
			printf("logarithm argument is negative\n");
			while (1) {
			}
		}
		/* calculation of control input from modified swing-up algorithm */
		Ek = DEF_m2 * DEF_l * DEF_l / 2 * xhat_o[3] * xhat_o[3];
		Ep = DEF_m2 * DEF_g * DEF_lg * cos(xhat_o[2]);
		u1 = DEF_k1 * sign(xhat_o[3] * cos(xhat_o[2]));
		u2 = DEF_k2 * sign(xhat_o[0]) * log10(log_l);
		u3 = DEF_k3 * sign(xhat_o[1]) * log10(log_vmax);
		u_act = u1 + u2 + u3;
		/* applying the control input */
		setPWM(u_act);
	}
	/* checking the energy level of the system to switch to LQR */
	if (Ep + Ek < -0.32) {
		reached_bot = true;
	}
	/* final phase of swing-down; LQR control and system reset */
	if (reached_bot) {
		u_act = 0.0;
		/* Modification of the angular position of the pendulum for LQR control.
		 * The controller was designed based on a model linearized around bottom
		 * equillibrium position of the system.
		 */
		double xnorm = 0.0;
		if (xhat_o[2] < 0.0) {
			deltaX[2] = -(xhat_o[2] + DEF_pi);
		} else if (xhat_o[2] == 0.0) {
			deltaX[2] = 0.0;
		} else {
			deltaX[2] = -(xhat_o[2] - DEF_pi);
		}
		/* calculation of the control input and the quadratic norm of the state vector */
		for (int i = 0; i < DEF_nx; i++) {
			u_act += K_sdlqr[i] * deltaX[i];
			xnorm += deltaX[i] * deltaX[i];
		}
		/* saturation of the control input */
		if (u_act > 6.0) {
			u_act = 6.0;
		}
		if (u_act < -6.0) {
			u_act = -6.0;
		}
		/* applying the control input */
		setPWM(u_act);
		/* checking is the system is at rest, then stopping the motor and calling internal reset */
		if (xnorm < 0.00006) {
			TIM2->CNT = 0;
			TIM2->ARR = 2147483647;
			TIM2->CCR1 = (int) (TIM2->ARR / 2);

			NVIC_SystemReset();
		}
	}
}


/**
  * @brief  mod function with equivalent functionality as in MATLAB
  * @param  dividend double, number to be divided.
  *
  * @param  divisor double, number that will divide.
  *
  * @note	Used to limit the angular position estimate of the pendulum
  * 		within interval of [-pi pi].
  * @retval Positive complement after the division.
  */
double pmod(double dividend, double divisor) {
	if (dividend > 0.0) {
		return fmod(dividend, divisor);
	} else if (dividend == 0.0) {
		return 0.0;
	} else {
		return (divisor + fmod(dividend, divisor));
	}
}


/**
  * @brief  function required by the tiny_ekf implementation
  * @param  ekf struct, internal structure used by the library.
  *
  * @note	User has to input the initial values of parameters for
  * 		the extended Kalman filter.
  * @retval None
  */
static void init_kal(ekf_t * ekf) {
	/* setting the initial position (assuming it is the bottom equillibrium) */
	xhat_k[0] = 0.0;
	xhat_k[1] = 0.0;
	xhat_k[2] = DEF_pi;
	xhat_k[3] = 0.0;

	ekf->x[0] = 0.0;
	ekf->x[1] = 0.0;
	ekf->x[2] = DEF_pi;
	ekf->x[3] = 0.0;
	/* defining covariance matrices */
	ekf->Q[0][0] = 1.0e-004;
	ekf->Q[1][1] = 1.0e-004;
	ekf->Q[2][2] = 1.0e-005;
	ekf->Q[3][3] = 1.0e-003;

	double P0 = 1.0e-006;
	int i;
	for (i = 0; i < DEF_nx; ++i) {
		ekf->P[i][i] = P0;
	}

	ekf->R[0][0] = 1.0e-012;
	ekf->R[1][1] = 1.0e-006;
}


/**
  * @brief  function required by the tiny_ekf implementation
  * @param  ekf struct, internal structure used by the library.
  *
  *	@param 	xhat[DEF_nx] double, estimated state vector from
  *			the previous sampling moment.
  *
  *	@param 	u_kal double, control input from the previous sampling
  *			moment.
  *
  * @note	User has to define the state-space equations with their
  * 		Jacobians respectively.
  * @retval None
  */
static void model(ekf_t * ekf, double xhat[DEF_nx],
		double u_kal) {
	/* definition of the discrete f(x), by integration of the continuous model */
	volatile double xdot[DEF_nx];
	xdot[0] = xhat[1];
	xdot[1] = u_kal;
	xdot[2] = xhat[3];
	xdot[3] = -DEF_b / DEF_lg * xhat[3] - u_kal / DEF_lg * cos(xhat[2])
			+ DEF_g / DEF_lg * sin(xhat[2]);
	for (int i = 0; i < DEF_nx; i++) {
		xhat[i] += DEF_Treg * xdot[i];
		ekf->fx[i] = xhat[i];
	}

	/* definition of the discrete F (Jacobian calculated using MATLAB) */
	ekf->F[0][0] = 1.0;
	ekf->F[0][1] = DEF_Treg;
	ekf->F[1][1] = 1.0;
	ekf->F[2][2] = 1.0;
	ekf->F[2][3] = DEF_Treg;
	ekf->F[3][2] = (u_kal / DEF_lg * sin(xhat[2])
			+ DEF_g / DEF_lg * cos(xhat[2])) * DEF_Treg;
	ekf->F[3][3] = 1.0 - DEF_b * DEF_Treg / DEF_lg;

	/* definition of the discrete h(x) */
	ekf->hx[0] = xhat[0];
	ekf->hx[1] = xhat[2];

	/* definition of the discrete H (Jacobian calculated using MATLAB) */
	ekf->H[0][0] = 1.0;
	ekf->H[1][2] = 1.0;
}


/**
  * @brief  implementation of signum function
  * @param  num double, number to be tested for its sign.
  *
  * @retval 1.0 for positive argument, -1.0 for negative argument else it is 0.0
  */
static double sign(double num) {
	if (num > 0)
		return 1.0;
	if (num < 0)
		return -1.0;
	return 0.0;
}


/**
  * @brief  function to calculate the swing-up control input
  *	@param 	q[DEF_nx] double, requires the state vector.
  *
  * @note	User has to define the state-space equations with their
  * 		Jacobians respectively.
  * @retval Swing-up control input.
  */
double swingup(double q[DEF_nx]) {
	double u_swingUp, u1, u2, u3, u4, log_l, log_vmax;
	/* calculation of arguments used in logarithms */
	log_l = 1 - fabs(q[0]) / DEF_L;
	log_vmax = 1 - fabs(q[1]) / DEF_vmaxSU;
	/* if agruments are negative motor is stopped and system goes into infinite loop */
	if (log_l < 0 || log_vmax < 0) {
		TIM2->CNT = 0;
		TIM2->ARR = 2147483647;
		TIM2->CCR1 = (int) (TIM2->ARR / 2);
		printf("logarithm argument is negative\n");
		while (1) {
		}
	}
	/* calculation of control input from energy-based swing-up algorithm */
	Ek = DEF_m2 * DEF_l * DEF_l / 2 * q[3] * q[3];
	Ep = DEF_m2 * DEF_g * DEF_lg * cos(q[2]);

	u1 = -DEF_k1 * sign(q[3] * cos(q[2]));
	u2 = DEF_k2 * sign(q[0]) * log10(log_l);
	u3 = DEF_k3 * sign(q[1]) * log10(log_vmax);
	u4 = DEF_k4 * sign(q[3] * cos(q[2])) * sign(Ek + Ep - DEF_Eup)
			* (exp(Ek + Ep - DEF_k5 * DEF_Eup) - 1);
	u_swingUp = u1 + u2 + u3 + u4;
	return u_swingUp;
}


/**
  * @brief  calculates velocity of the cart and applying correct PWM signal
  *	@param 	u_pwm double, control input to the system.
  *
  * @note	This function modifies necessary registers of the TIM2 responsible
  * 		for PWM generation to control the stepper motor.
  * @note	It also contains safety constraining limits on maximal velocity
  * 		that can be applied. User can modify these in the userDefines.h.
  * @note	Calculated velocity of the cart is inversely proportional to the
  * 		value written to the timer register. Small velocities
  * 		could cause overflow and therefore those cases need to be checked
  * 		for.
  * @retval None.
  */
void setPWM(double u_pwm) {
	double pwm_counter, v_pwm;
	/* integration of the control input to calculate the velocity of the cart */
	v_pwm = v_prior + (u_prior + u_pwm) * DEF_Treg / 2.0;
	u_prior = u_pwm;
	v_prior = v_pwm;
	/* changing the appropriate direction of rotation of the stepper motor */
	if (v_pwm <= 0.0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	}
	/* modification of TIM2 registers */
	if (fabs(v_pwm) > 1.0e-006)	// check for register overflow
			{
		/* check if the velocity does not exceed maximal allowed value */
		if (fabs(v_pwm) < DEF_vmaxPWM)
		{
			pwm_counter = 84000000 * 2 * DEF_pi * DEF_r / (fabs(v_pwm) * 10000);
			TIM2->CNT = 0;
			TIM2->ARR = (int) pwm_counter;
			TIM2->CCR1 = (int) (TIM2->ARR / 2);
		} else
		{
			/* set register to minimal value */
			TIM2->CNT = 0;
			TIM2->ARR = (int) (84000000 * 2 * DEF_pi * DEF_r
					/ (DEF_vmaxPWM * 10000));
			TIM2->CCR1 = (int) (TIM2->ARR / 2);
		}
	} else
	{
		/* set register to maximal value */
		TIM2->CNT = 0;
		TIM2->ARR = 2147483647;
		TIM2->CCR1 = (int) (TIM2->ARR / 2);
	}
}
