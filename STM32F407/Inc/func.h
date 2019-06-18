/*
 * func.h: file with user functions
 *
 * Copyright (C) 2019 Jakub Kulhanek
 *
 * MIT Licence
 */

#ifndef INC_FUNC_H_
#define INC_FUNC_H_

#include "userDefines.h"
#include <stdbool.h>
#include <math.h>
#include "stm32f4xx_hal.h"

/** system variables */
extern ADC_HandleTypeDef hadc1;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;

/** extern variables */
extern long unsigned int period_count;		// used too keep count of elapsed periods
extern volatile bool period_elapsed;		// flag used to enable and track main loop
extern bool swingup_start;					// flag used for switching between loop functions
extern bool stabilization_start;			// flag used for switching between loop functions
extern volatile bool userStart;				// flag used for switching between loop functions
extern bool swing_down;						// flag used for switching between loop functions
extern bool swing_down_start;				// flag used for switching between loop functions
extern double xhat_o[DEF_nx];				// state estimates vector with bounded angular position

/** extern functions */
void systemInit();
void getObs();
void kalmanStep();
void calibrateKalman();
void swingupStep();
void controlInit();
void controlStep();
void printout();
void swingDown();
double pmod(double dividend, double divisor);

#endif /* INC_FUNC_H_ */
