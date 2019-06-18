/*
 * MPC_setup: Generated file with necessary variables for MPC control
 *
 * Copyright (C) 2019 Jakub Kulhanek
 *
 * MIT Licence
 */


#ifndef MPC_setup_H    /* Include guard */
#define MPC_setup_H

#include <userDefines.h>
#include <qpOASES_wrapper.h>


/**
  * Initializes QPOases intput arguments and sets up the problem
  * 
  * constants:
        H -> Hessian matrix
        G -> Gradient matrix to be multiplied by state
        g -> Zero out gradient matrix
        lb -> lower input bounds matrix
        ub -> upper input bounds matrix
        
        If defined QProblem with state bounds
        following equation is used
        lb0 + lB0 * x <= A * u <= ub0 + uB0 * x
        where:
        A -> input multiplication matrix
        lb0 -> lower state bounds matrix
        ub0 -> upper state bounds matrix
        lB0 -> to be multiplied by state
        uB0 -> to be multiplied by state
  *
  */

#ifdef MPC_control

#define DEF_nV 5    /** number of variables (np) */
#define DEF_nC 20    /** number of constraints (length of lbA) */
#define DEF_nWSR 20    /** maximum number of working set recalculations */
#define DEF_cputime 0.007000    /** maximum time allocated for qpOASES [sec] */


extern real_t H[5*5];
extern float G[5][4];
extern real_t g[5];
extern real_t lb[5];
extern real_t ub[5];

#ifdef MPC_qproblem
extern real_t A[20*5];
extern real_t lb0[20];
extern real_t ub0[20];
extern float lB0[20][4];
extern float uB0[20][4];
extern real_t lbA[20];
extern real_t ubA[20];
#endif    /** MPC_qproblem */

extern int nWSR;
extern real_t cputime;

extern real_t uOpt[DEF_nV];
extern real_t yOpt[DEF_nV+DEF_nC];
extern real_t obj;
extern int status;

extern qpOASES_Options options;

#endif    /** MPC_control */

#endif    /** MPC_setup */
