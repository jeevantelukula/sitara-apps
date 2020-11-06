/*
 * Copyright (C) 2017-2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MULTI_AXIS_MASTER_CTRL_H
#define MULTI_AXIS_MASTER_CTRL_H

#include "pi_cntl.h"		/* Include header for the PI  object */
#include "rmp_cntl.h"		/* Include header for the RMPCNTL object */
#include "pid_cntl_grando.h"
#include "hw_types.h"

#include "fcl_enum.h"

#include "motor_ctrl_user.h"
#include "multi_axis_fsi_shared.h"

/* Node number */
#define SYS_NODE_NUM        FSI_NODE_NUM

#define POS_BUF_NUM         4
#define POS_PTR_MAX         4
#define POS_CNTR_MAX        1000

/* Enumeration for controlled node */
typedef enum
{
    SYS_NODE1  = 0,
    SYS_NODE2  = 1,
    SYS_NODE3  = 2,
    SYS_NODE4  = 3,
    SYS_NODE5  = 4,
    SYS_NODE6  = 5,
    SYS_NODE7  = 6,
    SYS_NODE8  = 7
} SysNode_e;

#if(SPD_CNTLR == SPD_PID_CNTLR)
/* Default values for controller variables */
#define CTRL_DEFAULTS  {                                                       \
    {2.5, -2.5, 3.5, -3.5},            /*posArray[POS_BUF_NUM] */              \
    0.001,                             /* posSlewRate */                       \
    (float32_t)M_BASE_FREQ,            /* baseFreq */                          \
                                                                               \
    0.0,                               /* IdRefStart */                        \
    0.0,                               /* IqRefStart */                        \
    0.0,                               /* ctrlIdRef */                         \
    0.0,                               /* ctrlIqRef */                         \
    0.0,                               /* ctrlSpeedRef */                      \
    0.0,                               /* ctrlPosRef */                        \
                                                                               \
    0.0,                               /* ctrlSpdOut */                        \
    0.0,                               /* ctrlPosOut */                        \
    1.0,                               /* ctrlSpdMaxOut */                     \
    0.1,                               /* ctrlSpdMinOut */                     \
    1.0,                               /* ctrlPosMaxOut */                     \
                                                                               \
    0.0,                               /* IdRefSet */                          \
    0.0,                               /* IqRefSet */                          \
                                                                               \
    0.0,                               /* IdRef */                             \
    0.05,                              /* IqRef */                             \
                                                                               \
    0.1,                               /* speedSet */                          \
    0.2,                               /* positionSet */                       \
                                                                               \
    0.1,                               /* speedRef */                          \
    0.0,                               /* positionRef */                       \
                                                                               \
    0.0,                               /* posElecTheta */                      \
    0.0,                               /* posMechTheta */                      \
    0.0,                               /* speedWe */                           \
    0.0,                               /* speedMech */                         \
    0.0,                               /* torque */                            \
                                                                               \
    0.0,                               /* speedWePrev */                       \
    0.0,                               /* posMechThetaPrev */                  \
    0.0,                               /* speedWeError */                      \
    0.0,                               /* posMechThetaError */                 \
    0.0,                               /* speedWeDelta */                      \
    0.0,                               /* posMechThetaDelta */                 \
                                                                               \
    0.0,                               /* Kp_Id; */                            \
    0.0,                               /* Ki_Id; */                            \
    0.0,                               /* Kp_Iq; */                            \
    0.0,                               /* Ki_Iq; */                            \
                                                                               \
    0.0,                               /* Umax_Id; */                          \
    0.0,                               /* Umin_Id; */                          \
    0.0,                               /* Umax_Iq; */                          \
    0.0,                               /* Umin_Iq;     */                      \
                                                                               \
    6.0,                               /* curLimit */                          \
                                                                               \
    RMPCNTL_DEFAULTS,                  /* rc */                                \
                                                                               \
    {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS},  /* pid_spd */ \
    PI_CONTROLLER_DEFAULTS,            /* pi_pos */                            \
                                                                               \
    POS_CNTR_MAX,                      /* posRampMax */                        \
    0,                                 /* posRampCntr */                       \
    POS_PTR_MAX,                       /* posBufMax */                         \
    0,                                 /* posBufPtr */                         \
                                                                               \
    0,                                 /* faultFlag */                         \
    0,                                 /* runState */                          \
    0,                                 /* fsiState */                          \
    CTRL_MODE_STOP,                    /* ctrlModeSet */                       \
    CTRL_MODE_STOP,                    /* ctrlModeCom */                       \
    CTRL_STOP,                         /* ctrlSateSet */                       \
    CTRL_STOP,                         /* ctrlSateCom */                       \
    CTRL_STOP                          /* ctrlSateFdb */                       \
}
#endif  /* (SPD_CNTLR == SPD_PID_CNTLR) */

/* Default values for system variables */
#define SYS_DEFAULTS  {                                                        \
    0,                                  /* isrTicker */                        \
                                                                               \
    0.1,                                /* speedSet */                         \
    0.2,                                /* positionSet */                      \
                                                                               \
    0,                                  /* focExecutionTime_us */              \
    0,                                  /* focClrCntr */                       \
    0,                                  /* focCycleCountMax */                 \
    0,                                  /* focCycleCount */                    \
                                                                               \
    0,                                  /* peedLoopPrescaler */                \
    0,                                  /* speedLoopCount */                   \
                                                                               \
    SYS_NODE1,                          /* dacNode */                          \
                                                                               \
    SYS_NODE1,                          /* fsiNode */                          \
    SYS_NODE1,                          /* fsiNodePrev */                      \
                                                                               \
    SYS_NODE1,                          /* ecatNode */                         \
                                                                               \
    SYS_NODE1,                          /* ctrlNode */                         \
    SYS_NODE1,                          /* ctrlNodePrev */                     \
                                                                               \
    CTRL_MODE_STOP,                     /* ctrlModeSet */                      \
    CTRL_STOP,                          /* ctrlStateSet */                     \
    CTRL_SYN_DISABLE,                   /* ctrlSynSet */                       \
    ECAT_CTRL_ENABLE                    /* ecatCtrlSet */                      \
}

/* Typedefs for ctrlVars */
typedef struct _CTRL_Vars_t_
{
    float32_t posArray[POS_BUF_NUM];
    float32_t posSlewRate;
    float32_t baseFreq;

    float32_t IdRefStart;           /* Id reference (pu) for startup */
    float32_t IqRefStart;           /* Iq reference (pu) for startup */
    float32_t ctrlIdRef;            /* Id reference (pu) to controller */
    float32_t ctrlIqRef;            /* Iq reference (pu) to controller */
    float32_t ctrlSpeedRef;         /* Speed reference (pu) to controller */
    float32_t ctrlPosRef;           /* Position reference (pu) to controller */

    float32_t ctrlSpdOut;           /* the output of speed controller */
    float32_t ctrlPosOut;           /* the output of position controller */
    float32_t ctrlSpdMaxOut;        /* the maximum output of speed controller */
    float32_t ctrlSpdMinOut;        /* the minimum output of speed controller */
    float32_t ctrlPosMaxOut;        /* the maximum output of position controller */

    float32_t IdRefSet;             /* Id reference setting (pu) */
    float32_t IqRefSet;             /* Iq reference setting (pu) */

    float32_t IdRef;                /* Id reference (pu) */
    float32_t IqRef;                /* Iq reference (pu) */

    float32_t speedSet;             /* For Closed Loop tests */
    float32_t positionSet;          /* For Position Loop tests */

    float32_t speedRef;             /* speed reference Closed Loop (pu) */
    float32_t positionRef;          /* speed reference Position Loop (pu) */

    float32_t posElecTheta;
    float32_t posMechTheta;
    float32_t speedWe;
    float32_t speedMech;
    float32_t torque;

    float32_t speedWePrev;
    float32_t posMechThetaPrev;
    float32_t speedWeError;
    float32_t posMechThetaError;
    float32_t speedWeDelta;
    float32_t posMechThetaDelta;

    float32_t Kp_Id;
    float32_t Ki_Id;

    float32_t Kp_Iq;
    float32_t Ki_Iq;

    float32_t Umax_Id;
    float32_t Umin_Id;

    float32_t Umax_Iq;
    float32_t Umin_Iq;

    float32_t curLimit;

    RMPCNTL rc;                      /* Ramp control */

    PID_CONTROLLER  pid_spd;
    PI_CONTROLLER   pi_pos;

    uint16_t posRampMax;
    uint16_t posRampCntr;
    uint16_t posBufMax;
    uint16_t posBufPtr;

    uint16_t faultFlag;
    uint16_t runState;
    uint16_t fsiState;
    CtrlMode_e  ctrlModeSet;
    CtrlMode_e  ctrlModeCom;
    CtrlState_e ctrlStateSet;
    CtrlState_e ctrlStateCom;
    CtrlState_e ctrlStateFdb;

} CTRL_Vars_t;

/* Typedefs for ctrlVars */
typedef struct _SYS_Vars_t_
{
    uint32_t isrTicker;

    float32_t speedSet;                 /* For Closed Loop tests */
    float32_t positionSet;              /* For Position Loop tests */

    float32_t focExecutionTime_us;      /* FOC execution time since sampling */
    uint16_t  focClrCntr;
    uint16_t  focCycleCountMax;
    uint16_t  focCycleCount;            /* FOC execution time variable */

    uint16_t  speedLoopPrescaler;       /* Speed loop pre scalar */
    uint16_t  speedLoopCount;           /* Speed loop counter */

    SysNode_e  dacNode;

    SysNode_e fsiNode;
    SysNode_e fsiNodePrev;

    SysNode_e ecatNode;

    SysNode_e ctrlNode;
    SysNode_e ctrlNodePrev;

    CtrlMode_e  ctrlModeSet;
    CtrlState_e ctrlStateSet;
    CtrlSync_e  ctrlSynSet;
    EcatCtrl_e  ecatCtrlSet;
} SYS_Vars_t;

/* Global variables used in this system */
extern CTRL_Vars_t ctrlVars[SYS_NODE_NUM];
extern SYS_Vars_t  sysVars;

/* Initializes the parameters of system */
extern void initSysParameters(SYS_Vars_t *pSys);

/* Initializes the parameters of controller */
extern void initCtrlParameters(CTRL_Vars_t *pCtrl);

/* Reset the control variables of motor */
extern void resetControllerVars(CTRL_Vars_t *pCtrl);

/* Build level function prototypes */
extern void buildLevel7_speedLoop(void);
extern void buildLevel7_ctrlStateMachine(void);
extern void buildLevel8_positionLoop(void);
extern void buildLevel8_ctrlStateMachine(void);
extern void runController(SysNode_e node);

#endif /* end of MULTI_AXIS_MASTER_CTRL_H definition */
