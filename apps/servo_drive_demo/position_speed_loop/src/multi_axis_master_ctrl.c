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
 
//
// includes
//
#include <ti/csl/tistdtypes.h>
#include <ti/csl/hw_types.h>
#include "multi_axis_fsi_shared.h"
#include "multi_axis_master_ctrl.h"

//
// Global variables used in this system
//
CTRL_Vars_t ctrlVars[SYS_NODE_NUM];
SYS_Vars_t  sysVars;


//
//   Various Incremental Build levels
//
static inline void switchActiveNode(void)
{
    switch(sysVars.ctrlNode)
    {
        case SYS_NODEM:
            sysVars.ctrlNode = SYS_NODE1;
            break;

        case SYS_NODE1:
            sysVars.ctrlNode = SYS_NODE2;
            break;
        case SYS_NODE2:
            sysVars.ctrlNode = SYS_NODE3;

            break;
        case SYS_NODE3:
            sysVars.ctrlNode = SYS_NODE4;

            break;
        case SYS_NODE4:
            sysVars.ctrlNode = SYS_NODEM;

            break;
        default:
            sysVars.ctrlNode = SYS_NODE1;
    }
    return;
}

//****************************************************************************
// INCRBUILD 7
//****************************************************************************
#if((BUILDLEVEL == FCL_LEVEL7))
inline void buildLevel7(void)
{
#if(SPD_CNTLR == SPD_PID_CNTLR)
    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        //
        //  Connect inputs of the RMP module and call the ramp control module
        //
        fclRampControl(&ctrlVars[sysVars.ctrlNode].rc);

        ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue;

        ctrlVars[sysVars.ctrlNode].pid_spd.term.Ref =
                ctrlVars[sysVars.ctrlNode].ctrlSpeedRef;

        ctrlVars[sysVars.ctrlNode].pid_spd.term.Fbk =
                ctrlVars[sysVars.ctrlNode].speedWe;

        if(ctrlVars[sysVars.ctrlNode].ctrlSpeedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin = 0.0;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax = 0.0;

            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin =
                    -ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
        }

        runPID(&ctrlVars[sysVars.ctrlNode].pid_spd);

        ctrlVars[sysVars.ctrlNode].ctrlSpdOut =
                ctrlVars[sysVars.ctrlNode].pid_spd.term.Out;

        sysVars.speedLoopCount = 0;
    }

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
    {
        ctrlVars[sysVars.ctrlNode].pid_spd.data.d1 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.d2 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.i1 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.ud = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.ui = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.up = 0;
    }
#endif  // (SPD_CNTLR == SPD_PID_CNTLR)

#if(SPD_CNTLR == SPD_DCL_CNTLR)
    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_RUN)
        {
            ctrlVars[sysVars.ctrlNode].rc.TargetValue =
                    ctrlVars[sysVars.ctrlNode].speedRef;
        }

        //
        //  Connect inputs of the RMP module and call the ramp control module
        //
        fclRampControl(&ctrlVars[sysVars.ctrlNode].rc);

        ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue;

        if(ctrlVars[sysVars.ctrlNode].ctrlSpeedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin = 0.0;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax = 0.0;

            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin =
                    -ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
        }

        if(ctrlVars[sysVars.ctrlNode].ctrlSpeedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umin = 0.0;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umax = 0.0;

            ctrlVars[sysVars.ctrlNode].dcl_spd.Umin =
                    -ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
        }

        ctrlVars[sysVars.ctrlNode].ctrlSpdOut =
                DCL_runPI_C1(&ctrlVars[sysVars.ctrlNode].dcl_spd,
                             ctrlVars[sysVars.ctrlNode].ctrlSpeedRef,
                             ctrlVars[sysVars.ctrlNode].speedWe);

        sysVars.speedLoopCount = 0;
    }

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
    {
        DCL_resetPI(&ctrlVars[sysVars.ctrlNode].dcl_spd);
    }

#endif  // (SPD_CNTLR == SPD_DCL_CNTLR)

    ctrlVars[sysVars.ctrlNode].ctrlStateCom =
            ctrlVars[sysVars.ctrlNode].ctrlStateSet;

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_RUN)
    {
        //#####BEGIN_INTERNAL#####
        #ifdef TEST_ENABLE
        GPIO_writePin(TGIO_MOTORS_NUM, 1);

        if(sysVars.ctrlNode == SYS_NODE1)
        {
            GPIO_writePin(TGIO_MOTOR1_NUM, 1);
        }
        #endif // TEST_ENABLE
        //#####END_INTERNAL#####

        ctrlVars[sysVars.ctrlNode].rc.TargetValue =
                ctrlVars[sysVars.ctrlNode].speedRef;

        ctrlVars[sysVars.ctrlNode].IdRef =
                ctrlVars[sysVars.ctrlNode].ctrlIdRef;

        ctrlVars[sysVars.ctrlNode].IqRef =
                ctrlVars[sysVars.ctrlNode].ctrlSpdOut;
    }
    else if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_FAULT)
    {
        //#####BEGIN_INTERNAL#####
        #ifdef TEST_ENABLE
        GPIO_writePin(TGIO_MOTORS_NUM, 0);

        if(sysVars.ctrlNode == SYS_NODE1)
        {
            GPIO_writePin(TGIO_MOTOR1_NUM, 0);
        }
        #endif // TEST_ENABLE
        //#####END_INTERNAL#####

        ctrlVars[sysVars.ctrlNode].IdRef = 0.0;
        ctrlVars[sysVars.ctrlNode].IqRef = 0.0;
        ctrlVars[sysVars.ctrlNode].ctrlModeCom = CTRL_MODE_FAULT;
    }
    else if(ctrlVars[sysVars.ctrlNode].ctrlStateCom == CTRL_RUN)
    {
        //#####BEGIN_INTERNAL#####
        #ifdef TEST_ENABLE
        GPIO_writePin(TGIO_MOTORS_NUM, 1);

        if(sysVars.ctrlNode == SYS_NODE1)
        {
            GPIO_writePin(TGIO_MOTOR1_NUM, 1);
        }
        #endif // TEST_ENABLE
        //#####END_INTERNAL#####

        ctrlVars[sysVars.ctrlNode].rc.TargetValue = 0.0;

        ctrlVars[sysVars.ctrlNode].IdRef =
                ctrlVars[sysVars.ctrlNode].IdRefStart;

        if(ctrlVars[sysVars.ctrlNode].speedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].IqRef =
                    ctrlVars[sysVars.ctrlNode].IqRefStart;
        }
        else if(ctrlVars[sysVars.ctrlNode].speedRef < 0.0)
        {
            ctrlVars[sysVars.ctrlNode].IqRef =
                    -ctrlVars[sysVars.ctrlNode].IqRefStart;
        }
    }
    else
    {
        //#####BEGIN_INTERNAL#####
        #ifdef TEST_ENABLE
        GPIO_writePin(TGIO_MOTORS_NUM, 0);

        if(sysVars.ctrlNode == SYS_NODE1)
        {
            GPIO_writePin(TGIO_MOTOR1_NUM, 0);
        }
        #endif // TEST_ENABLE
        //#####END_INTERNAL#####

        ctrlVars[sysVars.ctrlNode].ctrlModeCom = CTRL_MODE_STOP;
    }

    return;
}

#endif // ((BUILDLEVEL==FCL_LEVEL7))


//****************************************************************************
// INCRBUILD 8
//****************************************************************************
// TODO: buildLevel8()
#if(BUILDLEVEL == FCL_LEVEL8)
// FCL_LEVEL8: verifies the position control
// build level 8 subroutine for motor for all slave nodes (node1~4)
void buildLevel8(void)
{
#if(SPD_CNTLR == SPD_PID_CNTLR)
    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
        {
            ctrlVars[sysVars.ctrlNode].rc.SetpointValue =
                    ctrlVars[sysVars.ctrlNode].posMechTheta;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].rc.TargetValue =
                    refPosGen(ctrlVars[sysVars.ctrlNode].rc.TargetValue,
                              &ctrlVars[sysVars.ctrlNode]);

            ctrlVars[sysVars.ctrlNode].rc.SetpointValue =
                ctrlVars[sysVars.ctrlNode].rc.TargetValue -
                (float32_t)((int32_t)ctrlVars[sysVars.ctrlNode].rc.TargetValue);

            // Rolling in angle within 0 to 1pu
            if(ctrlVars[sysVars.ctrlNode].rc.SetpointValue < 0)
            {
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue += 1.0;
            }
        }

        ctrlVars[sysVars.ctrlNode].ctrlPosRef =
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue;
        ctrlVars[sysVars.ctrlNode].pi_pos.Ref =
                ctrlVars[sysVars.ctrlNode].ctrlPosRef;
        ctrlVars[sysVars.ctrlNode].pi_pos.Fbk =
                ctrlVars[sysVars.ctrlNode].posMechTheta;

        runPIPos(&ctrlVars[sysVars.ctrlNode].pi_pos);

        ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                ctrlVars[sysVars.ctrlNode].pi_pos.Out;

        ctrlVars[sysVars.ctrlNode].pid_spd.term.Ref =
                ctrlVars[sysVars.ctrlNode].ctrlSpeedRef;

        ctrlVars[sysVars.ctrlNode].pid_spd.term.Fbk =
                ctrlVars[sysVars.ctrlNode].speedWe;

        if(ctrlVars[sysVars.ctrlNode].ctrlSpeedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin = 0.0;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax = 0.0;

            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin =
                    -ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
        }

        runPID(&ctrlVars[sysVars.ctrlNode].pid_spd);

        ctrlVars[sysVars.ctrlNode].ctrlSpdOut =
                ctrlVars[sysVars.ctrlNode].pid_spd.term.Out;

        sysVars.speedLoopCount = 0;
    }

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
    {
        ctrlVars[sysVars.ctrlNode].pid_spd.data.d1 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.d2 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.i1 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.ud = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.ui = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.up = 0;

        ctrlVars[sysVars.ctrlNode].pi_pos.ui = 0;
        ctrlVars[sysVars.ctrlNode].pi_pos.i1 = 0;

    }

#endif  // (SPD_CNTLR == SPD_PID_CNTLR)

#if(SPD_CNTLR == SPD_DCL_CNTLR)
    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
        {
            ctrlVars[sysVars.ctrlNode].rc.SetpointValue =
                    ctrlVars[sysVars.ctrlNode].posMechTheta;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].rc.TargetValue =
                refPosGen(ctrlVars[sysVars.ctrlNode].rc.TargetValue,
                          &ctrlVars[sysVars.ctrlNode]);

            ctrlVars[sysVars.ctrlNode].rc.SetpointValue =
                ctrlVars[sysVars.ctrlNode].rc.TargetValue -
                (float32_t)((int32_t)ctrlVars[sysVars.ctrlNode].rc.TargetValue);

            // Rolling in angle within 0.0 to 1.0pu
            if(ctrlVars[sysVars.ctrlNode].rc.SetpointValue < 0.0)
            {
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue += 1.0;
            }
        }

        ctrlVars[sysVars.ctrlNode].ctrlPosRef =
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue;

        ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                DCL_runPI_C1(&ctrlVars[sysVars.ctrlNode].dcl_pos,
                             ctrlVars[sysVars.ctrlNode].ctrlPosRef,
                             ctrlVars[sysVars.ctrlNode].posMechTheta);

        if(ctrlVars[sysVars.ctrlNode].ctrlSpeedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umin = 0.0;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umax = 0.0;

            ctrlVars[sysVars.ctrlNode].dcl_spd.Umin =
                    -ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
        }

        ctrlVars[sysVars.ctrlNode].ctrlSpdOut =
                DCL_runPI_C1(&ctrlVars[sysVars.ctrlNode].dcl_spd,
                             ctrlVars[sysVars.ctrlNode].ctrlSpeedRef,
                             ctrlVars[sysVars.ctrlNode].speedWe);

        sysVars.speedLoopCount = 0;
    }

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
    {
        DCL_resetPI(&ctrlVars[sysVars.ctrlNode].dcl_pos);
        DCL_resetPI(&ctrlVars[sysVars.ctrlNode].dcl_spd);
    }

#endif  // (SPD_CNTLR == SPD_DCL_CNTLR)

    ctrlVars[sysVars.ctrlNode].ctrlStateCom =
            ctrlVars[sysVars.ctrlNode].ctrlStateSet;

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_RUN)
    {
        ctrlVars[sysVars.ctrlNode].IdRef =
                ctrlVars[sysVars.ctrlNode].ctrlIdRef;

        ctrlVars[sysVars.ctrlNode].IqRef =
                ctrlVars[sysVars.ctrlNode].ctrlSpdOut;
    }
    else if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_FAULT)
    {
        ctrlVars[sysVars.ctrlNode].IdRef = 0.0;
        ctrlVars[sysVars.ctrlNode].IqRef = 0.0;
        ctrlVars[sysVars.ctrlNode].ctrlModeCom = CTRL_MODE_FAULT;
    }
    else if(ctrlVars[sysVars.ctrlNode].ctrlStateCom == CTRL_RUN)
    {
        ctrlVars[sysVars.ctrlNode].IdRef =
                ctrlVars[sysVars.ctrlNode].IdRefStart;

        if(ctrlVars[sysVars.ctrlNode].speedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].IqRef =
                    ctrlVars[sysVars.ctrlNode].IqRefStart;
        }
        else if(ctrlVars[sysVars.ctrlNode].speedRef < 0.0)
        {
            ctrlVars[sysVars.ctrlNode].IqRef =
                    -ctrlVars[sysVars.ctrlNode].IqRefStart;
        }
    }

    return;
}

#endif // (BUILDLEVEL==FCL_LEVEL8)

//
// run the controller
//
// TODO: runController
void runController(SysNode_e node)
{
    if(sysVars.ctrlSynSet == CTRL_SYN_ENABLE)
    {
        ctrlVars[node].ctrlStateSet = sysVars.ctrlStateSet;
        ctrlVars[node].speedRef = sysVars.speedSet;
        ctrlVars[node].positionSet = sysVars.positionSet;
    }
    else
    {
        ctrlVars[node].speedRef = ctrlVars[node].speedSet;
        ctrlVars[node].positionSet = ctrlVars[node].positionSet;
    }

    return;
}

//
// reset the controller
//
void resetControllerVars(CTRL_Vars_t *pCtrl)
{
    pCtrl->ctrlStateCom = CTRL_STOP;
    pCtrl->ctrlStateFdb = CTRL_STOP;

#if(SPD_CNTLR == SPD_PID_CNTLR)
    pCtrl->pid_spd.data.d1 = 0;
    pCtrl->pid_spd.data.d2 = 0;
    pCtrl->pid_spd.data.i1 = 0;
    pCtrl->pid_spd.data.ud = 0;
    pCtrl->pid_spd.data.ui = 0;
    pCtrl->pid_spd.data.up = 0;
#endif  // (SPD_CNTLR == SPD_PID_CNTLR)

#if(SPD_CNTLR == SPD_DCL_CNTLR)
    DCL_resetPI(&pCtrl->dcl_spd);
#endif  // (SPD_CNTLR == SPD_DCL_CNTLR)

    return;
}

