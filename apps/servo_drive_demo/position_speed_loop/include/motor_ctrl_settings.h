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

#ifndef MOTOR_CTRL_SETTINGS_H
#define MOTOR_CTRL_SETTINGS_H

//
// Include project specific include files.
//

//
// Following is the list of the Build Level choices.
//
#define  FCL_LEVEL7   7     // Verify speed loop over FSI & IPC
#define  FCL_LEVEL8   8     // Verify position loop over FSI & IPC

//
// Here below, pick position and speed loop controller option
//
#define  SPD_PID_CNTLR      1
#define  SPD_DCL_CNTLR      2

//
// User can select choices from available control configurations
//
#define  BUILDLEVEL          FCL_LEVEL7         // 7 & 11 for demo
#define  SPD_CNTLR           SPD_PID_CNTLR      // SPD_DCL_CNTLR     //

#ifndef BUILDLEVEL
#error  Critical: BUILDLEVEL must be defined !!
#endif  // BUILDLEVEL

#ifndef PI
#define PI 3.14159265358979
#endif

#endif  // end of MOTOR_CTRL_SETTINGS_H definition

