/* ----------------------------------------------------------------------
 * Copyright (C) 2010-2020 ARM Limited. All rights reserved.
 *
* $Date:         17. January 2013
* $Revision:     V1.4.0
*
* Project:       CMSIS DSP Library
 * Title:        arm_fir_example_f32.c
 *
 * Description:  Example code demonstrating how an FIR filter can be used
 *               as a low pass filter.
 *
 * Target Processor: Cortex-M4/Cortex-M3
 *
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------- */

/**
 * @ingroup groupExamples
 */

/**
 * @defgroup PID Example
 *
 * \par Description:
 * \par
 * Removes high frequency signal components from the input using an FIR lowpass filter.
 * The example demonstrates how to configure an FIR filter and then pass data through
 * it in a block-by-block fashion.
 * \image html FIRLPF_signalflow.gif
 *
 * \par Algorithm:
 * \par
 * The input signal is a sum of two sine waves:  1 kHz and 15 kHz.
 * This is processed by an FIR lowpass filter with cutoff frequency 6 kHz.
 * The lowpass filter eliminates the 15 kHz signal leaving only the 1 kHz sine wave at the output.
 * \par
 * The lowpass filter was designed using MATLAB with a sample rate of 48 kHz and
 * a length of 29 points.
 * The MATLAB code to generate the filter coefficients is shown below:
 * <pre>
 *     h = fir1(28, 6/24);
 * </pre>
 * The first argument is the "order" of the filter and is always one less than the desired length.
 * The second argument is the normalized cutoff frequency.  This is in the range 0 (DC) to 1.0 (Nyquist).
 * A 6 kHz cutoff with a Nyquist frequency of 24 kHz lies at a normalized frequency of 6/24 = 0.25.
 * The CMSIS FIR filter function requires the coefficients to be in time reversed order.
 * <pre>
 *     fliplr(h)
 * </pre>
 * The resulting filter coefficients and are shown below.
 * Note that the filter is symmetric (a property of linear phase FIR filters)
 * and the point of symmetry is sample 14.  Thus the filter will have a delay of
 * 14 samples for all frequencies.
 * \par
 * \image html FIRLPF_coeffs.gif
 * \par
 * The frequency response of the filter is shown next.
 * The passband gain of the filter is 1.0 and it reaches 0.5 at the cutoff frequency 6 kHz.
 * \par
 * \image html FIRLPF_response.gif
 * \par
 * The input signal is shown below.
 * The left hand side shows the signal in the time domain while the right hand side is a frequency domain representation.
 * The two sine wave components can be clearly seen.
 * \par
 * \image html FIRLPF_input.gif
 * \par
 * The output of the filter is shown below.  The 15 kHz component has been eliminated.
 * \par
 * \image html FIRLPF_output.gif
 *
 * \par Variables Description:
 * \par
 * \li \c testInput_f32_1kHz_15kHz points to the input data
 * \li \c refOutput points to the reference output data
 * \li \c testOutput points to the test output data
 * \li \c firStateF32 points to state buffer
 * \li \c firCoeffs32 points to coefficient buffer
 * \li \c blockSize number of samples processed at a time
 * \li \c numBlocks number of frames
 *
 * \par CMSIS DSP Software Library Functions Used:
 * \par
 * - arm_fir_init_f32()
 * - arm_fir_f32()
 *
 * <b> Refer  </b>
 * \link arm_fir_example_f32.c \endlink
 *
 */


/** \example arm_fir_example_f32.c
 */

/* ----------------------------------------------------------------------
** Include Files
** ------------------------------------------------------------------- */

#include <stdint.h>

#ifndef IO_CONSOLE
#include <ti/board/board.h>
#include <ti/board/board_cfg.h>
#endif

#include "benchmark_log.h"
#include "profile.h"

#include "arm_math.h"
#include "math_helper.h"

/* ----------------------------------------------------------------------
** Macro Defines
** ------------------------------------------------------------------- */

#define NUM_ITERATION 10
#define NUM_PID_LOOP 1000

/* -------------------------------------------------------------------
 * The input signal and reference output (computed with MATLAB)
 * are defined externally in arm_fir_lpf_data.c.
 * ------------------------------------------------------------------- */

/* -------------------------------------------------------------------
 * Declare Test output buffer
 * ------------------------------------------------------------------- */

/* ------------------------------------------------------------------
 * Global variables for PID Example
 * ------------------------------------------------------------------- */
arm_pid_instance_f32 myPIDInstance __attribute__((section(".testInData")));

/* ----------------------------------------------------------------------
 * PID Example
 * ------------------------------------------------------------------- */

int32_t main(void)
{
    uint32_t i, j;
    float pidOutput;

#ifndef IO_CONSOLE
	Board_initCfg boardCfg;

    boardCfg = BOARD_INIT_PINMUX_CONFIG |
               BOARD_INIT_MODULE_CLOCK  |
               BOARD_INIT_UART_STDIO;
    Board_init(boardCfg);
#endif

#if PROFILE == COMPONENTS
    init_profiling();
    gStartTime = readPmu(); // two initial reads are necessary for correct overhead time
    gStartTime = readPmu();
    gEndTime = readPmu();
    gOverheadTime = gEndTime - gStartTime;    
    MCBENCH_log("\n %d overhead cycles\n", (uint32_t)gOverheadTime);
#endif

  /* Initialize PID instance */
  myPIDInstance.Kp = 350;
  myPIDInstance.Ki = 300;
  myPIDInstance.Kd = 50;
  
  MCBENCH_log("\n START PID benchmark\n");

  /* ----------------------------------------------------------------------
  ** Call the PID function for every input sample
  ** ------------------------------------------------------------------- */
  for(i=0; i < NUM_ITERATION; i++)
  {

    /* Call PID init function to initialize the instance structure. */
    arm_pid_init_f32(&myPIDInstance, 1);
	
#if PROFILE == COMPONENTS
    gStartTime = readPmu();
#endif        
    for (j=0; j<NUM_PID_LOOP; j++)
      pidOutput = arm_pid_f32(&myPIDInstance, 0.1);

#if PROFILE == COMPONENTS
  /*********** Compute benchmark in cycles ********/
    gEndTime = readPmu();
    gTotalTime = gEndTime - gStartTime - gOverheadTime;
    MCBENCH_log("\n Test used %d cycles\n", (uint32_t)gTotalTime);
#endif        
  }

  MCBENCH_log("PID output = %d\n", (int32_t)(pidOutput));
  MCBENCH_log("\n END PID benchmark\n");
  /* MCBENCH_log is blank, if the DEBUG_PRINT is not defined in benchmark_log.h */ 
  /* supress the unused variable build warning */
  (void) pidOutput;

  /* ----------------------------------------------------------------------
  ** Compare the generated output against the reference output computed
  ** in MATLAB.
  ** ------------------------------------------------------------------- */

  /* ----------------------------------------------------------------------
  ** Loop here if the signal does not match the reference output.
  ** ------------------------------------------------------------------- */

  while (1);                             /* main function does not return */
}

/** \endlink */
