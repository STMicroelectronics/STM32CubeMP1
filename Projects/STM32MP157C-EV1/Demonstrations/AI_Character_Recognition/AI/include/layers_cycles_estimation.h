/**
  ******************************************************************************
  * @file    layers_cycles_estimation.h
  * @author  AST Embedded Analytics Research Platform
  * @date    01-May-2018
  * @brief   header file of AI platform layers cycles measurements
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LAYERS_CYCLES_ESTIMATION_H__
#define __LAYERS_CYCLES_ESTIMATION_H__


/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "ai_platform.h"
#include "ai_math_helpers.h"
#include "ai_datatypes_internal.h"
#ifndef HAS_STM32
  #include <time.h>
#endif

AI_API_DECLARE_BEGIN

/* Exported constants --------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
/* Maximum number of stacked layers to measure */
#define CYCLES_LAYER_NB_MAX                  (20)


/* Exported types ------------------------------------------------------------*/
/* Layer indexes */
typedef enum {
  CYCLES_LAYER_FORWARD_ALL                = 0,
  CYCLES_LAYER_FORWARD_DENSE,
  CYCLES_LAYER_FAST_DOT_ARRAY,
  CYCLES_LAYER_DICT48_DOT_ARRAY_F32,
  CYCLES_LAYER_FORWARD_CONV2D_NL_POOL,
  CYCLES_LAYER_FORWARD_CONV2D,
  CYCLES_LAYER_FORWARD_TIME_DELAY,
  CYCLES_LAYER_FORWARD_SM,
  CYCLES_LAYER_FORWARD_MP,
  CYCLES_LAYER_FORWARD_AP,
  CYCLES_LAYER_FORWARD_LRN,
  CYCLES_LAYER_FORWARD_BN,
  CYCLES_LAYER_FORWARD_NORM,
  CYCLES_LAYER_MEAS_NUM_MAX
} CYCLES_LayersIdx_e;


/* External variables --------------------------------------------------------*/
extern ai_float CYCLES_theoMacs;
extern ai_float CYCLES_filtxAverage;
extern ai_float CYCLES_filtyAverage;
extern ai_float CYCLES_measuredMacs;
extern ai_u32   CYCLES_counter;


/* Exported functions ------------------------------------------------------- */ 
/*!
 * @brief Starts cycles measurement of current layer
 * @ingroup layers_cycles_estimation
 *
 * @param input idx is the layer idx used for current cycles measurement
 * @return none
 */
AI_API_ENTRY
void CYCLES_MeasurementsStart(const ai_u32 idx);


/*!
 * @brief Stops cycles measurement of current layer
 * @ingroup layers_cycles_estimation
 *
 * @param input idx is the layer idx used for current cycles measurement
 * @param input theoritical_macs of current layer
 * @param input innerloop_coef to estimate MACCs/MHz ratio
 * @param input overhead_coef to estimate MACCs/MHz ratio
 * @param input misc_coef to estimate MACCs/MHz ratio
 * @return none
 */
AI_API_ENTRY
void CYCLES_MeasurementsStop(const ai_u32 idx, 
                             const ai_float theoritical_macs, 
                             const ai_float innerloop_coef, 
                             const ai_float overhead_coef, 
                             const ai_float misc_coef);


AI_API_DECLARE_END

#endif  /* __LAYERS_CYCLES_ESTIMATION_H__ */
