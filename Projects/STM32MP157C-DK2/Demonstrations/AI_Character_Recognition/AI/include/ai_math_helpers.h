/**
  ******************************************************************************
  * @file    ai_math_helpers.h
  * @author  AST Embedded Analytics Research Platform
  * @date    01-May-2017
  * @brief   Math helpers routines header file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  ******************************************************************************
  */ 
#ifndef __AI_MATH_HELPERS_H_
#define __AI_MATH_HELPERS_H_
#include <math.h>

#include "ai_platform.h"
#include "ai_platform_interface.h"


#define AI_FLOAT_EPSILON        (1.19209290e-7F)

#define AI_MIN(x,y)             ( ((x)<(y)) ? (x) : (y) )
#define AI_MAX(x,y)             ( ((x)>(y)) ? (x) : (y) )
#define AI_CLAMP(x, min, max)   AI_MIN(AI_MAX(x,min), max)


#define AI_MATH_DOT_ARRAY(dst, src0, src1, size) \
          ai_math_dot_array(dst, src0, src1, size)

#define AI_MATH_SQRT(x)         ai_math_sqrt(x)
#define AI_MATH_EXP(x)          expf(x)
#define AI_MATH_POW(x, e)       powf((x), (e))
#define AI_MATH_TANH(x)         tanhf(x)
#define AI_MATH_SIGN(x)         (((x)>0) ? 1 : -1)


#define AI_MATH_LEAKY_RELU(x, neg_slope, pos_slope) \
          ( ((x)>0) ? (x)*(pos_slope) : (x)*(neg_slope) )

#define AI_MATH_PRELU(x,slope)  AI_MATH_LEAKY_RELU(x, slope, 1)

#define AI_MATH_RELU(x)         AI_MAX(x, 0)

#define AI_MATH_RELU_6(x)       AI_CLAMP(x, 0, 6)


#define AI_MATH_SIGMOID(x)      (1.0f / (1.0f + AI_MATH_EXP(-(x))))

#define AI_MATH_HARD_SIGMOID(x) (AI_MAX(0.0f, AI_MIN(1.0f, (x) * 0.2f + 0.5f)))


AI_API_DECLARE_BEGIN

/*!
 * @defgroup math_helpers Math helpers
 * @brief Common math functions
 *
 * Math functions are mapped to the underlying platform through those utility
 * functions. On x86 and ARM v7 they are mapped to the float math functions in
 * the C99 standard library; on MCUs they are mapped to the ARM DSP functions.
 */

/*!
 * @brief platform optimized dot product of float vectors
 *
 * Computes the dot product between vectors and adds the result to out.
 * @ingroup math_helpers
 * @param out scalar result of the dot product
 * @param data0 the first float vector
 * @param data1 the second float vector
 * @param data_size the size of both vectors
 */
AI_INTERFACE_ENTRY
void ai_math_dot_array(
        ai_float* out,
        const ai_float* data0,
        const ai_float* data1,
        const ai_size data_size);

/*!
 * @brief platform optimized square root on float value
 * @ingroup math_helpers
 * @param x input value
 * @return square root of the value
 */
AI_INTERFACE_ENTRY
ai_float ai_math_sqrt(const ai_float x);

/*!
 * @brief platform optimized exponential on float value
 * @ingroup math_helpers
 * @param x input value
 * @return exponential of the value
 */
AI_INTERFACE_ENTRY
ai_float ai_math_exp(const ai_float x);

/*!
 * @brief platform optimized pow on float value
 * @ingroup math_helpers
 * @param x input value
 * @param e input value
 * @return pow of the value ^ e
 */
AI_INTERFACE_ENTRY
ai_float ai_math_pow(const ai_float x, const ai_float e);

/*!
 * @brief platform optimized tangent on float value
 * @ingroup math_helpers
 * @param x input value
 * @return hyperbolic tangent of the value
 */
AI_INTERFACE_ENTRY
ai_float ai_math_tanh(const ai_float x);

/*!
 * @brief platform optimized relu on float value
 * @ingroup math_helpers
 * @param x input value
 * @return relu of the value ( x if x>0 else 0)
 */
AI_INTERFACE_ENTRY
ai_float ai_math_relu(const ai_float x);

/*!
 * @brief platform optimized parametric relu on float value
 * @ingroup math_helpers
 * @param x input value
 * @param slope input value
 * @return parametric relu of the value
 */
AI_INTERFACE_ENTRY
ai_float ai_math_prelu(const ai_float x, const ai_float slope);

/*!
 * @brief platform optimized parametric sigmoid on float value
 * @ingroup math_helpers
 * @param x input value
 * @return sigmoid of the value
 */
AI_INTERFACE_ENTRY
ai_float ai_math_sigmoid(const ai_float x);

/*!
 * @brief platform optimized parametric hard sigmoid on float value
 * @ingroup math_helpers
 * @param x input value
 * @return hard sigmoid of the value
 */
AI_INTERFACE_ENTRY
ai_float ai_math_hard_sigmoid(const ai_float x);


/*!
 * @brief optimized parametric rectified linear unit on float values
 * @ingroup math_helpers
 * @param x input value
 * @param slope parameter value
 * @return x if x is positive and x*slope otherwise
 */
AI_INTERFACE_ENTRY
ai_float ai_fast_prelu(const ai_float x,const ai_float slope);

AI_API_DECLARE_END

#endif /* __MATH_HELPERS_H_ */
