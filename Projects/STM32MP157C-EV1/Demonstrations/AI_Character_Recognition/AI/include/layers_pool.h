/**
  ******************************************************************************
  * @file    layers_pool.h
  * @author  AST Embedded Analytics Research Platform
  * @date    18-Apr-2018
  * @brief   header file of AI platform pooling layers datatypes
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright
  *      notice, this list of conditions and the following disclaimer in the
  *      documentation and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its
  *      contributors may be used to endorse or promote products derived from
  *      this software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  * POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#ifndef __LAYERS_POOL_H_
#define __LAYERS_POOL_H_
#pragma once

#include "layers_common.h"

/*!
 * @defgroup layers_pool Pooling Layers Definitions
 * @brief definition 
 *
 */

AI_API_DECLARE_BEGIN

/*!
 * @struct ai_layer_pool
 * @ingroup layers_pool
 * @brief Pooling layer
 *
 * The type of pooling function is handled by the specific forward function
 * @ref forward_pool
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_pool_ {
  AI_LAYER_SEQUENTIAL_FIELDS_DECLARE
  ai_shape_2d pool_size;    /*!< pooling size */
  ai_shape_2d pool_stride;  /*!< pooling stride */
  ai_shape_2d pool_pad;     /*!< pooling pad, y,x border sizes */
} ai_layer_pool;


/*!
 * @typedef (*func_pool)
 * @ingroup layers_pool
 * @brief Fuction pointer for generic pooling transform
 * this function pointer abstracts a generic pooling layer.
 * see @ref nl_func_sm_array_f32 and @ref pool_func_ap_array_f32 as examples
 */
typedef void (*func_pool)(ai_handle out, const ai_handle in,
                          const ai_size size, const ai_size step,
                          const ai_size pool_size);

/*!
 * @brief Max Pooling on a float data array
 * @ingroup layers_pool
 * @param in  opaque handler to float, size = 1
 * @param out opaque handler to float output elem
 * @param size size of the input data in bytes
 * @param step step in bytes between two consecutive elements
 * @param pool_size number of elements in the pooling operation
 */
AI_INTERNAL_API
void pool_func_mp_array_f32(ai_handle out, const ai_handle in,
                            const ai_size size, const ai_size step,
                            const ai_size pool_size);

/*!
 * @brief Average Pooling on a float data array
 * @ingroup layers_pool
 * @param in  opaque handler to float, size = 1
 * @param out opaque handler to float output elem
 * @param size size of the input data in bytes
 * @param step step in bytes between two consecutive elements
 * @param pool_size number of elements in the pooling operation
 */
AI_INTERNAL_API
void pool_func_ap_array_f32(ai_handle out, const ai_handle in,
                            const ai_size size, const ai_size step,
                            const ai_size pool_size);

/******************************************************************************/
/*  Forward Functions Section                                                 */
/******************************************************************************/

/*!
 * @brief Computes the activations of a max pooling layer.
 * @ingroup layers_pool
 * @param layer the pooling (pool) layer
 */
AI_INTERNAL_API
void forward_mp(ai_layer* layer);

/*!
 * @brief Computes the activations of an average pooling layer.
 * @ingroup layers_pool
 * @param layer the pooling (pool) layer
 */
AI_INTERNAL_API
void forward_ap(ai_layer* layer);


AI_API_DECLARE_END

#endif    /*__LAYERS_POOL_H_*/
