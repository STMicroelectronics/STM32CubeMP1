/**
  ******************************************************************************
  * @file    layers_pool.h
  * @author  AST Embedded Analytics Research Platform
  * @date    18-Apr-2018
  * @brief   header file of AI platform pooling layers datatypes
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
