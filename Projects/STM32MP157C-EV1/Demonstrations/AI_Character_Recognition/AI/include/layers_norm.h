/**
  ******************************************************************************
  * @file    layers_norm.h
  * @author  AST Embedded Analytics Research Platform
  * @date    18-Apr-2018
  * @brief   header file of AI platform normalization layers datatypes
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
#ifndef __LAYERS_NORM_H_
#define __LAYERS_NORM_H_
#pragma once

#include "layers_common.h"

/*!
 * @defgroup layers_norm Normalization Layers Definitions
 * @brief definition 
 *
 */

AI_API_DECLARE_BEGIN

/*!
 * @struct ai_layer_bn
 * @ingroup layers_norm
 * @brief Batch normalization (scale with bias) layer
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_bn_ {
  AI_LAYER_SEQUENTIAL_FIELDS_DECLARE
  ai_tensor* scale; /*!< layer scale factor, shape: (n_output,) */
  ai_tensor* bias;  /*!< layer bias, shape: (n_output,) */
} ai_layer_bn;

/*!
 * @struct ai_layer_lrn
 * @ingroup layers_norm
 * @brief Local Response Normalization layer
 *
 * Divides each element by a scale factor computed
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_lrn_ {
  AI_LAYER_SEQUENTIAL_FIELDS_DECLARE
  ai_u32 local_size;   /*!< size of the normalization window */
  ai_float k;          /*!< bias term */
  ai_float alpha;      /*!< input scale */
  ai_float beta;       /*!< scale exponent */
} ai_layer_lrn;

/*!
 * @struct ai_layer_norm
 * @ingroup layers_norm
 * @brief Lp Normalization layer
 *
 * Normalizes the tensor along the 'axis' direction using the Lp norm.
 * Optionally divides the result by the number of the elements.
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_norm_ {
  AI_LAYER_SEQUENTIAL_FIELDS_DECLARE
  ai_shape_type axis;    /*! normalization axis */
  ai_float exponent;     /*!< normalization exponent p */
  ai_bool scale;    /*!< multiplies by the pth root of the number of elements */
} ai_layer_norm;


/*!
 * @brief Local response normalization computed on a float array
 * @ingroup layers_norm
 * @param out opaque handler to float output channel
 * @param in  opaque handler to float input channel
 * @param pad amount of padding for the channels
 */
AI_INTERNAL_API
void func_lrn_array_f32(ai_handle out, const ai_handle in,
                        const ai_size in_size, const ai_size channel_size,
                        const ai_i32 pad, const ai_float k,
                        const ai_float alpha, const ai_float beta);

/*!
 * @brief Lp normalization computed on a float array
 * @ingroup layers_norm
 * @param out opaque handler to float output channel
 * @param in  opaque handler to float input channel
 * @param exponent p exponent for the Lp normalization
 * @param axis_stride stride (in array elements) of the normalization axis
 * @param axis_size size of the normalization axis
 * @param outer_size number of tensor slices (including the normalization axis)
 *   on which compute the normalization
 */
AI_INTERNAL_API
void func_norm_array_f32(ai_handle out, const ai_handle in,
                         const ai_float exponent,
                         const ai_float norm,
                         const ai_size axis_stride,
                         const ai_size axis_size,
                         const ai_size outer_size);

/******************************************************************************/
/*  Forward Functions Section                                                 */
/******************************************************************************/

/*!
 * @brief Computes the activations of a batchnorm (scale + bias) layer.
 * @ingroup layers_norm
 * @param layer the batch normalization (bn) layer
 */
AI_INTERNAL_API
void forward_bn(ai_layer* layer);

/*!
 * @brief Computes the activations of a Local Response Normalization Layer.
 * @ingroup layers_norm
 * @param layer the local response normalization (lrn) layer
 */
AI_INTERNAL_API
void forward_lrn(ai_layer* layer);

/*!
 * @brief Computes the activations of a normalization layer.
 * @ingroup layers_norm
 * @param layer the normalization (norm) layer
 */
AI_INTERNAL_API
void forward_norm(ai_layer* layer);


AI_API_DECLARE_END

#endif    /*__LAYERS_NORM_H_*/
