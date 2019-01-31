/**
  ******************************************************************************
  * @file    layers_conv2d.h
  * @author  AST Embedded Analytics Research Platform
  * @date    18-Apr-2018
  * @brief   header file of AI platform conv2d layers datatypes
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
#ifndef __LAYERS_CONV2D_H_
#define __LAYERS_CONV2D_H_
#pragma once

#include "layers.h"
#include "layers_nl.h"
#include "layers_pool.h"

#define AI_LAYER_CONV2D_FIELDS_DECLARE \
  AI_LAYER_SEQUENTIAL_FIELDS_DECLARE \
  ai_tensor*  weights; /*!< layer weights, shape: (filt_height, filt_width, \
                            n_chan_out, n_chan_in) */ \
  ai_tensor*  bias;    /*!< layer bias, shape: (n_chan_out,) */ \
  ai_u32      groups;  /*!< groups for separable convolution */ \
  func_nl     nl_func;        /*!< function pointer to non linear transform */ \
  ai_shape_2d filter_stride;  /*!< filter stride, how much the filter moves */ \
  ai_shape_2d filter_pad;     /*!< filter pad, y,x border sizes */ 

/*!
 * @defgroup layers_conv2d Convolutive Layers Definitions
 * @brief definition 
 *
 */

AI_API_DECLARE_BEGIN

/*!
 * @struct ai_layer_dense
 * @ingroup layers_conv2d
 * @brief Dense (fully connected) layer
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_dense_ {
  AI_LAYER_SEQUENTIAL_FIELDS_DECLARE
  ai_tensor* weights; /*!< layer weights, shape: (n_input, n_output) */
  ai_tensor* bias;    /*!< layer bias, shape: (n_output,) */
} ai_layer_dense;

/*!
 * @struct ai_layer_gemm
 * @ingroup layers_conv2d
 * @brief layer for General Matrix Multiplication
 *
 * Layer for General Matrix Multiplication (GEMM):
 * \f{equation}{ Y = \alpha A \cdot B + \beta C \f}
 * \f$\alpha\f$ and \f$\beta\f$ are paramaters, A and B are matrices,
 * C is a matrix or an array. Size checks for A, B, C, and Y are performed and
 * broadcast is applied on C if necessary.
 * This is a sequential layer (see @ref ai_layer_sequential).
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_gemm_ {
  AI_NODE_COMMON_FIELDS_DECLARE
  ai_tensor* in;     /*!< input tensors, interpreted as 2D matrices */
  ai_tensor* out;    /*!< output tensor, interpreted as a 2D matrix */
  ai_float alpha;    /*!< alpha coefficient */
  ai_float beta;     /*!< beta coefficient */
  ai_u8 tA;          /*!< transpose A flag */
  ai_u8 tB;          /*!< transpose B flag */
} ai_layer_gemm;

/*!
 * @struct ai_layer_conv2d
 * @ingroup layers_conv2d
 * @brief 2D convolutional layer with strides and pads
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_conv2d_ {
  AI_LAYER_CONV2D_FIELDS_DECLARE
} ai_layer_conv2d;

/*!
 * @struct ai_layer_conv2d_nl_pool
 * @ingroup layers_conv2d
 * @brief 2D convolutional layer + nl + pooling with strides and pads
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_conv2d_nl_pool_ {
  AI_LAYER_CONV2D_FIELDS_DECLARE
  ai_shape_2d filter_pad_r;  /*!< right filter pad, for asymmetric padding */

  ai_shape_2d pool_size;    /*!< pooling size */
  ai_shape_2d pool_stride;  /*!< pooling stride */
  ai_shape_2d pool_pad;     /*!< pooling pad */

  func_pool pool_func;      /*!< function pointer to pooling transform */
} ai_layer_conv2d_nl_pool;

/******************************************************************************/
/*  Forward Functions Section                                                 */
/******************************************************************************/

/*!
 * @brief Computes the activations of a 2D convolutional layer.
 * @ingroup layers_conv2d
 * @param layer the convolutional (conv) layer
 */
AI_INTERNAL_API
void forward_conv2d(ai_layer* layer);

/*!
 * @brief Computes the activations of a @ref ai_layer_conv2d_nl_pool layer
 * The @ref ai_layer_conv2d_nl_pool is a fused conv2D + optional nonlinear
 * layer + optional pooling / nonlinearity (average, max, softmax)
 * @ingroup layers_conv2d
 * @param layer see @ai_layer_conv2d_nl_pool
 */
AI_INTERNAL_API
void forward_conv2d_nl_pool(ai_layer* layer);

/*!
 * @brief Computes the activations of a GEMM layer.
 * @ingroup layers
 * @param layer the layer including output and input tensors
 */
AI_INTERNAL_API
void forward_gemm(ai_layer* layer);

/*!
 * @brief Computes the activations of a dense (fully connected) layer.
 * @ingroup layers_conv2d
 * @param layer the dense layer
 */
AI_INTERNAL_API
void forward_dense(ai_layer* layer);

AI_API_DECLARE_END

#endif    /*__LAYERS_CONV2D_H_*/
