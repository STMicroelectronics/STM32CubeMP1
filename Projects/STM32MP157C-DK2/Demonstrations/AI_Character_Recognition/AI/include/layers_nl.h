/**
  ******************************************************************************
  * @file    layers_nl.h
  * @author  AST Embedded Analytics Research Platform
  * @date    18-Apr-2018
  * @brief   header file of AI platform nonlinearity layers datatypes
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
#ifndef __LAYERS_NL_H_
#define __LAYERS_NL_H_
#pragma once

#include "layers_common.h"

/*!
 * @defgroup layers_nl Normalization Layers Definitions
 * @brief definition 
 *
 */

AI_API_DECLARE_BEGIN

/*!
 * @struct ai_layer_nl
 * @ingroup layers_nl
 * @brief Nonlinearity layer
 *
 * The type of nonlinearity is handled by the specific forward function.
 * It is a sequential layer. see @ref ai_layer_sequential
 */
typedef ai_layer_sequential ai_layer_nl;

/*!
 * @struct ai_layer_nl_selu
 * @ingroup layers_nl
 * @brief Scaled Exponential Linear Unit (SELU) layer
 *
 * Scaled Exponential Linear Unit (SELU).
 * It is a sequential layer. see @ref ai_layer_sequential
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_nl_selu_ {
  AI_LAYER_SEQUENTIAL_FIELDS_DECLARE
  ai_float alpha;    /*!< normalization exponent */
  ai_float gamma;    /*!< slope */
} ai_layer_nl_selu;

/*!
 * @struct ai_layer_nl_prelu
 * @ingroup layers_nl
 * @brief Parametric Rectified Linear Unit (PRELU) layer
 *
 * Parametric Rectified Linear Unit (PRELU).
 * It allows to define a slope tensor.
 * It is a sequential layer. see @ref ai_layer_sequential
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_nl_prelu_ {
  AI_LAYER_SEQUENTIAL_FIELDS_DECLARE
  ai_tensor* slope;     /*!< slope tensor */
} ai_layer_nl_prelu;

/*!
 * @struct ai_layer_sm
 * @ingroup layers_nl
 * @brief Softmax nonlinear layer
 *
 * The softmax layer is handled separately because it involves a normalization
 * step along the channel axis. It is a sequential layer. see @ref ai_layer_sequential
 */
typedef ai_layer_sequential ai_layer_sm;

/*!
 * @typedef (*func_nl)
 * @ingroup layers_nl
 * @brief Fuction pointer for generic non linear transform
 * this function pointer abstracts a generic non linear layer.
 * see @ref nl_func_tanh_array_f32 and similar as examples.
 */
typedef void (*func_nl)(ai_handle out, const ai_handle in,
                        const ai_size size);

/*!
 * @brief Softmax pooling computed on a single float channel
 * @ingroup layers_nl
 * @param out opaque handler to float output channel
 * @param in  opaque handler to float input channel
 * @param channel_size number of elements of the input channel
 */
AI_INTERNAL_API
void nl_func_sm_channel_f32(ai_handle out, const ai_handle in,
                            const ai_size channel_size);

/*!
 * @brief Softmax normalization computed on an array of float channels
 * @ingroup layers_nl
 * @param out opaque handler to float output channel array
 * @param in  opaque handler to float input channel array
 * @param in_size  total size (number of elements) to process on the input
 * @param channel_size number of elements of the input channel
 * @param in_channel_step number of elements to move to next input element
 * @param out_channel_step number of elements to move to next output element
 */
AI_INTERNAL_API
void nl_func_sm_array_f32(ai_handle out, const ai_handle in,
                          const ai_size in_size,
                          const ai_size channel_size,
                          const ai_size in_channel_step,
                          const ai_size out_channel_step);

/*!
 * @brief Computes the tanh function on a float data array
 * @ingroup layers_nl
 * @param in opaque handler to float, size should be 1
 * @param out opaque handler to float output elem
 * @param size number of elements in the input buffer
 */
AI_INTERNAL_API
void nl_func_tanh_array_f32(ai_handle out, const ai_handle in,
                            const ai_size size);

/*!
 * @brief Computes the sigmoid function on a float data array
 * @ingroup layers_nl
 * @param in opaque handler to float, size should be 1
 * @param out opaque handler to float output elem
 * @param size number of elements in the input buffer
 */
AI_INTERNAL_API
void nl_func_sigmoid_array_f32(ai_handle out, const ai_handle in,
                               const ai_size size);

/*!
 * @brief Computes the sign function on a single float element.
 * @ingroup layers_nl
 * @param in opaque handler to float, size should be 1
 * @param out opaque handler to float output elem
 * @param size number of elements in the input buffer
 */
AI_INTERNAL_API
void nl_func_sign_array_f32(ai_handle out, const ai_handle in,
                            const ai_size size);

/*!
 * @brief Computes the relu function on a float data array
 * @ingroup layers_nl
 * @param in opaque handler to float, size should be 1
 * @param out opaque handler to float output elem
 * @param size number of elements in the input buffer
 */
AI_INTERNAL_API
void nl_func_relu_array_f32(ai_handle out, const ai_handle in,
                            const ai_size size);

/*!
 * @brief Computes the relu6 function on a float data array
 * @ingroup layers_nl
 * @param in opaque handler to float, size should be 1
 * @param out opaque handler to float output elem
 * @param size number of elements in the input buffer
 */
AI_INTERNAL_API
void nl_func_relu6_array_f32(ai_handle out, const ai_handle in,
                             const ai_size size);

/*!
 * @brief Computes the selu function on a float data array
 * @ingroup layers_nl
 * @param in opaque handler to float, size should be 1
 * @param out opaque handler to float output elem
 * @param size number of elements in the input buffer
 * @param alpha normalization coefficient of selu
 * @param gamma slope coefficient of selu
 */
AI_INTERNAL_API
void nl_func_selu_array_f32(ai_handle out, const ai_handle in,
                             const ai_size size, const ai_float alpha,
                             const ai_float gamma);

/*!
 * @brief Computes the prelu function on a float data array
 * @ingroup layers_nl
 * @param in opaque handler to float, size should be 1
 * @param slope opaque handler to float, size should be 1
 * @param out opaque handler to float output elem
 * @param size size of the input data in bytes
 */
AI_INTERNAL_API
void nl_func_prelu_array_f32(ai_handle out, const ai_handle in,
                            const ai_handle slope, const ai_size size);


/******************************************************************************/
/** Forward Functions Section                                                **/
/******************************************************************************/

/*!
 * @brief Computes the activations of a ReLU nonlinear layer.
 * @ingroup layers_nl
 * @param layer the nonlinear (nl) layer
 */
AI_INTERNAL_API
void forward_relu(ai_layer* layer);

/*!
 * @brief Computes the activations of a ReLU6 nonlinear layer.
 * @ingroup layers_nl
 * @param layer the nonlinear (nl) layer
 */
AI_INTERNAL_API
void forward_relu6(ai_layer* layer);

/*!
 * @brief Computes the activations of a SELU nonlinear layer.
 * @ingroup layers_nl
 * @param layer the nonlinear (nl) layer
 */
AI_INTERNAL_API
void forward_selu(ai_layer* layer);

/*!
 * @brief Computes the activations of a PRELU nonlinear layer.
 * @ingroup layers_nl
 * @param layer the nonlinear (nl) layer
 */
AI_INTERNAL_API
void forward_prelu(ai_layer* layer);

/*!
 * @brief Computes the activations of a binary tanh (sign) nonlinear layer.
 * @ingroup layers
 * @param layer the nonlinear (nl) layer
 */
AI_INTERNAL_API
void forward_sign(ai_layer* layer);

/*!
 * @brief Computes the activations of a sigmoid nonlinear layer.
 * @ingroup layers_nl
 * @param layer the nonlinear (nl) layer
 */
AI_INTERNAL_API
void forward_sigmoid(ai_layer* layer);

/*!
 * @brief Computes the activations of a hyperbolic tangent (tanh) layer.
 * @ingroup layers_nl
 * @param layer the nonlinear (nl) layer
 */
AI_INTERNAL_API
void forward_tanh(ai_layer* layer);

/*!
 * @brief Computes the activations of a softmax nonlinear layer.
 * @ingroup layers_nl
 * @param layer the softmax (sm) layer
 */
AI_INTERNAL_API
void forward_sm(ai_layer* layer);

AI_API_DECLARE_END

#endif    /*__LAYERS_NL_H_*/
