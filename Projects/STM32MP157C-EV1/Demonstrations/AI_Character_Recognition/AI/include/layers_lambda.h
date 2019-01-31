/**
  ******************************************************************************
  * @file    layers_lambda.h
  * @author  AST Embedded Analytics Research Platform
  * @date    30-Lug-2018
  * @brief   header file of AI platform lambda layers datatype
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
#ifndef __LAYERS_LAMBDA_H_
#define __LAYERS_LAMBDA_H_
#pragma once

#include "layers_common.h"

#ifdef USE_OPERATORS

#include "operators_common.h"

/*!
 * @defgroup layers_lambda Lambda layer definitions
 * @brief Definition of structures for Lambda layers and generic operator
 * containers. These layers require the inclusion of the operator module.
 */

AI_API_DECLARE_BEGIN


/*!
 * @struct ai_layer_lambda
 * @ingroup layers_lambda
 * @brief Lambda layer wrapper
 *
 * The lambda layer wrapper includes a sub-graph of operators.
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_lambda_ {
  AI_LAYER_SEQUENTIAL_FIELDS_DECLARE
  ai_operator* sub_graph;
} ai_layer_lambda;

/*!
 * @struct ai_layer_container
 * @ingroup layers_lambda
 * @brief Container layer
 *
 * The container layer wraps generic operator in order to use them as layers
 * in a network structure.
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_container_ {
    AI_NODE_COMMON_FIELDS_DECLARE
    struct ai_operator_* operators;
} ai_layer_container;


/******************************************************************************/
/* Forward Functions Section                                                  */
/******************************************************************************/

/*!
 * @brief Computes the activations of a lambda layer.
 * @ingroup layers_generic
 *
 * Container forward layer function. This forward function
 * implements the activation of the operators chain.
 * @param layer the container layer
 */
AI_INTERNAL_API
void forward_container(ai_layer* layer);

/*!
 * @brief Computes the activations of a lambda layer.
 * @ingroup layers_lambda
 * @param layer the lambda layer
 */
AI_INTERNAL_API
void forward_lambda(ai_layer* layer);

AI_API_DECLARE_END

#endif /* USE_OPERATORS */

#endif    /*__LAYERS_LAMBDA_H_*/
