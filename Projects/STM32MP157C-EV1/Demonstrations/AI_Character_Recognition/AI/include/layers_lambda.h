/**
  ******************************************************************************
  * @file    layers_lambda.h
  * @author  AST Embedded Analytics Research Platform
  * @date    30-Lug-2018
  * @brief   header file of AI platform lambda layers datatype
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
