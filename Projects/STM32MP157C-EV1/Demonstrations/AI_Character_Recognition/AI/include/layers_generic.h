/**
  ******************************************************************************
  * @file    layers_generic.h
  * @author  AST Embedded Analytics Research Platform
  * @date    18-Apr-2018
  * @brief   header file of AI platform generic layers datatypes
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
#ifndef __LAYERS_GENERIC_H_
#define __LAYERS_GENERIC_H_
#pragma once

#include "core_common.h"
#include "layers_common.h"

/*!
 * @defgroup layers_generic Generic Layers Definitions
 * @brief definition 
 *
 */

AI_API_DECLARE_BEGIN

/*!
 * @struct ai_layer_time_delay
 * @ingroup layers_generic
 * @brief TimeDelay layer with sparse kernel
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_time_delay_ {
  AI_NODE_SEQUENTIAL_FIELDS_DECLARE
  ai_tensor* weights; /*!< layer weights, shape:
                         (filt_height, filt_width, n_chan_out, n_chan_in) */
  ai_tensor* bias;    /*!< layer bias, shape: (n_chan_out,) */

  ai_array* mask;      /*!< sparse filter mask */
} ai_layer_time_delay;

/*!
 * @struct ai_layer_split
 * @ingroup layers_generic
 * @brief Split layer definition
 *
 * This layer defines the params of a splitting layer. It is intended to be used
 * by his associated forward function @ref forward_split
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_split_ {
  AI_NODE_COMMON_FIELDS_DECLARE
  ai_u16             out_layers_count; /*!< number of output layers to split*/
  ai_u16             out_layer_curr;   /*!< current layer to split  */
  ai_node**          out_layers;  /*!< output layers list */
  ai_tensor**        out_tensors; /*!< output tensors list */
  ai_tensor*         in_tensor;   /*!< input tensor */
  func_copy_tensor   copy_to_out_tensor; /*!< pointer to copy tensor func
                                         (NULL = no copy) */ 
} ai_layer_split;

/*!
 * @struct ai_layer_add
 * @ingroup layers_generic
 * @brief Add layer definition
 *
 * This layer defines the params of an add layer. 
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_add_ {
  AI_NODE_COMMON_FIELDS_DECLARE
  ai_u16             in_layers_count; /*!< number of input layers to concat */
  ai_u16             in_layer_curr;   /*!< current layer to concat  */
  ai_tensor**        in_tensors;  /*!< input tensors list (if NULL==no copy) */
  ai_tensor*         out_tensor;  /*!< output tensor (if NULL==no copy) */
  func_copy_tensor   copy_to_out_tensor; /*!< pointer to copy tensor func
                                         (NULL = no copy) */ 
  ai_node*  split_layer; /*!< pointer to associated split layer */
  ai_node*  next_layer;  /*!< pointer to next layer to process */
} ai_layer_add;

/*!
 * @struct ai_layer_permute
 * @ingroup layers_generic
 * @brief Permute layer datastruct declaration. This defines the params of a
 * permute layer. It is intended to be used by his associated forward function
 * @ref forward_permute
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_permute_ {
  AI_NODE_SEQUENTIAL_FIELDS_DECLARE
  ai_shape out_mapping;       /*!< permute output mapping order. I.e. tt is a
                                   permutation of the input tensor shape */
} ai_layer_permute;


#define AI_TIME_DISTRIBUTED_AXIS    (AI_SHAPE_HEIGHT)

/*!
 * @struct ai_layer_time_distributed
 * @ingroup layers_generic
 * @brief Time distributed layer datastruct declaration. This defines the params
 * of a time distributed layer. It is intended to be used by his associated
 * forward function @ref forward_time_distributed
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_time_distributed_ {
  AI_NODE_SEQUENTIAL_FIELDS_DECLARE
  ai_node*  inner_layer;       /*!< inner layer to process */
} ai_layer_time_distributed;


/******************************************************************************/
/* Forward Functions Section                                                  */
/******************************************************************************/

/*!
 * @brief Computes the activations of a TimeDelay layer.
 * @ingroup layers_generic
 * @param layer the time delay layer
 */
AI_INTERNAL_API
void forward_time_delay(ai_layer* layer);

/*!
 * @brief Split network computation in N parallel branches.
 * @ingroup layers_generic
 * @param layer the split layer
 */
AI_INTERNAL_API
void forward_split(ai_layer* layer);

/*!
 * @brief Add network computation from N parallel branches.
 * @ingroup layers_generic
 * @param layer the add layer
 */
AI_INTERNAL_API
void forward_add(ai_layer* layer);

/*!
 * @brief Permute a tensor along a pivot and save permuted values into an output
 * tensor
 * @ingroup layers_generic
 * @param layer the permute layer
 */
AI_INTERNAL_API
void forward_permute(ai_layer* layer);

/*!
 * @brief TimeDistrubuted forward layer function. This forward function
 * implements the timedistributed layer. 
 * @ingroup layers_generic
 * @param layer the time distributed layer
 */
AI_INTERNAL_API
void forward_time_distributed(ai_layer* layer);

AI_API_DECLARE_END

#endif    /*__LAYERS_GENERIC_H_*/
