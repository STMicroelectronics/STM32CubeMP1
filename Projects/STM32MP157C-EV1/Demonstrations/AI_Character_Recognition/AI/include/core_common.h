/**
  ******************************************************************************
  * @file    core_common.h
  * @author  AST Embedded Analytics Research Platform
  * @date    20-Lug-2018
  * @brief   header file of common core datatypes
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

#ifndef __CORE_COMMON_H_
#define __CORE_COMMON_H_
#pragma once

#include "ai_platform.h"
#include "ai_platform_interface.h"
#include "ai_datatypes_internal.h"

#include "core_datatypes.h"
#include "core_log.h"

/*!
 * @defgroup core_common Common Core Library Routines
 * @brief Common macros, datatypes and routines of core common module
 * @details This module contains the definitons and handling of the @ref ai_node
 * datastructures. An ai_node is a generic abstraction for a network node that 
 * could be either a fixed function layer or an operator. Ideally the platform 
 * interface defined in api module should handle an process generic nodes in the
 * network, not relying on the fact that they are layers or operators datastructs
 * Specific implementative details should be kept inside layers and operators
 * modules. The core module implements additionally common routines used in the
 * layers and operators modules.
 */

#define AI_NODE_TYPE(type)    ( (ai_node_type)((ai_u32)(type)&0xFFFF) )

#define AI_NODE_OBJ(obj)      ((ai_node*)(obj))

#define AI_NODE_FORWARD_FUNC(func_)     ((node_forward_func)(func_))
          
#define AI_NODE_IS_FIRST(node) \
  (AI_NODE_OBJ(node)==AI_NODE_OBJ(AI_NODE_OBJ(node)->network->input_node))

#define AI_NODE_IS_LAST(node_) \
  ((AI_NODE_OBJ(node_)==AI_NODE_OBJ(node_)->next) || \
   (AI_NODE_OBJ(node_)->next==NULL))

#define AI_NODE_COMMON_FIELDS_DECLARE \
  ai_node_type type;           /*!< node type id (see @ref ai_node_type) */ \
  ai_id_obj id;                 /*!< node object instance id (see @ref ai_id_obj) */ \
  struct ai_network_* network;  /*!< handle to global network context */ \
  struct ai_node_* next;       /*!< the next node object in the sequence */ \
  node_forward_func forward;   /*!< forward function for the node */ \
  ai_klass_obj klass;       /*!< opaque handler to specific layer implementations */ \

#define AI_NODE_SEQUENTIAL_FIELDS_DECLARE \
  AI_NODE_COMMON_FIELDS_DECLARE \
  ai_tensor* in; \
  ai_tensor* out;

#define AI_NODE_COMMON_INIT(type_, id_, forward_, next_, network_, klass_) \
  .type = AI_NODE_TYPE(type_), \
  .id   = AI_ID_OBJ(id_), \
  .network = AI_NETWORK_OBJ(network_), \
  .next = AI_NODE_OBJ(next_), \
  .forward = AI_NODE_FORWARD_FUNC(forward_), \
  .klass = AI_KLASS_OBJ(klass_)

#define AI_NODE_SEQUENTIAL_INIT( \
        type_, forward_, next_, network_, in_, out_, ...) { \
  AI_NODE_COMMON_INIT(type_, 0x0, forward_, next_, network_, NULL), \
  .in = in_, .out = out_, \
  ## __VA_ARGS__ }


#define AI_FOR_EACH_NODE_DO(node_, nodes_) \
  for ( ai_node* node_ = AI_NODE_OBJ(nodes_); (node_); \
        node_ = ((AI_NODE_IS_LAST(node_)) ? NULL : (node_)->next) )

#define AI_TENSOR_IS_LAST(tensor_) \
  ((AI_TENSOR_OBJ(tensor_)==AI_TENSOR_OBJ(tensor_)->next) || \
   (AI_TENSOR_OBJ(tensor_)->next==NULL))

#define AI_FOR_EACH_TENSOR_DO(tensor_, tensors_) \
  for ( ai_tensor* tensor_ = AI_TENSOR_OBJ(tensors_); (tensor_); \
    tensor_ = ((AI_TENSOR_IS_LAST(tensor_)) ? NULL : (tensor_)->next) )

#if 1
  #define SECTION_SERIAL(expr)    expr
  #define SECTION_PARALLEL(expr)
#else
  #define SECTION_SERIAL(expr)
  #define SECTION_PARALLEL(expr)  expr
#endif

AI_API_DECLARE_BEGIN

/*!
 * @struct ai_node_type
 * @ingroup core_common
 * @brief generic network node numeric type ID
 *
 */
typedef uint16_t ai_node_type;

/*!
 * @typedef void (*node_forward_func)(ai_handle node)
 * @ingroup core_common
 * @brief Callback signatures for all forward functions
 */
typedef void (*node_forward_func)(struct ai_node_* node);

/*!
 * @struct ai_node
 * @ingroup core_common
 * @brief Structure encoding a generic node of the network
 *
 * The node struct includes information about the network it belong to, the
 * next node in a sequential network and the forward function. The forward
 * functions are implemented in the @ref layers module.
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_node_ {
  AI_NODE_SEQUENTIAL_FIELDS_DECLARE
} ai_node;

/*!
 * @struct ai_node_sequential
 * @ingroup core_common
 * @brief Abstract datatypes definition to cluster together all nodes that has
 * a single input tensor and a single output tensor. A node is an abstraction
 * datatypes from which a layer or an operator coudl be derived.
 * see for example @ref ai_layer_sm, @ref ai_layer_nl
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_node_sequential_ {
  AI_NODE_SEQUENTIAL_FIELDS_DECLARE
} ai_node_sequential;

/*!
 * @brief initialize core module
 * @ingroup core_common
 * @return false if initialization fails, false otherwise
 */
AI_INTERNAL_API
ai_bool core_init(void);

/*!
 * @brief get 1st error raised during processing 
 * @ingroup core_common
 * @param[out] error the @ref ai_error recorded during processing
 * @return the 1st error generated during processing. If no errors AI_ERROR_NONE
 */
AI_INTERNAL_API
ai_error core_get_error(ai_error* error);

/*!
 * @brief set error recorded during processing
 * @ingroup core_common
 * @param[out] error the @ref ai_error to set
 * @param[in] type the specific error type to set
 * @param[in] code the specific error code to set
 * @return true if the error is set, false in case a precedent error was already 
 * set
 */
AI_INTERNAL_API
ai_bool core_set_error(
  ai_error* error, const ai_error_type type, const ai_error_code code);

AI_API_DECLARE_END

#endif    /*__CORE_COMMON_H_*/
