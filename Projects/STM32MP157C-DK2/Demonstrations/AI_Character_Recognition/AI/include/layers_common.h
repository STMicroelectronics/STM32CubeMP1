/**
  ******************************************************************************
  * @file    layers_common.h
  * @author  AST Embedded Analytics Research Platform
  * @date    17-Nov-2017
  * @brief   header file of AI platform layers datatypes
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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

#ifndef __LAYERS_COMMON_H_
#define __LAYERS_COMMON_H_
#pragma once

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef USE_CYCLE_MEASUREMENTS
  #include "layers_cycles_estimation.h"
#endif
#include "ai_platform.h"
#include "ai_common_config.h"
#include "ai_datatypes_internal.h"

#include "core_common.h"

/* Activation of optimizations */
#define AI_OPTIM_DICT8_DOT_ARRAY_F32    (1)
#define AI_OPTIM_DICT8_DTCM             (1)
#define AI_OPTIM_FUNC_MP_ARRAY_F32      (0)

/* standard checks for in-place layers */
#define ASSERT_LAYER_INPLACE(l) \
  do { \
    AI_ASSERT(l->in && l->out) \
    AI_ASSERT(l->in->data && l->out->data) \
    AI_ASSERT(ai_shape_get_size(&l->in->shape) <= l->in->data->size)  \
    AI_ASSERT(ai_shape_get_size(&l->out->shape) <= l->out->data->size) \
  } while (0);


#define AI_LAYER_OBJ(obj)       AI_NODE_OBJ(obj)

#define AI_LAYER_FORWARD_FUNC(func_)     ((layer_forward_func)(func_))

#define AI_LAYER_TYPE_NAME(type_) \
  ai_layer_type_name((ai_layer_type)(type_))
  
#define AI_LAYER_TYPE_IS_VALID(type_) \
  ai_layer_type_is_valid(type_)

#define AI_LAYER_SEQUENTIAL_FIELDS_DECLARE AI_NODE_SEQUENTIAL_FIELDS_DECLARE


#define AI_LAYER_OBJ_INIT(type_, id_, network_, next_, forward_, ...) { \
  AI_NODE_COMMON_INIT(AI_CONCAT(AI_LAYER_, type_), id_, forward_, next_, network_, NULL), \
  ## __VA_ARGS__ }

#define AI_LAYER_OBJ_DECLARE(varname_, id_, type_, struct_, forward_func_, \
                             network_, next_, attr_, ...) \
  AI_ALIGNED(4) \
  attr_ AI_CONCAT(ai_layer_, struct_) varname_ = \
    AI_LAYER_OBJ_INIT( type_, id_, network_, \
      next_, forward_func_, \
      ## __VA_ARGS__ );

#define AI_LAYER_TYPE_ENTRY(type_)  \
   AI_CONCAT(AI_CONCAT(AI_LAYER_, type_), _TYPE)


AI_API_DECLARE_BEGIN

/*!
 * @defgroup layers_common Layers Common
 * @brief Implementation of the common layers datastructures 
 * This header enumerates the layers specific definition implemented in the
 * library toghether with the macros and datatypes used to manipulate them. 
 */

/*!
 * @enum ai_layer_type
 * @ingroup layers
 * @brief ai_tools supported layers type id
 */
typedef enum {
#define LAYER_ENTRY(type_, id_, struct_, forward_func_) \
   AI_LAYER_TYPE_ENTRY(type_) = id_,
#include "layers_list.h"

} ai_layer_type;

/*!
 * @typedef void (*layer_forward_func)(ai_node* layer)
 * @ingroup layers_common
 * @brief Callback signatures for all layers forward functions
 */
typedef node_forward_func layer_forward_func;

/*!
 * @struct ai_layer
 * @ingroup layers_common
 * @brief Structure encoding a layer in the network
 *
 * The layer struct is an alias for a generic @ref ai_node datastrutcture
 */
typedef ai_node ai_layer;

/*!
 * @struct ai_layer_sequential
 * @ingroup layers_common
 * @brief Abstract datatypes definition to cluster together all layer that has
 * a single input tensor and a single output tensor
 * see for example @ref ai_layer_sm, @ref ai_layer_nl
 */
typedef ai_node_sequential ai_layer_sequential;

/*!
 * @brief Check the custom network types against the internally compiled ones
 * Helper function to check if the private APIs where compiled with a different
 * `datatypes_network.h` than the one provided to the caller.
 * @ingroup layers_common
 * @param signatures list of type sizes signatures (first element is the number of types)
 * @return false if there is a type size mismatch
 */
AI_INTERNAL_API
ai_bool ai_check_custom_types(const ai_custom_type_signature* signatures);

/*!
 * @brief Helper API to retrieve a human readable layer type from enum
 * @ingroup layers_common
 * @param type in type of layer
 * @return string defining the type of the layer
 */
AI_INTERNAL_API
const char* ai_layer_type_name(const ai_layer_type type);

/*!
 * @brief Helper API to check if a node is a valid layer type
 * @ingroup layers_common
 * @param type in type of layer
 * @return true if the layer is one of the ones listed in the enum, 
 * false otherwise
 */
AI_INTERNAL_API
ai_bool ai_layer_type_is_valid(const ai_layer_type type);

AI_API_DECLARE_END

#endif /* __LAYERS_COMMON_H_ */
