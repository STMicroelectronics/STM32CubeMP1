/**
  ******************************************************************************
  * @file    layers_list.h
  * @author  AST Embedded Analytics Research Platform
  * @date    20-Jul-2018
  * @brief   header file of AI platform layers datatypes
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


/* No sentry. This is deliberate!! */
/* Template: LAYER_ENTRY(type_, id_, struct_, forward_func_) 
 * Where:
 *  - type_ is the (enum) type name of the layer. to have the complete enum 
 *      value you should use the macro @ref AI_LAYER_TYPE_ENTRY(type_) that adds
 *      the specific prefix and postfix tokens to the type_
 *  - id_ is the numeric id of the layer
 *  - struct_ is the name of the datastruct of the layer
 *  - forward_func_ is the forward function name of the routine implementing
 *        actual layer processing
 */

/*!< Elementwise addition layer */
LAYER_ENTRY(ADD, 10001, ai_layer_add, forward_add)
 /*!< Batch normalization layer */
LAYER_ENTRY(BN, 10002, ai_layer_bn,  forward_bn)
/*!< 2D Convolutional layer */
LAYER_ENTRY(CONV2D, 10004, ai_layer_conv2d, forward_conv2d)
/*!< Dense layer */
LAYER_ENTRY(DENSE, 10005, ai_layer_dense, forward_dense)
/*!< Gated Recurrent Unit layer */
LAYER_ENTRY(GRU, 10006, ai_layer_gru,  forward_gru)
/*!< Local Response Normalization layer */            
LAYER_ENTRY(LRN, 10007, ai_layer_lrn,  forward_lrn)
/*!< Long Short Time Memory layer */
LAYER_ENTRY(LSTM, 10008, ai_layer_lstm,  forward_lstm)
/*!< Nonlinearity layer */ 
LAYER_ENTRY(NL, 10009, ai_layer_nl, forward_nl)
/*!< Normalization layer */
LAYER_ENTRY(NORM, 10010, ai_layer_norm, forward_norm)
/*!< Merged Conv2d / Pool layer */
LAYER_ENTRY(OPTIMIZED_CONV2D, 10011, ai_layer_conv2d_nl_pool,  forward__conv2d_nl_pool)
/*!< Permute Tensor layer */
LAYER_ENTRY(PERMUTE, 10012, ai_layer_permute,  forward_permute)
/*!< Pooling layer */
LAYER_ENTRY(POOL, 10013, ai_layer_pool, forward_pool)
/*!< Softmax layer */
LAYER_ENTRY(SM, 10014, ai_layer_sm, forward_sm)
/*!< Slice layer */
LAYER_ENTRY(SPLIT, 10015, ai_layer_split, forward_split)
/*!< TimeDelay layer */
LAYER_ENTRY(TIME_DELAY, 10016, ai_layer_time_delay,  forward_time_delay)
/*!< TimeDistributed layer */
LAYER_ENTRY(TIME_DISTRIBUTED, 10017, ai_layer_time_distributed,  forward_time_distributed)
#ifdef USE_OPERATORS
/*!< Container layer for operators */
LAYER_ENTRY(CONTAINER, 10003, ai_layer_container,  forward_container)
/*!< Container layer for operators */
LAYER_ENTRY(LAMBDA, 10018, ai_layer_lambda,  forward_lambda)
#endif
#undef LAYER_ENTRY
