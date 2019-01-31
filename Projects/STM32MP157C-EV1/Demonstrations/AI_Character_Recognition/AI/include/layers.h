/**
  ******************************************************************************
  * @file    layers.h
  * @author  AST Embedded Analytics Research Platform
  * @date    01-May-2017
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

#ifndef __LAYERS_H_
#define __LAYERS_H_
#pragma once

#include "layers_common.h"
#include "layers_conv2d.h"
#include "layers_generic.h"
#include "layers_nl.h"
#include "layers_norm.h"
#include "layers_pool.h"
#include "layers_rnn.h"

#ifdef USE_OPERATORS
  #include "layers_lambda.h"
#endif /* USE_OPERATORS */


AI_API_DECLARE_BEGIN

/*!
 * @defgroup layers Layers
 * @brief Definition of the forward functions for the layers and the general
 * ai_layer datastructure used to abstract specific layer implementation in the
 * generic forward function definition
 *
 * The forward function for a layer computes the layer activations given the
 * activations of the previous layer. They are added to the layer as function
 * pointer and called implicitly by the @ref ai_layers_forward_all function.
 * The input activations are read from layer &rarr; in and the computed
 * activations stored in layer &rarr; out. The layer type needs to be compatible
 * with the forward function, but layers with the same layout (e.g. `mp` and
 * `ap`) can share the same structure.
 */

/******************************************************************************/
/* Forward Functions Section                                                  */
/******************************************************************************/

/*!
 * @brief Executes a single layer in the network.
 * @ingroup layers
 * @param layer the layer to process
 * @return pointer to the next layer
 */
AI_INTERNAL_API
ai_layer* ai_layers_forward_layer(ai_layer* layer);


/*!
 * @brief Computes the ouptut of the network given the input.
 * @ingroup layers
 *
 * Given a network with the input pre-loaded in the net &rarr; in tensor,
 * computes the output by calling the forward functions of each layer and
 * selecting the next layer. When the layer has no successor or it's in a
 * loop-back configuration (layer &rarr; next is again layer), the function
 * stops. The result is stored in net &rarr; out.
 *
 * @param net the network to evaluate
 */
AI_INTERNAL_API
void ai_layers_forward_all(ai_network* net);

AI_API_DECLARE_END

#endif /* __LAYERS_H_ */
