/**
  ******************************************************************************
  * @file    network.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Wed Oct 24 15:56:22 2018
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
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



#include "network.h"

#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "layers.h"

#undef AI_TOOLS_VERSION_MAJOR
#undef AI_TOOLS_VERSION_MINOR
#undef AI_TOOLS_VERSION_MICRO
#define AI_TOOLS_VERSION_MAJOR 3
#define AI_TOOLS_VERSION_MINOR 2
#define AI_TOOLS_VERSION_MICRO 0

#undef AI_TOOLS_API_VERSION_MAJOR
#undef AI_TOOLS_API_VERSION_MINOR
#undef AI_TOOLS_API_VERSION_MICRO
#define AI_TOOLS_API_VERSION_MAJOR 1
#define AI_TOOLS_API_VERSION_MINOR 0
#define AI_TOOLS_API_VERSION_MICRO 0

#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_network
 
#undef AI_NETWORK_MODEL_SIGNATURE
#define AI_NETWORK_MODEL_SIGNATURE     "e0ad7fe945575fb10f5bc61a180ff11a"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     "(rev-)"
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "Wed Oct 24 15:56:22 2018"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_NETWORK_N_BATCHES
#define AI_NETWORK_N_BATCHES         (1)

/* Forward network declaration */
AI_STATIC ai_network AI_NET_OBJ_INSTANCE;


/* Forward network arrays declaration */
AI_STATIC ai_array input_0_output_array;   /* Array #1 */
AI_STATIC ai_array conv2d_3_output_array;   /* Array #2 */
AI_STATIC ai_array conv2d_4_output_array;   /* Array #3 */
AI_STATIC ai_array conv2d_5_output_array;   /* Array #4 */
AI_STATIC ai_array conv2d_6_output_in_array;   /* Array #5 */
AI_STATIC ai_array conv2d_6_output_out_array;   /* Array #6 */
AI_STATIC ai_array dense_3_output_array;   /* Array #7 */
AI_STATIC ai_array dense_3_nl_output_array;   /* Array #8 */
AI_STATIC ai_array dense_4_output_array;   /* Array #9 */
AI_STATIC ai_array dense_4_nl_output_array;   /* Array #10 */


/* Forward network tensors declaration */
AI_STATIC ai_tensor input_0_output;   /* Tensor #1 */
AI_STATIC ai_tensor conv2d_3_output;   /* Tensor #2 */
AI_STATIC ai_tensor conv2d_4_output;   /* Tensor #3 */
AI_STATIC ai_tensor conv2d_5_output;   /* Tensor #4 */
AI_STATIC ai_tensor conv2d_6_output_in;   /* Tensor #5 */
AI_STATIC ai_tensor conv2d_6_output_out;   /* Tensor #6 */
AI_STATIC ai_tensor dense_3_output;   /* Tensor #7 */
AI_STATIC ai_tensor dense_3_nl_output;   /* Tensor #8 */
AI_STATIC ai_tensor dense_4_output;   /* Tensor #9 */
AI_STATIC ai_tensor dense_4_nl_output;   /* Tensor #10 */


/* Subgraph network operators declaration */



/* Forward network layers declaration */
AI_STATIC ai_layer_conv2d conv2d_3_layer; /* Layer #1 */
AI_STATIC ai_layer_conv2d_nl_pool conv2d_4_layer; /* Layer #2 */
AI_STATIC ai_layer_conv2d conv2d_5_layer; /* Layer #3 */
AI_STATIC ai_layer_conv2d_nl_pool conv2d_6_layer; /* Layer #4 */
AI_STATIC ai_layer_dense dense_3_layer; /* Layer #5 */
AI_STATIC ai_layer_nl dense_3_nl_layer; /* Layer #6 */
AI_STATIC ai_layer_dense dense_4_layer; /* Layer #7 */
AI_STATIC ai_layer_sm dense_4_nl_layer; /* Layer #8 */


/* Arrays declarations section */
AI_ARRAY_OBJ_DECLARE(
  input_0_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 784,
  AI_STATIC )
AI_ARRAY_OBJ_DECLARE(
  conv2d_3_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 5408,
  AI_STATIC )
AI_ARRAY_OBJ_DECLARE(
  conv2d_4_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 1152,
  AI_STATIC )
AI_ARRAY_OBJ_DECLARE(
  conv2d_5_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 1600,
  AI_STATIC )
AI_ARRAY_OBJ_DECLARE(
  conv2d_6_output_in_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 256,
  AI_STATIC )
AI_ARRAY_OBJ_DECLARE(
  conv2d_6_output_out_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 256,
  AI_STATIC )
AI_ARRAY_OBJ_DECLARE(
  dense_3_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 240,
  AI_STATIC )
AI_ARRAY_OBJ_DECLARE(
  dense_3_nl_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 240,
  AI_STATIC )
AI_ARRAY_OBJ_DECLARE(
  dense_4_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 36,
  AI_STATIC )
AI_ARRAY_OBJ_DECLARE(
  dense_4_nl_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 36,
  AI_STATIC )


/* Activations tensors declaration section */
AI_TENSOR_OBJ_DECLARE(
  input_0_output,
  AI_SHAPE_INIT(28, 28, 1, 1),
  AI_STRIDE_INIT(112, 4, 4, 4),
  &input_0_output_array,
  &input_0_output,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  conv2d_3_output,
  AI_SHAPE_INIT(26, 26, 8, 1),
  AI_STRIDE_INIT(832, 32, 4, 4),
  &conv2d_3_output_array,
  &conv2d_3_output,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  conv2d_4_output,
  AI_SHAPE_INIT(12, 12, 8, 1),
  AI_STRIDE_INIT(384, 32, 4, 4),
  &conv2d_4_output_array,
  &conv2d_4_output,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  conv2d_5_output,
  AI_SHAPE_INIT(10, 10, 16, 1),
  AI_STRIDE_INIT(640, 64, 4, 4),
  &conv2d_5_output_array,
  &conv2d_5_output,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  conv2d_6_output_in,
  AI_SHAPE_INIT(4, 4, 16, 1),
  AI_STRIDE_INIT(256, 64, 4, 4),
  &conv2d_6_output_in_array,
  &conv2d_6_output_in,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  conv2d_6_output_out,
  AI_SHAPE_INIT(1, 1, 256, 1),
  AI_STRIDE_INIT(1024, 1024, 4, 4),
  &conv2d_6_output_out_array,
  &conv2d_6_output_out,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  dense_3_output,
  AI_SHAPE_INIT(1, 1, 240, 1),
  AI_STRIDE_INIT(960, 960, 4, 4),
  &dense_3_output_array,
  &dense_3_output,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  dense_3_nl_output,
  AI_SHAPE_INIT(1, 1, 240, 1),
  AI_STRIDE_INIT(960, 960, 4, 4),
  &dense_3_nl_output_array,
  &dense_3_nl_output,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  dense_4_output,
  AI_SHAPE_INIT(1, 1, 36, 1),
  AI_STRIDE_INIT(144, 144, 4, 4),
  &dense_4_output_array,
  &dense_4_output,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  dense_4_nl_output,
  AI_SHAPE_INIT(1, 1, 36, 1),
  AI_STRIDE_INIT(144, 144, 4, 4),
  &dense_4_nl_output_array,
  &dense_4_nl_output,
  AI_STATIC)




/* Layer #1: "conv2d_3" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_3_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 72,
  AI_STATIC )

AI_TENSOR_OBJ_DECLARE(
  conv2d_3_weights,
  AI_SHAPE_INIT(8, 3, 3, 1),
  AI_STRIDE_INIT(36, 12, 4, 4),
  &conv2d_3_weights_array,
  &conv2d_3_weights,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_3_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 8,
  AI_STATIC )

AI_TENSOR_OBJ_DECLARE(
  conv2d_3_bias,
  AI_SHAPE_INIT(1, 1, 8, 1),
  AI_STRIDE_INIT(32, 32, 4, 4),
  &conv2d_3_bias_array,
  &conv2d_3_bias,
  AI_STATIC)

AI_LAYER_OBJ_DECLARE(
  conv2d_3_layer, 0,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_4_layer, AI_STATIC ,
  .in = &input_0_output, .out = &conv2d_3_output, 
  .weights = &conv2d_3_weights, 
  .bias = &conv2d_3_bias, 
  .groups = 1, 
  .nl_func = nl_func_relu_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 0), 
);
/* Layer #2: "conv2d_4" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_4_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 576,
  AI_STATIC )

AI_TENSOR_OBJ_DECLARE(
  conv2d_4_weights,
  AI_SHAPE_INIT(8, 3, 3, 8),
  AI_STRIDE_INIT(288, 96, 32, 4),
  &conv2d_4_weights_array,
  &conv2d_4_weights,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_4_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 8,
  AI_STATIC )

AI_TENSOR_OBJ_DECLARE(
  conv2d_4_bias,
  AI_SHAPE_INIT(1, 1, 8, 1),
  AI_STRIDE_INIT(32, 32, 4, 4),
  &conv2d_4_bias_array,
  &conv2d_4_bias,
  AI_STATIC)

AI_LAYER_OBJ_DECLARE(
  conv2d_4_layer, 1,
  OPTIMIZED_CONV2D_TYPE,
  conv2d_nl_pool, forward_conv2d_nl_pool,
  &AI_NET_OBJ_INSTANCE, &conv2d_5_layer, AI_STATIC ,
  .in = &conv2d_3_output, .out = &conv2d_4_output, 
  .weights = &conv2d_4_weights, 
  .bias = &conv2d_4_bias, 
  .groups = 1, 
  .nl_func = nl_func_relu_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 0), 
  .filter_pad_r = AI_SHAPE_2D_INIT(0, 0), 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_2D_INIT(0, 0), 
  .pool_func = pool_func_mp_array_f32, 
);
/* Layer #3: "conv2d_5" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_5_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 1152,
  AI_STATIC )

AI_TENSOR_OBJ_DECLARE(
  conv2d_5_weights,
  AI_SHAPE_INIT(16, 3, 3, 8),
  AI_STRIDE_INIT(288, 96, 32, 4),
  &conv2d_5_weights_array,
  &conv2d_5_weights,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_5_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 16,
  AI_STATIC )

AI_TENSOR_OBJ_DECLARE(
  conv2d_5_bias,
  AI_SHAPE_INIT(1, 1, 16, 1),
  AI_STRIDE_INIT(64, 64, 4, 4),
  &conv2d_5_bias_array,
  &conv2d_5_bias,
  AI_STATIC)

AI_LAYER_OBJ_DECLARE(
  conv2d_5_layer, 3,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_6_layer, AI_STATIC ,
  .in = &conv2d_4_output, .out = &conv2d_5_output, 
  .weights = &conv2d_5_weights, 
  .bias = &conv2d_5_bias, 
  .groups = 1, 
  .nl_func = nl_func_relu_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 0), 
);
/* Layer #4: "conv2d_6" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_6_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 2304,
  AI_STATIC )

AI_TENSOR_OBJ_DECLARE(
  conv2d_6_weights,
  AI_SHAPE_INIT(16, 3, 3, 16),
  AI_STRIDE_INIT(576, 192, 64, 4),
  &conv2d_6_weights_array,
  &conv2d_6_weights,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_6_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 16,
  AI_STATIC )

AI_TENSOR_OBJ_DECLARE(
  conv2d_6_bias,
  AI_SHAPE_INIT(1, 1, 16, 1),
  AI_STRIDE_INIT(64, 64, 4, 4),
  &conv2d_6_bias_array,
  &conv2d_6_bias,
  AI_STATIC)

AI_LAYER_OBJ_DECLARE(
  conv2d_6_layer, 4,
  OPTIMIZED_CONV2D_TYPE,
  conv2d_nl_pool, forward_conv2d_nl_pool,
  &AI_NET_OBJ_INSTANCE, &dense_3_layer, AI_STATIC ,
  .in = &conv2d_5_output, .out = &conv2d_6_output_in, 
  .weights = &conv2d_6_weights, 
  .bias = &conv2d_6_bias, 
  .groups = 1, 
  .nl_func = nl_func_relu_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 0), 
  .filter_pad_r = AI_SHAPE_2D_INIT(0, 0), 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_2D_INIT(0, 0), 
  .pool_func = pool_func_mp_array_f32, 
);
/* Layer #5: "dense_3" (Dense) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_weights_array, AI_DATA_FORMAT_LUT8_FLOAT, 
  NULL, NULL, 61440,
  AI_STATIC )

AI_TENSOR_OBJ_DECLARE(
  dense_3_weights,
  AI_SHAPE_INIT(1, 1, 240, 256),
  AI_STRIDE_INIT(61440, 61440, 256, 1),
  &dense_3_weights_array,
  &dense_3_weights,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 240,
  AI_STATIC )

AI_TENSOR_OBJ_DECLARE(
  dense_3_bias,
  AI_SHAPE_INIT(1, 1, 240, 1),
  AI_STRIDE_INIT(960, 960, 4, 4),
  &dense_3_bias_array,
  &dense_3_bias,
  AI_STATIC)

AI_LAYER_OBJ_DECLARE(
  dense_3_layer, 8,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_3_nl_layer, AI_STATIC ,
  .in = &conv2d_6_output_out, .out = &dense_3_output, 
  .weights = &dense_3_weights, 
  .bias = &dense_3_bias, 
);
/* Layer #6: "dense_3_nl" (Nonlinearity) */
  

AI_LAYER_OBJ_DECLARE(
  dense_3_nl_layer, 8,
  NL_TYPE,
  nl, forward_relu,
  &AI_NET_OBJ_INSTANCE, &dense_4_layer, AI_STATIC ,
  .in = &dense_3_output, .out = &dense_3_nl_output, 
);
/* Layer #7: "dense_4" (Dense) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  dense_4_weights_array, AI_DATA_FORMAT_LUT8_FLOAT, 
  NULL, NULL, 8640,
  AI_STATIC )

AI_TENSOR_OBJ_DECLARE(
  dense_4_weights,
  AI_SHAPE_INIT(1, 1, 36, 240),
  AI_STRIDE_INIT(8640, 8640, 240, 1),
  &dense_4_weights_array,
  &dense_4_weights,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  dense_4_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 36,
  AI_STATIC )

AI_TENSOR_OBJ_DECLARE(
  dense_4_bias,
  AI_SHAPE_INIT(1, 1, 36, 1),
  AI_STRIDE_INIT(144, 144, 4, 4),
  &dense_4_bias_array,
  &dense_4_bias,
  AI_STATIC)

AI_LAYER_OBJ_DECLARE(
  dense_4_layer, 10,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_4_nl_layer, AI_STATIC ,
  .in = &dense_3_nl_output, .out = &dense_4_output, 
  .weights = &dense_4_weights, 
  .bias = &dense_4_bias, 
);
/* Layer #8: "dense_4_nl" (Nonlinearity) */
  

AI_LAYER_OBJ_DECLARE(
  dense_4_nl_layer, 10,
  SM_TYPE,
  sm, forward_sm,
  &AI_NET_OBJ_INSTANCE, &dense_4_nl_layer, AI_STATIC ,
  .in = &dense_4_output, .out = &dense_4_nl_output, 
);


AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE,
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 89840, 1,
                     NULL),
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 26276, 1,
                     NULL),
  &input_0_output, &dense_4_nl_output,
  &conv2d_3_layer, 0)


AI_DECLARE_STATIC
ai_bool network_configure_activations(
  ai_network* net_ctx, const ai_buffer* activation_buffer)
{
  AI_ASSERT(net_ctx &&  activation_buffer && activation_buffer->data)

  ai_ptr activations = AI_PTR(AI_PTR_ALIGN(activation_buffer->data, 4));
  AI_ASSERT( activations )
  AI_FLAG_SET(net_ctx->flags, AI_NETWORK_FLAG_OUT_COPY);

  {
    /* Updating activations (byte) offsets */
    input_0_output_array.data = NULL;
  input_0_output_array.data_start = NULL;
  conv2d_3_output_array.data = activations + 0;
  conv2d_3_output_array.data_start = activations + 0;
  conv2d_4_output_array.data = activations + 21632;
  conv2d_4_output_array.data_start = activations + 21632;
  conv2d_5_output_array.data = activations + 0;
  conv2d_5_output_array.data_start = activations + 0;
  conv2d_6_output_in_array.data = activations + 6400;
  conv2d_6_output_in_array.data_start = activations + 6400;
  conv2d_6_output_out_array.data = activations + 6400;
  conv2d_6_output_out_array.data_start = activations + 6400;
  dense_3_output_array.data = activations + 0;
  dense_3_output_array.data_start = activations + 0;
  dense_3_nl_output_array.data = activations + 0;
  dense_3_nl_output_array.data_start = activations + 0;
  dense_4_output_array.data = activations + 960;
  dense_4_output_array.data_start = activations + 960;
  dense_4_nl_output_array.data = activations + 960;
  dense_4_nl_output_array.data_start = activations + 960;
  
  }
  return true;
}

AI_DECLARE_STATIC
ai_bool network_configure_weights(
  ai_network* net_ctx, const ai_buffer* weights_buffer)
{
  AI_ASSERT(net_ctx &&  weights_buffer && weights_buffer->data)

  ai_ptr weights = AI_PTR(weights_buffer->data);
  AI_ASSERT( weights )

  {
    /* Updating weights (byte) offsets */
    conv2d_3_weights_array.format |= AI_FMT_FLAG_CONST;
  conv2d_3_weights_array.data = weights + 0;
  conv2d_3_weights_array.data_start = weights + 0;
  conv2d_3_bias_array.format |= AI_FMT_FLAG_CONST;
  conv2d_3_bias_array.data = weights + 288;
  conv2d_3_bias_array.data_start = weights + 288;
  conv2d_4_weights_array.format |= AI_FMT_FLAG_CONST;
  conv2d_4_weights_array.data = weights + 320;
  conv2d_4_weights_array.data_start = weights + 320;
  conv2d_4_bias_array.format |= AI_FMT_FLAG_CONST;
  conv2d_4_bias_array.data = weights + 2624;
  conv2d_4_bias_array.data_start = weights + 2624;
  conv2d_5_weights_array.format |= AI_FMT_FLAG_CONST;
  conv2d_5_weights_array.data = weights + 2656;
  conv2d_5_weights_array.data_start = weights + 2656;
  conv2d_5_bias_array.format |= AI_FMT_FLAG_CONST;
  conv2d_5_bias_array.data = weights + 7264;
  conv2d_5_bias_array.data_start = weights + 7264;
  conv2d_6_weights_array.format |= AI_FMT_FLAG_CONST;
  conv2d_6_weights_array.data = weights + 7328;
  conv2d_6_weights_array.data_start = weights + 7328;
  conv2d_6_bias_array.format |= AI_FMT_FLAG_CONST;
  conv2d_6_bias_array.data = weights + 16544;
  conv2d_6_bias_array.data_start = weights + 16544;
  dense_3_weights_array.format |= AI_FMT_FLAG_CONST;
  dense_3_weights_array.data = weights + 16608;
  dense_3_weights_array.data_start = weights + 78048;
  dense_3_bias_array.format |= AI_FMT_FLAG_CONST;
  dense_3_bias_array.data = weights + 79072;
  dense_3_bias_array.data_start = weights + 79072;
  dense_4_weights_array.format |= AI_FMT_FLAG_CONST;
  dense_4_weights_array.data = weights + 80032;
  dense_4_weights_array.data_start = weights + 88672;
  dense_4_bias_array.format |= AI_FMT_FLAG_CONST;
  dense_4_bias_array.data = weights + 89696;
  dense_4_bias_array.data_start = weights + 89696;
  
  }

  return true;
}

/*** PUBLIC APIs SECTION  *****************************************************/

AI_API_ENTRY
ai_bool ai_network_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if ( report && net_ctx )
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = {AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR,
                            AI_TOOLS_API_VERSION_MICRO, 0x0},

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 732284,
      .n_inputs          = AI_NETWORK_IN_NUM,
      .inputs            = AI_BUFFER_OBJ_INIT(
                              AI_BUFFER_FORMAT_FLOAT,
                              28,
                              28,
                              1,
                              1, NULL),
      .n_outputs         = AI_NETWORK_OUT_NUM,
      .outputs           = AI_BUFFER_OBJ_INIT(
                              AI_BUFFER_FORMAT_FLOAT,
                              1,
                              1,
                              36,
                              1, NULL),
      .activations       = net_ctx->activations,
      .weights           = net_ctx->params,
      .n_nodes           = 0,
      .signature         = net_ctx->signature,
    };

    AI_FOR_EACH_NODE_DO(node, net_ctx->input_node)
    {
      r.n_nodes++;
    }

    *report = r;

    return ( r.n_nodes>0 ) ? true : false;
  }
  
  return false;
}

AI_API_ENTRY
ai_error ai_network_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}

AI_API_ENTRY
ai_error ai_network_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    &AI_NET_OBJ_INSTANCE,
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}

AI_API_ENTRY
ai_handle ai_network_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}

AI_API_ENTRY
ai_bool ai_network_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = ai_platform_network_init(network, params);
  if ( !net_ctx ) return false;

  ai_bool ok = network_configure_weights(net_ctx, &params->params);

  ok &= network_configure_activations(net_ctx, &params->activations);
  
  return ok;
}


AI_API_ENTRY
ai_i32 ai_network_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}

AI_API_ENTRY
ai_i32 ai_network_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}

#undef AI_NETWORK_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_VERSION_MAJOR
#undef AI_TOOLS_VERSION_MINOR
#undef AI_TOOLS_VERSION_MICRO
#undef AI_TOOLS_API_VERSION_MAJOR
#undef AI_TOOLS_API_VERSION_MINOR
#undef AI_TOOLS_API_VERSION_MICRO
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

