/**
  ******************************************************************************
  * @file    datatypes_network.h
  * @author  AST Embedded Analytics Research Platform
  * @date    30-Aug-2017
  * @brief   Definitions of code generated network types 
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

#ifndef __DATATYPES_NETWORK_H__
#define __DATATYPES_NETWORK_H__
#pragma once

/*
 * Header to be overriden by the generated version
 * by including with <> the include directories are searched in the order
 * specified in the compiler
 * To enable the override, put the generated path before the API path
 */

#include "ai_platform.h"

AI_API_DECLARE_BEGIN

#ifdef AI_OVERRIDE_CUSTOM_TYPES
#warning "Warning: Custom Types have been already defined!\n"
#endif

#define AI_CUSTOM_TYPES_COUNT      (3)

#define AI_CUSTOM_TYPES_SIGNATURE_DECLARE(name)  \
  const ai_custom_type_signature name[AI_CUSTOM_TYPES_COUNT+1] = { \
    AI_CUSTOM_TYPES_COUNT, \
    AI_CUSTOM_SIZE(ai_shape_dimension), \
    AI_CUSTOM_SIZE(ai_stride_dimension), \
    AI_CUSTOM_SIZE(ai_array_size), \
  };


typedef ai_u32 ai_shape_dimension;
typedef ai_i32 ai_stride_dimension;
typedef ai_u32 ai_array_size;


AI_API_DECLARE_END

#endif /*__DATATYPES_NETWORK_H__*/
