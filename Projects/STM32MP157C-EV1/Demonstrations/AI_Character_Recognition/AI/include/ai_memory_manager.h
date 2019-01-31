/**
  ******************************************************************************
  * @file    ai_memory_manager.h
  * @author  AST Embedded Analytics Research Platform
  * @date    18-Jun-2018
  * @brief   AI Library Memory Management Wrappers
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

#ifndef __AI_MEMORY_MANAGER_H__
#define __AI_MEMORY_MANAGER_H__
#pragma once

#include <string.h>  /* memcpy */
#include <stdlib.h>

#include "ai_datatypes_defines.h"

/*!
 * @section MemoryManager
 * @ingroup ai_memory_manager
 * Macros to handle memory allocation and management as generic wrappers.
 * Dynamic allocations, freeing, clearing and copy are provided.
 * @{
 */

#define AI_MEM_ALLOC(size, type) \
          ((type*)malloc((size)*sizeof(type)))          

#define AI_MEM_FREE(ptr) \
          { free((void*)(ptr)); }

#define AI_MEM_CLEAR(ptr, size) \
          { memset((void*)(ptr), 0, (size)); }

#define AI_MEM_COPY(dst, src, size) \
          { memcpy((void*)(dst), (const void*)(src), (size)); }

#define AI_MEM_MOVE(dst, src, size) \
          { memmove((void*)(dst), (const void*)(src), (size)); }

/*!
 * @brief Copy an array into another.
 * @ingroup ai_memory_manager
 * @param src the source array handle
 * @param dst the destination array handle
 * @param size the size in byte of the two arrays
 * @return a pointer to the destination buffer
 */
AI_DECLARE_STATIC
ai_handle ai_mem_copy_buffer(
  ai_handle dst, const ai_handle src, const ai_size byte_size)
{
  AI_ASSERT(src && dst && byte_size>0)
  AI_MEM_COPY(dst, src, byte_size)

  return dst;
}

/*! @} */
          
#endif    /*__AI_MEMORY_MANAGER_H__*/
