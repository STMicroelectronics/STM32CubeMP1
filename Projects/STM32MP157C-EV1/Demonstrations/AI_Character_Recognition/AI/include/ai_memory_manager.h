/**
  ******************************************************************************
  * @file    ai_memory_manager.h
  * @author  AST Embedded Analytics Research Platform
  * @date    18-Jun-2018
  * @brief   AI Library Memory Management Wrappers
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
