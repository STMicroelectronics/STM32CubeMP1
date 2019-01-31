/**
  ******************************************************************************
  * @file    core_utils.h
  * @author  AST Embedded Analytics Research Platform
  * @date    16-Aug-2018
  * @brief   header file of core utils routines
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

#ifndef __CORE_UTILS_H_
#define __CORE_UTILS_H_
#pragma once

#include "ai_platform.h"
#include "ai_platform_interface.h"

#include "core_common.h"

AI_API_DECLARE_BEGIN

/*!
 * @defgroup core_utils Core Utils Routines
 * @brief Implementation of core utils such has checksums algorithms, etc. 
 */


/*!
 * @brief Computes 32bit checksum from a buffer array of bytes
 * @ingroup core_utils
 * @param[in] buffer in an opaque handler to the buffer we want to compute crc code
 * @param[in] byte_size the size in byte of the buffer provided
 */
AI_INTERNAL_API
ai_u32    core_utils_compute_crc32(
  const ai_handle buffer, const ai_size byte_size);

/*!
 * @brief Computes network signature given a network context
 * @ingroup core_utils
 * @param[in] net_ctx a pointer to the network context to be signed
 * @param[out] signature a pointer to the checksum signature computed 
 * from the network context
 * @return false if failed to compute the signature, true otherwise
 */
AI_INTERNAL_API
ai_bool    core_utils_generate_network_signature(
  const ai_network* net_ctx, ai_signature* signature);

AI_API_DECLARE_END

#endif    /*__CORE_UTILS_H_*/
