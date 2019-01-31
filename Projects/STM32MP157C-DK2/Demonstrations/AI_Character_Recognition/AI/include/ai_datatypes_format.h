/**
  ******************************************************************************
  * @file    ai_datatypes_format.h
  * @author  AST Embedded Analytics Research Platform
  * @date    18-Oct-2017
  * @brief   Definitions of AI platform private APIs types 
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

#ifndef __AI_DATATYPES_FORMAT_H__
#define __AI_DATATYPES_FORMAT_H__
#pragma once

#include "ai_platform.h"
#include "ai_datatypes_defines.h"

/*!
 * @defgroup ai_datatypes_format Definiton and Macro of array and buffer formats
 * @brief Type definition and implementation of internal @ref ai_array and 
 * @ref ai_buffer formats.
 * @details The library handles 2 different kind of formats: an internal format
 * that is part of the @ref ai_array struct that is a packed 32bit representation
 * of the format attributes, and a public format (used in public APIs) associated
 * with @ref ai_buffer struct , defined as enum in @ref ai_platform.h,
 * that is just an enum type. Converters are provided in this header file to 
 * convert from one format representation to another.
 * Some MSB bits are reserved in both formats to code some bit flag useful to 
 * declare some special attribute. Two flags are actually implemented in both 
 * formats: the @ref AI_BUFFER_FMT_FLAG_CONST and @ref AI_FMT_FLAG_CONST used
 * to tag read-only memory buffers, and @ref AI_BUFFER_FMT_FLAG_STATIC and
 * @ref AI_FMT_FLAG_STATIC to mark statically allocated memory buffers.
 * All the formats are declared in a proper tuple organize table header named 
 * @ref format_lists.h that enumerates all the formats available for the library.
 * A new format could be added easily by adding a new FMY_ENTRY() as required.
 * The preprocessor automatically generates the code for the handling of the 
 * format according to this tuples entry. A rational for the methodology could
 * be found here:
 *   - https://codecraft.co/2012/10/29/how-enums-spread-disease-and-how-to-cure-it/
 *   
 * The 32bits internal format fields are organized as follows:
 *
 * MSB                                                                       LSB
 * 31     26      25       24       23      21      17        14       7       0
 * /---------------------------------------------------------------------------/
 * / FLAGS |  EXP  |  FLOAT |  SIGN  |  LDIV |  TYPE |  PBITS  |  BITS | FBITS /
 * /---------------------------------------------------------------------------/
 * Where:
 * - FLAGS: is the reserved bits to store additional format attributes
 * - EXP: 1 bit boolean flag to mark a format as public available (i.e. it has
 *      a twinned ai_buffer format declared for public APis)
 * - FLOAT: 1 bit mark the format as floating point type
 * - SIGN : 1 bit mark the format as signed type 
 * - LDIV : 2 bits is a log2 value that is used to compute elements size 
 *      with some special format such as the compressed ones. It is a shift 
 *      factor usually set to zero
 * - TYPE : 4 bits mark the format "family" type. Actually 5 families are coded,
 *      @ref AI_FMT_FLOAT (float types) @ref AI_FMT_I (integer types),
 *      @ref AI_FMT_Q (fixed-point types in Qm.n format)
 *      @ref AI_FMT_LUT4 (compressed lookup 16 formats)
 *      @ref AI_FMT_LUT8 (compressed lookup 256 formats)
 * - PBITS 3 bits padding bits used to set the number of padding bits 
 *      (per element) to handle special aligned formats/ E.g. a 6 bit format
 *      where each element is stored byte aligned (8 bits) has 2 padding bits.
 *      Usually this is set to 0
 * - BITS 7 bits set the total number of bits of the element, padding bits 
 *      excluded. The bits are thus = sign bit + fractional bits + integer bits
 *      The number of integer bits could thus be known using the @ref
 *      AI_FMT_GET_IBITS() macro.
 * - FBITS 7 bits set the number of fractional bits in the format
 *
 *
 * A reference code snippet for usage is the test unit that uses this header:
 *
 * \include test/test_lcut_formats.cpp
 *
 */

#define AI_BITS_ALIGN(bits_)  ( ((bits_)+0x7u) & (~0x7u) )

/*!
 * Format bitfields definition.  NOTE: 6 MSB are masked off 
 * for (optional) atributes setting using flags. see @ref AI_FMT_FLAG_CONST that
 * is used for marking a data as constant readonly
 * 1 bitfield to indicate that this format is exposed to the application as 
 * part of the AI_BUFFER_ formats
 */
#define _FMT_EXP_MASK         (0x1)
#define _FMT_EXP_BITS         (25)

/* 1 bit field to identify floating point values*/
#define _FMT_FLOAT_MASK       (0x1)
#define _FMT_FLOAT_BITS       (24)

/*! 1 bit sign info */
#define _FMT_SIGN_MASK        (0x1)
#define _FMT_SIGN_BITS        (23)

/*! fractional bits field (i.e. for Q formats see @ref AI_FMT_Q) */
#define _FMT_FBITS_MASK       (0x7F)
#define _FMT_FBITS_BITS       (0)

/*! TOTAL number of bits (fractional+integer+sign) (excluded padding ones) */
#define _FMT_BITS_MASK        (0x7F)
#define _FMT_BITS_BITS        (7)

/*! Padding bits for handling formats not aligned to multiples of 8 bits */
#define _FMT_PBITS_MASK       (0x7)
#define _FMT_PBITS_BITS       (14)

/*! bits reserved for identifying the family format, e.g. float, fixed-point.. */
#define _FMT_TYPE_MASK        (0xF)
#define _FMT_TYPE_BITS        (17)

#define _FMT_LDIV_MASK        (0x3)
#define _FMT_LDIV_BITS        (21)


/******************************************************************************/
#define AI_FMT_OBJ(fmt_)        ((ai_data_format)(fmt_))

/*!
 * Only 25 LSB bits are used for storing actual format bits. 7 bits are reserved
 * for format atrtributes, see @ref AI_FMT_FLAG_CONST flag
 */
#define AI_FMT_MASK             (0x03FFFFFF)

#define AI_FMT_FLAG_CONST       (0x1U<<31)
#define AI_FMT_FLAG_STATIC      (0x1U<<30)

/******************************************************************************/
/*!
 * Format "Class" type : this identify the family of the format: 
 * float, integer, fixed point (i.e. Q format), compressed via lookup table 
 */
#define AI_FMT_FLOAT            (0x0)
#define AI_FMT_I                (0x1)
#define AI_FMT_Q                (0x2)
#define AI_FMT_LUT4             (0x4)
#define AI_FMT_LUT8             (0x8)

#define AI_FMT_GET(val_)        (((ai_data_format)(val_))&AI_FMT_MASK)
#define AI_FMT_Q_MASK           ((_FMT_FBITS_MASK<<_FMT_FBITS_BITS) | \
                                 (_FMT_BITS_MASK<<_FMT_BITS_BITS))

#define _FMT_SET(val, mask, bits)   AI_FMT_GET(((val)&(mask))<<(bits))
#define _FMT_GET(fmt, mask, bits)   ((AI_FMT_GET(fmt)>>(bits))&(mask))

#define AI_FMT_SET_EXPORTED(val) _FMT_SET(val, _FMT_EXP_MASK, _FMT_EXP_BITS)
#define AI_FMT_GET_EXPORTED(fmt) _FMT_GET(fmt, _FMT_EXP_MASK, _FMT_EXP_BITS)
#define AI_FMT_SET_FLOAT(val)    _FMT_SET(val, _FMT_FLOAT_MASK, _FMT_FLOAT_BITS)
#define AI_FMT_GET_FLOAT(fmt)    _FMT_GET(fmt, _FMT_FLOAT_MASK, _FMT_FLOAT_BITS)
#define AI_FMT_SET_SIGN(val)     _FMT_SET(val, _FMT_SIGN_MASK, _FMT_SIGN_BITS)
#define AI_FMT_GET_SIGN(fmt)     _FMT_GET(fmt, _FMT_SIGN_MASK, _FMT_SIGN_BITS)
#define AI_FMT_SET_BITS(val)     _FMT_SET(val, _FMT_BITS_MASK, _FMT_BITS_BITS)
#define AI_FMT_GET_BITS(fmt)     _FMT_GET(fmt, _FMT_BITS_MASK, _FMT_BITS_BITS)
#define AI_FMT_SET_FBITS(val)    _FMT_SET(val, _FMT_FBITS_MASK, _FMT_FBITS_BITS)
#define AI_FMT_GET_FBITS(fmt)    _FMT_GET(fmt, _FMT_FBITS_MASK, _FMT_FBITS_BITS)
#define AI_FMT_SET_PBITS(val)    _FMT_SET(val, _FMT_PBITS_MASK, _FMT_PBITS_BITS)
#define AI_FMT_GET_PBITS(fmt)    _FMT_GET(fmt, _FMT_PBITS_MASK, _FMT_PBITS_BITS)
#define AI_FMT_SET_TYPE(val)     _FMT_SET(val, _FMT_TYPE_MASK, _FMT_TYPE_BITS)
#define AI_FMT_GET_TYPE(fmt)     _FMT_GET(fmt, _FMT_TYPE_MASK, _FMT_TYPE_BITS)
#define AI_FMT_SET_LDIV(val)     _FMT_SET(val, _FMT_LDIV_MASK, _FMT_LDIV_BITS)
#define AI_FMT_GET_LDIV(fmt)     _FMT_GET(fmt, _FMT_LDIV_MASK, _FMT_LDIV_BITS)

/*!
 * The total number of bits for a given format is supposed to be the sum of the 
 * bits + padding bits. This means that the number of integer bits is derived
 * as follow: int_bits = bits - fbits (fractional bits) - 1 (for the sign) 
 */
#define AI_FMT_GET_BITS_SIZE(fmt_) \
          (AI_FMT_GET_BITS(fmt_)+AI_FMT_GET_PBITS(fmt_))

/*! Macro used to compute the integer bits for a format */
#define AI_FMT_GET_IBITS(fmt_) \
          (AI_FMT_GET_BITS(fmt_)-AI_FMT_GET_FBITS(fmt_)-AI_FMT_GET_SIGN(fmt_))

/*!
 *  Macro used to select only the format entries with exp_ bit=1.
 * i.e. the public exported ai_buffer formats 
 */
#define AI_FMT_IIF(c)            AI_PRIMITIVE_CAT(AI_FMT_IIF_, c)
#define AI_FMT_IIF_0(...)
#define AI_FMT_IIF_1(...)        __VA_ARGS__

/*! Q format handlers *********************************************************/
#define AI_DATA_FMT_SET_Q(bits_, fbits_)    ( AI_DATA_FORMAT_Q | \
  _FMT_SET(bits_, _FMT_BITS_MASK, _FMT_BITS_BITS) | \
  _FMT_SET(fbits_, _FMT_FBITS_MASK, _FMT_FBITS_BITS) )

#define AI_DATA_FMT_IS_Q(fmt) \
  (AI_DATA_FORMAT_Q==(AI_FMT_GET(fmt)&~AI_FMT_Q_MASK))

/*! ai_array section **********************************************************/
#define AI_DATA_FMT_ENTRY(name_) \
  AI_CONCAT(AI_DATA_FORMAT_, name_)

#define AI_DATA_FMT_FROM_EXP_ID(exp_) \
  ai_data_fmt_from_exp_id(exp_id)

#define AI_DATA_FMT_NAME(type_) \
  ai_data_fmt_name(type_)

#define AI_DATA_FMT_VALID(type_) \
  ai_data_fmt_valid(type_)

#define AI_DATA_FMT_GET_FORMATS(formats_) \
  ai_data_fmt_get_formats(formats_)


#define AI_DATA_TO_BUFFER_FMT(fmt_) \
  ai_data_to_buffer_fmt(fmt_)

#define AI_ARRAY_GET_BYTE_SIZE(fmt_, count_) \
  ai_array_get_byte_size(fmt_, count_)

#define AI_ARRAY_GET_ELEMS_FROM_SIZE(fmt_, size_) \
  ai_array_get_elems_from_size(fmt_, size_)

AI_API_DECLARE_BEGIN

/*!
 * @typedef ai_data_format
 * @ingroup ai_datatypes_format
 * @brief Generic Data Format Specifier (32bits packed info) 
 */
typedef ai_u32 ai_data_format;

/*!
 * @enum internal data format enums
 * @ingroup ai_datatypes_format
 * @brief Generic Data Format Specifier (32bits packed info) 
 */
enum {
#define FMT_ENTRY(idx_, exp_, exp_name_, exp_id_, name_, type_id_, \
  sign_bit_, float_bit_, pbits_, bits_, fbits_, ldiv_bits_) \
    AI_DATA_FMT_ENTRY(name_) = (AI_FMT_SET_FLOAT(float_bit_) | \
                                AI_FMT_SET_SIGN(sign_bit_) | \
                                AI_FMT_SET_BITS(bits_) | \
                                AI_FMT_SET_FBITS(fbits_) | \
                                AI_FMT_SET_PBITS(pbits_) | \
                                AI_FMT_SET_TYPE(type_id_) | \
                                AI_FMT_SET_LDIV(ldiv_bits_) | \
                                AI_FMT_SET_EXPORTED(exp_) ),
#include "formats_list.h"
};


/*!
 * @brief Convert from the exp_id_ enum value of the @ref ai_buffer_format_enum 
 * to correspondent internal @ref ai_data_format.
 * @ingroup ai_datatypes_format
 * @param[in] exp_id the exp_id value as declared in @ref formats_list.h 
 * exp_id_ field
 * @return the ai_data_format that corresponds to the exp_id value, 
 * AI_DATA_FORMAT_NONE if no valid format was found
 */
AI_DECLARE_STATIC
ai_data_format ai_data_fmt_from_exp_id(const ai_u8 exp_id)
{
  switch ( exp_id ) {
#define FMT_ENTRY(idx_, exp_, exp_name_, exp_id_, name_, type_id_, \
  sign_bit_, float_bit_, pbits_, bits_, fbits_, ldiv_bits_) \
    case exp_id_: return AI_DATA_FMT_ENTRY(name_);
#include "formats_list.h"
    default: break;
  }
  return AI_DATA_FORMAT_NONE;
}

/*!
 * @brief Check if @ref ai_data_format is a valid format present in the list of
 * supported formats 
 * @ingroup ai_datatypes_format
 * @param[in] type the ai_data_format to check
 * @return true if the format is valid, false otherwise
 */
AI_DECLARE_STATIC
bool ai_data_fmt_valid(const ai_data_format type)
{
  switch ( AI_FMT_GET(type) ) {
#define FMT_ENTRY(idx_, exp_, exp_name_, exp_id_, name_, type_id_, \
  sign_bit_, float_bit_, pbits_, bits_, fbits_, ldiv_bits_) \
  case AI_DATA_FMT_ENTRY(name_): return true;
#include "formats_list.h"
    default: break;
  }
  return false;
}

/*!
 * @brief Get a human readable string from the format ID value
 * @ingroup ai_datatypes_format
 * @param[in] type the @ref ai_data_format to print out
 * @return a string with a human readable name of the format
 */
AI_DECLARE_STATIC
const char* ai_data_fmt_name(const ai_data_format type)
{
  ai_u32 masked_type = AI_FMT_GET(type);

  switch ( masked_type ) {
#define FMT_ENTRY(idx_, exp_, exp_name_, exp_id_, name_, type_id_, \
  sign_bit_, float_bit_, pbits_, bits_, fbits_, ldiv_bits_) \
  case AI_DATA_FMT_ENTRY(name_): return AI_STRINGIFY(AI_DATA_FMT_ENTRY(name_));
#include "formats_list.h"
    default: break;
  }

  /* Mask off Q format fractional and total bits field info */
  masked_type &= ~AI_FMT_Q_MASK;

  switch ( masked_type ) {
#define FMT_ENTRY(idx_, exp_, exp_name_, exp_id_, name_, type_id_, \
  sign_bit_, float_bit_, pbits_, bits_, fbits_, ldiv_bits_) \
  case AI_DATA_FMT_ENTRY(name_): return AI_STRINGIFY(AI_DATA_FMT_ENTRY(name_));
#include "formats_list.h"
    default: break;
  }

  return "";
}

/*!
 * @brief Get the complete list of supported @ref ai_data_format formats
 * @ingroup ai_datatypes_format
 * @param[out] formats a pointer to an array withj all supported formats listed
 * @return the number of supported formats
 */
AI_DECLARE_STATIC
ai_size ai_data_fmt_get_formats(const ai_data_format** formats)
{
  static const ai_data_format _formats[] = {
#define FMT_ENTRY(idx_, exp_, exp_name_, exp_id_, name_, type_id_, \
  sign_bit_, float_bit_, pbits_, bits_, fbits_, ldiv_bits_) \
    AI_DATA_FMT_ENTRY(name_),
#include "formats_list.h"
  };
  
  *formats = _formats;
  
  return AI_BUFFER_COUNT(_formats);
}

/*! ai_buffer section *********************************************************
 * Only 25 LSB bits are used for storing actual format bits. 7 bits are reserved
 * for format atrtributes, see @ref AI_FMT_FLAG_CONST flag
 */
#define AI_BUFFER_FMT_MASK             (0x00FF)

#define AI_BUFFER_FMT_GET(fmt_) \
  (AI_BUFFER_FMT(fmt_)&AI_BUFFER_FMT_MASK)

#define AI_BUFFER_FMT_ENTRY(name_) \
  name_

#define AI_BUFFER_FMT_NAME(type_) \
  ai_buffer_fmt_name(type_)

#define AI_BUFFER_FMT_VALID(type_) \
  ai_buffer_fmt_valid(type_)

#define AI_BUFFER_FMT_GET_FORMATS(formats_) \
  ai_buffer_fmt_get_formats(formats_)

#define AI_BUFFER_TO_DATA_FMT(fmt_) \
  ai_buffer_to_data_fmt(fmt_)

#define AI_BUFFER_GET_BITS_SIZE(fmt) \
  AI_ARRAY_GET_BITS_SIZE(AI_BUFFER_TO_DATA_FMT(fmt))

/*
 NOTE: already defined in platform.h
enum {
#define FMT_ENTRY(idx_, exp_, exp_name_, exp_id_, name_, type_id_, \
  sign_bit_, float_bit_, pbits_, bits_, fbits_, ldiv_bits_) \
    AI_FMT_IIF(exp_)(AI_BUFFER_FMT_ENTRY(exp_name_) = _FMT_SET_ID(exp_id_),)
#include "formats_list.h"
};
*/

/*!
 * @brief Get a human readable string from the format ID value
 * @ingroup ai_datatypes_format
 * @param[in] type the @ref ai_buffer_format to print out
 * @return a string with a human readable name of the format
 */
AI_DECLARE_STATIC
const char* ai_buffer_fmt_name(const ai_buffer_format type)
{
  switch ( AI_BUFFER_FMT_GET(type) ) {
#define FMT_ENTRY(idx_, exp_, exp_name_, exp_id_, name_, type_id_, \
  sign_bit_, float_bit_, pbits_, bits_, fbits_, ldiv_bits_) \
  AI_FMT_IIF(exp_)(case AI_BUFFER_FMT_ENTRY(exp_name_): return AI_STRINGIFY(AI_BUFFER_FMT_ENTRY(exp_name_));)
#include "formats_list.h"
    default: break;
  }
  return "";
}

/*!
 * @brief Check if @ref ai_buffer_format is a valid format present in the list of
 * supported formats 
 * @ingroup ai_datatypes_format
 * @param[in] type the @ref ai_buffer_format to check
 * @return true if the format is valid, false otherwise
 */
AI_DECLARE_STATIC
bool ai_buffer_fmt_valid(const ai_buffer_format type)
{
  switch ( AI_BUFFER_FMT_GET(type) ) {
#define FMT_ENTRY(idx_, exp_, exp_name_, exp_id_, name_, type_id_, \
  sign_bit_, float_bit_, pbits_, bits_, fbits_, ldiv_bits_) \
  AI_FMT_IIF(exp_)(case AI_BUFFER_FMT_ENTRY(exp_name_): return true;)
#include "formats_list.h"
    default: break;
  }
  return false;
}

/*!
 * @brief Get the complete list of supported @ref ai_buffer_format formats
 * @ingroup ai_datatypes_format
 * @param[out] formats a pointer to an array withj all supported formats listed
 * @return the number of supported formats
 */
AI_DECLARE_STATIC
ai_size ai_buffer_fmt_get_formats(const ai_buffer_format** formats)
{
  static const ai_buffer_format _formats[] = {
#define FMT_ENTRY(idx_, exp_, exp_name_, exp_id_, name_, type_id_, \
  sign_bit_, float_bit_, pbits_, bits_, fbits_, ldiv_bits_) \
    AI_FMT_IIF(exp_)(AI_BUFFER_FMT_ENTRY(exp_name_),)
#include "formats_list.h"
  };
  
  *formats = _formats;

  return AI_BUFFER_COUNT(_formats);
}

/*! Conversions section *******************************************************/
/*!
 * @brief Convert from ai_data_format to ai_buffer_format.
 * @ingroup ai_datatypes_format
 * @param fmt the input ai_data_format to convert
 * @return the converted format as a ai_buffer_format
 */
AI_DECLARE_STATIC
ai_buffer_format ai_data_to_buffer_fmt(const ai_data_format fmt)
{
  ai_buffer_format out = AI_BUFFER_FORMAT_NONE;

  switch ( AI_FMT_GET(fmt) ) {
#define FMT_ENTRY(idx_, exp_, exp_name_, exp_id_, name_, type_id_, \
  sign_bit_, float_bit_, pbits_, bits_, fbits_, ldiv_bits_) \
    AI_FMT_IIF(exp_)(case AI_DATA_FMT_ENTRY(name_): \
                      out = AI_BUFFER_FMT_ENTRY(exp_name_); \
                      break;)
#include "formats_list.h"
    default: break;
  }

  /* Manage reserved flags conversion */
  out |= ( fmt & AI_FMT_FLAG_CONST )  ? AI_BUFFER_FMT_FLAG_CONST  : 0x0;
  out |= ( fmt & AI_FMT_FLAG_STATIC ) ? AI_BUFFER_FMT_FLAG_STATIC : 0x0;

  return out;
}

/*!
 * @brief Convert from ai_buffer_format to ai_array_format.
 * @ingroup ai_datatypes_format
 * @param fmt the input ai_buffer_format to convert
 * @return the converted format as a ai_array_format
 */
AI_DECLARE_STATIC
ai_data_format ai_buffer_to_data_fmt(const ai_buffer_format fmt)
{
  ai_data_format out = AI_DATA_FORMAT_NONE;

  switch ( AI_BUFFER_FMT_GET(fmt) ) {
#define FMT_ENTRY(idx_, exp_, exp_name_, exp_id_, name_, type_id_, \
  sign_bit_, float_bit_, pbits_, bits_, fbits_, ldiv_bits_) \
    AI_FMT_IIF(exp_)(case AI_BUFFER_FMT_ENTRY(exp_name_): \
                      out = AI_DATA_FMT_ENTRY(name_); \
                      break;)
#include "formats_list.h"
    default: break;
  }

  /* Manage reserved flags conversion */
  out |= ( fmt & AI_BUFFER_FMT_FLAG_CONST )  ? AI_FMT_FLAG_CONST  : 0x0;
  out |= ( fmt & AI_BUFFER_FMT_FLAG_STATIC ) ? AI_FMT_FLAG_STATIC : 0x0;

  return out;
}

/** helpers section ***********************************************************/
/*!
 * @brief Computes the size in bytes given an ai_array_format and number of 
 * array elements.
 * @details This routine computes from the number of elements of the array its
 * size in bytes. If the arry is referred by a tensor structure, it is the task
 * of the latter to handle per-dimension padding (e.g. to align odd rows in a
 * 4-bit matrix. At array level the padding elements MUST be included in the
 * number of elements.
 * @ingroup ai_datatypes_format
 * @param[in] fmt the input array format as an ai_data_format
 * @param[in] count the number of elements stored in the data array
 * @return the size in bytes of the array given the specific format and number
 * of elements (including padding elements)
 */
AI_DECLARE_STATIC
ai_size ai_array_get_byte_size(const ai_data_format fmt, const ai_size count)
{
  if ( 0==count ) return 0;

  ai_size bits_size = count * AI_FMT_GET_BITS_SIZE(fmt);
  
  /* Compute the right shift ldiv bitfield if required
   * e.g. with compressed formats */
  bits_size = AI_BITS_ALIGN(bits_size) >> AI_FMT_GET_LDIV(fmt);

  /* If format is compressed add the size of the lookup table */
  switch ( AI_FMT_GET_TYPE(fmt) )
  {
    case AI_FMT_LUT4:
      bits_size += (16 * AI_FMT_GET_BITS_SIZE(fmt)); 
      break;
    case AI_FMT_LUT8:
      bits_size += (256 * AI_FMT_GET_BITS_SIZE(fmt)); 
      break;
    default:
      break;
  }

  /* converts the size in bits to the size in bytes
   * aligning to 1 byte if required */
  bits_size = AI_BITS_ALIGN(bits_size) >> 3;

  return bits_size;
}

/*!
 * @brief Computes the number of elements from ai_array_format and
 * the size in byte of the array.
 * @ingroup ai_datatypes_format
 * @param fmt the input array format as an ai_data_format
 * @param size the size in bytes of the array
 * @return the number of elements that could be stored given the format
 */
AI_DECLARE_STATIC
ai_size ai_array_get_elems_from_size(
  const ai_data_format fmt, const ai_size byte_size)
{
  if ( 0==byte_size ) return 0;

  ai_size bits_size = byte_size * 8;

  /* If format is compressed subtract the size of the lookup table */
  switch ( AI_FMT_GET_TYPE(fmt) )
  {
    case AI_FMT_LUT4:
      bits_size -= (16 * AI_FMT_GET_BITS_SIZE(fmt)); 
      break;
    case AI_FMT_LUT8:
      bits_size -= (256 * AI_FMT_GET_BITS_SIZE(fmt)); 
      break;
    default:
      break;
  }

  /* Compute the left shift ldiv bitfield if required
   * e.g. with compressed formats */
  bits_size = AI_BITS_ALIGN(bits_size << AI_FMT_GET_LDIV(fmt));

  bits_size /= AI_FMT_GET_BITS_SIZE(fmt);

  return bits_size;
}

AI_API_DECLARE_END

#undef AI_BITS_ALIGN
#undef AI_DATA_FMT_ENTRY
#undef AI_BUFFER_FMT_ENTRY

#endif /*__AI_DATATYPES_FORMAT_H__*/
