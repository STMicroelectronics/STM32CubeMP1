/**
 * Copyright (c) 2017 rxi
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license. See `log.c` for details.
 */

#ifndef __AI_LOG_H_
#define __AI_LOG_H_
#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

/*!
 * @defgroup log Core logger class definition and implementation
 * @brief Data structures and defines used to implementlogger module
 * functionalities
 */

#define LOG_VERSION       "0.2.0"
#define LOG_CR            "\r\n"

/***** Compilation options: define/undef as required **************************/
#define LOG_USE_COLOR
/* #define LOG_INFO_SOURCE_CODE */

#ifndef HAS_STM32
#define LOG_USE_FILE
#define LOG_INFO_TIME
#define LOG_INFO_SOURCE_CODE_STRIP_FILE_PATHS '/'
#else
#define LOG_INFO_SOURCE_CODE_STRIP_FILE_PATHS		'\\'
#endif

#if defined (__IAR_SYSTEMS_ICC__)
#define LOG_PRINT(fmt, ...)     printf(fmt, ## __VA_ARGS__);
#define LOG_VPRINT(fmt, ...)    vprintf(fmt, ## __VA_ARGS__);
#define LOG_PRINT_FLUSH
#else
#define LOG_PRINT(fmt, ...)     fprintf(stderr, fmt, ## __VA_ARGS__);
#define LOG_VPRINT(fmt, ...)    vfprintf(stderr, fmt, ## __VA_ARGS__);
#define LOG_PRINT_FLUSH         fflush(stderr);
#endif

/******************************************************************************/
#define LOG_SUDO          (0x0)
#define LOG_FATAL         (0x1)
#define LOG_ERROR         (0x2)
#define LOG_WARN          (0x3)
#define LOG_INFO          (0x4)
#define LOG_DEBUG         (0x5)
#define LOG_TRACE         (0x6)

#define _LOG_SUDO(...)  do { log_log(LOG_SUDO, __FILE__, __LINE__, __VA_ARGS__); } while(0);
#define _LOG_TRACE(...) do { log_log(LOG_TRACE, __FILE__, __LINE__, __VA_ARGS__); } while(0);
#define _LOG_DEBUG(...) do { log_log(LOG_DEBUG, __FILE__, __LINE__, __VA_ARGS__); } while(0);
#define _LOG_INFO(...)  do { log_log(LOG_INFO,  __FILE__, __LINE__, __VA_ARGS__); } while(0);
#define _LOG_WARN(...)  do { log_log(LOG_WARN,  __FILE__, __LINE__, __VA_ARGS__); } while(0);
#define _LOG_ERROR(...) do { log_log(LOG_ERROR, __FILE__, __LINE__, __VA_ARGS__); } while(0);
#define _LOG_FATAL(...) do { log_log(LOG_FATAL, __FILE__, __LINE__, __VA_ARGS__); } while(0);


/*!
 * @typedef log_LockFn
 * @ingroup log
 * @brief callback function for locking implementation (e.g. mutexes, etc.)
 */
typedef void (*log_LockFn)(const void *udata, const bool lock);

/*!
 * @typedef log_MsgFn
 * @ingroup log
 * @brief callback for listening at logged channels
 */
typedef void (*log_MsgFn)(
  const void *udata, const uint8_t level, 
  const char* msg, const uint32_t len);

/*!
 * @brief Set global log level
 * @ingroup log
 */
void log_set_level(const uint8_t level);

/*!
 * @brief Set global log quiet mode (no messages are emitted)
 * @ingroup log
 */
void log_set_quiet(const bool enable);

/*!
 * @brief Set callback for log messages locking
 * @ingroup log
 */
void log_set_lock(log_LockFn fn, const void *udata);

/*!
 * @brief Push on log stack a new listener with given log level
 * @ingroup log
 * @param[in] level the log level for this channel
 * @param[out] the callback function to emit when a message is available
 * @param[in] udata a pointer to the caller environment that is provided back 
 * when the callback is called
 * @return 0 if OK, value>0 that indicates the current size of the stack
 */
uint8_t log_channel_push(const uint8_t level, log_MsgFn fn, const void *udata);

/*!
 * @brief Pop from log stack a pushed listener
 * @ingroup log
 * @param[in] the callback function registered during @ref log_channel_push
 * @param[in] udata a pointer to the caller environment registered during @ref 
 * log_channel_push
 * @return 0 if OK, value>0 that indicates the max size of the callback stack
 */
uint8_t log_channel_pop(log_MsgFn fn, const void *udata);

#ifdef LOG_USE_FILE
/*!
 * @brief Enable file dumping of all logged messages to a file as well. 
 * @details NB: the quiet option does not apply to file logging. file log 
 * messages are recorded also when the log is in quiet mode.
 * @ingroup log
 * @param[out] fp the file pointer of the file used to log the massages
 */
void log_set_fp(FILE *fp);
#endif

/*!
 * @brief Main Routine: PLEASE invoke always by using defined macros
 * @ingroup log
 * @param[in] level the log level of the input message
 * @param[in] file the string containing the __FILE__ info about the source file
 * generating the message to log
 * @param[in] fmt the varargs format of the string to print 
 */
void log_log(const uint8_t level, const char *file,
  const int line, const char *fmt, ...);

#endif    /*__AI_LOG_H_*/
