/**
  ******************************************************************************
  * @file    core_lists.h
  * @author  AST Embedded Analytics Research Platform
  * @date    22-Aug-2018
  * @brief   implementation of macros for handling ai_dlist double-linked 
  * (circular) list data structures
  * 
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

#ifndef __AI_CORE_LISTS_H_
#define __AI_CORE_LISTS_H_
#pragma once

#include "ai_platform.h"
#include "ai_platform_interface.h"

/*!
 * @defgroup Core List to handle double-linked list
 * @brief Macros for handling @ref ai_dlist data structures
 * @details This datastructure and associated macros implements methods to 
 * create / add / remove / concatenate (see @ref AI_DLIST_SPLICE) and loops over
 * the items in a double-linked circular list 
 * (see @ref AI_DLIST_FOR_EACH_OBJ_NEXT_SAFE). 
 * In particular the iteration loops with postfix "SAFE" are used to safely 
 * remove and/or manipulate items in a list. DO NOT use non safe loops macro 
 * to modify the list.
 * The non SAFE methods are intended to quickly access items in read-only manner 
 * A reference code that shows how to use these structures is provided in the 
 * following sample code:
 * 
 * \include test/test_lcut_dlists.cpp
 *
 */

/**** BEGIN DOUBLE LINKED LIST MACROS SECTION *********************************/
/*! Initialize a list setting its prev and next fields */
#define AI_DLIST_SET__(item_, next_, prev_) \
  (item_)->next = (next_); (item_)->prev = (prev_);

/*! Initialize an empty list setting prev and next fields to point to itself */
#define AI_DLIST(item_) \
  { .next = (item_), .prev = (item_) }

/*! Initialize an empty list setting prev and next fields to point to itself */
#define AI_DLIST_INIT(item_) \
  AI_DLIST_SET__(item_, item_, item_)

/*! Insert an element into a list */
#define AI_DLIST_INSERT(item_, next_, prev_) \
  ai_dlist_insert(item_, next_, prev_);

/*! Insert an element before head in the list */
#define AI_DLIST_PUSH_FRONT(item_, head_) \
  AI_DLIST_INSERT(item_, head_, (head_)->prev)

/*! Insert an element after head on the list */
#define AI_DLIST_PUSH_BACK(item_, head_) \
  AI_DLIST_INSERT(item_, (head_)->next, head_)

/*! Check if the list is empty */
#define AI_DLIST_IS_EMPTY(item_) \
  ( (item_)==(item_)->next )

/**** END DOUBLE LINKED LIST MACROS SECTION ***********************************/
/*
#include <stddef.h>
#define AI_STRUCT_OFFSET(type_, field_) \
  ( (ai_u32)(offsetof(type_,field_)) )
*/

/*! Computes offset in byte of a  named "member_" into a container of type 
 * "type_" */
#define AI_STRUCT_OFFSET(type_, member_) \
  ( (ai_u32)((ai_uptr)&(((type_*)0)->member_)) )

/*! Cast the pointer as a type_* pointer */
#define AI_CONTAINER_TYPE(type_, ptr_) \
  ( (type_*)(ptr_) )

/*! Get the start address of the container datastruct */
#define AI_CONTAINER_PTR(ptr_, offset_) \
  ( ((ai_ptr)(ptr_))-(offset_) )

/*! Get start address of the container "type_" from the pointer to the list 
 * ai_dlist element named "member_" */
#define AI_DLIST_CONTAINER(ptr_, type_, member_) \
  AI_CONTAINER_TYPE( type_, \
                     AI_CONTAINER_PTR(ptr_, AI_STRUCT_OFFSET(type_, member_)) )

/*! Get the size of a list */
#define AI_DLIST_SIZE(head_) \
  ai_dlist_size(head_)

/*! For loop that iterates forward on the ai_dlist elements */
#define AI_DLIST_FOR_EACH_NEXT(item_, head_) \
  for ( ai_dlist *item_=(head_)->next; item_!=(head_); item_=item_->next )

/*! For loop that iterates backward on the ai_dlist elements */
#define AI_DLIST_FOR_EACH_PREV(item_, head_) \
  for ( ai_dlist *item_=(head_)->prev; item_!=(head_); item_=item_->prev )

/*! For loop that safely iterates forward on the ai_dlist elements */
#define AI_DLIST_FOR_EACH_NEXT_SAFE(item_, head_) \
  for ( ai_dlist *item_=(head_)->next, *n_=item_->next; \
        item_!=(head_); item_=n_, n_=n_->next )

/*! For loop that safely iterates backward on the ai_dlist elements */
#define AI_DLIST_FOR_EACH_PREV_SAFE(item_, head_) \
  for ( ai_dlist *item_=(head_)->prev, *p_=item_->prev; item_!=(head_); \
        item_=p_, p_=p_->prev )

/*! Remove an element from the list */
#define AI_DLIST_REMOVE(next_, prev_) \
  { (next_)->prev = (prev_); (prev_)->next = (next_); }

/*! Concatenate list head_ in position where_ of another list */
#define AI_DLIST_SPLICE(head_, where_) \
  ai_dlist_splice(head_, where_);

/*! Remove an element from the list using the pointer of the container */
#define AI_DLIST_OBJ_REMOVE(item_, member_) \
  AI_DLIST_REMOVE((item_)->member_.next, (item_)->member_.prev)

/*! For loop that iterates forward on the container elements */
#define AI_DLIST_FOR_EACH_OBJ_NEXT(obj_, head_, type_, member_) \
  for ( type_ *obj_=AI_DLIST_CONTAINER((head_)->next, type_, member_); \
        &obj_->member_!=(head_); \
        obj_=AI_DLIST_CONTAINER(obj_->member_.next, type_, member_ ) )

/*! For loop that iterates backward on the container elements */
#define AI_DLIST_FOR_EACH_OBJ_PREV(obj_, head_, type_, member_) \
  for ( type_ *obj_=AI_DLIST_CONTAINER((head_)->prev, type_, member_); \
        &obj_->member_!=(head_); \
        obj_=AI_DLIST_CONTAINER(obj_->member_.prev, type_, member_ ) )

/*! For loop that safely iterates forward on the container elements */
#define AI_DLIST_FOR_EACH_OBJ_NEXT_SAFE(obj_, head_, type_, member_) \
  for ( type_ *obj_=AI_DLIST_CONTAINER((head_)->next, type_, member_), \
        *n_= AI_DLIST_CONTAINER(obj_->member_.next, type_, member_); \
        &obj_->member_!=(head_); \
        obj_=n_, n_=AI_DLIST_CONTAINER(n_->member_.next, type_, member_ ) )

/*! For loop that safely iterates backward on the container elements */
#define AI_DLIST_FOR_EACH_OBJ_PREV_SAFE(obj_, head_, type_, member_) \
  for ( type_ *obj_=AI_DLIST_CONTAINER((head_)->prev, type_, member_), \
        *p_= AI_DLIST_CONTAINER(obj_->member_.prev, type_, member_); \
        &obj_->member_!=(head_); \
        obj_=p_, p_=AI_DLIST_CONTAINER(p_->member_.prev, type_, member_ ) )

/*!
 * @typedef ai_dlist
 * @ingroup core_lists
 * @brief Core implementation of a double linked datastracture to handle generic
 * double-linked lists see all @ref AI_DLIST_INIT() and all AI_DLIST_XX macros
 * to manage this datastructure
 */
typedef struct ai_dlist_ {
  struct ai_dlist_*     next; /*!< pointer to the next element on the list */
  struct ai_dlist_*     prev; /*!< pointer to the previous element on the list */
} ai_dlist;


/*!
 * @brief Insert an item inside the list
 * @details this routine inserts an item in a list between the next and prev 
 * element.
 * @ingroup core_lists
 * @param[in/out] item a pointer to the @ref ai_dlist datastructure to be inserted
 * @param[in/out] next a pointer to the next element in the list
 * @param[in/out] prev a pointer to the previous element in the list
 */
AI_DECLARE_STATIC
void ai_dlist_insert(ai_dlist* item, ai_dlist* next, ai_dlist* prev)
{
  AI_ASSERT(item && next && prev)
  item->next = next;
  item->prev = prev;
  next->prev = item;
  prev->next = item;
}

/*!
 * @brief Insert a list head into another list into position where 
 * @details this routine inserts an full list in position where on another
 * list
 * @ingroup core_lists
 * @param[in/out] head a pointer to the list to be inserted
 * @param[in/out] where a pointer to the position in the 2nd list where to insert the
 * 1st list
 */
AI_DECLARE_STATIC
void ai_dlist_splice(ai_dlist* head, ai_dlist* where)
{
  AI_ASSERT(head && where)
  if ( AI_DLIST_IS_EMPTY(head) ) return;

  ai_dlist* first = head->next;
  ai_dlist* last  = head->prev;
  ai_dlist* where_last = where->next;

  first->prev      = where;
  where->next      = first;

  last->next       = where_last;
  where_last->prev = last;

  AI_DLIST_INIT(head)
}

/*!
 * @brief Get the number of elements in the list
 * @ingroup core_lists
 * @param[in] head a pointer to the list
 */
AI_DECLARE_STATIC
ai_size ai_dlist_size(const ai_dlist* head)
{
  AI_ASSERT(head)
  ai_size size = 0;
  AI_DLIST_FOR_EACH_NEXT(item, head) { size++; }
  return size;
}

#endif    /*__AI_CORE_LISTS_H_*/
