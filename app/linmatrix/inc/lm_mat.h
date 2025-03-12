/**
 *******************************************************************************
 * Copyright 2022 Y.H.Kuo
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @noop    ___     ______  _   __
 * @noop   /  /    /_   _/ / \ / /  A lightweight matrix computation SW library.
 * @noop  /  /__  _ / /_  /   \ / __  Use at your own risk.
 * @noop /_____/ /_____/ /_/ \_/ /_/
 * @noop    ______    _____   _______  ____     ______ ___   ___
 * @noop   /      \  /  _  | /__  __/ / __ \   /_   _/ \  \ /  /
 * @noop  /  / /  / /  _   |  /  /   /  -- /  _ / /_   /  --  /
 * @noop /__/_/__/ /__/ \__| /__/   /__/ \_\ /_____/  /__/ \__\
 *
 * @brief   Lin matrix handle management functions
 * @file    lm_mat.h
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#ifndef __LM_MAT_H__
#define __LM_MAT_H__

#include <stdint.h>
#include <stdbool.h>

#include "lm_global.h"
#include "lm_assert.h"


/*
 *******************************************************************************
 * Constant value definition
 *******************************************************************************
 */

#define LM_MAT_STATS_INIT           (1 << 0)
#define LM_MAT_STATS_BUILTIN_MEM    (1 << 1)
#define LM_MAT_STATS_RESHAPED       (1 << 2)


/*
 *******************************************************************************
 * Macros
 *******************************************************************************
 */

/**
  @def LM_MAT_SET_STATS(__p_mat, __stats)
  To setup the stats of handle.
*/
#define LM_MAT_SET_STATS(__p_mat, __stats)                                      \
    do {                                                                        \
        ((lm_mat_t *)(__p_mat))->stats |= (uint8_t)(__stats);                   \
    } while(0)

/**
  @def LM_MAT_CLR_STATS(__p_mat, __stats)
  To clear the stats of handle.
*/
#define LM_MAT_CLR_STATS(__p_mat, __stats)                                      \
    do {                                                                        \
        ((lm_mat_t *)(__p_mat))->stats &= ~((uint8_t)(__stats));                \
    } while(0)

/**
  @def LM_MAT_GET_R_SIZE(__p_mat)
  To get current row size of handle.
*/
#define LM_MAT_GET_R_SIZE(__p_mat) (((lm_mat_t *)(__p_mat))->elem.dim.r)

/**
  @def LM_MAT_GET_C_SIZE(__p_mat)
  To get current column size of handle.
*/
#define LM_MAT_GET_C_SIZE(__p_mat) (((lm_mat_t *)(__p_mat))->elem.dim.c)

/**
  @def LM_MAT_GET_NXT_OFS(__p_mat)
  To get current element offset between two rows of handle.
*/
#define LM_MAT_GET_NXT_OFS(__p_mat) (((lm_mat_t *)(__p_mat))->elem.nxt_r_osf)

/**
  @def LM_MAT_GET_ELEM_PTR(__p_mat)
  To get address of first element of handle.
*/
#define LM_MAT_GET_ELEM_PTR(__p_mat) (((lm_mat_t *)(__p_mat))->elem.ptr)

/**
  @def LM_MAT_GET_ELEM_TOT(__p_mat)
  To get total element number of handle.
*/
#define LM_MAT_GET_ELEM_TOT(__p_mat) \
    ((lm_mat_elem_size_t)(((lm_mat_t *)(__p_mat))->elem.dim.r                   \
                          * __p_mat->elem.dim.c))

/**
  @def LM_MAT_GET_ROW_PTR(__p_mat, __nxt_row_ofs, __r_idx)
  To get start address of specified row of handle.
*/
#define LM_MAT_GET_ROW_PTR(__p_mat, __nxt_row_ofs, __r_idx)                     \
    (((lm_mat_t *)(__p_mat))->elem.ptr + (__nxt_row_ofs * __r_idx))

/**
  @def LM_MAT_GET_COL_PTR(__p_mat, __nxt_col_ofs, __c_idx)
  To get start address of specified column of handle.
*/
#define LM_MAT_GET_COL_PTR(__p_mat, __nxt_col_ofs, __c_idx)                     \
    (((lm_mat_t *)(__p_mat))->elem.ptr + (__nxt_col_ofs * __c_idx))

/**
  @def LM_MAT_TO_NXT_ELEM(__p_elem, __p_mat)
  To shift the pointer to the position of next element.
*/
#define LM_MAT_TO_NXT_ELEM(__p_elem, __p_mat)                                   \
    do {                                                                        \
                                                                                \
        LM_ASSERT_DBG((__p_elem < (((lm_mat_t *)(__p_mat))->mem.ptr             \
                                   + __p_mat->mem.elem_tot)),                   \
                      "Out of allocated memory range");                         \
        __p_elem++;                                                             \
                                                                                \
    } while(0)

/**
  @def LM_MAT_TO_NXT_N_ELEM(__p_elem, __n, __p_mat)
  To shift the pointer to the position of next N elements.
*/
#define LM_MAT_TO_NXT_N_ELEM(__p_elem, __n, __p_mat)                            \
    do {                                                                        \
                                                                                \
        LM_ASSERT_DBG((__p_elem < (((lm_mat_t *)(__p_mat))->mem.ptr             \
                                   + ((lm_mat_t *)(__p_mat))->mem.elem_tot)),   \
                      "Out of allocated memory range");                         \
        __p_elem += __n;                                                        \
                                                                                \
    } while(0)

/**
  @def LM_MAT_TO_NXT_ROW(__p_elem, __nxt_row_ofs, __p_mat)
  To shift the pointer to next row.
*/
#define LM_MAT_TO_NXT_ROW(__p_elem, __nxt_row_ofs, __p_mat)                     \
    do {                                                                        \
                                                                                \
        LM_ASSERT_DBG((__p_elem < (((lm_mat_t *)(__p_mat))->mem.ptr             \
                                   + ((lm_mat_t *)(__p_mat))->mem.elem_tot)),   \
                      "Out of allocated memory range");                         \
        __p_elem += __nxt_row_ofs;                                              \
                                                                                \
    } while(0)

/**
  @def LM_MAT_TO_NXT_COL(__p_elem, __nxt_col_ofs, __p_mat)
  To shift the pointer to next column.
*/
#define LM_MAT_TO_NXT_COL(__p_elem, __nxt_col_ofs, __p_mat)                     \
    do {                                                                        \
                                                                                \
        LM_ASSERT_DBG((__p_elem < (((lm_mat_t *)(__p_mat))->mem.ptr             \
                                   + ((lm_mat_t *)(__p_mat))->mem.elem_tot)),   \
                      "Out of allocated memory range");                         \
        __p_elem += __nxt_col_ofs;                                              \
                                                                                \
    } while(0)

/**
  @def LM_RETURN_IF_ERR(__err_code)
  Exit from this function immediately if error occupies.
*/
#define LM_RETURN_IF_ERR(__err_code)            \
    do {                                        \
        if (LM_IS_ERR(__err_code) == true) {    \
            return (__err_code);                \
        }                                       \
    } while (0)

/**
  @def LM_GOTO_IF_ERR(__err_code)
  Goto specific code section in this function immediately if error occupies.
*/
#define LM_GOTO_IF_ERR(__err_code)              \
    do {                                        \
        if (LM_IS_ERR(__err_code) == true) {    \
            goto LM_EXIT_LABEL;                 \
        }                                       \
    } while (0)


/*
 *******************************************************************************
 * Data type definition
 *******************************************************************************
 */

/** @brief Data type of matrix row and column size */
typedef uint16_t lm_mat_dim_size_t;

/** @brief Data type of matrix element size */
typedef uint16_t lm_mat_elem_size_t;

/** @brief Data type of matrix memory size */
typedef uint32_t lm_mat_mem_size_t;

/** @brief Data type of matrix element offset */
typedef int16_t lm_mat_dim_offset_t;

/** @brief Matrix dimension description */
typedef struct lm_mat_dim {
    /** @brief Row size of matrix  */
    lm_mat_dim_size_t r;

    /** @brief Column size of matrix  */
    lm_mat_dim_size_t c;
} lm_mat_dim_t;

/** @brief Data structure of LinMatrix handle */
typedef struct lm_mat {

#if LM_MAT_NAME_ENABLED
    /** @brief Name of this matrix */
    uint8_t name[5];
#endif // LM_MAT_NAME_ENABLED

    /** @brief Stats of this matrix */
    uint8_t stats;

    /** @brief Matrix element description */
    struct {
        /** @brief  Point to the first element of matrix */
        lm_mat_elem_t *ptr;

        /** @brief Dimension of this matrix */
        lm_mat_dim_t dim;

        /** @brief The element offset between two rows */
        lm_mat_elem_size_t nxt_r_osf;
    } elem;

    /** @brief Matrix memory block description */
    struct {

        /** @brief Point to start address of assigned memory block */
        lm_mat_elem_t *ptr;

        /** @brief Total number of elements that can be stored in this memory block */
        lm_mat_elem_size_t elem_tot;

        /** @brief Byte size of assigned memory block */
        lm_mat_mem_size_t bytes;

    } mem;

} lm_mat_t;


/*
 *******************************************************************************
 * Global variables
 *******************************************************************************
 */


/*
 *******************************************************************************
 * Public functions declaration
 *******************************************************************************
 */


/*
 *******************************************************************************
 * Private functions declaration
 *******************************************************************************
 */


/*
 *******************************************************************************
 * Public functions
 *******************************************************************************
 */

lm_rtn_t lm_mat_set(lm_mat_t *p_mat,
                    lm_mat_dim_size_t r_size,
                    lm_mat_dim_size_t c_size,
                    lm_mat_elem_t *p_men,
                    lm_mat_elem_size_t mem_elem_tot);

#if LM_MAT_NAME_ENABLED
lm_rtn_t lm_mat_set_name(lm_mat_t *p_mat, const char *p_name);
#endif // LM_MAT_NAME_ENABLED

lm_rtn_t lm_mat_clr(lm_mat_t *p_mat);

lm_rtn_t lm_mat_dump(const lm_mat_t *p_mat);

lm_rtn_t lm_mat_elem_set(lm_mat_t *p_mat,
                         lm_mat_dim_size_t r_idx,
                         lm_mat_dim_size_t c_idx,
                         lm_mat_elem_t value);

lm_rtn_t lm_mat_elem_get(lm_mat_t *p_mat,
                         lm_mat_dim_size_t r_idx,
                         lm_mat_dim_size_t c_idx,
                         lm_mat_elem_t *p_value);


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */


#endif // __LM_MAT_H__
