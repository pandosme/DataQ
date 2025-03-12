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
 * @file    lm_mat.c
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#include <string.h>

#include "lm_mat.h"
#include "lm_err.h"
#include "lm_log.h"


/*
 *******************************************************************************
 * Constant value definition
 *******************************************************************************
 */


/*
 *******************************************************************************
 * Data type definition
 *******************************************************************************
 */


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

/**
 * lm_mat_set - Function to initialize a matrix handle.
 *
 * @param   [in,out]    *p_mat          Address of new matrix handle.
 * @param   [in]        r_size          Row size of this matrix.
 * @param   [in]        c_size          Column size of this matrix.
 * @param   [in]        *p_men          Address of external memory buffer.
 * @param   [in]        mem_elem_tot    Number of total element which can be stored
 *                                      in external memory buffer.
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_mat_set(lm_mat_t *p_mat,
                    lm_mat_dim_size_t r_size,
                    lm_mat_dim_size_t c_size,
                    lm_mat_elem_t *p_men,
                    lm_mat_elem_size_t mem_elem_tot)
{
    if (p_mat == NULL || p_men == NULL) {
        return LM_ERR_CODE(LM_ERR_NULL_PTR);
    }

    if (r_size > LM_MAT_DIM_LIMIT || c_size > LM_MAT_DIM_LIMIT) {

        return LM_ERR_CODE(LM_ERR_MAT_DIM_LIMIT_EXCEEDED);
    }

    if (r_size == 0 || c_size == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    if ((r_size * c_size) > mem_elem_tot) {
        return LM_ERR_CODE(LM_ERR_NEED_MORE_MEM);
    }

#if LM_MAT_NAME_ENABLED
    strncpy((char *)p_mat->name, "LM", sizeof(p_mat->name) - 1);
#endif // LM_MAT_NAME_ENABLED

    p_mat->stats = 0;
    p_mat->elem.ptr = p_men;
    p_mat->elem.dim.r = r_size;
    p_mat->elem.dim.c = c_size;
    p_mat->elem.nxt_r_osf = c_size;
    p_mat->mem.ptr = p_men;
    p_mat->mem.elem_tot = mem_elem_tot;
    p_mat->mem.bytes = (lm_mat_mem_size_t)(mem_elem_tot * LM_MAT_SIZEOF_ELEM);

    LM_MAT_SET_STATS(p_mat, (LM_MAT_STATS_INIT));

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_mat_set - Function to assign a name to matrix handle.
 *
 * @param   [in,out]    *p_mat          Address of initialized handle.
 * @param   [in]        *p_name         Name string for this handle.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
#if LM_MAT_NAME_ENABLED
lm_rtn_t lm_mat_set_name(lm_mat_t *p_mat, const char *p_name)
{

    strncpy((char *)p_mat->name, (char *)p_name, sizeof(p_mat->name) - 1);


    return LM_ERR_CODE(LM_SUCCESS);
}
#endif // LM_MAT_NAME_ENABLED

/**
 * lm_mat_clr - Function to clear a matrix handle.
 *
 * Please note that the externally allocated memory buffer should be
 * released by the caller if the memory buffer is dynamic allocated.
 *
 * @param   [in,out]    *p_mat      Address of matrix handle that
 *                                  needs to be clear.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_mat_clr(lm_mat_t *p_mat)
{
    if (p_mat == NULL) {
        return LM_ERR_CODE(LM_ERR_NULL_PTR);
    }

#if LM_MAT_NAME_ENABLED
    p_mat->name[0] = 0;
#endif // LM_MAT_NAME_ENABLED

    p_mat->stats = 0;
    p_mat->elem.ptr = NULL;
    p_mat->elem.dim.r = 0;
    p_mat->elem.dim.c = 0;
    p_mat->elem.nxt_r_osf = 0;
    p_mat->mem.ptr = NULL;
    p_mat->mem.elem_tot = 0;
    p_mat->mem.bytes = 0;

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_mat_dump - Function to dump the elements of a matrix handle.
 *
 * @param   [in]        *p_mat   The matrix that needs to be dump.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_mat_dump(const lm_mat_t *p_mat)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat);
    const lm_mat_elem_size_t nxt_r_osf = LM_MAT_GET_NXT_OFS(p_mat);
    const lm_mat_elem_t *p_elem = NULL;

    if (p_mat == NULL) {
        return LM_ERR_CODE(LM_ERR_NULL_PTR);
    }

    LM_LOG_INFO("%s = [\n", p_mat->name);

    if (p_mat->elem.ptr == NULL) {
        LM_LOG_INFO("\t{NULL}\n");
    }
    else if (p_mat->elem.dim.r == 0 || p_mat->elem.dim.c == 0) {
        LM_LOG_INFO("\t{DIM = 0}\n");
    }
    else {
        for (r_idx = 0; r_idx < r_size; r_idx++) {

            p_elem = LM_MAT_GET_ELEM_PTR(p_mat)
                   + (r_idx * nxt_r_osf);

            for (c_idx = 0; c_idx < c_size; c_idx++) {

                if (*p_elem >= LM_MAT_ZERO_VAL) {
                    LM_LOG_INFO("\t " LM_MAT_ELEM_PRINT_FMT, *p_elem);
                }
                else {
                    LM_LOG_INFO("\t" LM_MAT_ELEM_PRINT_FMT, *p_elem);
                }

                LM_MAT_TO_NXT_ELEM(p_elem, p_mat);
            }

            LM_LOG_INFO(";\n");
        }
    }


    LM_LOG_INFO("]\n");

    LM_LOG_INFO("\n");

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_mat_elem_set - Function to set value of specific element in given matrix
 *
 * @param   [in,out]    *p_mat          Matrix handle.
 * @param   [in]        r_idx           Row index of element (start from 0).
 * @param   [in]        c_idx           Column index of element (start from 0).
 * @param   [in]        value           New value of specific element.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_mat_elem_set(lm_mat_t *p_mat,
                         lm_mat_dim_size_t r_idx,
                         lm_mat_dim_size_t c_idx,
                         lm_mat_elem_t value)
{
    if (r_idx >= p_mat->elem.dim.r) {
        return LM_ERR_CODE(LM_ERR_MAT_ROW_IDX_OUT_OF_RANGE);
    }

    if (c_idx >= p_mat->elem.dim.c) {
        return LM_ERR_CODE(LM_ERR_MAT_COL_IDX_OUT_OF_RANGE);
    }

    *(LM_MAT_GET_ROW_PTR(p_mat,
                         p_mat->elem.nxt_r_osf,
                         r_idx) + c_idx) = value;

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_mat_elem_get - Function to get value of specific element in given matrix
 *
 * @param   [in,out]    *p_mat          Matrix handle.
 * @param   [in]        r_idx           Row index of element (start from 0).
 * @param   [in]        c_idx           Column index of element (start from 0).
 * @param   [in]        *p_value        Value output buffer.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_mat_elem_get(lm_mat_t *p_mat,
                         lm_mat_dim_size_t r_idx,
                         lm_mat_dim_size_t c_idx,
                         lm_mat_elem_t *p_value)
{
    if (r_idx >= p_mat->elem.dim.r) {
        return LM_ERR_CODE(LM_ERR_MAT_ROW_IDX_OUT_OF_RANGE);
    }

    if (c_idx >= p_mat->elem.dim.c) {
        return LM_ERR_CODE(LM_ERR_MAT_COL_IDX_OUT_OF_RANGE);
    }

    *p_value = *(LM_MAT_GET_ROW_PTR(p_mat,
                                    p_mat->elem.nxt_r_osf,
                                    r_idx) + c_idx);

    return LM_ERR_CODE(LM_SUCCESS);
}


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */

