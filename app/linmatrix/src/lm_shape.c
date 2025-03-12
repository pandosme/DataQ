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
 * @file    lm_shape.c
 * @brief   Lin matrix shaping functions
 *
 * @note    Please note that all the "shape" functions are designed to
 *          temporarily change the row and column access order and row
 *          and column size of matrices which have been assigned to specific
 *          matrix handle. This does not really change the arrangement of
 *          matrix elements in memory, nor does it produce any memory copies.
 *          Any write operation to matrix elements via "shaped matrix handle"
 *          will directly overwrite the values in the original matrix.
 *
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#include "lm_shape.h"
#include "lm_global.h"
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
 * lm_shape_row_vect - Function to setup a new matrix handle that point
 *                     to the specific row vector of the matrix stored in
 *                     specific matrix handle.
 *
 * @param   [in]        *p_mat_a        Handle of matrix A.
 * @param   [in]        row_idx         The index of specific row of matrix A.
 * @param   [in,out]    p_mat_shaped    Handle of reshaped matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_shape_row_vect(const lm_mat_t *p_mat_a,
                           lm_mat_dim_size_t row_idx,
                           lm_mat_t *p_mat_shaped)
{
    const lm_mat_dim_size_t r_size_a = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    if (row_idx >= r_size_a) {
        return LM_ERR_CODE(LM_ERR_MAT_ROW_IDX_OUT_OF_RANGE);
    }

    *p_mat_shaped = *p_mat_a;

    p_mat_shaped->elem.ptr += (row_idx * nxt_r_osf_a);
    p_mat_shaped->elem.dim.r = 1;

    LM_MAT_SET_STATS(p_mat_shaped, (LM_MAT_STATS_RESHAPED));

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_shape_col_vect - Function to setup a new matrix handle that point
 *                     to the specific column vector of the matrix stored
 *                     in specific matrix handle.
 *
 * @param   [in]        *p_mat_a        Handle of matrix A.
 * @param   [in]        col_idx         The index of specific column of matrix A.
 * @param   [in,out]    p_mat_shaped    Handle of reshaped matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_shape_col_vect(const lm_mat_t *p_mat_a,
                           lm_mat_dim_size_t col_idx,
                           lm_mat_t *p_mat_shaped)
{
    const lm_mat_dim_size_t c_size_a = LM_MAT_GET_C_SIZE(p_mat_a);

    if (col_idx >= c_size_a) {
        return LM_ERR_CODE(LM_ERR_MAT_COL_IDX_OUT_OF_RANGE);
    }

    *p_mat_shaped = *p_mat_a;

    p_mat_shaped->elem.ptr += col_idx;
    p_mat_shaped->elem.dim.c = 1;

    LM_MAT_SET_STATS(p_mat_shaped, (LM_MAT_STATS_RESHAPED));

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_shape_submatrix - Function to setup a new matrix handle that point to
 *                      a specific subblock (submatrix) of the matrix stored
 *                      in specific matrix handle.
 *
 * @param   [in]        *p_mat_a        Handle of matrix A.
 * @param   [in]        row_idx         The index of specific row of matrix A.
 * @param   [in]        col_idx         The index of specific column of matrix A.
 * @param   [in]        row_idx         Row size of sub matrix.
 * @param   [in]        col_idx         Column size of sub matrix.
 * @param   [in,out]    p_mat_shaped    Handle of reshaped matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_shape_submatrix(const lm_mat_t *p_mat_a,
                            lm_mat_dim_size_t row_idx,
                            lm_mat_dim_size_t col_idx,
                            lm_mat_dim_size_t row_num,
                            lm_mat_dim_size_t col_num,
                            lm_mat_t *p_mat_shaped)
{
    const lm_mat_dim_size_t r_size_a = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size_a = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    if (row_idx + row_num > r_size_a) {
        return LM_ERR_CODE(LM_ERR_MAT_ROW_IDX_OUT_OF_RANGE);
    }

    if (col_idx + col_num > c_size_a) {
        return LM_ERR_CODE(LM_ERR_MAT_COL_IDX_OUT_OF_RANGE);
    }

    *p_mat_shaped = *p_mat_a;

    p_mat_shaped->elem.ptr += (row_idx * nxt_r_osf_a + col_idx);
    p_mat_shaped->elem.dim.r = row_num;
    p_mat_shaped->elem.dim.c = col_num;

    LM_MAT_SET_STATS(p_mat_shaped, (LM_MAT_STATS_RESHAPED));

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_shape_diag - Function to setup a new matrix handle that point to the
 *                 diagonal of the matrix stored in specific matrix handle.
 *
 * Please note that the diagonal elements of matrix A will be presented
 * as N by 1 vector by the new matrix handle.
 *
 * @param   [in]        *p_mat_a        Handle of matrix A.
 * @param   [in]        diag_osf        The diagonal offset.
 *                                      = 0: main diagonal,
 *                                      > 0: offset to specific superdiagonal,
 *                                      < 0: offset to specific subdiagonal,
 * @param   [in,out]    p_mat_shaped    Handle of reshaped matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_shape_diag(const lm_mat_t *p_mat_a,
                       lm_mat_dim_offset_t diag_osf,
                       lm_mat_t *p_mat_shaped)
{
    *p_mat_shaped = *p_mat_a;
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    if (p_mat_a->elem.dim.r == 0 || p_mat_a->elem.dim.c == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    /* Main diagonal, Subdiagonal */
    if (diag_osf <= 0) {

        diag_osf = (lm_mat_dim_offset_t)(-diag_osf);

        if (diag_osf >= p_mat_a->elem.dim.r) {

            return LM_ERR_CODE(LM_ERR_MAT_ROW_IDX_OUT_OF_RANGE);
        }

        p_mat_shaped->elem.ptr = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, diag_osf);
        p_mat_shaped->elem.dim.r = LM_MIN((p_mat_a->elem.dim.r - diag_osf),
                                          p_mat_a->elem.dim.c);
        p_mat_shaped->elem.dim.c = 1;
    }
    /* Superdiagonal */
    else {

        if (diag_osf >= p_mat_a->elem.dim.c) {

            return LM_ERR_CODE(LM_ERR_MAT_COL_IDX_OUT_OF_RANGE);
        }

        p_mat_shaped->elem.ptr = LM_MAT_GET_COL_PTR(p_mat_a, 1, diag_osf);
        p_mat_shaped->elem.dim.r = LM_MIN(p_mat_a->elem.dim.r,
                                          (p_mat_a->elem.dim.c - diag_osf));
    }

    p_mat_shaped->elem.dim.c = 1;
    p_mat_shaped->elem.nxt_r_osf += 1;

    LM_MAT_SET_STATS(p_mat_shaped, (LM_MAT_STATS_RESHAPED));

    return LM_ERR_CODE(LM_SUCCESS);
}


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */



