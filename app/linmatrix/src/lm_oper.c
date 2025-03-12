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
 * @file    lm_oper.c
 * @brief   Lin matrix operating functions
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#include <math.h>

#include "lm_oper.h"
#include "lm_mat.h"
#include "lm_err.h"
#include "lm_chk.h"
#include "lm_permute.h"
#include "lm_shape.h"


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
 * lm_oper_zeros - Function to all elements of the matrix to zero.
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 * @param   [out]       *p_norm         Frobenius norm of matrix A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_zeros(lm_mat_t *p_mat_a)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_elem_t *p_elem_a = NULL;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    for (r_idx = 0; r_idx < r_size; r_idx++) {

        p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

        for (c_idx = 0; c_idx < c_size; c_idx++) {

            /* A[i][j] = zero */
            p_elem_a[0] = LM_MAT_ZERO_VAL;

            LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
        }

    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_zeros_diagonal - Function to set diagonal elements of the matrix to zero.
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 * @param   [in]        diag_osf        The diagonal offset.
 *                                      = 0: main diagonal,
 *                                      > 0: offset to specific superdiagonal,
 *                                      < 0: offset to specific subdiagonal,
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_zeros_diagonal(const lm_mat_t *p_mat_a,
                                lm_mat_dim_offset_t diag_osf)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_dim_size_t dim;
    const lm_mat_dim_size_t r_size_a = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size_a = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    /* Main diagonal, Subdiagonal */
    if (diag_osf <= 0) {

        diag_osf = (lm_mat_dim_offset_t)(-diag_osf);

        if (diag_osf >= r_size_a) {

            return LM_ERR_CODE(LM_ERR_MAT_ROW_IDX_OUT_OF_RANGE);
        }

        r_idx = (lm_mat_dim_size_t)(diag_osf);
        c_idx = 0;
        dim = (lm_mat_dim_size_t)(r_idx + (LM_MIN((r_size_a - diag_osf), c_size_a)));
    }
    /* Superdiagonal */
    else {

        if (diag_osf >= c_size_a) {

            return LM_ERR_CODE(LM_ERR_MAT_COL_IDX_OUT_OF_RANGE);
        }

        r_idx = 0;
        c_idx = (lm_mat_dim_size_t)(diag_osf);
        dim = (lm_mat_dim_size_t)(LM_MIN(r_size_a, (c_size_a - diag_osf)));
    }

    for (; r_idx < dim; r_idx++) {

        *(LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx) + c_idx) = LM_MAT_ZERO_VAL;

        c_idx++;
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_zeros_triu - Function to set upper triangular elements
 *                               of the matrix to zero.
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 * @param   [in]        diag_osf        The diagonal offset.
 *                                      = 0: main diagonal,
 *                                      > 0: offset to specific superdiagonal,
 *                                      < 0: offset to specific subdiagonal,
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_zeros_triu(lm_mat_t *p_mat_a,
                            lm_mat_dim_offset_t diag_osf)
{
    lm_rtn_t result;
    const lm_mat_dim_size_t r_size_a = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size_a = LM_MAT_GET_C_SIZE(p_mat_a);

    result = LM_ERR_CODE(LM_ERR_UNKNOWN);

    if (diag_osf >= 0) {

        if (diag_osf >= c_size_a) {

            return LM_ERR_CODE(LM_ERR_MAT_COL_IDX_OUT_OF_RANGE);
        }

    }
    else {

        if ((-diag_osf) >= r_size_a) {

            return LM_ERR_CODE(LM_ERR_MAT_ROW_IDX_OUT_OF_RANGE);
        }
    }

    for (; diag_osf < (lm_mat_dim_offset_t)(c_size_a); diag_osf++) {

        result = lm_oper_zeros_diagonal(p_mat_a, diag_osf);

        if (LM_IS_ERR(result) == true) {

            break;
        }

    }

    return result;
}

/**
 * lm_oper_zeros_tril - Function to set lower triangular elements
 *                      of the matrix to zero.
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 * @param   [in]        diag_osf        The diagonal offset.
 *                                      = 0: main diagonal,
 *                                      > 0: offset to specific superdiagonal,
 *                                      < 0: offset to specific subdiagonal,
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_zeros_tril(lm_mat_t *p_mat_a,
                            lm_mat_dim_offset_t diag_osf)
{
    lm_rtn_t result;
    const lm_mat_dim_size_t r_size_a = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size_a = LM_MAT_GET_C_SIZE(p_mat_a);

    result = LM_ERR_CODE(LM_ERR_UNKNOWN);

    if (diag_osf >= 0) {

        if (diag_osf >= c_size_a) {

            return LM_ERR_CODE(LM_ERR_MAT_COL_IDX_OUT_OF_RANGE);
        }

    }
    else {

        if ((-diag_osf) >= r_size_a) {

            return LM_ERR_CODE(LM_ERR_MAT_ROW_IDX_OUT_OF_RANGE);
        }
    }

    diag_osf = (-diag_osf);

    for (; diag_osf < (lm_mat_dim_offset_t)(r_size_a); diag_osf++) {

        result = lm_oper_zeros_diagonal(p_mat_a, (-diag_osf));

        if (LM_IS_ERR(result) == true) {

            break;
        }
    }

    return result;
}

/**
 * lm_oper_identity - Function to set diagonal elements of the matrix to one
 *                    and the remaining elements to zero.
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_identity(lm_mat_t *p_mat_a)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_elem_t *p_elem_a = NULL;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    for (r_idx = 0; r_idx < r_size; r_idx++) {

        p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

        for (c_idx = 0; c_idx < c_size; c_idx++) {

            /* A[i][j] = zero */
            if (r_idx != c_idx) {
                p_elem_a[0] = LM_MAT_ZERO_VAL;
            }
            /* A[i][i] = one */
            else {
                p_elem_a[0] = LM_MAT_ONE_VAL;
            }

            LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
        }

    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_abs - Function to change all elements of the matrix to
 *               corresponding absolute value.
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_abs(lm_mat_t *p_mat_a)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_elem_t *p_elem_a = NULL;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    for (r_idx = 0; r_idx < r_size; r_idx++) {

        p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

        for (c_idx = 0; c_idx < c_size; c_idx++) {

            /* Turn negative number into positive number */
            if (p_elem_a[0] < LM_MAT_ZERO_VAL) {
                p_elem_a[0] = (-p_elem_a[0]);
            }

            LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
        }

    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_max - Function to find out the maximum value from all elements
 *               of the matrix.
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 * @param   [out]       *p_r_idx        The row position of the element
 *                                      with the maximum value.
 * @param   [out]       *p_c_idx        The column position of the element
 *                                      with the maximum value.
 * @param   [out]       *p_max          Maximum value found.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_max(const lm_mat_t *p_mat_a,
                     lm_mat_dim_size_t *p_r_idx,
                     lm_mat_dim_size_t *p_c_idx,
                     lm_mat_elem_t *p_max)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_elem_t max_tmp;
    lm_mat_dim_size_t match_r_idx;
    lm_mat_dim_size_t match_c_idx;
    const lm_mat_elem_t *p_elem_a = LM_MAT_GET_ELEM_PTR(p_mat_a);
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    if (r_size == 0 || c_size == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    max_tmp = p_elem_a[0];
    match_r_idx = 0;
    match_c_idx = 0;

    for (r_idx = 0; r_idx < r_size; r_idx++) {

        p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

        for (c_idx = 0; c_idx < c_size; c_idx++) {

            /* Turn negative number into positive number */
            if (p_elem_a[0] > max_tmp) {
                max_tmp = p_elem_a[0];
                match_r_idx = r_idx;
                match_c_idx = c_idx;
            }

            LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
        }

    }

    *p_max = max_tmp;
    *p_r_idx = match_r_idx;
    *p_c_idx = match_c_idx;

    LM_ASSERT_DBG((*p_r_idx < r_size), "Exceeded row range");
    LM_ASSERT_DBG((*p_c_idx < c_size), "Exceeded column range");

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_max_abs - Function to find out the absolute maximum value
 *                   from all elements of the matrix.
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 * @param   [out]       *p_r_idx        The row position of the element
 *                                      with the maximum absolute value.
 * @param   [out]       *p_c_idx        The column position of the element
 *                                      with the maximum absolute value.
 * @param   [out]       *p_max          Maximum absolute value found.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_max_abs(const lm_mat_t *p_mat_a,
                         lm_mat_dim_size_t *p_r_idx,
                         lm_mat_dim_size_t *p_c_idx,
                         lm_mat_elem_t *p_max)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_elem_t max_tmp;
    lm_mat_dim_size_t match_r_idx;
    lm_mat_dim_size_t match_c_idx;
    const lm_mat_elem_t *p_elem_a = LM_MAT_GET_ELEM_PTR(p_mat_a);
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    if (r_size == 0 || c_size == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    max_tmp = (p_elem_a[0] < 0) ? (-p_elem_a[0]) : p_elem_a[0];
    match_r_idx = 0;
    match_c_idx = 0;

    for (r_idx = 0; r_idx < r_size; r_idx++) {

        p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

        for (c_idx = 0; c_idx < c_size; c_idx++) {

            /* Find out the maximum abs(X) */
            if (p_elem_a[0] > max_tmp) {
                max_tmp = p_elem_a[0];
                match_r_idx = r_idx;
                match_c_idx = c_idx;
            }
            else if (-p_elem_a[0] > max_tmp) {
                max_tmp = (-p_elem_a[0]);
                match_r_idx = r_idx;
                match_c_idx = c_idx;
            }

            LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
        }

    }

    *p_max = max_tmp;
    *p_r_idx = match_r_idx;
    *p_c_idx = match_c_idx;

    LM_ASSERT_DBG((*p_max >= LM_MAT_ZERO_VAL), "Should be positive number");
    LM_ASSERT_DBG((*p_r_idx < r_size), "Exceeded row range");
    LM_ASSERT_DBG((*p_c_idx < c_size), "Exceeded column range");

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_min - Function to find out the minimum value from all elements
 *               of the matrix.
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 * @param   [out]       *p_r_idx        The row position of the element
 *                                      with the minimum value.
 * @param   [out]       *p_c_idx        The column position of the element
 *                                      with the minimum value.
 * @param   [out]       *p_min          Minimum value found.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_min(const lm_mat_t *p_mat_a,
                     lm_mat_dim_size_t *p_r_idx,
                     lm_mat_dim_size_t *p_c_idx,
                     lm_mat_elem_t *p_min)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_elem_t min_tmp;
    lm_mat_dim_size_t match_r_idx;
    lm_mat_dim_size_t match_c_idx;
    const lm_mat_elem_t *p_elem_a = LM_MAT_GET_ELEM_PTR(p_mat_a);
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    if (r_size == 0 || c_size == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    min_tmp = p_elem_a[0];
    match_r_idx = 0;
    match_c_idx = 0;

    for (r_idx = 0; r_idx < r_size; r_idx++) {

        p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

        for (c_idx = 0; c_idx < c_size; c_idx++) {

            /* Turn negative number into positive number */
            if (p_elem_a[0] < min_tmp) {
                min_tmp = p_elem_a[0];
                match_r_idx = r_idx;
                match_c_idx = c_idx;
            }

            LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
        }

    }

    *p_min = min_tmp;
    *p_r_idx = match_r_idx;
    *p_c_idx = match_c_idx;

    LM_ASSERT_DBG((*p_r_idx < r_size), "Exceeded row range");
    LM_ASSERT_DBG((*p_c_idx < c_size), "Exceeded column range");

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_min_abs - Function to find out the minimum absolute value from
 *                   all elements of the matrix.
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 * @param   [out]       *p_r_idx        The row position of the element
 *                                      with the minimum absolute value.
 * @param   [out]       *p_c_idx        The column position of the element
 *                                      with the minimum absolute value.
 * @param   [out]       *p_min          Minimum absolute value found.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_min_abs(const lm_mat_t *p_mat_a,
                         lm_mat_dim_size_t *p_r_idx,
                         lm_mat_dim_size_t *p_c_idx,
                         lm_mat_elem_t *p_min)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_elem_t min_tmp;
    lm_mat_dim_size_t match_r_idx;
    lm_mat_dim_size_t match_c_idx;
    const lm_mat_elem_t *p_elem_a = LM_MAT_GET_ELEM_PTR(p_mat_a);
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    if (r_size == 0 || c_size == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    min_tmp = (p_elem_a[0] < 0) ? (-p_elem_a[0]) : p_elem_a[0];
    match_r_idx = 0;
    match_c_idx = 0;

    for (r_idx = 0; r_idx < r_size; r_idx++) {

        p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

        for (c_idx = 0; c_idx < c_size; c_idx++) {

            /* Find out the minimum abs(X) */
            if (p_elem_a[0] < LM_MAT_ZERO_VAL) {

                if ((-p_elem_a[0]) < min_tmp) {
                    min_tmp = (-p_elem_a[0]);
                    match_r_idx = r_idx;
                    match_c_idx = c_idx;
                }
            }
            else if (p_elem_a[0] < min_tmp) {
                min_tmp = p_elem_a[0];
                match_r_idx = r_idx;
                match_c_idx = c_idx;
            }

            LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
        }

    }

    *p_min = min_tmp;
    *p_r_idx = match_r_idx;
    *p_c_idx = match_c_idx;

    LM_ASSERT_DBG((*p_min >= LM_MAT_ZERO_VAL), "Should be positive number");
    LM_ASSERT_DBG((*p_r_idx < r_size), "Exceeded row range");
    LM_ASSERT_DBG((*p_c_idx < c_size), "Exceeded column range");

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_swap_row - Function to swap specific 2 rows of the matrix.
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 * @param   [in]        src_r_idx       The index of source row.
 * @param   [in]        dst_r_idx       The index of destination row.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_swap_row(lm_mat_t *p_mat_a,
                          lm_mat_dim_size_t src_r_idx,
                          lm_mat_dim_size_t dst_r_idx)
{
    lm_mat_dim_size_t c_idx;
    lm_mat_elem_t tmp;
    lm_mat_elem_t *p_elem_src = NULL;
    lm_mat_elem_t *p_elem_dst = NULL;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    if (src_r_idx >= r_size || dst_r_idx >= r_size) {

        return LM_ERR_CODE(LM_ERR_MAT_ROW_IDX_OUT_OF_RANGE);
    }

    p_elem_src = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, src_r_idx);
    p_elem_dst = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, dst_r_idx);

    for (c_idx = 0; c_idx < c_size; c_idx++) {

        tmp = p_elem_src[c_idx];
        p_elem_src[c_idx] = p_elem_dst[c_idx];
        p_elem_dst[c_idx] = tmp;

    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_swap_col - Function to swap specific 2 columns of the matrix.
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 * @param   [in]        src_c_idx       The index of source column.
 * @param   [in]        dst_c_idx       The index of destination column.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_swap_col(lm_mat_t *p_mat_a,
                          lm_mat_dim_size_t src_c_idx,
                          lm_mat_dim_size_t dst_c_idx)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_elem_t tmp;
    lm_mat_elem_t *p_elem_src = NULL;
    lm_mat_elem_t *p_elem_dst = NULL;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    if (src_c_idx >= c_size || dst_c_idx >= c_size) {

        return LM_ERR_CODE(LM_ERR_MAT_COL_IDX_OUT_OF_RANGE);
    }

    p_elem_src = LM_MAT_GET_COL_PTR(p_mat_a, 1, src_c_idx);
    p_elem_dst = LM_MAT_GET_COL_PTR(p_mat_a, 1, dst_c_idx);

    for (r_idx = 0; r_idx < r_size; r_idx++) {

        tmp = p_elem_src[0];
        p_elem_src[0] = p_elem_dst[0];
        p_elem_dst[0] = tmp;

        LM_MAT_TO_NXT_ROW(p_elem_src, nxt_r_osf_a, p_mat_a);
        LM_MAT_TO_NXT_ROW(p_elem_dst, nxt_r_osf_a, p_mat_a);

    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_permute_row - Function to permute rows of the matrix
 *                       according to the cycle notations stored
 *                       in permutation list.
 *
 * https://en.wikipedia.org/wiki/Cyclic_permutation
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 * @param   [in]        *p_list         Handle of permutation list
 *                                      which contains cycle notations.
 *                                      The length of permutation list
 *                                      should be 2 * row size of matrix A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_permute_row(lm_mat_t *p_mat_a,
                             const lm_permute_list_t *p_list)
{
    lm_rtn_t result;
    lm_permute_size_t perm_elem_idx;
    lm_permute_size_t found_grp_cnt;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_permute_size_t perm_grp_num = p_list->elem.cyc_grp_num;
    const lm_permute_size_t elem_num = p_list->elem.num;
    const lm_permute_elem_t *p_perm_elem = p_list->elem.ptr;

    if (elem_num > p_list->mem.elem_tot) {
        return LM_ERR_CODE(LM_ERR_PM_ELEM_NUM_EXCEED_MEM_LIMIT);
    }

    if (perm_grp_num == 0) {
        return LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_NUM_IS_ZERO);
    }

    if (perm_grp_num > r_size) {
        return LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_NUM_OUT_OF_RANGE);
    }

    found_grp_cnt = 0;

    for (perm_elem_idx = 0; perm_elem_idx < (elem_num - 1); perm_elem_idx++) {

        /* End of this cycle group */
        if (p_perm_elem[perm_elem_idx] == LM_PERMUTE_CYCLE_END_SYM) {
            found_grp_cnt++;
        }
        else if (p_perm_elem[perm_elem_idx + 1] != LM_PERMUTE_CYCLE_END_SYM) {

            /* Swap [n_th][:] and [m_th][:] */
            result = lm_oper_swap_row(p_mat_a,
                                      p_perm_elem[perm_elem_idx],
                                      p_perm_elem[perm_elem_idx + 1]);

            LM_RETURN_IF_ERR(result);
        }
    }

    /*
     * Check the last element in the cycle notation buffer if this function can't find
     * all the cycle groups in the loop above after it scanned the (N - 1) elements.
     */
    if (found_grp_cnt != perm_grp_num) {

        if (p_perm_elem[perm_elem_idx] == LM_PERMUTE_CYCLE_END_SYM) {
            found_grp_cnt++;
        }
    }

    if (found_grp_cnt != perm_grp_num) {
        result = LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_NUM_MISMATCHED);
    }
    else {
        result = LM_ERR_CODE(LM_SUCCESS);
    }

    return result;
}

/**
 * lm_oper_permute_row_inverse - Function to permute rows of the matrix
 *                               according to the cycle notations stored
 *                               in permutation list inversely.
 *
 * https://en.wikipedia.org/wiki/Cyclic_permutation
 *
 * E.g. if the cycle notations stored in permutation list is
 *      (1 4 6 8 3 7)(2)(5), the corresponding inverse cycle
 *      notations is (5)(2)(7 3 8 6 4 1).
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 * @param   [in]        *p_list         Handle of permutation list
 *                                      which contains cycle notations.
 *                                      The length of permutation list
 *                                      should be 2 * row size of matrix A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_permute_row_inverse(lm_mat_t *p_mat_a,
                                     const lm_permute_list_t *p_list)
{
    lm_rtn_t result;
    lm_permute_size_t perm_elem_idx;
    lm_permute_size_t found_grp_cnt;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_permute_size_t perm_grp_num = p_list->elem.cyc_grp_num;
    const lm_permute_size_t elem_num = p_list->elem.num;
    const lm_permute_elem_t *p_perm_elem = p_list->elem.ptr;

    if (elem_num > p_list->mem.elem_tot) {
        return LM_ERR_CODE(LM_ERR_PM_ELEM_NUM_EXCEED_MEM_LIMIT);
    }

    if (perm_grp_num == 0) {
        return LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_NUM_IS_ZERO);
    }

    if (perm_grp_num > r_size) {
        return LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_NUM_OUT_OF_RANGE);
    }

    found_grp_cnt = 0;

    for (perm_elem_idx = elem_num; perm_elem_idx > 1 ; perm_elem_idx--) {

        /* End of this cycle group */
        if (p_perm_elem[(perm_elem_idx - 1)] == LM_PERMUTE_CYCLE_END_SYM) {
            found_grp_cnt++;
        }
        else if (p_perm_elem[(perm_elem_idx - 2)] != LM_PERMUTE_CYCLE_END_SYM) {

            /* Swap [n_th][:] and [m_th][:] */
            result = lm_oper_swap_row(p_mat_a,
                                      p_perm_elem[(perm_elem_idx - 1)],
                                      p_perm_elem[(perm_elem_idx - 2)]);
            LM_RETURN_IF_ERR(result);
        }
    }

    if (found_grp_cnt != perm_grp_num) {
        result = LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_NUM_MISMATCHED);
    }
    else {
        result = LM_ERR_CODE(LM_SUCCESS);
    }

    return result;
}

/**
 * lm_oper_permute_col - Function to permute columns of the matrix
 *                       according to the cycle notations stored
 *                       in permutation list.
 *
 * https://en.wikipedia.org/wiki/Cyclic_permutation
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 * @param   [in]        *p_list         Handle of permutation list
 *                                      which contains cycle notations.
 *                                      The length of permutation list
 *                                      should be 2 * column size of matrix A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_permute_col(lm_mat_t *p_mat_a,
                             const lm_permute_list_t *p_list)
{
    lm_rtn_t result;
    lm_permute_size_t perm_elem_idx;
    lm_permute_size_t found_grp_cnt;
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_permute_size_t perm_grp_num = p_list->elem.cyc_grp_num;
    const lm_permute_size_t elem_num = p_list->elem.num;
    const lm_permute_elem_t *p_perm_elem = p_list->elem.ptr;

    if (elem_num > p_list->mem.elem_tot) {
        return LM_ERR_CODE(LM_ERR_PM_ELEM_NUM_EXCEED_MEM_LIMIT);
    }

    if (perm_grp_num == 0) {
        return LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_NUM_IS_ZERO);
    }

    if (perm_grp_num > c_size) {
        return LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_NUM_OUT_OF_RANGE);
    }

    found_grp_cnt = 0;

    for (perm_elem_idx = 0; perm_elem_idx < (elem_num - 1); perm_elem_idx++) {

        /* End of this cycle group */
        if (p_perm_elem[perm_elem_idx] == LM_PERMUTE_CYCLE_END_SYM) {
            found_grp_cnt++;
        }
        else if (p_perm_elem[perm_elem_idx + 1] != LM_PERMUTE_CYCLE_END_SYM) {

            /* Swap [:][n_th] and [:][m_th] */
            result = lm_oper_swap_col(p_mat_a,
                                      p_perm_elem[perm_elem_idx],
                                      p_perm_elem[perm_elem_idx + 1]);

            LM_RETURN_IF_ERR(result);
        }
    }

    /*
     * Check the last element in the cycle notation buffer if this function can't find
     * all the cycle groups in the loop above after it scanned the (N - 1) elements.
     */
    if (found_grp_cnt != perm_grp_num) {

        if (p_perm_elem[perm_elem_idx] == LM_PERMUTE_CYCLE_END_SYM) {
            found_grp_cnt++;
        }
    }

    if (found_grp_cnt != perm_grp_num) {
        result = LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_NUM_MISMATCHED);
    }
    else {
        result = LM_ERR_CODE(LM_SUCCESS);
    }

    return result;
}

/**
 * lm_oper_permute_col_inverse - Function to permute columns of the matrix
 *                               according to the cycle notations stored
 *                               in permutation list inversely.
 *
 * https://en.wikipedia.org/wiki/Cyclic_permutation
 *
 * E.g. if the cycle notations stored in permutation list is
 *      (1 4 6 8 3 7)(2)(5), the corresponding inverse cycle
 *      notations is (5)(2)(7 3 8 6 4 1).
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 * @param   [in]        *p_list         Handle of permutation list
 *                                      which contains cycle notations.
 *                                      The length of permutation list
 *                                      should be 2 * column size of matrix A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_permute_col_inverse(lm_mat_t *p_mat_a,
                                     const lm_permute_list_t *p_list)
{
    lm_rtn_t result;
    lm_permute_size_t perm_elem_idx;
    lm_permute_size_t found_grp_cnt;
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_permute_size_t perm_grp_num = p_list->elem.cyc_grp_num;
    const lm_permute_size_t elem_num = p_list->elem.num;
    const lm_permute_elem_t *p_perm_elem = p_list->elem.ptr;

    if (elem_num > p_list->mem.elem_tot) {
        return LM_ERR_CODE(LM_ERR_PM_ELEM_NUM_EXCEED_MEM_LIMIT);
    }

    if (perm_grp_num == 0) {
        return LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_NUM_IS_ZERO);
    }

    if (perm_grp_num > c_size) {
        return LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_NUM_OUT_OF_RANGE);
    }

    found_grp_cnt = 0;

    for (perm_elem_idx = elem_num; perm_elem_idx > 1 ; perm_elem_idx--) {

        /* End of this cycle group */
        if (p_perm_elem[(perm_elem_idx - 1)] == LM_PERMUTE_CYCLE_END_SYM) {
            found_grp_cnt++;
        }
        else if (p_perm_elem[(perm_elem_idx - 2)] != LM_PERMUTE_CYCLE_END_SYM) {

            /* Swap [:][n_th] and [:][m_th] */
            result = lm_oper_swap_col(p_mat_a,
                                      p_perm_elem[(perm_elem_idx - 1)],
                                      p_perm_elem[(perm_elem_idx - 2)]);

            LM_RETURN_IF_ERR(result);
        }
    }

    if (found_grp_cnt != perm_grp_num) {
        result = LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_NUM_MISMATCHED);
    }
    else {
        result = LM_ERR_CODE(LM_SUCCESS);
    }

    return result;
}

/**
 * lm_oper_copy - Function to copy all the values of elements of source matrix
 *                to destination matrix.
 *
 * @param   [in]        *p_mat_src      Handle of source matrix.
 * @param   [out]       *p_mat_dst      Handle of destination matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_copy(const lm_mat_t *p_mat_src,
                      lm_mat_t *p_mat_dst)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    const lm_mat_elem_t *p_elem_src = NULL;
    lm_mat_elem_t *p_elem_dst = NULL;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_src);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_src);
    const lm_mat_elem_size_t nxt_r_osf_src = LM_MAT_GET_NXT_OFS(p_mat_src);
    const lm_mat_elem_size_t nxt_r_osf_dst = LM_MAT_GET_NXT_OFS(p_mat_dst);

    /* Matrix dimensions must be equal */
    if (p_mat_src->elem.dim.r != p_mat_dst->elem.dim.r
        || p_mat_src->elem.dim.c != p_mat_dst->elem.dim.c) {

        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    for (r_idx = 0; r_idx < r_size; r_idx++) {

        p_elem_src = LM_MAT_GET_ROW_PTR(p_mat_src, nxt_r_osf_src, r_idx);
        p_elem_dst = LM_MAT_GET_ROW_PTR(p_mat_dst, nxt_r_osf_dst, r_idx);

        for (c_idx = 0; c_idx < c_size; c_idx++) {

            /* B[i][j] = A[i][j] */
            p_elem_dst[0] = p_elem_src[0];

            LM_MAT_TO_NXT_ELEM(p_elem_src, p_mat_src);
            LM_MAT_TO_NXT_ELEM(p_elem_dst, p_mat_dst);
        }

    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_copy_diagonal - Function to copy the values of diagonal elements of
 *                         source matrix to destination matrix.
 *
 * @param   [in]        *p_mat_src      Handle of source matrix.
 * @param   [out]       *p_mat_dst      Handle of destination matrix.
 * @param   [in]        diag_osf        The diagonal offset.
 *                                      = 0: main diagonal,
 *                                      > 0: offset to specific superdiagonal,
 *                                      < 0: offset to specific subdiagonal,
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_copy_diagonal(const lm_mat_t *p_mat_src,
                               lm_mat_t *p_mat_dst,
                               lm_mat_dim_offset_t diag_osf)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_dim_size_t dim;
    const lm_mat_elem_t *p_elem_src = NULL;
    const lm_mat_dim_size_t r_size_src = LM_MAT_GET_R_SIZE(p_mat_src);
    const lm_mat_dim_size_t c_size_src = LM_MAT_GET_C_SIZE(p_mat_src);
    const lm_mat_elem_size_t nxt_r_osf_src = LM_MAT_GET_NXT_OFS(p_mat_src);

    lm_mat_elem_t *p_elem_dst = NULL;
    const lm_mat_dim_size_t r_size_dst = LM_MAT_GET_R_SIZE(p_mat_dst);
    const lm_mat_dim_size_t c_size_dst = LM_MAT_GET_C_SIZE(p_mat_dst);
    const lm_mat_elem_size_t nxt_r_osf_dst = LM_MAT_GET_NXT_OFS(p_mat_dst);

    if (r_size_src != r_size_dst || c_size_src != c_size_dst) {

        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    /* Main diagonal, Subdiagonal */
    if (diag_osf <= 0) {

        diag_osf = (lm_mat_dim_offset_t)(-diag_osf);

        if (diag_osf >= r_size_src) {

            return LM_ERR_CODE(LM_ERR_MAT_ROW_IDX_OUT_OF_RANGE);
        }

        r_idx = (lm_mat_dim_size_t)(diag_osf);
        c_idx = 0;
        dim = (lm_mat_dim_size_t)(r_idx + (LM_MIN((r_size_src - diag_osf), c_size_src)));
    }
    /* Superdiagonal */
    else {

        if (diag_osf >= c_size_src) {

            return LM_ERR_CODE(LM_ERR_MAT_COL_IDX_OUT_OF_RANGE);
        }

        r_idx = 0;
        c_idx = (lm_mat_dim_size_t)(diag_osf);
        dim = (lm_mat_dim_size_t)(LM_MIN(r_size_src, (c_size_src - diag_osf)));
    }

    for (; r_idx < dim; r_idx++) {

        p_elem_src = LM_MAT_GET_ROW_PTR(p_mat_src, nxt_r_osf_src, r_idx) + c_idx;
        p_elem_dst = LM_MAT_GET_ROW_PTR(p_mat_dst, nxt_r_osf_dst, r_idx) + c_idx;

        p_elem_dst[0] = p_elem_src[0];

        c_idx++;
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_copy_triu - Function to copy values of upper triangular elements
 *                     of the source matrix to destination matrix.
 *
 * @param   [in]        *p_mat_src      Handle of source matrix.
 * @param   [out]       *p_mat_dst      Handle of destination matrix.
 * @param   [in]        diag_osf        The diagonal offset.
 *                                      = 0: main diagonal,
 *                                      > 0: offset to specific superdiagonal,
 *                                      < 0: offset to specific subdiagonal,
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_copy_triu(const lm_mat_t *p_mat_src,
                           lm_mat_t *p_mat_dst,
                           lm_mat_dim_offset_t diag_osf)
{
    lm_rtn_t result;
    const lm_mat_dim_size_t r_size_src = LM_MAT_GET_R_SIZE(p_mat_src);
    const lm_mat_dim_size_t c_size_src = LM_MAT_GET_C_SIZE(p_mat_src);

    result = LM_ERR_CODE(LM_ERR_UNKNOWN);

    if (diag_osf >= 0) {

        if (diag_osf >= c_size_src) {

            return LM_ERR_CODE(LM_ERR_MAT_COL_IDX_OUT_OF_RANGE);
        }

    }
    else {

        if ((-diag_osf) >= r_size_src) {

            return LM_ERR_CODE(LM_ERR_MAT_ROW_IDX_OUT_OF_RANGE);
        }
    }

    for (; diag_osf < (lm_mat_dim_offset_t)(c_size_src); diag_osf++) {

        result = lm_oper_copy_diagonal(p_mat_src, p_mat_dst, diag_osf);

        if (LM_IS_ERR(result) == true) {

            break;
        }

    }

    return result;
}

/**
 * lm_oper_copy_tril - Function to copy values of lower triangular elements
 *                     of the source matrix to destination matrix.
 *
 * @param   [in]        *p_mat_src      Handle of source matrix.
 * @param   [out]       *p_mat_dst      Handle of destination matrix.
 * @param   [in]        diag_osf        The diagonal offset.
 *                                      = 0: main diagonal,
 *                                      > 0: offset to specific superdiagonal,
 *                                      < 0: offset to specific subdiagonal,
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_copy_tril(const lm_mat_t *p_mat_src,
                           lm_mat_t *p_mat_dst,
                           lm_mat_dim_offset_t diag_osf)
{
    lm_rtn_t result;
    const lm_mat_dim_size_t r_size_src = LM_MAT_GET_R_SIZE(p_mat_src);
    const lm_mat_dim_size_t c_size_src = LM_MAT_GET_C_SIZE(p_mat_src);

    result = LM_ERR_CODE(LM_ERR_UNKNOWN);

    if (diag_osf >= 0) {

        if (diag_osf >= c_size_src) {

            return LM_ERR_CODE(LM_ERR_MAT_COL_IDX_OUT_OF_RANGE);
        }

    }
    else {

        if ((-diag_osf) >= r_size_src) {

            return LM_ERR_CODE(LM_ERR_MAT_ROW_IDX_OUT_OF_RANGE);
        }
    }

    diag_osf = (-diag_osf);

    for (; diag_osf < (lm_mat_dim_offset_t)(r_size_src); diag_osf++) {

        result = lm_oper_copy_diagonal(p_mat_src,
                                       p_mat_dst,
                                       (lm_mat_dim_offset_t)(-diag_osf));

        if (LM_IS_ERR(result) == true) {

            break;
        }
    }

    return result;
}

/**
 * lm_oper_copy_transpose - Function to copy the transpose matrix A to
 *                          destination matrix.
 *
 * If the dimension of matrix is M by N, dimension of destination
 * matrix is N by M.
 *
 * @param   [in]        *p_mat_a        Handle of matrix A.
 * @param   [out]       *p_mat_trans    Handle of destination matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_copy_transpose(const lm_mat_t *p_mat_a,
                                lm_mat_t *p_mat_trans)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    const lm_mat_elem_t *p_elem_a = NULL;
    lm_mat_elem_t *p_elem_trans = NULL;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_trans = LM_MAT_GET_NXT_OFS(p_mat_trans);

    /* Need a different matrix to store the transpose output  */
    if (p_mat_a == p_mat_trans) {

        return LM_ERR_CODE(LM_ERR_MAT_NEED_DIFFERENT_MAT_TO_STORE_OUTPUT);
    }

    /* Matrix dimensions must be equal */
    if (p_mat_a->elem.dim.r != p_mat_trans->elem.dim.c
        || p_mat_a->elem.dim.c != p_mat_trans->elem.dim.r) {

        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    for (r_idx = 0; r_idx < r_size; r_idx++) {

        p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);
        p_elem_trans = LM_MAT_GET_COL_PTR(p_mat_trans, 1, r_idx);

        for (c_idx = 0; c_idx < c_size; c_idx++) {

            /* T[i][j] = A[j][i] */
            p_elem_trans[0] = p_elem_a[0];

            LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
            LM_MAT_TO_NXT_ROW(p_elem_trans, nxt_r_osf_trans, p_mat_trans);
        }

    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_transpose - Function to perform in-place transpose of matrix A.
 *
 * The matrix A should be a square matrix.
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_transpose(lm_mat_t *p_mat_a)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_elem_t tmp;
    lm_mat_elem_t *p_elem_a = NULL;
    lm_mat_elem_t *p_elem_b = NULL;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_b = LM_MAT_GET_NXT_OFS(p_mat_a);

    if (r_size == 0 || c_size == 0) {

        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    /* For square matrix only */
    if (r_size != c_size) {

        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_SQUARE);
    }

    for (r_idx = 0; r_idx < r_size - 1; r_idx++) {

        p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx)
                 + (r_idx + 1);
        p_elem_b = LM_MAT_GET_COL_PTR(p_mat_a, 1, r_idx)
                 + ((r_idx + 1) * nxt_r_osf_b);

        for (c_idx = r_idx + 1; c_idx < c_size; c_idx++) {

            tmp = p_elem_a[0];
            p_elem_a[0] = p_elem_b[0];
            p_elem_b[0] = tmp;

            LM_MAT_TO_NXT_COL(p_elem_a, 1, p_mat_a);
            LM_MAT_TO_NXT_ROW(p_elem_b, nxt_r_osf_b, p_mat_a);
        }

    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_scalar - Function to perform scalar operation on matrix A.
 *
 *      A := scalar .* A
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 * @param   [in]        scalar          Scalar.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_scalar(lm_mat_t *p_mat_a, lm_mat_elem_t scalar)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_elem_t *p_elem_a = NULL;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    for (r_idx = 0; r_idx < r_size; r_idx++) {

        p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

        for (c_idx = 0; c_idx < c_size; c_idx++) {

            /* A[i][j] *= scalar */
            p_elem_a[0] *= scalar;

            LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
        }

    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_scalar - Function to compute the bandwidth of matrix A.
 *
 * The lower bandwidth of a matrix is the number of subdiagonals
 * with nonzero entries.
 *
 * The upper bandwidth of a matrix is the number of superdiagonals
 * with nonzero entries.
 *
 * @param   [in]    *p_mat_a        Handle of matrix A.
 * @param   [out]   *p_lower        The lower bandwidth.
 * @param   [out]   *p_upper        The upper bandwidth.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_bandwidth(const lm_mat_t *p_mat_a,
                           lm_mat_dim_size_t *p_lower,
                           lm_mat_dim_size_t *p_upper)
{
    lm_rtn_t result;
    lm_mat_dim_size_t dim_idx;
    lm_mat_dim_size_t diag_idx;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_dim_size_t min_dim = LM_MIN(r_size, c_size);

    lm_mat_t mat_subm = {0};
    lm_mat_elem_t *p_elem_diag = NULL;
    lm_mat_elem_size_t nxt_r_osf_diag;

    if (p_lower == NULL || p_upper == NULL) {
        return LM_ERR_CODE(LM_ERR_NULL_PTR);
    }

    *p_lower = 0;
    *p_upper = 0;

    /* Check lower triangular */
    for (dim_idx = (r_size - 1); dim_idx > 0; dim_idx--) {

        /* The lower left sub-matrix */
        result = lm_shape_submatrix(p_mat_a, dim_idx, 0,
                                    (lm_mat_dim_size_t)(LM_MIN((r_size - dim_idx), min_dim)),
                                    (lm_mat_dim_size_t)(LM_MIN((r_size - dim_idx), min_dim)),
                                    &mat_subm);
        LM_RETURN_IF_ERR(result);

        /* The main diagonal of sub-matrix */
        result = lm_shape_diag(&mat_subm, 0, &mat_subm);
        LM_RETURN_IF_ERR(result);

        p_elem_diag = LM_MAT_GET_ELEM_PTR(&mat_subm);
        nxt_r_osf_diag = LM_MAT_GET_NXT_OFS(&mat_subm);

        /* Check the elements on the main diagonal */
        for (diag_idx = 0; diag_idx < LM_MAT_GET_R_SIZE(&mat_subm); diag_idx++) {

            /* Break this loop if any of element on the main diagonal is not equal to zero */
            if (LM_CHK_VAL_ALMOST_EQ_ZERO(p_elem_diag[0]) == false) {

                /* Setup the lower bandwidth */
                *p_lower = dim_idx;

                break;
            }

            LM_MAT_TO_NXT_ROW(p_elem_diag, nxt_r_osf_diag, &mat_subm);
        }

        if (*p_lower != 0) {
            break;
        }
    }

    /* Check upper triangular */
    for (dim_idx = (c_size - 1); dim_idx > 0; dim_idx--) {

        /* The upper right sub-matrix */
        result = lm_shape_submatrix(p_mat_a, 0, dim_idx,
                                    (lm_mat_dim_size_t)(LM_MIN((c_size - dim_idx), min_dim)),
                                    (lm_mat_dim_size_t)(LM_MIN((c_size - dim_idx), min_dim)),
                                    &mat_subm);
        LM_RETURN_IF_ERR(result);

        /* The main diagonal of sub-matrix */
        result = lm_shape_diag(&mat_subm, 0, &mat_subm);
        LM_RETURN_IF_ERR(result);

        p_elem_diag = LM_MAT_GET_ELEM_PTR(&mat_subm);
        nxt_r_osf_diag = LM_MAT_GET_NXT_OFS(&mat_subm);

        /* Check the elements on the main diagonal */
        for (diag_idx = 0; diag_idx < LM_MAT_GET_R_SIZE(&mat_subm); diag_idx++) {

            /* Break this loop if any of element on the main diagonal is not equal to zero */
            if (LM_CHK_VAL_ALMOST_EQ_ZERO(p_elem_diag[0]) == false) {

                /* Setup the upper bandwidth */
                *p_upper = dim_idx;

                break;
            }

            LM_MAT_TO_NXT_ROW(p_elem_diag, nxt_r_osf_diag, &mat_subm);
        }

        if (*p_upper != 0) {

            break;
        }
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_givens - Function to compute the Givens rotation factors.
 *
 * The computed sine theta and cosine theta satisfy:
 *
 * \f$\left[ \begin{matrix} c & s \\ -s & c \end{matrix} \right]
 * \left[ \begin{matrix} a \\ b \end{matrix} \right]
 * = \left[ \begin{matrix} r \\ 0 \end{matrix} \right]\f$
 *
 * @noop -       -     -   -     -   -
 * @noop |  c s  |     | a |     | r |
 * @noop |       |  X  |   |  =  |   |
 * @noop | -s c  |     | b |     | 0 |
 * @noop -       -     -   -     -   -
 *
 * @param   [in]    elem_a        Scalar a.
 * @param   [in]    elem_b        Scalar b.
 * @param   [out]   *p_sin        Sine theta factor.
 * @param   [out]   *p_cos        Cosine theta factor.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_givens(const lm_mat_elem_t elem_a,
                        const lm_mat_elem_t elem_b,
                        lm_mat_elem_t *p_sin,
                        lm_mat_elem_t *p_cos)
{
    lm_mat_elem_t tmp;

    if (p_sin == NULL || p_cos == NULL) {

        return LM_ERR_CODE(LM_ERR_NULL_PTR);
    }

    /*
     * Calculate the sin theta and cos theta for givens rotation
     *      -       -     -   -     -   -
     *      |  c s  |     | a |     | r |
     *      |       |  X  |   |  =  |   |
     *      | -s c  |     | b |     | 0 |
     *      -       -     -   -     -   -
     */

    if (elem_b == LM_MAT_ZERO_VAL) {
        *p_sin = LM_MAT_ZERO_VAL;
        *p_cos = LM_MAT_ONE_VAL;
    }
    else {
        if (fabs(elem_b) > fabs(elem_a)) {
            tmp = (elem_a) / elem_b;
            *p_sin = -(LM_MAT_ONE_VAL / sqrt((LM_MAT_ONE_VAL + tmp * tmp)));
            *p_cos = (*p_sin) * tmp;
        }
        else {
            tmp = (elem_b) / elem_a;
            *p_cos = (LM_MAT_ONE_VAL / sqrt((LM_MAT_ONE_VAL + tmp * tmp)));
            *p_sin = (*p_cos) * tmp;
        }
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_trace - Function to compute the trace of matrix A.
 *
 * @param   [in]    *p_mat_a        Handle of matrix A.
 * @param   [out]   *p_trace        The trace.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_trace(const lm_mat_t *p_mat_a,
                       lm_mat_elem_t *p_trace)
{
    lm_mat_dim_size_t dim_idx;

    const lm_mat_elem_t *p_elem_a = LM_MAT_GET_ELEM_PTR(p_mat_a);
    const lm_mat_dim_size_t dim = LM_MIN(LM_MAT_GET_R_SIZE(p_mat_a),
                                         LM_MAT_GET_C_SIZE(p_mat_a));
    const lm_mat_elem_size_t nxt_r_osf_a = (lm_mat_elem_size_t)(LM_MAT_GET_NXT_OFS(p_mat_a) + 1);

    if (p_trace == NULL) {

        return LM_ERR_CODE(LM_ERR_NULL_PTR);
    }

    p_trace[0] = LM_MAT_ZERO_VAL;

    for (dim_idx = 0; dim_idx < dim; dim_idx++) {

        p_trace[0] += p_elem_a[0];

        LM_MAT_TO_NXT_ROW(p_elem_a, nxt_r_osf_a, p_mat_a);
    }

    return LM_ERR_CODE(LM_SUCCESS);
}


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */

