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
 * @file    lm_chk.c
 * @brief   Lin matrix auxiliary check functions
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#include <stdint.h>
#include <stddef.h>
#include <math.h>

#include "lm_chk.h"
#include "lm_global.h"
#include "lm_mat.h"
#include "lm_err.h"
#include "lm_log.h"
#include "lm_shape.h"
#include "lm_oper_gemm.h"
#include "lm_oper.h"


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
 * lm_chk_machine_eps - Function to check the distance between given value and
 *                      the next largest value in the machine's floating point
 *                      system.
 *
 * @note
 *
 *      Machine epsilon gives an upper bound on the relative error due to
 *      rounding in floating point arithmetic.
 *
 *      Reference:
 *          - Machine_epsilon
 *            https://en.wikipedia.org/wiki/Machine_epsilon
 *          - How to determine machine epsilon
 *            https://en.wikipedia.org/wiki/Machine_epsilon
 *
 * @attention
 *
 *      The machine epsilon is platform and datatype dependent.
 *
 * @param   [in,out]    *p_value      Address of scalar variable.
 *
 *      On entry:
 *          The variable contains given value.
 *
 *      On exit:
 *          The distance between given value and the next largest value is
 *          stored in this variable, the distance value is always positive.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_chk_machine_eps(lm_mat_elem_t *p_value)
{
    lm_mat_elem_union_t tmp_val;

    if (p_value == NULL) {
        return LM_ERR_CODE(LM_ERR_NULL_PTR);
    }

    tmp_val.flt = *p_value;
    tmp_val.uint++;

    *p_value = tmp_val.flt - *p_value;

    if (*p_value < LM_MAT_ZERO_VAL) {
        *p_value = -(*p_value);
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_chk_elem_almost_equal - Function to check if two values are approximately equal.
 *
 * https://floating-point-gui.de/errors/comparison/
 * https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
 * https://bitbashing.io/comparing-floats.html
 *
 * @param   [in]        elem_a      value a.
 * @param   [in]        elem_b      value b.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_chk_elem_almost_equal(lm_mat_elem_t elem_a, lm_mat_elem_t elem_b)
{
    lm_mat_elem_union_t val_a;
    lm_mat_elem_union_t val_b;
    lm_mat_elem_t flt_diff;
    lm_mat_elem_ulp_diff_t ulp_diff;

    /* NaN */
    if (isnan(elem_a) || isnan(elem_b)) {
        return LM_ERR_CODE(LM_ERR_MAT_ELEM_VALUE_NOT_EQUAL);
    }

    /* If one's infinite and they're not equal */
    if (isinf(elem_a) || isinf(elem_b)) {
        return LM_ERR_CODE(LM_ERR_MAT_ELEM_VALUE_NOT_EQUAL);
    }

    if (elem_a == elem_b) {
        return LM_ERR_CODE(LM_SUCCESS);
    }

    /*
     * Handle the near-zero case.
     */
    flt_diff = (lm_mat_elem_t)fabs(elem_a - elem_b);

    if (flt_diff <= LM_MAT_EPSILON_MAX) {
        return LM_ERR_CODE(LM_SUCCESS);
    }

    /*
     * ULP difference
     */
    val_a.flt = elem_a;
    val_b.flt = elem_b;

    /* Different signs */
    if ((val_a.uint < LM_MAT_ZERO_VAL) != (val_b.uint < LM_MAT_ZERO_VAL)) {
        return LM_ERR_CODE(LM_ERR_MAT_ELEM_VALUE_NOT_EQUAL);
    }

    ulp_diff = (lm_mat_elem_ulp_diff_t)(val_a.uint - val_b.uint);
    if (ulp_diff < 0) {
        ulp_diff = -ulp_diff;
    }

    if (ulp_diff > LM_MAT_ULP_MAX) {
        return LM_ERR_CODE(LM_ERR_MAT_ELEM_VALUE_NOT_EQUAL);
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_chk_mat_almost_equal - Function to check if two matrices are approximately equal.
 *
 * https://floating-point-gui.de/errors/comparison/
 * https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
 * https://bitbashing.io/comparing-floats.html
 *
 * @param   [in]        *p_mat_a    Handle of matrix A.
 * @param   [in]        *p_mat_b    Handle of matrix B.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_chk_mat_almost_equal(const lm_mat_t *p_mat_a, const lm_mat_t *p_mat_b)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    const lm_mat_elem_t *p_elem_a = NULL;
    const lm_mat_elem_t *p_elem_b = NULL;
    const lm_mat_dim_size_t r_size_a = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size_a = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_dim_size_t r_size_b = LM_MAT_GET_R_SIZE(p_mat_b);
    const lm_mat_dim_size_t c_size_b = LM_MAT_GET_C_SIZE(p_mat_b);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_b = LM_MAT_GET_NXT_OFS(p_mat_b);

    if (r_size_a != r_size_b || c_size_a != c_size_b ) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    for (r_idx = 0; r_idx < r_size_a; r_idx++) {

        p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);
        p_elem_b = LM_MAT_GET_ROW_PTR(p_mat_b, nxt_r_osf_b, r_idx);

        for (c_idx = 0; c_idx < c_size_a; c_idx++) {

            if (LM_IS_ERR(lm_chk_elem_almost_equal(p_elem_a[0], p_elem_b[0])) == true) {
                return LM_ERR_CODE(LM_ERR_MAT_ELEM_VALUE_NOT_EQUAL);
            }

            LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
            LM_MAT_TO_NXT_ELEM(p_elem_b, p_mat_b);
        }
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_chk_square_mat - Function to check if matrix is square.
 *
 * @param   [in]        *p_mat_a    Handle of matrix A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_chk_square_mat(const lm_mat_t *p_mat_a)
{
    if (p_mat_a->elem.dim.r != p_mat_a->elem.dim.c) {

        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_SQUARE);
    }

    if (p_mat_a->elem.dim.r == 0 || p_mat_a->elem.dim.c == 0) {

        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_SQUARE);
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_chk_triu_mat - Function to check if matrix is upper triangular.
 *
 * @param   [in]        *p_mat_a    Handle of matrix A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_chk_triu_mat(const lm_mat_t *p_mat_a)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    const lm_mat_elem_t *p_elem_a = NULL;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    if (r_size == 0 || c_size == 0) {

        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_UPPER_TRIANGULAR);
    }

    for (r_idx = 1; r_idx < r_size; r_idx++) {

        p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

        if (r_idx < c_size) {

            for (c_idx = 0; c_idx < r_idx; c_idx++) {

                if (LM_CHK_VAL_ALMOST_EQ_ZERO(p_elem_a[0]) == false) {

                    return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_UPPER_TRIANGULAR);
                }

                LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
            }
        }
        else {
            for (c_idx = 0; c_idx < c_size; c_idx++) {

                if (LM_CHK_VAL_ALMOST_EQ_ZERO(p_elem_a[0]) == false) {

                    return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_UPPER_TRIANGULAR);
                }

                LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
            }
        }

    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_chk_tril_mat - Function to check if matrix is lower triangular.
 *
 * @param   [in]        *p_mat_a    Handle of matrix A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_chk_tril_mat(const lm_mat_t *p_mat_a)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    const lm_mat_elem_t *p_elem_a = NULL;
    lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    if (r_size == 0 || c_size == 0) {

        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_LOWER_TRIANGULAR);
    }

    r_size = LM_MIN(r_size, c_size);

    for (r_idx = 0; r_idx < r_size; r_idx++) {

        /* Point to A[i][i+1] */
        p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx)
                 + (r_idx + 1);

        for (c_idx = r_idx + 1; c_idx < c_size; c_idx++) {

            if (LM_CHK_VAL_ALMOST_EQ_ZERO(p_elem_a[0]) == false) {

                return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_LOWER_TRIANGULAR);
            }

            LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
        }
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_chk_diagonal_mat - Function to check if matrix is diagonal.
 *
 * @param   [in]        *p_mat_a    Handle of matrix A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_chk_diagonal_mat(const lm_mat_t *p_mat_a)
{
    /*
     * If the input matrix conforms to both the upper triangular matrix
     * and the lower triangular matrix, it means that it is a diagonal matrix.
     */
    if (LM_IS_ERR(lm_chk_triu_mat(p_mat_a)) == false
        && LM_IS_ERR(lm_chk_tril_mat(p_mat_a)) == false) {

        return LM_ERR_CODE(LM_SUCCESS);
    }
    else {

        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_DIAGONAL);
    }

}

/**
 * lm_chk_identity_mat - Function to check if matrix is an identity matrix.
 *
 * @param   [in]        *p_mat_a    Handle of matrix A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_chk_identity_mat(const lm_mat_t *p_mat_a)
{
    lm_mat_dim_size_t r_idx;
    const lm_mat_elem_t *p_elem_a = LM_MAT_GET_ELEM_PTR(p_mat_a);
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = (lm_mat_elem_size_t)(LM_MAT_GET_NXT_OFS(p_mat_a) + 1);

    /* Given matrix should be a square matrix */
    if (r_size != c_size) {
        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_IDENTITY);
    }

    if (r_size == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_IDENTITY);
    }

    if (LM_IS_ERR(lm_chk_diagonal_mat(p_mat_a)) == true) {
        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_IDENTITY);
    }

    for (r_idx = 0; r_idx < r_size; r_idx++) {

        if (LM_CHK_VAL_ALMOST_EQ_ONE(p_elem_a[0]) == false) {
            return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_IDENTITY);
        }

        p_elem_a += nxt_r_osf_a;

    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_chk_orthogonal_mat - Function to check if matrix is orthogonal.
 *
 * @param   [in]        *p_mat_a    Handle of matrix A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_chk_orthogonal_mat(const lm_mat_t *p_mat_q)
{
    lm_rtn_t result;
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    const lm_mat_dim_size_t r_size_q = LM_MAT_GET_R_SIZE(p_mat_q);
    const lm_mat_dim_size_t c_size_q = LM_MAT_GET_C_SIZE(p_mat_q);

    lm_mat_t mat_row_vec_shaped = {0};
    lm_mat_t mat_col_vec_shaped = {0};
    lm_mat_t mat_dot = {0};
    lm_mat_elem_t q_elem_dot;

    /*
     * A orthogonal matrix must be a square matrix.
     */
    if (r_size_q != c_size_q) {
        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_ORTHOGONAL);
    }

    if (r_size_q == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_ORTHOGONAL);
    }

    result = lm_mat_set(&mat_dot, 1, 1, &q_elem_dot,
                        (sizeof(q_elem_dot) / sizeof(lm_mat_elem_t)));
    LM_RETURN_IF_ERR(result);

    /*
     * A orthogonal matrix multiple by its transpose
     * should equal to a Identity matrix.
     *
     * Q' = inv(Q), Q * Q' = Q * inv(Q) = I
     */

    for (r_idx = 0; r_idx < r_size_q; r_idx++) {

        result = lm_shape_row_vect(p_mat_q, r_idx, &mat_row_vec_shaped);
        LM_RETURN_IF_ERR(result);

        for (c_idx = 0; c_idx < c_size_q; c_idx++) {

            result = lm_shape_row_vect(p_mat_q, c_idx, &mat_col_vec_shaped);
            LM_RETURN_IF_ERR(result);

            /* v * v' */
            result = lm_oper_gemm(false, true,
                                  LM_MAT_ONE_VAL,
                                  &mat_row_vec_shaped,
                                  &mat_col_vec_shaped,
                                  LM_MAT_ZERO_VAL,
                                  &mat_dot);
            LM_RETURN_IF_ERR(result);

            /* The value of diagonal elements should equal to one */
            if (r_idx == c_idx) {

                if (LM_CHK_VAL_ALMOST_EQ_ONE(q_elem_dot) == false) {
                    return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_ORTHOGONAL);
                }
            }
            /* Others should equal to zero */
            else {
                if (LM_CHK_VAL_ALMOST_EQ_ZERO(q_elem_dot) == false) {
                    return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_ORTHOGONAL);
                }
            }
        }

    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_chk_banded_mat - Function to check if matrix is within specific bandwidth.
 *
 * https://en.wikipedia.org/wiki/Band_matrix
 *
 * @param   [in]        *p_mat_a    Handle of matrix A.
 * @param   [in]        lower_bw    Lower bandwidth (1 ~ N).
 * @param   [in]        upper_bw    Upper bandwidth (1 ~ N).
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_chk_banded_mat(const lm_mat_t *p_mat_a,
                           lm_mat_dim_size_t lower_bw,
                           lm_mat_dim_size_t upper_bw)
{
    lm_rtn_t result;
    lm_mat_dim_size_t low_bandwidth;
    lm_mat_dim_size_t up_bandwidth;

    result = lm_oper_bandwidth(p_mat_a, &low_bandwidth, &up_bandwidth);

    if (LM_IS_ERR(result) == false) {

        /*
         * Check whether matrix A is within the specified
         * lower and upper bandwidth.
         */
        if (low_bandwidth <= lower_bw && up_bandwidth <= upper_bw) {
            result = LM_ERR_CODE(LM_SUCCESS);
        }
        else {
            result = LM_ERR_CODE(LM_ERR_MAT_IS_NOT_BANDED);
        }
    }

    return result;
}


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */



