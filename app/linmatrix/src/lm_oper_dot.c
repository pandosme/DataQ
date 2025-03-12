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
 * @file    lm_oper_dot.c
 * @brief   Lin matrix arithmetic dot product functions
 * @note
 *
 * Reference:
 *      - https://zhuanlan.zhihu.com/p/66958390
 *      - https://www.youtube.com/watch?v=cKkF690TuG8
 *
 * Abbreviation:
 *
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#include "lm_oper_dot.h"
#include "lm_mat.h"
#include "lm_err.h"


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
 * lm_oper_dot - Function to perform dot product computation
 *               (Optimized with block computing).
 *
 * @note
 *      OUT := A * B
 *
 *      where OUT is a M by N matrix,
 *              A is a M by L matrix,
 *              B is a L by N matrix.
 *
 * @attention
 *      This function should no longer be used.
 *      To compute the matrix multiplication,
 *      please using the GEMM function.
 *
 * @param   [in]        *p_mat_a        Handle of matrix A.
 * @param   [in]        *p_mat_b        Handle of matrix B.
 * @param   [in,out]    *p_mat_out      Handle of matrix OUT.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_dot(const lm_mat_t *p_mat_a,
                     const lm_mat_t *p_mat_b,
                     lm_mat_t *p_mat_out)
{
    if (p_mat_a->elem.dim.r < 4) {
        if (p_mat_b->elem.dim.c < 4) {
            return lm_oper_dot_gemm11(p_mat_a, p_mat_b, p_mat_out);
        }
        else {
            return lm_oper_dot_gemm14(p_mat_a, p_mat_b, p_mat_out);
        }
    }
    else {
        if (p_mat_b->elem.dim.c < 4) {
            return lm_oper_dot_gemm41(p_mat_a, p_mat_b, p_mat_out);
        }
        else {
            return lm_oper_dot_gemm44(p_mat_a, p_mat_b, p_mat_out);
        }
    }
}

/**
 * lm_oper_dot_gemm11 - Function to perform dot product computation
 *                      (Optimized with 1 by 1 block computing).
 *
 * @note
 *      OUT := A * B
 *
 *      where OUT is a M by N matrix,
 *              A is a M by L matrix,
 *              B is a L by N matrix.
 *
 * @attention
 *      This function should no longer be used.
 *      To compute the matrix multiplication,
 *      please using the GEMM function.
 *
 * @param   [in]        *p_mat_a        Handle of matrix A.
 * @param   [in]        *p_mat_b        Handle of matrix B.
 * @param   [in,out]    *p_mat_out      Handle of matrix OUT.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_dot_gemm11(const lm_mat_t *p_mat_a,
                            const lm_mat_t *p_mat_b,
                            lm_mat_t *p_mat_out)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_elem_size_t elem_idx;
    lm_mat_elem_t elem_sum_val;
    const lm_mat_elem_t *p_elem_a = NULL;
    const lm_mat_elem_t *p_elem_b = NULL;
    lm_mat_elem_t *p_elem_out = NULL;
    const lm_mat_dim_size_t r_size_a = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size_a = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_dim_size_t r_size_b = LM_MAT_GET_R_SIZE(p_mat_b);
    const lm_mat_dim_size_t c_size_b = LM_MAT_GET_C_SIZE(p_mat_b);
    const lm_mat_dim_size_t r_size_out = LM_MAT_GET_R_SIZE(p_mat_out);
    const lm_mat_dim_size_t c_size_out = LM_MAT_GET_C_SIZE(p_mat_out);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_b = LM_MAT_GET_NXT_OFS(p_mat_b);
    const lm_mat_elem_size_t nxt_r_osf_out = LM_MAT_GET_NXT_OFS(p_mat_out);
    lm_mat_dim_size_t elem_size_tmp;

    /*
     * The number of columns in the first matrix must be
     * equal to the number of rows in the second matrix.
     */
    if (c_size_a != r_size_b) {
        return LM_ERR_CODE(LM_ERR_MAT_DOT_PRODUCT_DIM_MISMATCHED);
    }

    /*
     * The dimension of output matrix must equal to the dimension
     * of [number of columns of A] * [number of rows of B]
     */
    if (r_size_out != r_size_a || c_size_out != c_size_b) {

        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    elem_size_tmp = (lm_mat_dim_size_t)(c_size_a - (c_size_a % 4));

    /* Compute the row vector * column vector sequentially */
    for (r_idx = 0; r_idx < r_size_out; r_idx++) {

        p_elem_out = LM_MAT_GET_ROW_PTR(p_mat_out, nxt_r_osf_out, r_idx);

        for (c_idx = 0; c_idx < c_size_out; c_idx++) {

            /* To store a11 * b11 + .... a_mn * b_nm */
            elem_sum_val = 0;

            /* Point to row 0, 1, ... N of Matrix A */
            p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

            /* Point to column 0, 1, ... N of Matrix B */
            p_elem_b = LM_MAT_GET_COL_PTR(p_mat_b, 1, c_idx);

            for (elem_idx = 0; elem_idx < elem_size_tmp; elem_idx += 4) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val += p_elem_a[0] * p_elem_b[0 * nxt_r_osf_b]
                              + p_elem_a[1] * p_elem_b[1 * nxt_r_osf_b]
                              + p_elem_a[2] * p_elem_b[2 * nxt_r_osf_b]
                              + p_elem_a[3] * p_elem_b[3 * nxt_r_osf_b];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_N_ELEM(p_elem_a, 4, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, (nxt_r_osf_b * 4), p_mat_b);
            }

            for (elem_idx = elem_size_tmp; elem_idx < c_size_a; elem_idx++) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val += p_elem_a[0] * p_elem_b[0];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, nxt_r_osf_b, p_mat_b);
            }

            /* Store the sum of products to product matrix */
            p_elem_out[0] = elem_sum_val;

            LM_MAT_TO_NXT_ELEM(p_elem_out, p_mat_out);

        }

    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_dot_gemm14 - Function to perform dot product computation
 *                      (Optimized with 1 by 4 block computing).
 *
 * @note
 *      OUT := A * B
 *
 *      where OUT is a M by N matrix,
 *              A is a M by L matrix,
 *              B is a L by N matrix.
 *
 * @attention
 *      This function should no longer be used.
 *      To compute the matrix multiplication,
 *      please using the GEMM function.
 *
 * @param   [in]        *p_mat_a        Handle of matrix A.
 * @param   [in]        *p_mat_b        Handle of matrix B.
 * @param   [in,out]    *p_mat_out      Handle of matrix OUT.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_dot_gemm14(const lm_mat_t *p_mat_a,
                            const lm_mat_t *p_mat_b,
                            lm_mat_t *p_mat_out)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_elem_size_t elem_idx;
    lm_mat_elem_t elem_sum_val[4];
    const lm_mat_elem_t *p_elem_a = NULL;
    const lm_mat_elem_t *p_elem_b = NULL;
    lm_mat_elem_t *p_elem_out = NULL;
    const lm_mat_dim_size_t r_size_a = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size_a = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_dim_size_t r_size_b = LM_MAT_GET_R_SIZE(p_mat_b);
    const lm_mat_dim_size_t c_size_b = LM_MAT_GET_C_SIZE(p_mat_b);
    const lm_mat_dim_size_t r_size_out = LM_MAT_GET_R_SIZE(p_mat_out);
    const lm_mat_dim_size_t c_size_out = LM_MAT_GET_C_SIZE(p_mat_out);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_b = LM_MAT_GET_NXT_OFS(p_mat_b);
    const lm_mat_elem_size_t nxt_r_osf_out = LM_MAT_GET_NXT_OFS(p_mat_out);
    lm_mat_dim_size_t c_size_tmp;
    lm_mat_dim_size_t elem_size_tmp;

    /*
     * The number of columns in the first matrix must be
     * equal to the number of rows in the second matrix.
     */
    if (c_size_a != r_size_b) {

        return LM_ERR_CODE(LM_ERR_MAT_DOT_PRODUCT_DIM_MISMATCHED);
    }

    /*
     * The dimension of output matrix must equal to the dimension
     * of [number of columns of A] * [number of rows of B]
     */
    if (r_size_out != r_size_a || c_size_out != c_size_b) {

        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    c_size_tmp = (lm_mat_dim_size_t)(c_size_out - (c_size_out % 4));
    elem_size_tmp = (lm_mat_dim_size_t)(c_size_a - (c_size_a % 4));

    /* Compute the row vector * column vector sequentially */
    for (r_idx = 0; r_idx < r_size_out; r_idx++) {

        for (c_idx = 0; c_idx < c_size_tmp; c_idx += 4) {

            /* To store a11 * b11 + .... a_mn * b_nm */
            elem_sum_val[0] = LM_MAT_ZERO_VAL;
            elem_sum_val[1] = LM_MAT_ZERO_VAL;
            elem_sum_val[2] = LM_MAT_ZERO_VAL;
            elem_sum_val[3] = LM_MAT_ZERO_VAL;

            /* Point to row 0, 1, ... N of Matrix A */
            p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

            /* Point to column 0, 1, ... N of Matrix B */
            p_elem_b = LM_MAT_GET_COL_PTR(p_mat_b, 1, c_idx);

            p_elem_out = LM_MAT_GET_ROW_PTR(p_mat_out, nxt_r_osf_out, r_idx)
                       + c_idx;

            for (elem_idx = 0; elem_idx < elem_size_tmp; elem_idx += 4) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0] += p_elem_a[0] * p_elem_b[0 * nxt_r_osf_b + 0]
                                 + p_elem_a[1] * p_elem_b[1 * nxt_r_osf_b + 0]
                                 + p_elem_a[2] * p_elem_b[2 * nxt_r_osf_b + 0]
                                 + p_elem_a[3] * p_elem_b[3 * nxt_r_osf_b + 0];

                elem_sum_val[1] += p_elem_a[0] * p_elem_b[0 * nxt_r_osf_b + 1]
                                 + p_elem_a[1] * p_elem_b[1 * nxt_r_osf_b + 1]
                                 + p_elem_a[2] * p_elem_b[2 * nxt_r_osf_b + 1]
                                 + p_elem_a[3] * p_elem_b[3 * nxt_r_osf_b + 1];

                elem_sum_val[2] += p_elem_a[0] * p_elem_b[0 * nxt_r_osf_b + 2]
                                 + p_elem_a[1] * p_elem_b[1 * nxt_r_osf_b + 2]
                                 + p_elem_a[2] * p_elem_b[2 * nxt_r_osf_b + 2]
                                 + p_elem_a[3] * p_elem_b[3 * nxt_r_osf_b + 2];

                elem_sum_val[3] += p_elem_a[0] * p_elem_b[0 * nxt_r_osf_b + 3]
                                 + p_elem_a[1] * p_elem_b[1 * nxt_r_osf_b + 3]
                                 + p_elem_a[2] * p_elem_b[2 * nxt_r_osf_b + 3]
                                 + p_elem_a[3] * p_elem_b[3 * nxt_r_osf_b + 3];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_N_ELEM(p_elem_a, 4, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, (nxt_r_osf_b * 4), p_mat_b);
            }

            for (elem_idx = elem_size_tmp; elem_idx < c_size_a; elem_idx++) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0] += p_elem_a[0] * p_elem_b[0];
                elem_sum_val[1] += p_elem_a[0] * p_elem_b[1];
                elem_sum_val[2] += p_elem_a[0] * p_elem_b[2];
                elem_sum_val[3] += p_elem_a[0] * p_elem_b[3];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, nxt_r_osf_b, p_mat_b);
            }

            /* Store the sum of products to product matrix */
            p_elem_out[0] = elem_sum_val[0];
            p_elem_out[1] = elem_sum_val[1];
            p_elem_out[2] = elem_sum_val[2];
            p_elem_out[3] = elem_sum_val[3];

        }

        for (c_idx = c_size_tmp; c_idx < c_size_out; c_idx++) {

            /* To store a11 * b11 + .... a_mn * b_nm */
            elem_sum_val[0] = LM_MAT_ZERO_VAL;

            /* Point to row 0, 1, ... N of Matrix A */
            p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

            /* Point to column 0, 1, ... N of Matrix B */
            p_elem_b = LM_MAT_GET_COL_PTR(p_mat_b, 1, c_idx);

            p_elem_out = LM_MAT_GET_ROW_PTR(p_mat_out, nxt_r_osf_out, r_idx)
                       + c_idx;

            for (elem_idx = 0; elem_idx < elem_size_tmp; elem_idx += 4) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0] += p_elem_a[0] * p_elem_b[0 * nxt_r_osf_b]
                                 + p_elem_a[1] * p_elem_b[1 * nxt_r_osf_b]
                                 + p_elem_a[2] * p_elem_b[2 * nxt_r_osf_b]
                                 + p_elem_a[3] * p_elem_b[3 * nxt_r_osf_b];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_N_ELEM(p_elem_a, 4, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, (nxt_r_osf_b * 4), p_mat_b);
            }

            for (elem_idx = elem_size_tmp; elem_idx < c_size_a; elem_idx++) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0] += p_elem_a[0] * p_elem_b[0];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, nxt_r_osf_b, p_mat_b);
            }

            /* Store the sum of products to product matrix */
            p_elem_out[0] = elem_sum_val[0];

        }
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_dot_gemm41 - Function to perform dot product computation
 *                      (Optimized with 4 by 1 block computing).
 *
 * @note
 *      OUT := A * B
 *
 *      where OUT is a M by N matrix,
 *              A is a M by L matrix,
 *              B is a L by N matrix.
 *
 * @attention
 *      This function should no longer be used.
 *      To compute the matrix multiplication,
 *      please using the GEMM function.
 *
 * @param   [in]        *p_mat_a        Handle of matrix A.
 * @param   [in]        *p_mat_b        Handle of matrix B.
 * @param   [in,out]    *p_mat_out      Handle of matrix OUT.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_dot_gemm41(const lm_mat_t *p_mat_a,
                            const lm_mat_t *p_mat_b,
                            lm_mat_t *p_mat_out)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_elem_size_t elem_idx;
    lm_mat_elem_t elem_sum_val[4];
    const lm_mat_elem_t *p_elem_a = NULL;
    const lm_mat_elem_t *p_elem_b = NULL;
    lm_mat_elem_t *p_elem_out = NULL;
    const lm_mat_dim_size_t r_size_a = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size_a = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_dim_size_t r_size_b = LM_MAT_GET_R_SIZE(p_mat_b);
    const lm_mat_dim_size_t c_size_b = LM_MAT_GET_C_SIZE(p_mat_b);
    const lm_mat_dim_size_t r_size_out = LM_MAT_GET_R_SIZE(p_mat_out);
    const lm_mat_dim_size_t c_size_out = LM_MAT_GET_C_SIZE(p_mat_out);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_b = LM_MAT_GET_NXT_OFS(p_mat_b);
    const lm_mat_elem_size_t nxt_r_osf_out = LM_MAT_GET_NXT_OFS(p_mat_out);
    lm_mat_dim_size_t r_size_tmp;
    lm_mat_dim_size_t elem_size_tmp;

    /*
     * The number of columns in the first matrix must be
     * equal to the number of rows in the second matrix.
     */
    if (c_size_a != r_size_b) {

        return LM_ERR_CODE(LM_ERR_MAT_DOT_PRODUCT_DIM_MISMATCHED);
    }

    /*
     * The dimension of output matrix must equal to the dimension
     * of [number of columns of A] * [number of rows of B]
     */
    if (r_size_out != r_size_a || c_size_out != c_size_b) {

        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    r_size_tmp = (lm_mat_dim_size_t)(r_size_out - (r_size_out % 4));
    elem_size_tmp = (lm_mat_dim_size_t)(c_size_a - (c_size_a % 4));

    /* Compute the row vector * column vector sequentially */
    for (c_idx = 0; c_idx < c_size_out; c_idx++) {

        for (r_idx = 0; r_idx < r_size_tmp; r_idx += 4) {

            /* To store a11 * b11 + .... a_mn * b_nm */
            elem_sum_val[0] = LM_MAT_ZERO_VAL;
            elem_sum_val[1] = LM_MAT_ZERO_VAL;
            elem_sum_val[2] = LM_MAT_ZERO_VAL;
            elem_sum_val[3] = LM_MAT_ZERO_VAL;

            /* Point to row 0, 1, ... N of Matrix A */
            p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

            /* Point to column 0, 1, ... N of Matrix B */
            p_elem_b = LM_MAT_GET_COL_PTR(p_mat_b, 1, c_idx);

            p_elem_out = LM_MAT_GET_ROW_PTR(p_mat_out, nxt_r_osf_out, r_idx)
                       + c_idx;

            for (elem_idx = 0; elem_idx < elem_size_tmp; elem_idx += 4) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0] += p_elem_a[nxt_r_osf_a * 0 + 0] * p_elem_b[0 * nxt_r_osf_b]
                                 + p_elem_a[nxt_r_osf_a * 0 + 1] * p_elem_b[1 * nxt_r_osf_b]
                                 + p_elem_a[nxt_r_osf_a * 0 + 2] * p_elem_b[2 * nxt_r_osf_b]
                                 + p_elem_a[nxt_r_osf_a * 0 + 3] * p_elem_b[3 * nxt_r_osf_b];

                elem_sum_val[1] += p_elem_a[nxt_r_osf_a * 1 + 0] * p_elem_b[0 * nxt_r_osf_b]
                                 + p_elem_a[nxt_r_osf_a * 1 + 1] * p_elem_b[1 * nxt_r_osf_b]
                                 + p_elem_a[nxt_r_osf_a * 1 + 2] * p_elem_b[2 * nxt_r_osf_b]
                                 + p_elem_a[nxt_r_osf_a * 1 + 3] * p_elem_b[3 * nxt_r_osf_b];

                elem_sum_val[2] += p_elem_a[nxt_r_osf_a * 2 + 0] * p_elem_b[0 * nxt_r_osf_b]
                                 + p_elem_a[nxt_r_osf_a * 2 + 1] * p_elem_b[1 * nxt_r_osf_b]
                                 + p_elem_a[nxt_r_osf_a * 2 + 2] * p_elem_b[2 * nxt_r_osf_b]
                                 + p_elem_a[nxt_r_osf_a * 2 + 3] * p_elem_b[3 * nxt_r_osf_b];

                elem_sum_val[3] += p_elem_a[nxt_r_osf_a * 3 + 0] * p_elem_b[0 * nxt_r_osf_b]
                                 + p_elem_a[nxt_r_osf_a * 3 + 1] * p_elem_b[1 * nxt_r_osf_b]
                                 + p_elem_a[nxt_r_osf_a * 3 + 2] * p_elem_b[2 * nxt_r_osf_b]
                                 + p_elem_a[nxt_r_osf_a * 3 + 3] * p_elem_b[3 * nxt_r_osf_b];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_N_ELEM(p_elem_a, 4, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, (nxt_r_osf_b * 4), p_mat_b);
            }

            for (elem_idx = elem_size_tmp; elem_idx < c_size_a; elem_idx++) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0] += p_elem_a[nxt_r_osf_a * 0] * p_elem_b[0];
                elem_sum_val[1] += p_elem_a[nxt_r_osf_a * 1] * p_elem_b[0];
                elem_sum_val[2] += p_elem_a[nxt_r_osf_a * 2] * p_elem_b[0];
                elem_sum_val[3] += p_elem_a[nxt_r_osf_a * 3] * p_elem_b[0];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, nxt_r_osf_b, p_mat_b);
            }

            /* Store the sum of products to product matrix */
            p_elem_out[nxt_r_osf_out * 0] = elem_sum_val[0];
            p_elem_out[nxt_r_osf_out * 1] = elem_sum_val[1];
            p_elem_out[nxt_r_osf_out * 2] = elem_sum_val[2];
            p_elem_out[nxt_r_osf_out * 3] = elem_sum_val[3];

        }

        for (r_idx = r_size_tmp; r_idx < r_size_out; r_idx++) {

            /* To store a11 * b11 + .... a_mn * b_nm */
            elem_sum_val[0] = LM_MAT_ZERO_VAL;

            /* Point to row 0, 1, ... N of Matrix A */
            p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

            /* Point to column 0, 1, ... N of Matrix B */
            p_elem_b = LM_MAT_GET_COL_PTR(p_mat_b, 1, c_idx);

            p_elem_out = LM_MAT_GET_ROW_PTR(p_mat_out, nxt_r_osf_out, r_idx)
                       + c_idx;

            for (elem_idx = 0; elem_idx < elem_size_tmp; elem_idx += 4) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0] += p_elem_a[0] * p_elem_b[0 * nxt_r_osf_b]
                                 + p_elem_a[1] * p_elem_b[1 * nxt_r_osf_b]
                                 + p_elem_a[2] * p_elem_b[2 * nxt_r_osf_b]
                                 + p_elem_a[3] * p_elem_b[3 * nxt_r_osf_b];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_N_ELEM(p_elem_a, 4, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, (nxt_r_osf_b * 4), p_mat_b);
            }

            for (elem_idx = elem_size_tmp; elem_idx < c_size_a; elem_idx++) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0] += p_elem_a[0] * p_elem_b[0];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, nxt_r_osf_b, p_mat_b);
            }

            /* Store the sum of products to product matrix */
            p_elem_out[0] = elem_sum_val[0];

        }

    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_dot_gemm44 - Function to perform dot product computation
 *                      (Optimized with 4 by 4 block computing).
 *
 * @note
 *      OUT := A * B
 *
 *      where OUT is a M by N matrix,
 *              A is a M by L matrix,
 *              B is a L by N matrix.
 *
 * @attention
 *      This function should no longer be used.
 *      To compute the matrix multiplication,
 *      please using the GEMM function.
 *
 * @param   [in]        *p_mat_a        Handle of matrix A.
 * @param   [in]        *p_mat_b        Handle of matrix B.
 * @param   [in,out]    *p_mat_out      Handle of matrix OUT.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_dot_gemm44(const lm_mat_t *p_mat_a,
                            const lm_mat_t *p_mat_b,
                            lm_mat_t *p_mat_out)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_elem_size_t elem_idx;
    lm_mat_elem_t elem_sum_val[4][4];
    const lm_mat_elem_t *p_elem_a = NULL;
    const lm_mat_elem_t *p_elem_b = NULL;
    lm_mat_elem_t *p_elem_out = NULL;
    const lm_mat_dim_size_t r_size_a = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size_a = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_dim_size_t r_size_b = LM_MAT_GET_R_SIZE(p_mat_b);
    const lm_mat_dim_size_t c_size_b = LM_MAT_GET_C_SIZE(p_mat_b);
    const lm_mat_dim_size_t r_size_out = LM_MAT_GET_R_SIZE(p_mat_out);
    const lm_mat_dim_size_t c_size_out = LM_MAT_GET_C_SIZE(p_mat_out);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_b = LM_MAT_GET_NXT_OFS(p_mat_b);
    const lm_mat_elem_size_t nxt_r_osf_out = LM_MAT_GET_NXT_OFS(p_mat_out);
    lm_mat_dim_size_t c_size_tmp;
    lm_mat_dim_size_t r_size_tmp;
    lm_mat_dim_size_t elem_size_tmp;

    /*
     * The number of columns in the first matrix must be
     * equal to the number of rows in the second matrix.
     */
    if (c_size_a != r_size_b) {

        return LM_ERR_CODE(LM_ERR_MAT_DOT_PRODUCT_DIM_MISMATCHED);
    }

    /*
     * The dimension of output matrix must equal to the dimension
     * of [number of columns of A] * [number of rows of B]
     */
    if (r_size_out != r_size_a || c_size_out != c_size_b) {

        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    r_size_tmp = (lm_mat_dim_size_t)(r_size_out - (r_size_out % 4));
    c_size_tmp = (lm_mat_dim_size_t)(c_size_out - (c_size_out % 4));
    elem_size_tmp = (lm_mat_dim_size_t)(c_size_a - (c_size_a % 4));

    /*
     * Resolve the 4 by 4 block in the output matrix sequentially
     */
    for (r_idx = 0; r_idx < r_size_tmp; r_idx += 4) {

        for (c_idx = 0; c_idx < c_size_tmp; c_idx += 4) {

            /* To store a11 * b11 + .... a_mn * b_nm */
            elem_sum_val[0][0] = LM_MAT_ZERO_VAL;
            elem_sum_val[0][1] = LM_MAT_ZERO_VAL;
            elem_sum_val[0][2] = LM_MAT_ZERO_VAL;
            elem_sum_val[0][3] = LM_MAT_ZERO_VAL;

            elem_sum_val[1][0] = LM_MAT_ZERO_VAL;
            elem_sum_val[1][1] = LM_MAT_ZERO_VAL;
            elem_sum_val[1][2] = LM_MAT_ZERO_VAL;
            elem_sum_val[1][3] = LM_MAT_ZERO_VAL;

            elem_sum_val[2][0] = LM_MAT_ZERO_VAL;
            elem_sum_val[2][1] = LM_MAT_ZERO_VAL;
            elem_sum_val[2][2] = LM_MAT_ZERO_VAL;
            elem_sum_val[2][3] = LM_MAT_ZERO_VAL;

            elem_sum_val[3][0] = LM_MAT_ZERO_VAL;
            elem_sum_val[3][1] = LM_MAT_ZERO_VAL;
            elem_sum_val[3][2] = LM_MAT_ZERO_VAL;
            elem_sum_val[3][3] = LM_MAT_ZERO_VAL;

            /* Point to row 0, 1, ... N of Matrix A */
            p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

            /* Point to column 0, 1, ... N of Matrix B */
            p_elem_b = LM_MAT_GET_COL_PTR(p_mat_b, 1, c_idx);

            p_elem_out = LM_MAT_GET_ROW_PTR(p_mat_out, nxt_r_osf_out, r_idx)
                       + c_idx;

            for (elem_idx = 0; elem_idx < elem_size_tmp; elem_idx += 4) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0][0] += p_elem_a[0 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 0]
                                    + p_elem_a[0 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 0]
                                    + p_elem_a[0 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 0]
                                    + p_elem_a[0 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 0];
                elem_sum_val[0][1] += p_elem_a[0 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 1]
                                    + p_elem_a[0 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 1]
                                    + p_elem_a[0 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 1]
                                    + p_elem_a[0 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 1];
                elem_sum_val[0][2] += p_elem_a[0 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 2]
                                    + p_elem_a[0 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 2]
                                    + p_elem_a[0 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 2]
                                    + p_elem_a[0 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 2];
                elem_sum_val[0][3] += p_elem_a[0 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 3]
                                    + p_elem_a[0 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 3]
                                    + p_elem_a[0 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 3]
                                    + p_elem_a[0 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 3];

                elem_sum_val[1][0] += p_elem_a[1 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 0]
                                    + p_elem_a[1 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 0]
                                    + p_elem_a[1 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 0]
                                    + p_elem_a[1 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 0];
                elem_sum_val[1][1] += p_elem_a[1 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 1]
                                    + p_elem_a[1 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 1]
                                    + p_elem_a[1 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 1]
                                    + p_elem_a[1 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 1];
                elem_sum_val[1][2] += p_elem_a[1 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 2]
                                    + p_elem_a[1 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 2]
                                    + p_elem_a[1 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 2]
                                    + p_elem_a[1 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 2];
                elem_sum_val[1][3] += p_elem_a[1 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 3]
                                    + p_elem_a[1 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 3]
                                    + p_elem_a[1 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 3]
                                    + p_elem_a[1 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 3];

                elem_sum_val[2][0] += p_elem_a[2 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 0]
                                    + p_elem_a[2 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 0]
                                    + p_elem_a[2 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 0]
                                    + p_elem_a[2 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 0];
                elem_sum_val[2][1] += p_elem_a[2 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 1]
                                    + p_elem_a[2 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 1]
                                    + p_elem_a[2 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 1]
                                    + p_elem_a[2 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 1];
                elem_sum_val[2][2] += p_elem_a[2 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 2]
                                    + p_elem_a[2 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 2]
                                    + p_elem_a[2 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 2]
                                    + p_elem_a[2 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 2];
                elem_sum_val[2][3] += p_elem_a[2 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 3]
                                    + p_elem_a[2 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 3]
                                    + p_elem_a[2 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 3]
                                    + p_elem_a[2 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 3];

                elem_sum_val[3][0] += p_elem_a[3 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 0]
                                    + p_elem_a[3 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 0]
                                    + p_elem_a[3 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 0]
                                    + p_elem_a[3 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 0];
                elem_sum_val[3][1] += p_elem_a[3 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 1]
                                    + p_elem_a[3 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 1]
                                    + p_elem_a[3 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 1]
                                    + p_elem_a[3 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 1];
                elem_sum_val[3][2] += p_elem_a[3 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 2]
                                    + p_elem_a[3 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 2]
                                    + p_elem_a[3 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 2]
                                    + p_elem_a[3 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 2];
                elem_sum_val[3][3] += p_elem_a[3 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 3]
                                    + p_elem_a[3 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 3]
                                    + p_elem_a[3 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 3]
                                    + p_elem_a[3 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 3];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_N_ELEM(p_elem_a, 4, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, (nxt_r_osf_b * 4), p_mat_b);
            }

            for (elem_idx = elem_size_tmp; elem_idx < c_size_a; elem_idx++) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0][0] += p_elem_a[0 * nxt_r_osf_a] * p_elem_b[0];
                elem_sum_val[0][1] += p_elem_a[0 * nxt_r_osf_a] * p_elem_b[1];
                elem_sum_val[0][2] += p_elem_a[0 * nxt_r_osf_a] * p_elem_b[2];
                elem_sum_val[0][3] += p_elem_a[0 * nxt_r_osf_a] * p_elem_b[3];

                elem_sum_val[1][0] += p_elem_a[1 * nxt_r_osf_a] * p_elem_b[0];
                elem_sum_val[1][1] += p_elem_a[1 * nxt_r_osf_a] * p_elem_b[1];
                elem_sum_val[1][2] += p_elem_a[1 * nxt_r_osf_a] * p_elem_b[2];
                elem_sum_val[1][3] += p_elem_a[1 * nxt_r_osf_a] * p_elem_b[3];

                elem_sum_val[2][0] += p_elem_a[2 * nxt_r_osf_a] * p_elem_b[0];
                elem_sum_val[2][1] += p_elem_a[2 * nxt_r_osf_a] * p_elem_b[1];
                elem_sum_val[2][2] += p_elem_a[2 * nxt_r_osf_a] * p_elem_b[2];
                elem_sum_val[2][3] += p_elem_a[2 * nxt_r_osf_a] * p_elem_b[3];

                elem_sum_val[3][0] += p_elem_a[3 * nxt_r_osf_a] * p_elem_b[0];
                elem_sum_val[3][1] += p_elem_a[3 * nxt_r_osf_a] * p_elem_b[1];
                elem_sum_val[3][2] += p_elem_a[3 * nxt_r_osf_a] * p_elem_b[2];
                elem_sum_val[3][3] += p_elem_a[3 * nxt_r_osf_a] * p_elem_b[3];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, nxt_r_osf_b, p_mat_b);
            }

            /* Store the sum of products to product matrix */
            p_elem_out[nxt_r_osf_out * 0 + 0] = elem_sum_val[0][0];
            p_elem_out[nxt_r_osf_out * 0 + 1] = elem_sum_val[0][1];
            p_elem_out[nxt_r_osf_out * 0 + 2] = elem_sum_val[0][2];
            p_elem_out[nxt_r_osf_out * 0 + 3] = elem_sum_val[0][3];

            p_elem_out[nxt_r_osf_out * 1 + 0] = elem_sum_val[1][0];
            p_elem_out[nxt_r_osf_out * 1 + 1] = elem_sum_val[1][1];
            p_elem_out[nxt_r_osf_out * 1 + 2] = elem_sum_val[1][2];
            p_elem_out[nxt_r_osf_out * 1 + 3] = elem_sum_val[1][3];

            p_elem_out[nxt_r_osf_out * 2 + 0] = elem_sum_val[2][0];
            p_elem_out[nxt_r_osf_out * 2 + 1] = elem_sum_val[2][1];
            p_elem_out[nxt_r_osf_out * 2 + 2] = elem_sum_val[2][2];
            p_elem_out[nxt_r_osf_out * 2 + 3] = elem_sum_val[2][3];

            p_elem_out[nxt_r_osf_out * 3 + 0] = elem_sum_val[3][0];
            p_elem_out[nxt_r_osf_out * 3 + 1] = elem_sum_val[3][1];
            p_elem_out[nxt_r_osf_out * 3 + 2] = elem_sum_val[3][2];
            p_elem_out[nxt_r_osf_out * 3 + 3] = elem_sum_val[3][3];

        }
    }

    /*
     * Resolve  4 by 1 dummy columns in the output matrix sequentially
     */
    for (c_idx = c_size_tmp; c_idx < c_size_out; c_idx++) {

        for (r_idx = 0; r_idx < r_size_tmp; r_idx += 4) {

            /* To store a11 * b11 + .... a_mn * b_nm */
            elem_sum_val[0][0] = LM_MAT_ZERO_VAL;
            elem_sum_val[0][1] = LM_MAT_ZERO_VAL;
            elem_sum_val[0][2] = LM_MAT_ZERO_VAL;
            elem_sum_val[0][3] = LM_MAT_ZERO_VAL;

            /* Point to row 0, 1, ... N of Matrix A */
            p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

            /* Point to column 0, 1, ... N of Matrix B */
            p_elem_b = LM_MAT_GET_COL_PTR(p_mat_b, 1, c_idx);

            p_elem_out = LM_MAT_GET_ROW_PTR(p_mat_out, nxt_r_osf_out, r_idx)
                       + c_idx;

            for (elem_idx = 0; elem_idx < elem_size_tmp; elem_idx += 4) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0][0] += p_elem_a[0 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 0]
                                    + p_elem_a[0 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 0]
                                    + p_elem_a[0 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 0]
                                    + p_elem_a[0 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 0];
                elem_sum_val[0][1] += p_elem_a[1 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 0]
                                    + p_elem_a[1 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 0]
                                    + p_elem_a[1 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 0]
                                    + p_elem_a[1 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 0];
                elem_sum_val[0][2] += p_elem_a[2 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 0]
                                    + p_elem_a[2 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 0]
                                    + p_elem_a[2 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 0]
                                    + p_elem_a[2 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 0];
                elem_sum_val[0][3] += p_elem_a[3 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 0]
                                    + p_elem_a[3 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 0]
                                    + p_elem_a[3 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 0]
                                    + p_elem_a[3 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 0];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_N_ELEM(p_elem_a, 4, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, (nxt_r_osf_b * 4), p_mat_b);
            }

            for (elem_idx = elem_size_tmp; elem_idx < c_size_a; elem_idx++) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0][0] += p_elem_a[0 * nxt_r_osf_a] * p_elem_b[0];
                elem_sum_val[0][1] += p_elem_a[1 * nxt_r_osf_a] * p_elem_b[0];
                elem_sum_val[0][2] += p_elem_a[2 * nxt_r_osf_a] * p_elem_b[0];
                elem_sum_val[0][3] += p_elem_a[3 * nxt_r_osf_a] * p_elem_b[0];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, nxt_r_osf_b, p_mat_b);
            }

            /* Store the sum of products to product matrix */
            p_elem_out[nxt_r_osf_out * 0] = elem_sum_val[0][0];
            p_elem_out[nxt_r_osf_out * 1] = elem_sum_val[0][1];
            p_elem_out[nxt_r_osf_out * 2] = elem_sum_val[0][2];
            p_elem_out[nxt_r_osf_out * 3] = elem_sum_val[0][3];

        }
    }

    /*
     * Resolve 1 by 4 dummy columns in the output matrix sequentially
     */
    for (r_idx = r_size_tmp; r_idx < r_size_out; r_idx++) {

        for (c_idx = 0; c_idx < c_size_tmp; c_idx += 4) {

            /* To store a11 * b11 + .... a_mn * b_nm */
            elem_sum_val[0][0] = LM_MAT_ZERO_VAL;
            elem_sum_val[0][1] = LM_MAT_ZERO_VAL;
            elem_sum_val[0][2] = LM_MAT_ZERO_VAL;
            elem_sum_val[0][3] = LM_MAT_ZERO_VAL;

            /* Point to row 0, 1, ... N of Matrix A */
            p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

            /* Point to column 0, 1, ... N of Matrix B */
            p_elem_b = LM_MAT_GET_COL_PTR(p_mat_b, 1, c_idx);

            p_elem_out = LM_MAT_GET_ROW_PTR(p_mat_out, nxt_r_osf_out, r_idx)
                       + c_idx;

            for (elem_idx = 0; elem_idx < elem_size_tmp; elem_idx += 4) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0][0] += p_elem_a[0 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 0]
                                    + p_elem_a[0 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 0]
                                    + p_elem_a[0 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 0]
                                    + p_elem_a[0 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 0];
                elem_sum_val[0][1] += p_elem_a[0 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 1]
                                    + p_elem_a[0 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 1]
                                    + p_elem_a[0 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 1]
                                    + p_elem_a[0 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 1];
                elem_sum_val[0][2] += p_elem_a[0 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 2]
                                    + p_elem_a[0 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 2]
                                    + p_elem_a[0 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 2]
                                    + p_elem_a[0 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 2];
                elem_sum_val[0][3] += p_elem_a[0 * nxt_r_osf_a + 0] * p_elem_b[0 * nxt_r_osf_b + 3]
                                    + p_elem_a[0 * nxt_r_osf_a + 1] * p_elem_b[1 * nxt_r_osf_b + 3]
                                    + p_elem_a[0 * nxt_r_osf_a + 2] * p_elem_b[2 * nxt_r_osf_b + 3]
                                    + p_elem_a[0 * nxt_r_osf_a + 3] * p_elem_b[3 * nxt_r_osf_b + 3];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_N_ELEM(p_elem_a, 4, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, (nxt_r_osf_b * 4), p_mat_b);
            }

            for (elem_idx = elem_size_tmp; elem_idx < c_size_a; elem_idx++) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0][0] += p_elem_a[0] * p_elem_b[0];
                elem_sum_val[0][1] += p_elem_a[0] * p_elem_b[1];
                elem_sum_val[0][2] += p_elem_a[0] * p_elem_b[2];
                elem_sum_val[0][3] += p_elem_a[0] * p_elem_b[3];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, nxt_r_osf_b, p_mat_b);
            }

            /* Store the sum of products to product matrix */
            p_elem_out[0] = elem_sum_val[0][0];
            p_elem_out[1] = elem_sum_val[0][1];
            p_elem_out[2] = elem_sum_val[0][2];
            p_elem_out[3] = elem_sum_val[0][3];

        }

    }

    /*
     * Resolve 1 by 1 dummy blocks in the output matrix sequentially
     */
    for (r_idx = r_size_tmp; r_idx < r_size_out; r_idx++) {

        for (c_idx = c_size_tmp; c_idx < c_size_out; c_idx++) {

            /* To store a11 * b11 + .... a_mn * b_nm */
            elem_sum_val[0][0] = LM_MAT_ZERO_VAL;

            /* Point to row 0, 1, ... N of Matrix A */
            p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

            /* Point to column 0, 1, ... N of Matrix B */
            p_elem_b = LM_MAT_GET_COL_PTR(p_mat_b, 1, c_idx);

            p_elem_out = LM_MAT_GET_ROW_PTR(p_mat_out, nxt_r_osf_out, r_idx)
                       + c_idx;

            for (elem_idx = 0; elem_idx < elem_size_tmp; elem_idx += 4) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0][0] += p_elem_a[0] * p_elem_b[0 * nxt_r_osf_b]
                                    + p_elem_a[1] * p_elem_b[1 * nxt_r_osf_b]
                                    + p_elem_a[2] * p_elem_b[2 * nxt_r_osf_b]
                                    + p_elem_a[3] * p_elem_b[3 * nxt_r_osf_b];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_N_ELEM(p_elem_a, 4, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, (nxt_r_osf_b * 4), p_mat_b);
            }

            for (elem_idx = elem_size_tmp; elem_idx < c_size_a; elem_idx++) {

                /*
                 * Accumulate the product of element of first matrix
                 * and the element of second matrix
                 */
                elem_sum_val[0][0] += p_elem_a[0] * p_elem_b[0];

                /* Shift to next column of first matrix, A[r][c+1]. */
                LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);

                /* Shift to next row of second matrix, B[r+1][c]. */
                LM_MAT_TO_NXT_ROW(p_elem_b, nxt_r_osf_b, p_mat_b);
            }

            /* Store the sum of products to product matrix */
            p_elem_out[0] = elem_sum_val[0][0];

        }
    }

    return LM_ERR_CODE(LM_SUCCESS);
}


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */

