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
 * @file    lm_oper_gemm.c
 * @brief   Lin matrix GEMM-like computation functions
 * @note
 *
 * Reference:
 *      - https://en.wikipedia.org/wiki/Matrix_multiplication
 *      - https://en.wikipedia.org/wiki/Basic_Linear_Algebra_Subprograms#Level_3
 *      - https://en.wikipedia.org/wiki/Loop_unrolling
 *
 * Abbreviation:
 *     - GEMM: General Matrix Multiply
 *
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#include "lm_oper_gemm.h"
#include "lm_oper.h"
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

static lm_rtn_t lm_oper_gemm_basic(const bool is_transpose_a,
                                   const bool is_transpose_b,
                                   const lm_mat_elem_t alpha,
                                   const lm_mat_t *p_mat_a,
                                   const lm_mat_t *p_mat_b,
                                   const lm_mat_elem_t beta,
                                   lm_mat_t *p_mat_c);

static lm_rtn_t lm_oper_gemm_unrolled(const bool is_transpose_a,
                                      const bool is_transpose_b,
                                      const lm_mat_elem_t alpha,
                                      const lm_mat_t *p_mat_a,
                                      const lm_mat_t *p_mat_b,
                                      const lm_mat_elem_t beta,
                                      lm_mat_t *p_mat_c);


/*
 *******************************************************************************
 * Public functions
 *******************************************************************************
 */

/**
 * lm_oper_gemm - Function to perform GEMM computation.
 *
 * The meaning of GEMM is General Matrix Multiply defined in BLAS.
 * (https://zh.wikipedia.org/zh-tw/BLAS).
 *
 * The GEMM function accept numbers of different matrix multiply
 * operations shown below:
 *
 *      (1) C := alpha * A  * B  + beta * C
 *      (2) C := alpha * A' * B  + beta * C
 *      (3) C := alpha * A  * B' + beta * C
 *      (4) C := alpha * A' * B' + beta * C
 *
 *      (the single quotation mark represent transpose matrix)
 *
 * This function accept vectors or matrices.
 *
 * The dimensions of the input 3 matrices should match the combination shown below:
 *
 *      (1) If A (or its transpose) is an M by N matrix
 *      (2) B (or its transpose) should be a N by L matrix
 *      (3) C should be a M by L matrix.
 *
 *         A     *    B     =>    C
 *      [M by N] * [N by L] => [M by L]
 *
 *
 * @param   [in]        is_transpose_a  Set true to specify  the form of A' to
 *                                      be used in the matrix multiplication.
 * @param   [in]        is_transpose_b  Set true to specify  the form of B' to
 *                                      be used in the matrix multiplication.
 * @param   [in]        alpha           Scalar for alpha * A * B.
 * @param   [in]        *p_mat_a        Handle of matrix A.
 * @param   [in]        *p_mat_b        Handle of matrix B.
 * @param   [in]        beta            Scalar for beta * C.
 * @param   [in,out]    *p_mat_c        Handle of matrix C.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_gemm(const bool is_transpose_a,
                      const bool is_transpose_b,
                      const lm_mat_elem_t alpha,
                      const lm_mat_t *p_mat_a,
                      const lm_mat_t *p_mat_b,
                      const lm_mat_elem_t beta,
                      lm_mat_t *p_mat_c)
{
    return lm_oper_gemm_unrolled(is_transpose_a,
                                 is_transpose_b,
                                 alpha,
                                 p_mat_a,
                                 p_mat_b,
                                 beta,
                                 p_mat_c);
}


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */

/**
 * lm_oper_gemm_basic - Function to perform GEMM computation (basic version).
 *
 *  Basic version GEMM without optimization.
 *
 * @param   [in]        is_transpose_a  Set true to specify  the form of A' to
 *                                      be used in the matrix multiplication.
 * @param   [in]        is_transpose_b  Set true to specify  the form of B' to
 *                                      be used in the matrix multiplication.
 * @param   [in]        alpha           Scalar for alpha * A * B.
 * @param   [in]        *p_mat_a        Handle of matrix A.
 * @param   [in]        *p_mat_b        Handle of matrix B.
 * @param   [in]        beta            Scalar for beta * C.
 * @param   [in,out]    *p_mat_c        Handle of matrix C.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
static lm_rtn_t lm_oper_gemm_basic(const bool is_transpose_a,
                                   const bool is_transpose_b,
                                   const lm_mat_elem_t alpha,
                                   const lm_mat_t *p_mat_a,
                                   const lm_mat_t *p_mat_b,
                                   const lm_mat_elem_t beta,
                                   lm_mat_t *p_mat_c)
{
    lm_rtn_t result;

    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_dim_size_t elem_idx;

    const lm_mat_elem_t *p_elem_a = NULL;
    lm_mat_dim_size_t r_size_a = LM_MAT_GET_R_SIZE(p_mat_a);
    lm_mat_dim_size_t c_size_a = LM_MAT_GET_C_SIZE(p_mat_a);
    lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);
    lm_mat_elem_size_t nxt_c_osf_a = 1;

    const lm_mat_elem_t *p_elem_b = NULL;
    lm_mat_dim_size_t r_size_b = LM_MAT_GET_R_SIZE(p_mat_b);
    lm_mat_dim_size_t c_size_b = LM_MAT_GET_C_SIZE(p_mat_b);
    lm_mat_elem_size_t nxt_r_osf_b = LM_MAT_GET_NXT_OFS(p_mat_b);
    lm_mat_elem_size_t nxt_c_osf_b = 1;

    lm_mat_elem_t *p_elem_c = NULL;
    const lm_mat_dim_size_t r_size_c = LM_MAT_GET_R_SIZE(p_mat_c);
    const lm_mat_dim_size_t c_size_c = LM_MAT_GET_C_SIZE(p_mat_c);
    lm_mat_elem_size_t nxt_r_osf_c = LM_MAT_GET_NXT_OFS(p_mat_c);

    /*
     * (1) alpha * A  * B  + beta * C
     * (2) alpha * A' * B  + beta * C
     * (3) alpha * A  * B' + beta * C
     * (4) alpha * A' * B' + beta * C
     *
     * Step 1: calculate C := beta * C
     * Step 2: calculate C := C + alpha * A(') * B (')
     *
     */
    if (is_transpose_a == true) {
        r_size_a = LM_MAT_GET_C_SIZE(p_mat_a);
        c_size_a = LM_MAT_GET_R_SIZE(p_mat_a);
        nxt_r_osf_a = 1;
        nxt_c_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);
    }

    if (is_transpose_b == true) {
        r_size_b = LM_MAT_GET_C_SIZE(p_mat_b);
        c_size_b = LM_MAT_GET_R_SIZE(p_mat_b);
        nxt_r_osf_b = 1;
        nxt_c_osf_b = LM_MAT_GET_NXT_OFS(p_mat_b);
    }

    /*
     * To calculate the dot product of A and B,
     * he column number of A should equal to row number of B.
     */
    if (c_size_a != r_size_b) {
        return LM_ERR_CODE(LM_ERR_MAT_DOT_PRODUCT_DIM_MISMATCHED);
    }

    /* The dimension of C[M][N] should equal to A[M][L] * B[L][N]. */
    if (r_size_c != r_size_a || c_size_c != c_size_b) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    /* Process beta * C */
    if (beta != LM_MAT_ZERO_VAL) {
        result = lm_oper_scalar(p_mat_c, beta);
        LM_RETURN_IF_ERR(result);
    }
    else {
        result = lm_oper_zeros(p_mat_c);
        LM_RETURN_IF_ERR(result);
    }

    if (alpha != LM_MAT_ZERO_VAL) {

        /* Process C := C + alpha * A(') * B (') */
        for (r_idx = 0; r_idx < r_size_c; r_idx++) {

            p_elem_c = LM_MAT_GET_ROW_PTR(p_mat_c, nxt_r_osf_c, r_idx);

            for (c_idx = 0; c_idx < c_size_c; c_idx++) {

                /* Point to row 0, 1, ... N of Matrix A */
                p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

                /* Point to column 0, 1, ... N of Matrix B */
                p_elem_b = LM_MAT_GET_COL_PTR(p_mat_b, nxt_c_osf_b, c_idx);

                /*
                 * Accumulate the alpha * row vector of A * column vector of B.
                 * and store the C + A * B in C.
                 */

                for (elem_idx = 0; elem_idx < c_size_a; elem_idx++) {

                    p_elem_c[0] += (alpha * p_elem_a[0] * p_elem_b[0]);

                    /* Shift to next column of matrix A */
                    LM_MAT_TO_NXT_COL(p_elem_a, nxt_c_osf_a, p_mat_a);

                    /* Shift to next row of matrix B */
                    LM_MAT_TO_NXT_ROW(p_elem_b, nxt_r_osf_b, p_mat_b);
                }

                LM_MAT_TO_NXT_ELEM(p_elem_c, p_mat_c);

            }
        }
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_gemm_unrolled - Function to perform GEMM computation.
 *
 * Unrolled loop version GEMM
 *
 * @param   [in]        is_transpose_a  Set true to specify  the form of A' to
 *                                      be used in the matrix multiplication.
 * @param   [in]        is_transpose_b  Set true to specify  the form of B' to
 *                                      be used in the matrix multiplication.
 * @param   [in]        alpha           Scalar for alpha * A * B.
 * @param   [in]        *p_mat_a        Handle of matrix A.
 * @param   [in]        *p_mat_b        Handle of matrix B.
 * @param   [in]        beta            Scalar for beta * C.
 * @param   [in,out]    *p_mat_c        Handle of matrix C.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
static lm_rtn_t lm_oper_gemm_unrolled(const bool is_transpose_a,
                                      const bool is_transpose_b,
                                      const lm_mat_elem_t alpha,
                                      const lm_mat_t *p_mat_a,
                                      const lm_mat_t *p_mat_b,
                                      const lm_mat_elem_t beta,
                                      lm_mat_t *p_mat_c)
{
    lm_rtn_t result;

    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_dim_size_t elem_idx;

    const lm_mat_elem_t *p_elem_a = NULL;
    lm_mat_dim_size_t r_size_a = LM_MAT_GET_R_SIZE(p_mat_a);
    lm_mat_dim_size_t c_size_a = LM_MAT_GET_C_SIZE(p_mat_a);
    lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);
    lm_mat_elem_size_t nxt_c_osf_a = 1;
    lm_mat_dim_size_t c_size_a_align_4 = 0;

    const lm_mat_elem_t *p_elem_b = NULL;
    lm_mat_dim_size_t r_size_b = LM_MAT_GET_R_SIZE(p_mat_b);
    lm_mat_dim_size_t c_size_b = LM_MAT_GET_C_SIZE(p_mat_b);
    lm_mat_elem_size_t nxt_r_osf_b = LM_MAT_GET_NXT_OFS(p_mat_b);
    lm_mat_elem_size_t nxt_c_osf_b = 1;

    lm_mat_elem_t *p_elem_c = NULL;
    const lm_mat_dim_size_t r_size_c = LM_MAT_GET_R_SIZE(p_mat_c);
    const lm_mat_dim_size_t c_size_c = LM_MAT_GET_C_SIZE(p_mat_c);
    lm_mat_elem_size_t nxt_r_osf_c = LM_MAT_GET_NXT_OFS(p_mat_c);

    lm_mat_elem_t elem_tmp;

    /*
     * (1) alpha * A  * B  + beta * C
     * (2) alpha * A' * B  + beta * C
     * (3) alpha * A  * B' + beta * C
     * (4) alpha * A' * B' + beta * C
     *
     * Step 1: calculate C := beta * C
     * Step 2: calculate C := C + alpha * A(') * B (')
     *
     */
    if (is_transpose_a == true) {
        r_size_a = LM_MAT_GET_C_SIZE(p_mat_a);
        c_size_a = LM_MAT_GET_R_SIZE(p_mat_a);
        nxt_r_osf_a = 1;
        nxt_c_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);
    }

    if (is_transpose_b == true) {
        r_size_b = LM_MAT_GET_C_SIZE(p_mat_b);
        c_size_b = LM_MAT_GET_R_SIZE(p_mat_b);
        nxt_r_osf_b = 1;
        nxt_c_osf_b = LM_MAT_GET_NXT_OFS(p_mat_b);
    }

    /*
     * To calculate the dot product of A and B,
     * he column number of A should equal to row number of B.
     */
    if (c_size_a != r_size_b) {
        return LM_ERR_CODE(LM_ERR_MAT_DOT_PRODUCT_DIM_MISMATCHED);
    }

    /* The dimension of C[M][N] should equal to A[M][L] * B[L][N]. */
    if (r_size_c != r_size_a || c_size_c != c_size_b) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    /* Process beta * C */
    if (beta != LM_MAT_ZERO_VAL) {
        result = lm_oper_scalar(p_mat_c, beta);
        LM_RETURN_IF_ERR(result);
    }
    else {
        result = lm_oper_zeros(p_mat_c);
        LM_RETURN_IF_ERR(result);
    }

    c_size_a_align_4 = (lm_mat_dim_size_t)(c_size_a - (c_size_a % 4));

    if (alpha != LM_MAT_ZERO_VAL) {

        /* Process C := C + alpha * A(') * B (') */
        for (r_idx = 0; r_idx < r_size_c; r_idx++) {

            p_elem_c = LM_MAT_GET_ROW_PTR(p_mat_c, nxt_r_osf_c, r_idx);

            for (c_idx = 0; c_idx < c_size_c; c_idx++) {

                /* Point to row 0, 1, ... N of Matrix A */
                p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

                /* Point to column 0, 1, ... N of Matrix B */
                p_elem_b = LM_MAT_GET_COL_PTR(p_mat_b, nxt_c_osf_b, c_idx);

                /*
                 * Accumulate the alpha * row vector of A * column vector of B.
                 * and store the C + A * B in C.
                 */

                for (elem_idx = 0; elem_idx < c_size_a_align_4; elem_idx += 4) {

                    /*
                     * p_elem_c[0] += (alpha * p_elem_a[0 * nxt_c_osf_a] * p_elem_b[0 * nxt_r_osf_b])
                     *              + (alpha * p_elem_a[1 * nxt_c_osf_a] * p_elem_b[1 * nxt_r_osf_b])
                     *              + (alpha * p_elem_a[2 * nxt_c_osf_a] * p_elem_b[2 * nxt_r_osf_b])
                     *              + (alpha * p_elem_a[3 * nxt_c_osf_a] * p_elem_b[3 * nxt_r_osf_b]);
                     */

                    elem_tmp = (p_elem_a[0 * nxt_c_osf_a] * p_elem_b[0 * nxt_r_osf_b])
                             + (p_elem_a[1 * nxt_c_osf_a] * p_elem_b[1 * nxt_r_osf_b])
                             + (p_elem_a[2 * nxt_c_osf_a] * p_elem_b[2 * nxt_r_osf_b])
                             + (p_elem_a[3 * nxt_c_osf_a] * p_elem_b[3 * nxt_r_osf_b]);

                    p_elem_c[0] += alpha * elem_tmp;

                    /* Shift to next column of matrix A */
                    LM_MAT_TO_NXT_COL(p_elem_a, (nxt_c_osf_a * 4), p_mat_a);

                    /* Shift to next row of matrix B */
                    LM_MAT_TO_NXT_ROW(p_elem_b, (nxt_r_osf_b * 4), p_mat_b);
                }

                for (elem_idx = c_size_a_align_4; elem_idx < c_size_a; elem_idx++) {

                    p_elem_c[0] += (alpha * p_elem_a[0] * p_elem_b[0]);

                    /* Shift to next column of matrix A */
                    LM_MAT_TO_NXT_COL(p_elem_a, nxt_c_osf_a, p_mat_a);

                    /* Shift to next column of matrix B */
                    LM_MAT_TO_NXT_ROW(p_elem_b, nxt_r_osf_b, p_mat_b);
                }

                LM_MAT_TO_NXT_ELEM(p_elem_c, p_mat_c);

            }
        }
    }

    return LM_ERR_CODE(LM_SUCCESS);
}
