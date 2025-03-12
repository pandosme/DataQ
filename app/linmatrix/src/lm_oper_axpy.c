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
 * @file    lm_oper_axpy.c
 * @brief   Lin matrix AXPY-like computation functions
 * @note
 *
 * Reference:
 *     - https://zh.wikipedia.org/zh-tw/BLAS
 *
 * Abbreviation:
 *     - AXPY: Alpha * X Plus Y
 *
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#include "lm_oper_axpy.h"
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

static lm_rtn_t lm_oper_axpy_basic(const lm_mat_elem_t alpha,
                                   const lm_mat_t *p_mat_a,
                                   lm_mat_t *p_mat_b);
static lm_rtn_t lm_oper_axpy_unrolled(const lm_mat_elem_t alpha,
                                      const lm_mat_t *p_mat_a,
                                      lm_mat_t *p_mat_b);


/*
 *******************************************************************************
 * Public functions
 *******************************************************************************
 */

/**
 * lm_oper_axpy - Function to perform AXPY computation.
 *
 * The meaning of AXPY is Y := alpha * X + Y defined in BLAS.
 * (https://zh.wikipedia.org/zh-tw/BLAS).
 *
 * This function accept vectors or matrices.
 *
 * The dimensions of the input 2 matrices should be the same.
 *
 * If the alpha is equal to LM_MAT_ONE_VALUE (1.0), this function performs
 * Y := X + Y computation. If the alpha is equal to LM_MAT_ZERO_VALUE (0.0),
 * this function performs Y := Y. Otherwise this function performs
 * Y := (alpha .* X) + Y computation.
 *
 * @param   [in]        alpha       Scalar for alpha * X.
 * @param   [in]        *p_mat_a    Handle of matrix X.
 * @param   [in,out]    *p_mat_b    Handle of matrix Y.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_axpy(const lm_mat_elem_t alpha,
                      const lm_mat_t *p_mat_a,
                      lm_mat_t *p_mat_b)
{
    return lm_oper_axpy_unrolled(alpha, p_mat_a, p_mat_b);
}


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */

/**
 * lm_oper_axpy_basic - Function to perform AXPY computation (basic version).
 *
 *
 * @param   [in]        alpha       Scalar for alpha * X.
 * @param   [in]        *p_mat_a    Handle of matrix X.
 * @param   [in,out]    *p_mat_b    Handle of matrix Y.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
static lm_rtn_t lm_oper_axpy_basic(const lm_mat_elem_t alpha,
                                   const lm_mat_t *p_mat_a,
                                   lm_mat_t *p_mat_b)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    const lm_mat_elem_t *p_elem_a = NULL;
    lm_mat_elem_t *p_elem_b = NULL;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_b = LM_MAT_GET_NXT_OFS(p_mat_b);

    /* Matrix dimensions must be equal */
    if (p_mat_a->elem.dim.r != p_mat_b->elem.dim.r
        || p_mat_a->elem.dim.c != p_mat_b->elem.dim.c) {

        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    if (alpha != LM_MAT_ZERO_VAL) {

        /* Calculate B := 1 * A + B = B + A */
        if (alpha == LM_MAT_ONE_VAL) {
            for (r_idx = 0; r_idx < r_size; r_idx++) {

                p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);
                p_elem_b = LM_MAT_GET_ROW_PTR(p_mat_b, nxt_r_osf_b, r_idx);

                for (c_idx = 0; c_idx < c_size; c_idx++) {

                    /* B[i][j] += A[i][j] */
                    p_elem_b[0] += p_elem_a[0];

                    LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
                    LM_MAT_TO_NXT_ELEM(p_elem_b, p_mat_b);
                }

            }
        }
        /* Calculate B := -1 * A + B = B - A */
        else if (alpha == (-LM_MAT_ONE_VAL)) {

            for (r_idx = 0; r_idx < r_size; r_idx++) {

                p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);
                p_elem_b = LM_MAT_GET_ROW_PTR(p_mat_b, nxt_r_osf_b, r_idx);

                for (c_idx = 0; c_idx < c_size; c_idx++) {

                    /* B[i][j] -= A[i][j] */
                    p_elem_b[0] -= p_elem_a[0];

                    LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
                    LM_MAT_TO_NXT_ELEM(p_elem_b, p_mat_b);
                }

            }
        }
        /* Calculate B := alpha * A + B */
        else {

            for (r_idx = 0; r_idx < r_size; r_idx++) {

                p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);
                p_elem_b = LM_MAT_GET_ROW_PTR(p_mat_b, nxt_r_osf_b, r_idx);

                for (c_idx = 0; c_idx < c_size; c_idx++) {

                    /* B[i][j] += alpha * A[i][j] */
                    p_elem_b[0] += (alpha * p_elem_a[0]);

                    LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
                    LM_MAT_TO_NXT_ELEM(p_elem_b, p_mat_b);
                }

            }
        }

    }
    /* Calculate B = 0 * A + B */
    else {
        /* B = B, Do nothing */
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_oper_axpy_unrolled - Function to perform AXPY computation (basic version).
 *
 *
 * @param   [in]        alpha       Scalar for alpha * X.
 * @param   [in]        *p_mat_a    Handle of matrix X.
 * @param   [in,out]    *p_mat_b    Handle of matrix Y.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
static lm_rtn_t lm_oper_axpy_unrolled(const lm_mat_elem_t alpha,
                                      const lm_mat_t *p_mat_a,
                                      lm_mat_t *p_mat_b)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    const lm_mat_elem_t *p_elem_a = NULL;
    lm_mat_elem_t *p_elem_b = NULL;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_b = LM_MAT_GET_NXT_OFS(p_mat_b);
    lm_mat_dim_size_t c_size_align_4;

    /* Matrix dimensions must be equal */
    if (p_mat_a->elem.dim.r != p_mat_b->elem.dim.r
        || p_mat_a->elem.dim.c != p_mat_b->elem.dim.c) {

        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    c_size_align_4 = (c_size - (c_size % 4));

    if (alpha != LM_MAT_ZERO_VAL) {

        /* Calculate B := 1 * A + B = B + A */
        if (alpha == LM_MAT_ONE_VAL) {
            for (r_idx = 0; r_idx < r_size; r_idx++) {

                p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);
                p_elem_b = LM_MAT_GET_ROW_PTR(p_mat_b, nxt_r_osf_b, r_idx);

                for (c_idx = 0; c_idx < c_size_align_4; c_idx += 4) {

                    /* B[i][j] += A[i][j] */
                    p_elem_b[0] += p_elem_a[0];
                    p_elem_b[1] += p_elem_a[1];
                    p_elem_b[2] += p_elem_a[2];
                    p_elem_b[3] += p_elem_a[3];

                    LM_MAT_TO_NXT_N_ELEM(p_elem_a, 4, p_mat_a);
                    LM_MAT_TO_NXT_N_ELEM(p_elem_b, 4, p_mat_b);
                }

                for (c_idx = c_size_align_4; c_idx < c_size; c_idx++) {

                    /* B[i][j] += A[i][j] */
                    p_elem_b[0] += p_elem_a[0];

                    LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
                    LM_MAT_TO_NXT_ELEM(p_elem_b, p_mat_b);
                }

            }
        }
        /* Calculate B := -1 * A + B = B - A */
        else if (alpha == (-LM_MAT_ONE_VAL)) {

            for (r_idx = 0; r_idx < r_size; r_idx++) {

                p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);
                p_elem_b = LM_MAT_GET_ROW_PTR(p_mat_b, nxt_r_osf_b, r_idx);

                for (c_idx = 0; c_idx < c_size_align_4; c_idx += 4) {

                    /* B[i][j] -= A[i][j] */
                    p_elem_b[0] -= p_elem_a[0];
                    p_elem_b[1] -= p_elem_a[1];
                    p_elem_b[2] -= p_elem_a[2];
                    p_elem_b[3] -= p_elem_a[3];

                    LM_MAT_TO_NXT_N_ELEM(p_elem_a, 4, p_mat_a);
                    LM_MAT_TO_NXT_N_ELEM(p_elem_b, 4, p_mat_b);
                }

                for (c_idx = c_size_align_4; c_idx < c_size; c_idx++) {

                    /* B[i][j] -= A[i][j] */
                    p_elem_b[0] -= p_elem_a[0];

                    LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
                    LM_MAT_TO_NXT_ELEM(p_elem_b, p_mat_b);
                }

            }
        }
        /* Calculate B := alpha * A + B */
        else {

            for (r_idx = 0; r_idx < r_size; r_idx++) {

                p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);
                p_elem_b = LM_MAT_GET_ROW_PTR(p_mat_b, nxt_r_osf_b, r_idx);

                for (c_idx = 0; c_idx < c_size_align_4; c_idx += 4) {

                    /* B[i][j] += alpha * A[i][j] */
                    p_elem_b[0] += alpha * p_elem_a[0];
                    p_elem_b[1] += alpha * p_elem_a[1];
                    p_elem_b[2] += alpha * p_elem_a[2];
                    p_elem_b[3] += alpha * p_elem_a[3];

                    LM_MAT_TO_NXT_N_ELEM(p_elem_a, 4, p_mat_a);
                    LM_MAT_TO_NXT_N_ELEM(p_elem_b, 4, p_mat_b);
                }

                for (c_idx = c_size_align_4; c_idx < c_size; c_idx++) {

                    /* B[i][j] += alpha * A[i][j] */
                    p_elem_b[0] += alpha * p_elem_a[0];

                    LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
                    LM_MAT_TO_NXT_ELEM(p_elem_b, p_mat_b);
                }
            }
        }

    }
    /* Calculate B = 0 * A + B */
    else {
        /* B = B, Do nothing */
    }

    return LM_ERR_CODE(LM_SUCCESS);
}
