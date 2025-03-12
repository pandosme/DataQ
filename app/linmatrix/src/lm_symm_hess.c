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
 * @file    lm_symm_hess.c
 * @brief   Lin matrix Hessenberg similarity transformation functions for
 *          symmetric matrix
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#include <math.h>

#include "lm_symm_hess.h"
#include "lm_qr.h"
#include "lm_mat.h"
#include "lm_err.h"
#include "lm_shape.h"
#include "lm_oper.h"
#include "lm_oper_gemm.h"
#include "lm_oper_axpy.h"


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
 * lm_symm_hess - Function to compute the Hessenberg similar matrix and
 *                the corresponding transformation vectors of given matrix.
 *
 * @note
 *
 *      Please note that this function computes and generates T and Q in
 *      implicit format.
 *
 *      Hessenberg similarity transformation:
 *
 *          T = inv(Q) * A * Q
 *            =   Q'   * A * Q
 *
 *      where:
 *          T is the Hessenberg similar matrix (tridiagonal form).
 *          Q is the orthogonal similarity transformation matrix.
 *          A is the given matrix.
 *
 *      The given matrix must be a square matrix.
 *
 *      Reference:
 *          - 8.3.1 Reduction to Tridiagonal Form, "Matrix Computation 4th
 *            edition" written by Golub and Van Loan.
 *
 * @param   [in,out]    *p_mat_t        Handle of given matrix.
 *
 *      On entry:
 *          The T matrix contains the original data to be transformed.
 *          The given matrix QR must be a square matrix.
 *
 *      On exit:
 *          The implicit Q (Householder vectors) is stored in lower
 *          triangular part of T matrix (expect the main diagonal and
 *          subdiagonal). The tridiagonal elements of generated Hessenberg
 *          similar matrix are stored in the corresponding position of
 *          matrix T.
 *
 *         Given matrix                              Implicit Q and T
 *        -           -                             -                -
 *        | a a a a a |         Hessenberg          |  t  t  0  0  0 |
 *        | a a a a a |         similarity          |  t  t  t  0  0 |
 *        | a a a a a |       ==============>       | v1  t  t  t  0 |
 *        | a a a a a |       transformation        | v1 v2  t  t  t |
 *        | a a a a a |                             | v1 v2 v3  t  t |
 *        -           -                             -                -
 *
 *      v1 ~ vn are Householder vectors.
 *
 * @param   [out]       *p_mat_beta     Handle of beta vector.
 *
 *      On entry:
 *          The size of the beta vector should be M by 1, where M is
 *          equal to row size of the given matrix. This vector is needed
 *          for storing the beta value corresponding to each Householder
 *          vector.
 *
 *      On exit:
 *          The beta values are stored in this vector. these beta values
 *          are needed for converting implicit T and Q to explicit format.
 *
 * @param   [in,out]    *p_mat_work     Handle of working matrix needed for
 *                                      completing Hessenberg similarity
 *                                      transformation process, its output data
 *                                      is not meaningful. The size of working
 *                                      matrix should be M by 1, where M is equal
 *                                      to row size of the given matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_symm_hess(lm_mat_t *p_mat_t,
                      lm_mat_t *p_mat_beta,
                      lm_mat_t *p_mat_work)
{
    lm_rtn_t result;
    lm_mat_elem_t alpha;
    lm_mat_elem_size_t r_idx;
    lm_mat_elem_size_t c_idx;

    lm_mat_t mat_scalar = {0};
    lm_mat_elem_t elem_scalar;

    const lm_mat_dim_size_t r_size_t = LM_MAT_GET_R_SIZE(p_mat_t);
    const lm_mat_dim_size_t c_size_t = LM_MAT_GET_C_SIZE(p_mat_t);
    const lm_mat_elem_size_t nxt_r_osf_t = LM_MAT_GET_NXT_OFS(p_mat_t);

    lm_mat_elem_t *p_elem_beta = LM_MAT_GET_ELEM_PTR(p_mat_beta);
    const lm_mat_dim_size_t r_size_beta = LM_MAT_GET_R_SIZE(p_mat_beta);
    const lm_mat_dim_size_t c_size_beta = LM_MAT_GET_C_SIZE(p_mat_beta);

    const lm_mat_dim_size_t r_size_work = LM_MAT_GET_R_SIZE(p_mat_work);
    const lm_mat_dim_size_t c_size_work = LM_MAT_GET_C_SIZE(p_mat_work);

    lm_mat_t mat_h_subm_shaped = {0};
    lm_mat_t mat_housh_vec_shaped = {0};
    lm_mat_t mat_work_shaped = {0};

    if (r_size_t == 0 || c_size_t == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    /* Must be square matrix */
    if (r_size_t != c_size_t) {
        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_SQUARE);
    }

    /*
     * A M by 1 matrix is required for storing the Householder
     * reflection beta (scalar factor) value
     */
    if (r_size_beta != r_size_t || c_size_beta != 1) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    /* A M by 1 matrix (work space) is required for QR calculation */
    if (r_size_work != r_size_t || c_size_work != 1) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    /* Nothing to do for 1 by N and 2 by N matrix */
    if (r_size_t <= 2) {

        result = lm_oper_zeros(p_mat_beta);

        return result;
    }

    result = lm_mat_set(&mat_scalar, 1, 1, &elem_scalar,
                        (sizeof(elem_scalar) / sizeof(lm_mat_elem_t)));
    LM_RETURN_IF_ERR(result);

    for (r_idx = 1, c_idx = 0; c_idx < (c_size_t - 1); r_idx++, c_idx++) {

        /* Get the sub-column vector that needs to be reflected */
        result = lm_shape_submatrix(p_mat_t,
                                    r_idx, c_idx,
                                    (r_size_t - r_idx), 1,
                                    &mat_housh_vec_shaped);
        LM_RETURN_IF_ERR(result);

        /* Calculate the Householder reflection vector and the beta value */
        result = lm_qr_housh_v(&mat_housh_vec_shaped, &alpha, &(p_elem_beta[0]));
        LM_RETURN_IF_ERR(result);

        if (p_elem_beta[0] != LM_MAT_ZERO_VAL) {

            /*
             * p = beta * A(k + l:n, k + l:n) * v
             * Note: p is a L by 1 vector,
             *       a L by 1 working matrix is required to store calculation results.
             */

            /* Setup sub-matrix */
            result = lm_shape_submatrix(p_mat_t,
                                        r_idx, c_idx + 1,
                                        (r_size_t - r_idx),
                                        (lm_mat_dim_size_t)(c_size_t - c_idx - 1),
                                        &mat_h_subm_shaped);
            LM_RETURN_IF_ERR(result);

            /* Setup the work matrix */
            result = lm_shape_submatrix(p_mat_work,
                                        0, 0,
                                        (r_size_t - r_idx), 1,
                                        &mat_work_shaped);
            LM_RETURN_IF_ERR(result);

            /* p = beta * A(k + l:n, k + l:n) * v */
            result = lm_oper_gemm(false, false,
                                  p_elem_beta[0],
                                  &mat_h_subm_shaped,
                                  &mat_housh_vec_shaped,
                                  LM_MAT_ZERO_VAL,
                                  &mat_work_shaped);
            LM_RETURN_IF_ERR(result);

            /*
             * w = p - (beta * p' * v / 2) * v
             * Note: (beta * p' * v / 2) is a scalar C.
             *       w =  p - C * v
             *         = -C * v + p, is a L by 1 vector
             */

            result = lm_oper_gemm(true, false,
                                  (p_elem_beta[0] * 0.5),
                                  &mat_work_shaped,
                                  &mat_housh_vec_shaped,
                                  LM_MAT_ZERO_VAL,
                                  &mat_scalar);
            LM_RETURN_IF_ERR(result);

            result = lm_oper_axpy((-elem_scalar),
                                  &mat_housh_vec_shaped,
                                  &mat_work_shaped);
            LM_RETURN_IF_ERR(result);

            /*
             * Complete the Hessenberg similar matrix (tridiagonal form)
             *
             * A(k + l:n, k + l:n) = A(k + l:n, k + l:n) - v * w' - w * v'
             *
             * 2 different methods for completing the calculation above:
             *
             * Method 1: - v * w' - w * v' = - ((v * w') + (w * v'))
             *                             = - ((v * w') + (v * w')')
             *
             *           (v * w') is a L by L matrix, so an L x L
             *           working matrix is required for this calculation
             *
             * Method 2: A(k + l:n, k + l:n) -= v * w'
             *           A(k + l:n, k + l:n) -= w * v'
             *
             *           No additional work matrix is needed for this calculation,
             *           but more flops are needed to calculate v * w'and w * v'.
             *
             * Use method 2 to calculate the formula above!
             *
             */
            result = lm_oper_gemm(false, true,
                                  (-LM_MAT_ONE_VAL),
                                  &mat_housh_vec_shaped,
                                  &mat_work_shaped,
                                  LM_MAT_ONE_VAL,
                                  &mat_h_subm_shaped);
            LM_RETURN_IF_ERR(result);

            result = lm_oper_gemm(false, true,
                                  (-LM_MAT_ONE_VAL),
                                  &mat_work_shaped,
                                  &mat_housh_vec_shaped,
                                  LM_MAT_ONE_VAL,
                                  &mat_h_subm_shaped);
            LM_RETURN_IF_ERR(result);

            /*
             * A(k + 1, k) = norm(A(k + l:n, k))
             * Note: Store the alpha in subdiagonal.
             *
             * A = [
             *          X X X X X;
             *          a X X X X;
             *          X X X X X;
             *          X X X X X;
             *          X X X X X;
             * ]
             *
             */
            mat_housh_vec_shaped.elem.ptr[0] = (-alpha);

            /*
             * A(k, k + 1) = A(k + 1, k)
             * Note: The values of superdiagonal and subdiagonal are symmetric.
             * A = [
             *          X a X X X;
             *          a X X X X;
             *          X X X X X;
             *          X X X X X;
             *          X X X X X;
             * ]
             *
             */
            *(LM_MAT_GET_ROW_PTR(p_mat_t, nxt_r_osf_t, c_idx) + (r_idx)) = (-alpha);

        }

        LM_MAT_TO_NXT_ELEM(p_elem_beta, p_mat_beta);
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_symm_hess_explicit - Function to convert implicit Hessenberg similar
 *                         matrix and the corresponding transformation
 *                         vectors to explicit format.
 *
 * @param   [in,out]    *p_mat_t       Handle of given matrix.
 *
 *      On entry:
 *          The T matrix contains the data of implicit Hessenberg similar
 *          matrix and the corresponding transformation vectors.
 *
 *      On exit:
 *          The explicit T (tridiagonal) is stored in this matrix.
 *
 *        Implicit Hessenberg                Explicit Q             Explicit T
 *        -                -             -               -     -                -
 *        |  t  t  0  0  0 |             | q  q  q  q  q |     |  t  t  0  0  0 |
 *        |  t  t  t  0  0 |   convert   | q  q  q  q  q |     |  t  t  t  0  0 |
 *        | v1  t  t  t  0 | ==========> | q  q  q  q  q | and |  0  t  t  t  0 |
 *        | v1 v2  t  t  t |             | q  q  q  q  q |     |  0  0  t  t  t |
 *        | v1 v2 v3  t  t |             | q  q  q  q  q |     |  0  0  0  t  t |
 *        -                -             -               -     -                -
 *
 * @param   [in]        *p_mat_beta      Handle of beta vector.
 *
 *      On entry:
 *          This beta vector should contains the beta values generated
 *          during Hessenberg similarity transformation procedure. The
 *          size of this beta vector should be M by 1, where M is equal
 *          to row size of the given implicit Hessenberg similar matrix.
 *
 * @param   [out]       *p_mat_q        Handle of Q matrix.
 *
 *      On entry:
 *          The size of this Q matrix should be M by M, where M is equal
 *          to row size of the given matrix.
 *
 *      On exit:
 *          The explicit Q (orthogonal matrix) is stored in this matrix.
 *
 * @param   [in,out]    *p_mat_work     Handle of working matrix needed for
 *                                      completing implicit /explicit Hessenberg
 *                                      similar matrix conversion process, its
 *                                      output data is not meaningful. The size
 *                                      of working matrix should be 1 by M, where
 *                                      M is equal to row size of the given matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_symm_hess_explicit(lm_mat_t *p_mat_t,
                               const lm_mat_t *p_mat_beta,
                               lm_mat_t *p_mat_q,
                               lm_mat_t *p_mat_work)
{
    lm_rtn_t result;
    lm_mat_elem_t v1_tmp;
    lm_mat_elem_size_t r_idx;
    lm_mat_elem_size_t c_idx;

    const lm_mat_dim_size_t r_size_t = LM_MAT_GET_R_SIZE(p_mat_t);
    const lm_mat_dim_size_t c_size_t = LM_MAT_GET_C_SIZE(p_mat_t);

    const lm_mat_elem_t *p_elem_beta = LM_MAT_GET_ELEM_PTR(p_mat_beta);
    const lm_mat_dim_size_t r_size_beta = LM_MAT_GET_R_SIZE(p_mat_beta);
    const lm_mat_dim_size_t c_size_beta = LM_MAT_GET_C_SIZE(p_mat_beta);

    const lm_mat_dim_size_t r_size_q = LM_MAT_GET_R_SIZE(p_mat_q);
    const lm_mat_dim_size_t c_size_q = LM_MAT_GET_C_SIZE(p_mat_q);

    const lm_mat_dim_size_t r_size_work = LM_MAT_GET_R_SIZE(p_mat_work);
    const lm_mat_dim_size_t c_size_work = LM_MAT_GET_C_SIZE(p_mat_work);

    lm_mat_t mat_housh_vec_shaped = {0};
    lm_mat_t mat_q_subm_shaped = {0};
    lm_mat_t mat_work_shaped = {0};

    if (r_size_t == 0 || c_size_t == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    /* Must be square matrix */
    if (r_size_t != c_size_t) {
        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_SQUARE);
    }

    /*
     * A M by M square matrix is required for storing the
     * Householder orthogonal matrix Q
     */
    if (r_size_q != r_size_t || c_size_q != r_size_t) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    /* The Householder beta list should contains M elements (M by 1) */
    if (r_size_beta != r_size_t || c_size_beta != 1) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    /* A 1 x M matrix (work space) is required for explicit QR transformation */
    if (r_size_work != 1 || c_size_work != r_size_t) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    result = lm_oper_identity(p_mat_q);
    LM_RETURN_IF_ERR(result);

    for (r_idx = 1, c_idx = 0; c_idx < (c_size_t - 1); r_idx++, c_idx++) {

        /* Do the reflection on QR sub-matrix if needed */
        if (p_elem_beta[0] != LM_MAT_ZERO_VAL) {

            /* Get the Householder reflection vector from QR matrix*/
            result = lm_shape_submatrix(p_mat_t,
                                        r_idx, c_idx,
                                        (r_size_t - r_idx), 1,
                                        &mat_housh_vec_shaped);
            LM_RETURN_IF_ERR(result);

            /* Setup Q sub-matrix that needs to be reflected */
            result = lm_shape_submatrix(p_mat_q,
                                        r_idx, 0,
                                        (r_size_q - r_idx), c_size_q,
                                        &mat_q_subm_shaped);
            LM_RETURN_IF_ERR(result);

            /* Setup the work matrix */
            result = lm_shape_submatrix(p_mat_work,
                                        0, 0,
                                        1, r_size_q,
                                        &mat_work_shaped);
            LM_RETURN_IF_ERR(result);

            /*
             * Backup the first element of Householder reflection vector,
             * and change the value of first element to "one" for completing
             * Householder reflection vector
             */
            v1_tmp = mat_housh_vec_shaped.elem.ptr[0];
            mat_housh_vec_shaped.elem.ptr[0] = LM_MAT_ONE_VAL;

            /*
             * Do the reflection on Q matrix
             */
            result = lm_qr_housh_refl(&mat_q_subm_shaped,
                                      p_elem_beta[0],
                                      &mat_housh_vec_shaped,
                                      &mat_work_shaped);
            LM_RETURN_IF_ERR(result);

            /* Restore the first element */
            mat_housh_vec_shaped.elem.ptr[0] = v1_tmp;
        }

        LM_MAT_TO_NXT_ELEM(p_elem_beta, p_mat_beta);
    }

    /* Output the inverse of matrix Q, => inv(Q) = Q' */
    result = lm_oper_transpose(p_mat_q);
    LM_RETURN_IF_ERR(result);

    /*
     * Clear the upper triangular and lower triangular part
     * (except the tridiagonal) of QR matrix and output R
     */
    if (r_size_t > 2) {
        result = lm_oper_zeros_triu(p_mat_t, 2);
        LM_RETURN_IF_ERR(result);

        result = lm_oper_zeros_tril(p_mat_t, -2);
        LM_RETURN_IF_ERR(result);
    }

    return LM_ERR_CODE(LM_SUCCESS);
}


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */

