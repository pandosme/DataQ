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
 * @file    lm_qr.c
 * @brief   Lin matrix QR decomposition functions
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#include <math.h>

#include "lm_qr.h"
#include "lm_mat.h"
#include "lm_err.h"
#include "lm_shape.h"
#include "lm_oper.h"
#include "lm_oper_gemm.h"


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
 * lm_qr_housh_v - Function to compute Householder vector for given row vector.
 *
 * @note
 *
 *      Reference:
 *          - 5.2.2 Householder QR, "Matrix Computation 4th edition" written
 *            by Golub and Van Loan
 *
 * @param   [in,out]    *p_mat_v    Handle of given row vector.
 *
 *      On entry:
 *          The row vector contains the original data that needs to be
 *          reflected.
 *
 *      On exit:
 *          The rescaled Householder vector is stored in this row vector.
 *
 *         Given vector                               Rescaled Householder vector
 *             -   -                                             -   -
 *             | a |         Compute Householder vector          | 1 |
 *             | b |              for given vector               | h |
 *             | c |       ==============================>       | h |
 *             | d |                                             | h |
 *             | e |                                             | h |
 *             -   -                                             -   -
 *
 * @param   [out]       *p_alpha    Scalar alpha.
 *
 *      On exit, the value of first element of reflection vector is stored
 *      in this variable (scalar alpha must be equal to +norm(v) or -norm (v)).
 *
 *          Given vector                                     Reflection vector
 *             -   -                                             -       -
 *             | a |                                             | alpha |
 *             | b |         Householder transformation          |   0   |
 *             | c |       ==============================>       |   0   |
 *             | d |                                             |   0   |
 *             | e |                                             |   0   |
 *             -   -                                             -       -
 *
 * @param   [out]       *p_beta     Scalar beta.
 *
 *      On exit, the rescaled (2 / (v' * v)) value is stored in this variable.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_qr_housh_v(lm_mat_t *p_mat_v,
                       lm_mat_elem_t *p_alpha,
                       lm_mat_elem_t *p_beta)
{
    lm_rtn_t result;
    lm_mat_t mat_v_shaped = {0};
    lm_mat_t mat_dot = {0};
    lm_mat_elem_t u_elem_rest_squared;
    lm_mat_elem_t u_elem_1_squared;
    lm_mat_elem_t *p_elem_v = LM_MAT_GET_ELEM_PTR(p_mat_v);
    const lm_mat_dim_size_t r_size_v = LM_MAT_GET_R_SIZE(p_mat_v);
    const lm_mat_dim_size_t c_size_v = LM_MAT_GET_C_SIZE(p_mat_v);

    if (r_size_v == 0 || c_size_v == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    if (c_size_v != 1) {
        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_VECTOR);
    }

    if (r_size_v == 1) {
        *p_alpha = p_elem_v[0];
        *p_beta = LM_MAT_ZERO_VAL;

        return LM_ERR_CODE(LM_SUCCESS);
    }

    /* Point to v(2:end) */
    result = lm_shape_submatrix(p_mat_v, 1, 0,
                                (r_size_v - 1), 1,
                                &mat_v_shaped);
    LM_RETURN_IF_ERR(result);

    /* For storing v[2:end]' * v[2:end] */
    result = lm_mat_set(&mat_dot, 1, 1, &u_elem_rest_squared,
                        (sizeof(u_elem_rest_squared) / sizeof(lm_mat_elem_t)));
    LM_RETURN_IF_ERR(result);

    /* Calculate v[2:end]' * v[2:end] and store the result in u_elem_rest_squared */
    result = lm_oper_gemm(true, false,
                          LM_MAT_ONE_VAL, &mat_v_shaped, &mat_v_shaped,
                          LM_MAT_ZERO_VAL, &mat_dot);
    LM_RETURN_IF_ERR(result);

    /* Calculate v[1] * v[1] */
    u_elem_1_squared = p_elem_v[0] * p_elem_v[0];

    /*
     * Compute Householder vector only if the given row vector is not a
     * zero vector.
     */
    if ((u_elem_1_squared + u_elem_rest_squared) > LM_MAT_ZERO_VAL) {

        /*
         * Step 1: Calculate the scalar alpha
         *
         * The goal of Householder reflection is to reflect a vector X_orig to X_refl.
         *
         * X_orig is the the original vector [a, b, c, ...]' that needs to be reflected.
         *
         * X_refl is the reflection vector [+-norm(X_orig), 0, 0, ...]' aligned with a
         * specific coordinate axis.
         *
         * We first calculate the +-norm(X_orig) and store it in alpha.
         *
         */
        *p_alpha = (lm_mat_elem_t)(LM_SIGN(p_elem_v[0])
                                   * sqrt(u_elem_1_squared + u_elem_rest_squared));

        /*
         * Step 2: Generate Householder vector by calculating X_orig + X_refl.
         */
         p_elem_v[0] += *p_alpha;

        /*
         * Step 3: Calculate the scalar beta
         *
         * A complete Householder transform formula is:
         *           2 * v  * v'
         *     I - ------------- , (v represents the original Householder vector)
         *               v' * v
         *
         * But because we will rescale the Householder vector v to v_new = v ./ v(1)
         * for changing the value of first element of Householder vector to be equal
         * to 1.
         * The result of the above Householder transformation to will be incorrect
         * if we input the rescaled Householder vector v_new to the equation above
         * directly.
         *
         * To solve this problem, we must modify part of the original transformation
         * to be:
         *
         *      2 * v  * v'    2 * (v_new  * v(1)) * (v_new' * v(1))
         *     ------------ = ---------------------------------------
         *          v' * v                 v'      *       v
         *
         *                     2 * v(1) * v(1)
         *                  = ----------------- * (v_new * v_new')
         *                           v' *  v
         *
         *                       2 * v(1)^2
         *                  = ----------------- * (v_new * v_new')
         *                         norm(v)
         *
         *                  =        beta       * (v_new * v_new')
         *
         *
         * Now calculate (2 * v(1)^2) / norm(v) and store the reult in beta.
         *
         * Note: The reason we don¡¦t change v'*v to (v_new'*v(1))*(v_new*v(1))
         *       and knock out all the v(1) is because this requires more flops
         *       to recalculate the norm of v_new. We have already calculated
         *       norm(v) in the previous step, why not reuse the norm(v) we have
         *       already calculated.
         */
        *p_beta = ((lm_mat_elem_t)(2.0) * (p_elem_v[0] * p_elem_v[0]))
                / (p_elem_v[0] * p_elem_v[0] + u_elem_rest_squared);

        /*
         * Step 4: rescale the rest elements of original Householder vector
         * v(2:end) := v(2:end) / v(1)
         */
        result = lm_oper_scalar(p_mat_v, (LM_MAT_ONE_VAL / p_elem_v[0]));
        LM_RETURN_IF_ERR(result);

    }
    else {
        /* No need to do reflection */
        *p_alpha = LM_MAT_ZERO_VAL;
        *p_beta = LM_MAT_ZERO_VAL;
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_qr_housh_refl - Function to perform Householder reflection on given matrix.
 *
 * @param   [in,out]    *p_mat_a        Handle of given matrix.
 *
 *      On entry:
 *          The given matrix contains the data that needs to be reflected.
 *
 *      On exit:
 *          The reflected data is stored in this matrix.
 *
 * @param   [in]        beta            Scalar beta of corresponding Householder vector.
 *
 * @param   [in]        *p_mat_houshv   Handle of Householder vector.
 *                                      The size of the household vector should be M by 1,
 *                                      where M is equal to the row size of the given matrix.
 *
 * @param   [in,out]    *p_mat_work     Handle of working matrix needed for completing
 *                                      reflection process, its output data is not meaningful.
 *                                      The size of working matrix should be same as the given
 *                                      matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_qr_housh_refl(lm_mat_t *p_mat_a,
                          lm_mat_elem_t beta,
                          lm_mat_t *p_mat_houshv,
                          lm_mat_t *p_mat_work)
{
    lm_rtn_t result;

    /*
     * Householder reflection:
     * Calculate A := A - (beta * (u * u')) * A, separate the calculation into 2 steps:
     *      (1) Calculate the u' * A and store the result in the temporary 'work' matrix.
     *      (2) Calculate the A -= beta * u * work.
     */

    result = lm_oper_gemm(true, false,
                          LM_MAT_ONE_VAL, p_mat_houshv, p_mat_a,
                          LM_MAT_ZERO_VAL, p_mat_work);
    LM_RETURN_IF_ERR(result);

    result = lm_oper_gemm(false, false,
                          (-beta), p_mat_houshv, p_mat_work,
                          LM_MAT_ONE_VAL, p_mat_a);

    return result;
}

/**
 * lm_qr_decomp - Function to compute the QR decomposition of given matrix.
 *
 * @note
 *
 *      Please note that this function computes and generates Q and R in
 *      implicit format.
 *
 *      Reference:
 *          - 5.2.2 Householder QR, "Matrix Computation 4th edition" written
 *            by Golub and Van Loan
 *
 * @param   [in,out]    *p_mat_qr       Handle of given matrix.
 *
 *      On entry:
 *          The QR matrix contains the original data to be factored.
 *
 *      On exit:
 *          The implicit Q (Householder vectors) is stored in lower
 *          triangular part of QR matrix (expect the main diagonal).
 *          the R is stored in upper triangular part of QR matrix.
 *
 *         Given matrix                              Implicit Q and R
 *        -           -                             -                -
 *        | a a a a a |                             |  r  r  r  r  r |
 *        | a a a a a |        QR decompose         | v1  r  r  r  r |
 *        | a a a a a |       ==============>       | v1 v2  r  r  r |
 *        | a a a a a |                             | v1 v2 v3  r  r |
 *        | a a a a a |                             | v1 v2 v3 v4  r |
 *        -           -                             -                -
 *
 *      The given matrix QR can be a square or non-square M by N matrix.
 *
 * @param   [out]       *p_mat_beta     Handle of beta vector.
 *
 *      On entry:
 *          The size of the beta vector should be M by 1, where M is equal
 *          to row size of the given matrix. This vector is needed for storing
 *          the beta value corresponding to each Householder vector.
 *
 *      On exit:
 *          The beta values are stored in this vector. these beta values are
 *          needed for converting implicit QR to explicit QR.
 *
 * @param   [in,out]    *p_mat_work     Handle of working matrix needed for completing QR
 *                                      decomposition process, its output data is not meaningful.
 *                                      The size of working matrix should be 1 by N, where
 *                                      N is equal to column size of the given matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_qr_decomp(lm_mat_t *p_mat_qr,
                      lm_mat_t *p_mat_beta,
                      lm_mat_t *p_mat_work)
{
    lm_rtn_t result;
    lm_mat_elem_t alpha;
    lm_mat_elem_size_t dim_idx;

    const lm_mat_dim_size_t r_size_qr = LM_MAT_GET_R_SIZE(p_mat_qr);
    const lm_mat_dim_size_t c_size_qr = LM_MAT_GET_C_SIZE(p_mat_qr);
    const lm_mat_elem_size_t dim_size_qr = LM_MIN(r_size_qr, c_size_qr);

    lm_mat_elem_t *p_elem_beta = LM_MAT_GET_ELEM_PTR(p_mat_beta);
    const lm_mat_dim_size_t r_size_beta = LM_MAT_GET_R_SIZE(p_mat_beta);
    const lm_mat_dim_size_t c_size_beta = LM_MAT_GET_C_SIZE(p_mat_beta);

    const lm_mat_dim_size_t r_size_work = LM_MAT_GET_R_SIZE(p_mat_work);
    const lm_mat_dim_size_t c_size_work = LM_MAT_GET_C_SIZE(p_mat_work);

    lm_mat_t mat_qr_subm_shaped = {0};
    lm_mat_t mat_housh_vec_shaped = {0};
    lm_mat_t mat_work_shaped = {0};

    if (r_size_qr == 0 || c_size_qr == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    /*
     * A M, N by 1 matrix is required for storing the Householder
     * reflection beta (scalar factor) value
     */
    if (r_size_beta != r_size_qr || c_size_beta != 1) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    /* A 1 x N matrix (work space) is required for QR calculation */
    if (r_size_work != 1 || c_size_work != c_size_qr) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    /* Nothing to do for 1 by N matrix */
    if (r_size_qr == 1) {

        p_elem_beta[0] = LM_MAT_ZERO_VAL;

        return LM_ERR_CODE(LM_SUCCESS);
    }

    for (dim_idx = 0; dim_idx < dim_size_qr; dim_idx++) {

        /* Get the sub-column vector that needs to be reflected */
        result = lm_shape_submatrix(p_mat_qr,
                                    dim_idx, dim_idx,
                                    (r_size_qr - dim_idx), 1,
                                    &mat_housh_vec_shaped);
        LM_RETURN_IF_ERR(result);

        /* Calculate the Householder reflection vector and the beta value */
        result = lm_qr_housh_v(&mat_housh_vec_shaped, &alpha, &(p_elem_beta[0]));
        LM_RETURN_IF_ERR(result);

        /* Do the reflection on QR sub-matrix if needed */
        if (p_elem_beta[0] != LM_MAT_ZERO_VAL) {

            /* Setup QR sub-matrix */
            result = lm_shape_submatrix(p_mat_qr,
                                        dim_idx,
                                        dim_idx + 1,
                                        (r_size_qr - dim_idx),
                                        (lm_mat_dim_size_t)(c_size_qr - dim_idx - 1),
                                        &mat_qr_subm_shaped);
            LM_RETURN_IF_ERR(result);

            /* Setup the work matrix */
            result = lm_shape_submatrix(p_mat_work,
                                        0, 0,
                                        1, (lm_mat_dim_size_t)(c_size_qr - dim_idx - 1),
                                        &mat_work_shaped);
            LM_RETURN_IF_ERR(result);

            result = lm_qr_housh_refl(&mat_qr_subm_shaped, p_elem_beta[0],
                                      &mat_housh_vec_shaped, &mat_work_shaped);
            LM_RETURN_IF_ERR(result);

            /* Store the alpha in upper triangular of QR matrix */
            mat_housh_vec_shaped.elem.ptr[0] = (-alpha);
        }

        LM_MAT_TO_NXT_ELEM(p_elem_beta, p_mat_beta);
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_qr_explicit - Function to convert implicit QR to explicit QR.
 *
 * @param   [in,out]    *p_mat_qr       Handle of given matrix.
 *
 *      On entry:
 *          The QR matrix contains implicit QR.
 *
 *      On exit:
 *          The explicit R (upper triangular) is stored in this matrix.
 *
 *            Implicit QR                    Explicit Q             Explicit R
 *        -                -             -                -     -                -
 *        |  r  r  r  r  r |             |  q  q  q  q  q |     |  r  r  r  r  r |
 *        | v1  r  r  r  r |   convert   |  q  q  q  q  q |     |  0  r  r  r  r |
 *        | v1 v2  r  r  r | ==========> |  q  q  q  q  q | and |  0  0  r  r  r |
 *        | v1 v2 v3  r  r |             |  q  q  q  q  q |     |  0  0  0  r  r |
 *        | v1 v2 v3 v4  r |             |  q  q  q  q  q |     |  0  0  0  0  r |
 *        -                -             -                -     -                -
 *
 * @param   [in]       *p_mat_beta      Handle of beta vector.
 *
 *      On entry:
 *          This beta vector should contains the beta values generated
 *          during QR decomposition procedure. The size of this beta
 *          vector should be M by 1, where M is equal to row size of the
 *          given implicit QR matrix.
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
 * @param   [in,out]    *p_mat_work     Handle of working matrix needed for completing
 *                                      implicit /explicit QR conversion process, its
 *                                      output data is not meaningful. The size of working
 *                                      matrix should be 1 by M, where M is equal to row
 *                                      size of the given matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_qr_explicit(lm_mat_t *p_mat_qr,
                        const lm_mat_t *p_mat_beta,
                        lm_mat_t *p_mat_q,
                        lm_mat_t *p_mat_work)
{
    lm_rtn_t result;
    lm_mat_elem_t v1_tmp;
    lm_mat_elem_size_t dim_idx;

    const lm_mat_dim_size_t r_size_qr = LM_MAT_GET_R_SIZE(p_mat_qr);
    const lm_mat_dim_size_t c_size_qr = LM_MAT_GET_C_SIZE(p_mat_qr);
    const lm_mat_elem_size_t dim_size_qr = LM_MIN(r_size_qr, c_size_qr);

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

    if (r_size_qr == 0 || c_size_qr == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    /* A M by M square matrix is required for storing the Householder orthogonal matrix Q */
    if (r_size_q != r_size_qr || c_size_q != r_size_qr) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    /* The Householder beta list should contains M elements (M by 1) */
    if (r_size_beta != r_size_qr || c_size_beta != 1) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    /* A 1 x M matrix (work space) is required for explicit QR transformation */
    if (r_size_work != 1 || c_size_work != r_size_qr) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    result = lm_oper_identity(p_mat_q);
    LM_RETURN_IF_ERR(result);

    for (dim_idx = 0; dim_idx < dim_size_qr; dim_idx++) {

        /* Do the reflection on QR sub-matrix if needed */
        if (p_elem_beta[0] != LM_MAT_ZERO_VAL) {

            /* Get the Householder reflection vector from QR matrix*/
            result = lm_shape_submatrix(p_mat_qr,
                                        dim_idx, dim_idx,
                                        (r_size_qr - dim_idx), 1,
                                        &mat_housh_vec_shaped);
            LM_RETURN_IF_ERR(result);

            /* Setup Q sub-matrix that needs to be reflected */
            result = lm_shape_submatrix(p_mat_q,
                                        dim_idx, 0,
                                        (r_size_q - dim_idx), c_size_q,
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
             * and change the value of first element to one for completing
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

    /* Clear the lower triangular (except the diagonal) of QR matrix and output R */
    if (r_size_qr > 1) {
        result = lm_oper_zeros_tril(p_mat_qr, -1);
        LM_RETURN_IF_ERR(result);
    }

    return LM_ERR_CODE(LM_SUCCESS);
}


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */

