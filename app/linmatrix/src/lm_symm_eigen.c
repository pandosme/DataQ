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
 * @file    lm_symm_eigen.c
 * @brief   Lin matrix eigenvalue and eigenvector functions for symmetric matrix
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#include <math.h>

#include "lm_symm_eigen.h"
#include "lm_mat.h"
#include "lm_err.h"
#include "lm_shape.h"
#include "lm_oper.h"
#include "lm_oper_gemm.h"
#include "lm_chk.h"


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

static lm_rtn_t lm_symm_eigen_qr_step(lm_mat_t *p_mat_d,
                                      lm_mat_t *p_mat_sd,
                                      lm_mat_t *p_mat_q);


/*
 *******************************************************************************
 * Public functions
 *******************************************************************************
 */

/**
 * lm_symm_eigen - Function to compute the eigenvalues and eigenvectors of
 *                 Hessenberg similar matrix (tridiagonal form).
 *
 * @note
 *
 *      This function is designed for computing the eigenvalues and
 *      eigenvectors of Hessenberg similar matrix (tridiagonal form)
 *      satisfies the following formula:
 *
 *          T = Q * D * Q',
 *
 *      Where:
 *          Q is the orthogonal similarity transformation matrix.
 *          D is the diagonal similar matrix of A.
 *
 *      Reference:
 *          - 8.3 The Symmetric QR Algorithm, "Matrix Computation 4th
 *            edition" written by Golub and Van Loan.
 *
 * @param   [in,out]    *p_mat_d        Handle of row vector.
 *
 *      On entry:
 *          This row vector contains the main diagonal elements of
 *          Hessenberg similar matrix (tridiagonal form). The size
 *          of row vector should be M by 1, where M is equal to row
 *          size of the Hessenberg matrix.
 *
 *      On exit:
 *          The eigenvalues of given Hessenberg matrix are stored in
 *          this row vector in ascending order.
 *
 *               Complete                        Main diagonal
 *          Hessenberg matrix                    in row vector
 *          -                -                       -   -
 *          |  d sd  0  0  0 |                       | d |
 *          | sd  d sd  0  0 |          to           | d |
 *          |  0 sd  d sd  0 |    ==============>    | d |
 *          |  0  0 sd  d sd |                       | d |
 *          |  0  0  0 sd  d |                       | d |
 *          -                -                       -   -
 *
 * @param   [in,out]    *p_mat_sd       Handle of row vector.
 *
 *      On entry:
 *          This row vector contains the subdiagonal elements of
 *          Hessenberg similar matrix (tridiagonal form). The size
 *          of row vector should be M by 1, where M is equal to row
 *          size of the Hessenberg matrix. Set this argument to NULL
 *          if M is equal to 1.
 *
 *      On exit:
 *          The value of elements are changed and these values are
 *          not meaningful.
 *
 *               Complete                        Main diagonal
 *          Hessenberg matrix                    in row vector
 *          -                -                       -   -
 *          |  d sd  0  0  0 |                       | sd |
 *          | sd  d sd  0  0 |          to           | sd |
 *          |  0 sd  d sd  0 |    ==============>    | sd |
 *          |  0  0 sd  d sd |                       | sd |
 *          |  0  0  0 sd  d |                       -    -
 *          -                -
 *
 * @param   [out]       *p_mat_q        Handle of matrix Q.
 *
 *      On entry:
 *          The size of matrix Q should be M by M, where M is equal to
 *          row size of Hessenberg similar matrix. Set this argument to
 *          NULL if the eigenvectors are not needed.
 *
 *      On exit:
 *          The eigenvectors of given Hessenberg similar matrix are stored
 *          in matrix Q.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_symm_eigen(lm_mat_t *p_mat_d,
                       lm_mat_t *p_mat_sd,
                       lm_mat_t *p_mat_q)
{
    uint32_t iter_per_cyc;

    lm_rtn_t result;
    lm_mat_elem_size_t c_idx;

    const lm_mat_dim_size_t r_size_d = LM_MAT_GET_R_SIZE(p_mat_d);
    const lm_mat_dim_size_t c_size_d = LM_MAT_GET_C_SIZE(p_mat_d);
    const lm_mat_elem_size_t nxt_r_osf_d = LM_MAT_GET_NXT_OFS(p_mat_d);

    lm_mat_dim_size_t idx_i;
    lm_mat_dim_size_t idx_j;
    lm_mat_dim_size_t idx_min;

    lm_mat_elem_t eigen_val_min;
    lm_mat_elem_t eigen_val_tmp;

    lm_mat_elem_t *p_elem_sd_tail;
    lm_mat_t mat_d_shaped;
    lm_mat_t mat_sd_shaped;
    lm_mat_t mat_q_shaped;
    lm_mat_t *p_mat_q_shaped = NULL;

    /*
     * A M by 1 matrix is required for storing the main diagonal
     * input and eigenvalue output
     */
    if (r_size_d == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    if (c_size_d != 1) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    /*
     * A (M - 1) by 1 matrix is required for storing the sub-diagonal
     * input. if M is equal to one, then no need to input sub-diagonal.
     */
    if (r_size_d > 1) {

        if (p_mat_sd == NULL) {
            return LM_ERR_CODE(LM_ERR_NULL_PTR);
        }

        if (LM_MAT_GET_R_SIZE(p_mat_sd) != (r_size_d - 1)
            || LM_MAT_GET_C_SIZE(p_mat_sd) != c_size_d) {

            return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
        }
    }

    /*
     * Check the dimension of Q matrix (if provided).
     */
    if (p_mat_q != NULL) {

        /* The dimension of Q matrix should equal to M by M */
        if (LM_MAT_GET_R_SIZE(p_mat_q) != r_size_d
            || LM_MAT_GET_C_SIZE(p_mat_q) != r_size_d) {

            return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
        }
    }

    if (r_size_d == 1) {
        return LM_ERR_CODE(LM_SUCCESS);
    }

    mat_d_shaped = *p_mat_d;
    mat_sd_shaped = *p_mat_sd;

    if (p_mat_q != NULL) {
        mat_q_shaped = *p_mat_q;
        p_mat_q_shaped = &mat_q_shaped;
    }
    else {
        p_mat_q_shaped = NULL;
    }

    /* Francis QR algorithm */
    for (c_idx = (r_size_d - 1); c_idx > 0; c_idx--) {

        p_elem_sd_tail = LM_MAT_GET_ROW_PTR(&mat_sd_shaped,
                                            mat_sd_shaped.elem.nxt_r_osf,
                                            (mat_sd_shaped.elem.dim.r - 1));

        iter_per_cyc = 0;

        /*
         * QR iteration:
         *      Break this loop if the value of last sub-diagonal
         *      is very close enough to zero.
         */
        while ((fabs(p_elem_sd_tail[0]) > LM_MAT_EIGEN_TOLERANCE)) {

            result = lm_symm_eigen_qr_step(&mat_d_shaped,
                                           &mat_sd_shaped,
                                           p_mat_q_shaped);
            LM_RETURN_IF_ERR(result);

            iter_per_cyc++;

            /*
             * Stop and return an error if the value of last
             * sub-diagonal cannot be converging to "close zero"
             * within maximum iteration limitation
             */
            if (iter_per_cyc >= LM_MAT_EIGEN_MAX_ITER_PER_CYC) {

                return LM_ERR_CODE(LM_ERR_MAT_EXCEEDED_MAX_ITERATION);
            }
        }

        /* Deflation */
        if (c_idx > 1) {

            result = lm_shape_submatrix(&mat_d_shaped, 0, 0,
                                        (mat_d_shaped.elem.dim.r - 1), 1,
                                        &mat_d_shaped);
            LM_RETURN_IF_ERR(result);

            result = lm_shape_submatrix(&mat_sd_shaped, 0, 0,
                                        (mat_sd_shaped.elem.dim.r - 1), 1,
                                        &mat_sd_shaped);
            LM_RETURN_IF_ERR(result);
        }
    }

    /* Selection sort (ascending order) */
    for (idx_i = 0; idx_i < (r_size_d - 1); idx_i++) {

        eigen_val_min = *(LM_MAT_GET_ROW_PTR(p_mat_d, nxt_r_osf_d, idx_i));
        idx_min = idx_i;

        for (idx_j = (idx_i + 1); idx_j < r_size_d; idx_j++) {

            eigen_val_tmp = *(LM_MAT_GET_ROW_PTR(p_mat_d, nxt_r_osf_d, idx_j));

            if (eigen_val_tmp < eigen_val_min) {
                eigen_val_min = eigen_val_tmp;
                idx_min = idx_j;
            }

        }

        if (idx_min != idx_i) {
            result = lm_oper_swap_row(p_mat_d, idx_min, idx_i);
            LM_RETURN_IF_ERR(result);

            if (p_mat_q != NULL) {
                result = lm_oper_swap_col(p_mat_q, idx_min, idx_i);
                LM_RETURN_IF_ERR(result);
            }
        }
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_symm_eigen_sqrtm - Function to compute the square root of given
 *                       diagonal similar matrix and perform reverse
 *                       similarity transformation.
 *
 * @note
 *
 *      This function is designed for finding the square root of a
 *      matrix indirectly according to following method:
 *
 *      If matrix A is diagonalizable, then:
 *
 *          A = Q * D * inv(Q),
 *
 *      And the square root of A is equal to:
 *
 *          sqrtm(A) = Q * sqrtm(D) * inv(Q)
 *                   = Q * sqrtm(D) * Q'
 *
 *      Where:
 *          A is the original matrix.
 *          Q is the orthogonal similarity transformation matrix (eigenvectors).
 *          D is the diagonal similar matrix of A (eigenvalues).
 *
 *      Since D is a diagonal matrix, its square root can be found
 *      easily because:
 *
 *                            -          -     -                            -
 *                            | d1  0  0 |     | sqrt(d1)    0        0     |
 *          sqrtm(D) = sqrtm( |  0 d2  0 | ) = |    0     sqrt(d2)    0     |
 *                            |  0  0 d3 |     |    0        0     sqrt(d3) |
 *                            -          -     -                            -
 *
 *      After the calculation of sqrtm(D) is completed, then the sqrtm(A)
 *      can be obtained by performing the reverse similarity transformation
 *      Q * sqrtm(D) * inv(Q).
 *
 *      Reference:
 *          - https://en.wikipedia.org/wiki/Square_root_of_a_matrix
 *
 * @param   [in]        *p_mat_q        Handle of matrix Q.
 *
 *      On entry:
 *          Matrix Q contains the eigenvectors (orthogonal similarity
 *          transformation matrix) of given diagonal similar matrix.
 *          The matrix Q should be a M by M square matrix.
 *
 * @param   [in,out]    *p_mat_a        Handle of matrix A.
 *
 *      On entry:
 *          Matrix A contains the data of diagonal similar
 *          matrix D (eigenvalues).
 *
 *          The matrix A should be a M by M square matrix.
 *
 *                     -          -
 *                     | d1  0  0 |
 *          Matrix A = |  0 d2  0 |
 *                     |  0  0 d3 |
 *                     -          -
 *
 *      On exit:
 *          The result of Q * sqrtm(D) * inv(Q) is stored in this matrix.
 *
 *                     -       -   -                            -   -       -'
 *                     | q q q |   | sqrt(d1)    0        0     |   | q q q |
 *          Matrix A = | q q q | x |    0     sqrt(d2)    0     | x | q q q |
 *                     | q q q |   |    0        0     sqrt(d3) |   | q q q |
 *                     -       -   -                            -   -       -
 *
 * @param   [in,out]    *p_mat_work     Handle of working matrix needed for
 *                                      completing square root and similarity
 *                                      transformation process, its output data
 *                                      is not meaningful. The size of working
 *                                      matrix should be M by M, where M is equal
 *                                      to row size of the matrix Q and A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_symm_eigen_sqrtm(const lm_mat_t *p_mat_q,
                             lm_mat_t *p_mat_a,
                             lm_mat_t *p_mat_work)
{
    lm_rtn_t result;
    lm_mat_elem_size_t dim_idx;
    lm_mat_t mat_vec_shaped = {0};
    lm_mat_elem_t tol_around_zero = LM_MAT_ZERO_VAL;

    lm_mat_elem_t *p_elem_a = LM_MAT_GET_ELEM_PTR(p_mat_a);
    const lm_mat_dim_size_t dim = LM_MIN(LM_MAT_GET_R_SIZE(p_mat_a),
                                         LM_MAT_GET_C_SIZE(p_mat_a));
    const lm_mat_elem_size_t nxt_r_osf_a = (LM_MAT_GET_NXT_OFS(p_mat_a) + 1);

    /*
     *  Compute the acceptable tolerance around zero to identify
     *  if input matrix is positive or semi-positive definite.
     *
     *  tol = max(size(A)) * eps(max(eigenvalues))
     *
     *  The above formula is equivalent to:
     *      max(size(A)) * eps(norm(A, 2))
     *
     *  Positive definite:      all eigenvalues > tol
     *  Semi-positive definite: all eigenvalues >= tol
     *
     */

    /* Get the maximum eigenvalue */
    tol_around_zero = *(LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, (dim - 1)));

    /* Get the corresponding machine eps */
    result = lm_chk_machine_eps(&tol_around_zero);
    LM_RETURN_IF_ERR(result);

    /* Compute tolerance for later use */
    tol_around_zero = dim * tol_around_zero;

    result = lm_oper_copy(p_mat_q, p_mat_work);
    LM_RETURN_IF_ERR(result);

    /*
     * Step 1:
     *      Calculate TMP = Q * sqrt(lambda)
     */
    for (dim_idx = 0; dim_idx < dim; dim_idx++) {

        result = lm_shape_col_vect(p_mat_work, dim_idx, &mat_vec_shaped);
        LM_RETURN_IF_ERR(result);

        /* Calculate Q[:,dim_idx] = sqrt(A[dim_idx, dim_idx]) * Q[:,dim_idx] */
        if (p_elem_a[0] == LM_MAT_ZERO_VAL) {

            result = lm_oper_scalar(&mat_vec_shaped,
                                    LM_MAT_ZERO_VAL);

            LM_RETURN_IF_ERR(result);
        }
        else if (p_elem_a[0] > LM_MAT_ZERO_VAL) {

            result = lm_oper_scalar(&mat_vec_shaped,
                                    (lm_mat_elem_t)sqrt(p_elem_a[0]));

            LM_RETURN_IF_ERR(result);
        }

        /* For negative eigenvalue */
        else {
            /**
             *
             *  Negative eigenvalues are not allowed because this library
             *  does't support complex number calculation.
             *
             *  However, if the negative eigenvalue is close to zero within
             *  specific tolerance, then we assume that the the input matrix
             *  is positive or semi-positive definite, the negative eigenvalue
             *  may be caused by the rounding error so we can treat the negative
             *  eigenvalue as zero.
             *
             *  @ref
             *
             *      https://math.stackexchange.com/questions/909814/how-to-avoid-
             *      complex-value-for-square-root-of-a-symmetric-matrix
             *
             *      https://math.stackexchange.com/questions/1873559/which-non-
             *      negative-matrices-have-negative-eigenvalues
             *
             *      https://www.mathworks.com/help/matlab/math/determine-whether-
             *      matrix-is-positive-definite.html
             *
             */
            if (p_elem_a[0] > (-tol_around_zero)) {
                result = lm_oper_scalar(&mat_vec_shaped,
                                        LM_MAT_ZERO_VAL);

                LM_RETURN_IF_ERR(result);
            }
            else {
                return LM_ERR_CODE(LM_ERR_MAT_ELEM_VALUE_NEGATIVE);
            }
        }

        LM_MAT_TO_NXT_ROW(p_elem_a, nxt_r_osf_a, p_mat_a);
    }

    /*
     * Step 2:
     *      Calculate Q * sqrt(lambda) * Q' = TMP * Q',
     *      store the result in matrix A
     */
    result = lm_oper_gemm(false, true,
                          LM_MAT_ONE_VAL, p_mat_work, p_mat_q,
                          LM_MAT_ZERO_VAL, p_mat_a);
    LM_RETURN_IF_ERR(result);

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_symm_eigen_similar_tf - Function to compute similarity transformation,
 *                            for given diagonal and transformation matrix.
 *
 * @note
 *
 *      This function is designed for computing:
 *
 *          A := Q * D * Q',
 *
 *      Where:
 *          Q is the orthogonal similarity transformation matrix.
 *          D is the diagonal similar matrix of A.
 *
 * @param   [in]        *p_mat_q        Handle of matrix Q.
 *
 *      On entry:
 *          Matrix Q contains the eigenvectors (orthogonal similarity
 *          transformation matrix) of given diagonal similar matrix.
 *          The matrix Q should be a M by M square matrix.
 *
 * @param   [in,out]    *p_mat_a       Handle of matrix A.
 *
 *      On entry:
 *          Matrix A contains the data of diagonal similar matrix D.
 *          The matrix A should be a M by M square matrix.
 *
 *                     -          -
 *                     | d1  0  0 |
 *          Matrix A = |  0 d2  0 |
 *                     |  0  0 d3 |
 *                     -          -
 *
 *      On exit:
 *          The result of Q * sqrtm(D) * inv(Q) is stored in this matrix.
 *
 *                     -       -   -          -   -       -'
 *                     | q q q |   | d1  0  0 |   | q q q |
 *          Matrix A = | q q q | x |  0 d2  0 | x | q q q |
 *                     | q q q |   |  0  0 d3 |   | q q q |
 *                     -       -   -          -   -       -
 *
 * @param   [in,out]    *p_mat_work     Handle of working matrix needed for
 *                                      completing square root and similarity
 *                                      transformation process, its output data
 *                                      is not meaningful. The size of working
 *                                      matrix should be M by M, where M is equal
 *                                      to row size of the matrix Q and A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_symm_eigen_similar_tf(const lm_mat_t *p_mat_q,
                                  lm_mat_t *p_mat_a,
                                  lm_mat_t *p_mat_work)
{
    lm_rtn_t result;
    lm_mat_elem_size_t dim_idx;
    lm_mat_t mat_vec_shaped = {0};

    lm_mat_elem_t *p_elem_a = LM_MAT_GET_ELEM_PTR(p_mat_a);
    const lm_mat_dim_size_t dim = LM_MIN(LM_MAT_GET_R_SIZE(p_mat_a),
                                         LM_MAT_GET_C_SIZE(p_mat_a));
    const lm_mat_elem_size_t nxt_r_osf_a = (LM_MAT_GET_NXT_OFS(p_mat_a) + 1);

    result = lm_oper_copy(p_mat_q, p_mat_work);
    LM_RETURN_IF_ERR(result);

    /* Calculate lambda TMP = Q * D */
    for (dim_idx = 0; dim_idx < dim; dim_idx++) {

        result = lm_shape_col_vect(p_mat_work, dim_idx, &mat_vec_shaped);
        LM_RETURN_IF_ERR(result);

        /* Calculate Q[:,dim_idx] = A[dim_idx, dim_idx] * Q[:,dim_idx] */
        result = lm_oper_scalar(&mat_vec_shaped, p_elem_a[0]);
        LM_RETURN_IF_ERR(result);

        LM_MAT_TO_NXT_ROW(p_elem_a, nxt_r_osf_a, p_mat_a);
    }

    /* Calculate Q * D * Q' = TMP * Q', store the result in matrix A */
    result = lm_oper_gemm(false, true,
                          LM_MAT_ONE_VAL, p_mat_work, p_mat_q,
                          LM_MAT_ZERO_VAL, p_mat_a);
    LM_RETURN_IF_ERR(result);

    return LM_ERR_CODE(LM_SUCCESS);
}


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */

/**
 * function_example - Function example
 *
 * @param   [in]    input       Example input.
 * @param   [out]   *p_output   Example output.
 *
 * @return  [int]   Function executing result.
 * @retval  [0]     Success.
 * @retval  [-1]    Fail.
 *
 */
static lm_rtn_t lm_symm_eigen_qr_step(lm_mat_t *p_mat_d,
                                      lm_mat_t *p_mat_sd,
                                      lm_mat_t *p_mat_q)
{
    lm_rtn_t result;
    lm_mat_elem_size_t dim_idx;
    lm_mat_elem_size_t q_r_idx;

    const lm_mat_dim_size_t r_size_d = LM_MAT_GET_R_SIZE(p_mat_d);
    const lm_mat_elem_size_t nxt_r_osf_d = LM_MAT_GET_NXT_OFS(p_mat_d);

    const lm_mat_dim_size_t r_size_sd = LM_MAT_GET_R_SIZE(p_mat_sd);
    const lm_mat_elem_size_t nxt_r_osf_sd = LM_MAT_GET_NXT_OFS(p_mat_sd);

    lm_mat_elem_t *p_q_x;
    lm_mat_elem_t *p_q_y;
    lm_mat_elem_t q_x_tmp;
    lm_mat_elem_size_t r_size_q;
    lm_mat_elem_size_t nxt_r_osf_q;

    lm_mat_elem_t *p_d1;
    lm_mat_elem_t *p_d2;
    lm_mat_elem_t *p_sd1;
    lm_mat_elem_t *p_sd2;
    lm_mat_elem_t sin_theta;
    lm_mat_elem_t cos_theta;
    lm_mat_elem_t d1;
    lm_mat_elem_t d2;
    lm_mat_elem_t sd1;
    lm_mat_elem_t sd2;
    lm_mat_elem_t delta;
    lm_mat_elem_t mu;
    lm_mat_elem_t x;
    lm_mat_elem_t z;

    LM_ASSERT_DBG((r_size_d >= 2), "Diagonal size should equal to or larger than 2");
    LM_ASSERT_DBG((r_size_sd >= 1), "Sub diagonal size should equal to or larger than 1");

    /*
     * Wilkinson shift for symmetric matrix
     * [ x   x   x   x   x   x
     *   x   x   x   x   x   x
     *   x   x   x   x   x   x
     *   x   x   x   x   x   x
     *   x   x   x   x  d1 sd1
     *   x   x   x   x sd1  d2]
     *
     * delta = (d1 - d2) / 2
     * mu = d2 - (sd1 ^ 2) / (delta + sign(delta) * sqrt(delta ^ 2 + sd1 ^ 2))
     */

    d1 = *(LM_MAT_GET_ROW_PTR(p_mat_d, nxt_r_osf_d, (r_size_d - 2)));
    d2 = *(LM_MAT_GET_ROW_PTR(p_mat_d, nxt_r_osf_d, (r_size_d - 1)));
    sd1 = *(LM_MAT_GET_ROW_PTR(p_mat_sd, nxt_r_osf_sd, (r_size_sd - 1)));
    delta = (d1 - d2) / 2;

    LM_ASSERT_DBG(((delta != 0.0) || (sd1 != 0.0)), "Divisor should not be zero");

    mu = (lm_mat_elem_t)(d2 - (sd1 * sd1) / (delta + LM_SIGN(delta)
                                             * sqrt(delta * delta + sd1 * sd1)));

    x = *(LM_MAT_GET_ROW_PTR(p_mat_d, nxt_r_osf_d, 0)) - mu;
    z = *(LM_MAT_GET_ROW_PTR(p_mat_sd, nxt_r_osf_sd, 0));

    for (dim_idx = 0; dim_idx < (r_size_d - 1); dim_idx++) {

        /*
         *
         * The method for computing the QR decomposition and R * Q is based on
         * implicit symmetric QR step with Wilkinson shift algorithm.
         *
         *
         * Step 1:
         *
         * In this step, we will update the first and second element on the diagonal,
         * also the first and second element of sub-diagonal
         *
         * The original tridiagonal matrix                               R
         *   -                    -                           -                    -
         *   |  X  X  0  0  0  0  |                           |  R  R  t  0  0  0  |
         *   |  X  X  X  0  0  0  |          Rotate           |  0  R  R  0  0  0  |
         *   |  0  X  X  X  0  0  |      => the first =>      |  0  X  X  X  0  0  |
         *   |  0  0  X  X  X  0  |          2 rows           |  0  0  X  X  X  0  |
         *   |  0  0  0  X  X  X  |       (QR decompose)      |  0  0  0  X  X  X  |
         *   |  0  0  0  0  X  X  |                           |  0  0  0  0  X  X  |
         *   -                    -                           -                    -
         *
         *
         * Step 2: R * Q
         * In this step, we will only update the first and second element on the diagonal,
         * and the first element of sub-diagonal, no needs to calculate the rest elements
         * in these 2 columns is because that [+ R]' = [+ R] (symmetrical).
         *
         * We will also update the x and z for next Givens iteration.
         *
         *             R                                               R * Q
         *   -                    -                            -                    -
         *   |  R  R  R  0  0  0  |                            |  R  R  t  0  0  0  |
         *   |  0  R  R  0  0  0  |           Apply            |  R  R  R  0  0  0  |
         *   |  0  X  X  X  0  0  |      => Q to first =>      |  t  R  X  X  0  0  |
         *   |  0  0  X  X  X  0  |          2 columns         |  0  0  X  X  X  0  |
         *   |  0  0  0  X  X  X  |                            |  0  0  0  X  X  X  |
         *   |  0  0  0  0  X  X  |                            |  0  0  0  0  X  X  |
         *   -                    -                            -                    -
         *
         */

        result = lm_oper_givens(x, z, &sin_theta, &cos_theta);
        LM_RETURN_IF_ERR(result);

        /*
         * Step 3: Get rid of unwanted element and correct the element of sub-diagonal
         *
         * For the 2 ~ Nth iteration, we need to perform one more Givens rotation
         * to the [R t]' which are generated in previous iteration. The [R t]' now
         * will be rotated to [new_R 0]', and the new_R should be store in the
         * sub-diagonal buffer.
         *
         *      The R generated                                     Get rid of
         *   in previous iteration                               unwanted element t
         *   -                    -                            -                    -
         *   |  R  R  t  0  0  0  |                            |  R nR  0  0  0  0  |
         *   |  R  R  R  0  0  0  |           Apply            | nR  R  R  0  0  0  |
         *   |  t  R  X  X  0  0  |      => Q to first =>      |  0  R  X  X  0  0  |
         *   |  0  0  X  X  X  0  |          2 columns         |  0  0  X  X  X  0  |
         *   |  0  0  0  X  X  X  |                            |  0  0  0  X  X  X  |
         *   |  0  0  0  0  X  X  |                            |  0  0  0  0  X  X  |
         *   -                    -                            -                    -
         *
         */

        if (dim_idx > 0) {

            /* Calculate
             *    -    -     -       -     -   -     -                -
             *    | nR |  =  |  c  s |  *  | R |  =  |  c * R + s * t |
             *    |  0 |     | -s  c |     | t |     | -s * R + c * t |
             *    -    -     -       -     -   -     -                -
             */
             p_sd1[0] = cos_theta * p_sd1[0] + sin_theta * z;

        }

        /*
         * Update the main 2 by 2 matrix, overwrite the first and second element on diagonal,
         * and the first element on sub-diagonal
         * -                  -     -      -     -          -     -      -
         * |  new_d1  new_sd1 |  =  |  c s |  x  |  d1  sd1 |  x  | c -s |
         * | new_sd1   new_d2 |     | -s c |     | sd1   d2 |     | s  c |
         * -                  -     -      -     -          -     -      -
         *                          -      -     -                          -
         *                       =  |  c s |  x  | c*d1+s*sd1   -s*d1+c*sd1 |
         *                          | -s c |     | c*sd1+s*d2   -s*sd1+c*d2 |
         *                          -      -     -                          -
         *                          -                                                                                 -
         *                       =  |  c * (c*d1+s*sd1) + s * (c*sd1+s*d2)      c * (-s*d1+c*sd1) + s * (-s*sd1+c*d2) |
         *                          | -s * (c*d1+s*sd1) + c * (c*sd1+s*d2)     -s * (-s*d1+c*sd1) + c * (-s*sd1+c*d2) |
         *                          -                                                                                 -
         *
         */

        p_d1 = LM_MAT_GET_ROW_PTR(p_mat_d, nxt_r_osf_d, dim_idx);
        p_d2 = LM_MAT_GET_ROW_PTR(p_mat_d, nxt_r_osf_d, (dim_idx + 1));
        p_sd1 = LM_MAT_GET_ROW_PTR(p_mat_sd, nxt_r_osf_sd, dim_idx);

        d1 = p_d1[0];
        d2 = p_d2[0];
        sd1 = p_sd1[0];

        /*
         * In order to maintain the readability of the code, I do not simplify the following code.
         * I hope the compiler will automatically optimize and reduce the following repeated calculations.
         */
        p_d1[0] = cos_theta * (cos_theta * d1 + sin_theta * sd1)
                + sin_theta * (cos_theta * sd1 + sin_theta * d2);
        p_d2[0] = -sin_theta * (-sin_theta * d1 + cos_theta * sd1)
                + cos_theta * (-sin_theta * sd1 + cos_theta * d2);
        p_sd1[0] = -sin_theta * (cos_theta * d1 + sin_theta * sd1)
                 + cos_theta * (cos_theta * sd1 + sin_theta * d2);

        /*
         * Calculate the rest elements [t R]', update the second element on sub-diagonal
         * update the Givens arguments if needed.
         *    -   -     -       -     -   -     -       -
         *    | t |  =  |  c  s |  *  | 0 |  =  | s * X |
         *    | R |     | -s  c |     | X |     | c * X |
         *    -   -     -       -     -   -     -       -
         */
        if (dim_idx < (r_size_d - 2)) {
            p_sd2 = LM_MAT_GET_ROW_PTR(p_mat_sd, nxt_r_osf_sd, (dim_idx + 1));
            sd2 = p_sd2[0];
            p_sd2[0] = cos_theta * sd2;

            x = p_sd1[0];
            z = sin_theta * sd2;
        }

        /* Update orthogonal matrix Q */
        if (p_mat_q != NULL) {

            /*
             * Q(:, dim_idx:dim_idx+1) = Q(:, dim_idx:dim_idx+1) * givens_inv;
             *
             *    -         -     -         -
             *    | x1   y1 |     | x1   y1 |     -      -
             *    | x2   y2 |     | x2   y2 |     | c -s |
             *    |  .    . |  =  |  .    . |  x  |      |
             *    |  .    . |     |  .    . |     | s  c |
             *    | xn   yn |     | xn   yn |     -      -
             *    -         -     -         -
             */

            r_size_q = p_mat_q->elem.dim.r;
            nxt_r_osf_q = p_mat_q->elem.nxt_r_osf;
            p_q_x = LM_MAT_GET_COL_PTR(p_mat_q, 1, dim_idx);
            p_q_y = LM_MAT_GET_COL_PTR(p_mat_q, 1, (dim_idx + 1));

            for (q_r_idx = 0; q_r_idx < r_size_q; q_r_idx++) {

                q_x_tmp = p_q_x[0];
                p_q_x[0] = cos_theta * q_x_tmp + sin_theta * p_q_y[0];
                p_q_y[0] = (-sin_theta) * q_x_tmp + cos_theta * p_q_y[0];

                LM_MAT_TO_NXT_ROW(p_q_x, nxt_r_osf_q, p_mat_q);
                LM_MAT_TO_NXT_ROW(p_q_y, nxt_r_osf_q, p_mat_q);
            }
        }
    }

    return LM_ERR_CODE(LM_SUCCESS);
}
