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
 * @file    lm_lu.c
 * @brief   Lin matrix LU decomposition functions
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#include "lm_lu.h"
#include "lm_mat.h"
#include "lm_err.h"
#include "lm_shape.h"
#include "lm_oper.h"
#include "lm_oper_axpy.h"
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

static lm_rtn_t lm_lu_invert_tril(lm_mat_t *p_mat_lu);
static lm_rtn_t lm_lu_invert_triu(lm_mat_t *p_mat_lu);
static lm_rtn_t lm_lu_triu_mul_tril(lm_mat_t *p_mat_lu);


/*
 *******************************************************************************
 * Public functions
 *******************************************************************************
 */

/**
 * lm_lu_decomp - Function to compute the LU decomposition of given matrix.
 *
 * @note
 *
 *      This function decomposes the given matrix into P * A = L * U
 *      by using partial pivot method. Please refer to section 3.4.2
 *      "partial pivot" of the book "Matrix Computation 4th edition"
 *      written by Golub and Van Loan.
 *
 *      The given matrix LU can be a square or non-square M by N matrix.
 *
 * @param   [in,out]    *p_mat_lu           Handle of matrix LU.
 *
 *      On entry:
 *          The LU matrix contains the original data to be factored.
 *
 *      On exit:
 *          The factors L is a unit lower triangular matrix and all its
 *          elements below main diagonal are stored in lower triangular
 *          part of LU matrix. The factors U is a upper diagonal matrix
 *          and its elements on and above the main diagonal are stored
 *          in upper diagonal part of LU matrix.
 *
 *          Permutation   Original                Decomposed LU
 *           -       -    -       -                 -       -
 *           | P P P |    | A A A |    Decompose    | U U U |
 *           | P P P |    | A A A |        =>       | L U U |
 *           | P P P |    | A A A |                 | L L U |
 *           -       -    -       -                 -       -
 *
 * @param   [out]       *p_perm_p           Handle of permutation list P.
 *
 *      The size of given permutation list P should be 2 * M.
 *      (M is row size of matrix LU)
 *      On exit, the permutation list P contains the permutation information
 *      in cycle notation format.
 *
 * @param   [out]       *p_invert_sgn_det   Integer scalar.
 *
 *      On exit, the integer scalar contains the sign information needed
 *      for calculation of the determinant value of original matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_lu_decomp(lm_mat_t *p_mat_lu,
                      lm_permute_list_t *p_perm_p,
                      int32_t *p_invert_sgn_det)
{
    lm_rtn_t result;
    lm_mat_elem_t mult;
    lm_mat_elem_t max_pivot;
    lm_mat_dim_size_t max_pivot_r_idx;
    lm_mat_dim_size_t max_pivot_c_idx;
    lm_mat_dim_size_t min_dim;
    lm_mat_dim_size_t dim_idx;
    lm_mat_dim_size_t pivot_c_idx;
    lm_mat_dim_size_t pivot_r_idx;
    lm_mat_dim_size_t next_r_idx;
    lm_mat_t mat_lu_subm_shaped;
    lm_mat_t mat_lu_vec_shaped;
    lm_mat_t mat_pivot_row_shaped;
    lm_mat_t mat_next_row_shaped;

    const lm_mat_dim_size_t r_size_lu = LM_MAT_GET_R_SIZE(p_mat_lu);
    const lm_mat_dim_size_t c_size_lu = LM_MAT_GET_C_SIZE(p_mat_lu);
    const lm_mat_elem_size_t nxt_r_osf_lu = LM_MAT_GET_NXT_OFS(p_mat_lu);

    lm_permute_elem_t *p_perm_elem_p = (p_perm_p->elem.ptr);
    lm_permute_elem_t perm_elem_tmp;
    lm_permute_size_t perm_elem_idx;

    min_dim = LM_MIN(r_size_lu, c_size_lu);

    if (min_dim == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    if ((r_size_lu * 2) > p_perm_p->mem.elem_tot) {
        return LM_ERR_CODE(LM_ERR_PM_ONE_LINE_BUFF_TOO_SMALL);
    }

    /* Initialize the row permutation list */
    for (perm_elem_idx = 0; perm_elem_idx < r_size_lu; perm_elem_idx++) {
        p_perm_elem_p[perm_elem_idx] = perm_elem_idx;
    }

    p_perm_p->elem.num = r_size_lu;

    *p_invert_sgn_det = 1;

    if (r_size_lu == 1) {
        return lm_permute_oline_to_cycle(p_perm_p);
    }

    pivot_r_idx = 0;
    pivot_c_idx = 0;

    for (dim_idx = 0; dim_idx < min_dim; dim_idx++) {

        result = lm_shape_submatrix(p_mat_lu, pivot_r_idx, dim_idx,
                                    (r_size_lu - pivot_r_idx), (c_size_lu - dim_idx),
                                    &mat_lu_subm_shaped);
        LM_RETURN_IF_ERR(result);

        /*
         * Partial Pivoting
         * Find out the maximum absolute pivot from current column vector.
         */
        result = lm_shape_submatrix(p_mat_lu, pivot_r_idx, dim_idx,
                                    (r_size_lu - pivot_r_idx), 1,
                                    &mat_lu_vec_shaped);
        LM_RETURN_IF_ERR(result);

        result = lm_oper_max_abs(&mat_lu_vec_shaped,
                                 &max_pivot_r_idx,
                                 &max_pivot_c_idx,
                                 &max_pivot);
        LM_RETURN_IF_ERR(result);

        /* Found non-zero pivot */
        if (max_pivot != LM_MAT_ZERO_VAL) {

            /* Swap the rows if needed */
            if (pivot_r_idx != (pivot_r_idx + max_pivot_r_idx)) {
                /*
                 * Permute the row vectors so that
                 * the maximum absolute pivot is at A[m][m]
                 */
                result = lm_oper_swap_row(p_mat_lu,
                                          pivot_r_idx,
                                          (pivot_r_idx + max_pivot_r_idx));
                LM_RETURN_IF_ERR(result);

                /*
                 * Swap the elements of "one-line" notation
                 */
                LM_PERMUTE_SWAP_OLINE_ELEM(p_perm_elem_p[pivot_r_idx],
                                           p_perm_elem_p[(pivot_r_idx + max_pivot_r_idx)],
                                           perm_elem_tmp);

                /* Row swap changing sign of determinant */
                *p_invert_sgn_det = (-(*p_invert_sgn_det));
            }

            result = lm_shape_row_vect(&mat_lu_subm_shaped, 0, &mat_pivot_row_shaped);
            LM_RETURN_IF_ERR(result);

            for (next_r_idx = 1; next_r_idx < LM_MAT_GET_R_SIZE(&mat_lu_subm_shaped); next_r_idx++) {

                result = lm_shape_row_vect(&mat_lu_subm_shaped, next_r_idx, &mat_next_row_shaped);
                LM_RETURN_IF_ERR(result);

                /* Perform Gaussian elimination when needed */
                if (mat_next_row_shaped.elem.ptr[0] != LM_MAT_ZERO_VAL) {

                    mult = mat_next_row_shaped.elem.ptr[0]
                         / mat_pivot_row_shaped.elem.ptr[0];

                    result = lm_oper_axpy((-mult), &mat_pivot_row_shaped, &mat_next_row_shaped);
                    LM_RETURN_IF_ERR(result);
                }
                else {

                    mult = LM_MAT_ZERO_VAL;
                }

                /* Store the multiplier in lower triangular portion (L) of output matrix */
                *(LM_MAT_GET_ROW_PTR(p_mat_lu, nxt_r_osf_lu,
                                     (pivot_r_idx + next_r_idx)) + pivot_c_idx) = mult;
            }

            pivot_r_idx++;
            pivot_c_idx++;

        }
    }

    result = lm_permute_oline_to_cycle(p_perm_p);
    LM_RETURN_IF_ERR(result);

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_lu_det - Function to calculate the determinant of given matrix
 *             that has been decomposed into LU.
 *
 * @note
 *
 *      This function calculates the determinant of given LU matrix by
 *      multiplying all the elements on the main diagonal of the upper
 *      triangular matrix and the given integer scalar contains the
 *      sign information.
 *
 *      Determinant = d1 * d2 * d3 * ... dn * integer scalar
 *
 *                        Decomposed LU
 *                  -                        -
 *                  | d1   U   U   U   U   U |
 *                  |  L  d2   U   U   U   U |
 *                  |  L   L  d3   U   U   U |
 *                  |  L   L   L   .   U   U |
 *                  |  L   L   L   L   .   U |
 *                  |  L   L   L   L   L  dn |
 *                  -                        -
 *
 * @param   [in]        *p_mat_lu       Handle of matrix LU.
 *
 *      On entry:
 *          The LU matrix should contains factors L and factors U.
 *
 * @param   [in]        inv_sgn_det     Integer scalar.
 *
 *      On entry:
 *          The integer scalar should contains the sign information
 *          needed for calculation of the determinant value of original
 *          matrix.
 *
 * @param   [out]       *p_det          Determinant of given matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_lu_det(const lm_mat_t *p_mat_lu,
                   int32_t inv_sgn_det,
                   lm_mat_elem_t *p_det)
{
    lm_rtn_t result;
    lm_mat_t mat_diag_shaped;
    lm_mat_dim_size_t diag_idx;
    lm_mat_elem_t *p_diag_elem;
    lm_mat_elem_size_t nxt_r_osf;

    const lm_mat_dim_size_t min_dim = LM_MIN(LM_MAT_GET_R_SIZE(p_mat_lu),
                                             LM_MAT_GET_C_SIZE(p_mat_lu));

    if (p_det == NULL) {
        return LM_ERR_CODE(LM_ERR_NULL_PTR);
    }

    if (min_dim == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    if (LM_MAT_GET_R_SIZE(p_mat_lu) != LM_MAT_GET_C_SIZE(p_mat_lu)) {
        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_SQUARE);
    }

    if (inv_sgn_det != 1 && inv_sgn_det != (-1)) {
        return LM_ERR_CODE(LM_ERR_MAT_INVALID_INVERT_SIGN_DETERMINANT);
    }

    result = lm_shape_diag(p_mat_lu, 0, &mat_diag_shaped);
    LM_RETURN_IF_ERR(result);

    p_diag_elem = LM_MAT_GET_ROW_PTR(&mat_diag_shaped, nxt_r_osf, 0);
    nxt_r_osf = LM_MAT_GET_NXT_OFS(&mat_diag_shaped);

    p_det[0] = LM_MAT_ONE_VAL;

    for (diag_idx = 0; diag_idx < min_dim; diag_idx++) {

        /* Multiply all diagonal elements */
        p_det[0] *= p_diag_elem[0];

        LM_MAT_TO_NXT_ROW(p_diag_elem, nxt_r_osf, &mat_diag_shaped);
    }

    /* Invert the sign of determinant */
    if (inv_sgn_det == -1) {
        *p_det = (-(*p_det));
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_lu_rank - Function to calculate the rank of given matrix that has been
 *              decomposed into LU.
 * @note
 *
 *      This function calculates the rank of given LU matrix by scanning and
 *      finding out the rows of upper triangular U that contain non-zero valued
 *      elements
 *
 * @todo
 * @attention
 *
 *      Please note that using this function to calculate the rank of the matrix
 *      is unreliable.
 *
 *          - One reason is that this function uses a fixed tolerance configuration
 *            to check whether the elements of the row are equal to (or almost equal
 *            to) zero,
 *          - And the other is caused by LU decomposition and floating errors.
 *
 *      Reference:
 *
 *          - http://www.math.sjsu.edu/singular/matrices/html/NYPA/Maragal_3.html
 *
 *          - http://www.math.sjsu.edu/singular/matrices/numerical_rank.html
 *
 *          - https://www.ilovematlab.cn/thread-439322-1-1.html
 *
 *          - 5.4.1 Numerical Rank and the SVD, "Matrix Computation 4th edition"
 *            written by Golub and Van Loan.
 *
 *          - When applied to floating point computations on computers, basic Gaussian
 *            elimination (LU decomposition) can be unreliable
 *            https://en.wikipedia.org/wiki/Rank_(linear_algebra)#Computation
 *
 *          - https://en.wikipedia.org/wiki/RRQR_factorization
 *
 *          - https://scicomp.stackexchange.com/questions/1771/what-is-the-corresponding-
 *            lapack-function-behind-matlab-q-r-e-qra/1781#1781
 *
 *          - Understanding how Numpy does SVD
 *            https://scicomp.stackexchange.com/questions/1861/understanding-how-numpy-
 *            does-svd/1863#1863
 *
 *          - Numerically singular matrices
 *            http://www.math.sjsu.edu/singular/matrices/numerical_rank.html
 *
 * @param   [in]        *p_mat_lu       Handle of matrix LU.
 *
 *      On entry:
 *          The LU matrix should contains factors L and factors U.
 *
 * @param   [out]       *p_rank         Rank of given matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_lu_rank(const lm_mat_t *p_mat_lu,
                    lm_mat_elem_size_t *p_rank)
{
    lm_mat_dim_size_t dim_idx;
    lm_mat_dim_size_t elem_idx;

    lm_mat_elem_t *p_elem_lu = LM_MAT_GET_ELEM_PTR(p_mat_lu);
    const lm_mat_dim_size_t min_dim = LM_MIN(LM_MAT_GET_R_SIZE(p_mat_lu),
                                             LM_MAT_GET_C_SIZE(p_mat_lu));
    const lm_mat_dim_size_t c_size_lu = LM_MAT_GET_C_SIZE(p_mat_lu);
    const lm_mat_elem_size_t nxt_r_osf_lu = LM_MAT_GET_NXT_OFS(p_mat_lu);

    if (p_rank == NULL) {
        return LM_ERR_CODE(LM_ERR_NULL_PTR);
    }

    if (min_dim == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    p_rank[0] = 0;

    for (dim_idx = 0; dim_idx < min_dim; dim_idx++) {

        /* Scan each row on upper triangular */
        for (elem_idx = dim_idx; elem_idx < c_size_lu; elem_idx++) {

            /* Find out the non-zero pivot of this row */
            if (LM_CHK_VAL_ALMOST_EQ_ZERO(p_elem_lu[elem_idx]) == false) {

                p_rank[0] += 1;

                break;
            }
        }

        LM_MAT_TO_NXT_ROW(p_elem_lu, nxt_r_osf_lu, p_mat_lu);
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_lu_invert - Function to calculate the inverse of given matrix that has been
 *                decomposed into LU.
 *
 * @note
 *
 *      This function calculates the inverse of given matrix by calculating
 *
 *          inv(A) = inv(U) * inv(L) * inv(P)
 *
 *      Reference:
 *          - http://home.cc.umanitoba.ca/~farhadi/Math2120/Inverse%20
 *            Using%20LU%20decomposition.pdf
 *
 * @todo
 * @attention
 *
 *      It is unreliable to use the determinant (lm_lu_det) to check
 *      whether the matrix is invertible.
 *
 *      Reference:
 *          - https://www.mathworks.com/matlabcentral/answers/400327-
 *            why-is-det-a-bad-way-to-check-matrix-singularity
 *          - What is the Condition Number of a Matrix?
 *            https://blogs.mathworks.com/cleve/2017/07/17/what-is-the-
 *            condition-number-of-a-matrix/
 *
 * @param   [in]        *p_mat_lu       Handle of matrix LU.
 *
 *      On entry:
 *          The LU matrix should contains factors L and factors U.
 *
 * @param   [in]        *p_perm_p       Handle of permutation list P.
 * @param   [in,out]    *p_mat_inv      Handle of inverse matrix.
 *
 *      The size of given inverse matrix should be same as matrix LU.
 *      On exit, the inverse of given matrix is stored in this inverse matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_lu_invert(const lm_mat_t *p_mat_lu,
                      const lm_permute_list_t *p_perm_p,
                      lm_mat_t *p_mat_inv)
{
    lm_rtn_t result;

    lm_mat_elem_t *p_elem_inv = LM_MAT_GET_ELEM_PTR(p_mat_inv);
    const lm_mat_dim_size_t r_size_inv = LM_MAT_GET_R_SIZE(p_mat_inv);
    const lm_mat_dim_size_t c_size_inv = LM_MAT_GET_C_SIZE(p_mat_inv);

    lm_mat_elem_t *p_elem_lu = LM_MAT_GET_ELEM_PTR(p_mat_lu);
    const lm_mat_dim_size_t r_size_lu = LM_MAT_GET_R_SIZE(p_mat_lu);
    const lm_mat_dim_size_t c_size_lu = LM_MAT_GET_C_SIZE(p_mat_lu);

    if (r_size_lu == 0 || c_size_lu == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    if (r_size_lu != c_size_lu) {
        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_SQUARE);
    }

    if (r_size_lu != r_size_inv || c_size_lu != c_size_inv) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_MISMATCH);
    }

    if ((r_size_lu * 2) > p_perm_p->mem.elem_tot) {
        return LM_ERR_CODE(LM_ERR_PM_ONE_LINE_BUFF_TOO_SMALL);
    }

    result = lm_oper_copy(p_mat_lu, p_mat_inv);
    LM_RETURN_IF_ERR(result);

    if (r_size_lu == 1) {

        p_elem_inv[0] = p_elem_lu[0];

        if (p_elem_inv[0] != LM_MAT_ZERO_VAL) {

            p_elem_inv[0] = (LM_MAT_ONE_VAL / p_elem_inv[0]);

            return LM_ERR_CODE(LM_SUCCESS);
        }
        else {
            return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_INVERTIBLE);
        }
    }

    /*
     * The matrix A has already been decomposed to P * A = L * U,
     * according to the equation above, the inverse of matrix A
     * can be calculated according to the following process:
     *
     * P * A = L * U
     *      => inv(P * A)       = inv(L * U)
     *      => inv(A) * inv(P)  = inv(U) * inv(L)
     *      => inv(A)           = inv(U) * inv(L) * inv(P)
     *
     */

    /* Calculate inv(U) */
    result = lm_lu_invert_triu(p_mat_inv);
    LM_RETURN_IF_ERR(result);

    /* Calculate inv(L) */
    result = lm_lu_invert_tril(p_mat_inv);
    LM_RETURN_IF_ERR(result);

    /* Calculate inv(U) * inv(U) */
    result = lm_lu_triu_mul_tril(p_mat_inv);
    LM_RETURN_IF_ERR(result);

    /* Calculate inv(U) * inv(U) * P */
    result = lm_oper_permute_col_inverse(p_mat_inv, p_perm_p);
    LM_RETURN_IF_ERR(result);

    return LM_ERR_CODE(LM_SUCCESS);
}


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */

/**
 * lm_lu_invert_tril - Function to in-place calculate the inverse of unit lower
 *                     triangular L stored in lower triangular part of matrix LU.
 *
 * @note
 *
 *      Reference:
 *          - http://home.cc.umanitoba.ca/~farhadi/Math2120/Inverse%20
 *            Using%20LU%20decomposition.pdf
 *
 * @param   [in,out]    *p_mat_lu       Handle of matrix LU.
 *
 *      On entry:
 *          The LU matrix should contains factors L and factors U.
 *
 *      On exit:
 *          The inverse of L is stored in lower triangular part of
 *          this matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
static lm_rtn_t lm_lu_invert_tril(lm_mat_t *p_mat_lu)
{
    lm_rtn_t result;
    lm_mat_dim_size_t c_idx;
    lm_mat_dim_size_t r_idx;
    lm_mat_t mat_pivot_row_shaped = {0};
    lm_mat_t mat_next_row_shaped = {0};

    lm_mat_elem_t *p_elem_pivot;

    const lm_mat_dim_size_t r_size_lu = LM_MAT_GET_R_SIZE(p_mat_lu);
    const lm_mat_dim_size_t c_size_lu = LM_MAT_GET_C_SIZE(p_mat_lu);
    const lm_mat_elem_size_t nxt_r_osf_lu = LM_MAT_GET_NXT_OFS(p_mat_lu);

    if (r_size_lu != c_size_lu) {
        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_SQUARE);
    }

    if (r_size_lu == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    if (r_size_lu == 1) {
        return LM_ERR_CODE(LM_SUCCESS);
    }

    /*
     * Calculate the inverse of matrix L, and store the L' in
     * lower triangular part of LU matrix without storing the
     * pivot.
     */
    for (c_idx = 0; c_idx < c_size_lu; c_idx++) {

        /*
         * The pivot of L matrix is always equal to one, no need to re-scale it.
         *
         * The inverse of L matrix will be store in the lower triangular part of
         * inverse matrix (but not including the pivot elements).
         *
         */

        result = lm_shape_submatrix(p_mat_lu,
                                    (c_idx), 0,
                                    1, c_idx,
                                    &mat_pivot_row_shaped);
        LM_RETURN_IF_ERR(result);

        if (c_idx < (c_size_lu - 1)) {

            p_elem_pivot = LM_MAT_GET_ROW_PTR(p_mat_lu, nxt_r_osf_lu, (c_idx + 1))
                         + c_idx;

            for (r_idx = (c_idx + 1); r_idx < r_size_lu; r_idx++) {

                if (p_elem_pivot[0] != LM_MAT_ZERO_VAL) {

                    /* Change the sign of the elements next to pivot */
                    p_elem_pivot[0] = (-p_elem_pivot[0]);

                    /* Rescale the elements of related rows */
                    if (c_idx > 0) {

                        result = lm_shape_submatrix(p_mat_lu,
                                                    r_idx, 0,
                                                    1, c_idx,
                                                    &mat_next_row_shaped);
                        LM_RETURN_IF_ERR(result);

                        result = lm_oper_axpy(p_elem_pivot[0],
                                              &mat_pivot_row_shaped,
                                              &mat_next_row_shaped);
                        LM_RETURN_IF_ERR(result);

                    }

                    LM_MAT_TO_NXT_ROW(p_elem_pivot, nxt_r_osf_lu, p_mat_lu);
                }
            }
        }
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_lu_invert_triu - Function to in-place calculate the inverse of upper
 *                     triangular U stored in upper triangular part of matrix
 *                     LU.
 *
 * @note
 *
 *      Reference:
 *          - http://home.cc.umanitoba.ca/~farhadi/Math2120/Inverse%20
 *            Using%20LU%20decomposition.pdf
 *
 * @todo
 * @attention
 *
 *      It is unreliable to use the determinant (lm_lu_det) to check
 *      whether the matrix is invertible.
 *
 *      Reference:
 *          - Why is det a bad way to check matrix singularity?
 *            https://www.mathworks.com/matlabcentral/answers/400327-
 *            why-is-det-a-bad-way-to-check-matrix-singularity
 *          - Condition number
 *            https://en.wikipedia.org/wiki/Condition_number
 *          - What is the Condition Number of a Matrix?
 *            https://blogs.mathworks.com/cleve/2017/07/17/what-is-
 *            the-condition-number-of-a-matrix/
 *
 * @param   [in,out]    *p_mat_lu       Handle of matrix LU.
 *
 *      On entry:
 *          The LU matrix should contains factors L and factors U.
 *
 *      On exit:
 *          The inverse of U is stored in upper triangular part of
 *          this matrix.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
static lm_rtn_t lm_lu_invert_triu(lm_mat_t *p_mat_lu)
{
    lm_rtn_t result;
    lm_mat_dim_size_t c_idx;
    lm_mat_dim_size_t r_idx;
    lm_mat_t mat_pivot_row_shaped = {0};
    lm_mat_t mat_next_row_shaped = {0};

    lm_mat_elem_t det;
    lm_mat_elem_t mult;
    lm_mat_elem_t *p_elem_pivot;

    const lm_mat_dim_size_t r_size_lu = LM_MAT_GET_R_SIZE(p_mat_lu);
    const lm_mat_dim_size_t c_size_lu = LM_MAT_GET_C_SIZE(p_mat_lu);
    const lm_mat_elem_size_t nxt_r_osf_lu = LM_MAT_GET_NXT_OFS(p_mat_lu);

    if (r_size_lu != c_size_lu) {
        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_SQUARE);
    }

    if (r_size_lu == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    if (r_size_lu == 1) {
        return LM_ERR_CODE(LM_SUCCESS);
    }

    /* Check if the matrix U is invertible */
    result = lm_lu_det(p_mat_lu, LM_MAT_ONE_VAL, &det);
    LM_RETURN_IF_ERR(result);

    if (det == LM_MAT_ZERO_VAL) {
        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_INVERTIBLE);
    }

    /*
     * Calculate the inverse of matrix U, and store the U' in
     * upper triangular part of LU matrix.
     */
    for (c_idx = c_size_lu; c_idx > 0; c_idx--) {

        /* Point to the pivot element of matrix U */
        p_elem_pivot = LM_MAT_GET_ROW_PTR(p_mat_lu, nxt_r_osf_lu, (c_idx - 1))
                     + (c_idx - 1);

        /* Rescale the pivot to one, and also rescale the related elements */
        mult = p_elem_pivot[0];
        mult = (LM_MAT_ONE_VAL / mult);
        p_elem_pivot[0] = mult;

        if (c_idx < c_size_lu) {

            result = lm_shape_submatrix(p_mat_lu,
                                        (c_idx - 1), c_idx,
                                        1, (c_size_lu - c_idx),
                                        &mat_pivot_row_shaped);
            LM_RETURN_IF_ERR(result);

            result = lm_oper_scalar(&mat_pivot_row_shaped, mult);
            LM_RETURN_IF_ERR(result);
        }

        result = lm_shape_submatrix(p_mat_lu,
                                    (c_idx - 1), (c_idx - 1),
                                    1, (lm_mat_dim_size_t)(c_size_lu - (c_idx - 1)),
                                    &mat_pivot_row_shaped);
        LM_RETURN_IF_ERR(result);

        if (c_idx > 1) {

            p_elem_pivot = LM_MAT_GET_ROW_PTR(p_mat_lu, nxt_r_osf_lu, (c_idx - 2))
                         + (c_idx - 1);

            for (r_idx = (c_idx - 1); r_idx > 0; r_idx--) {

                if (p_elem_pivot[0] != LM_MAT_ZERO_VAL) {

                    mult = p_elem_pivot[0];
                    p_elem_pivot[0] = LM_MAT_ZERO_VAL;

                    result = lm_shape_submatrix(p_mat_lu,
                                                (r_idx - 1), (c_idx - 1),
                                                1, (lm_mat_dim_size_t)(c_size_lu - (c_idx - 1)),
                                                &mat_next_row_shaped);
                    LM_RETURN_IF_ERR(result);

                    result = lm_oper_axpy((-mult),
                                          &mat_pivot_row_shaped,
                                          &mat_next_row_shaped);
                    LM_RETURN_IF_ERR(result);

                }

                LM_MAT_TO_NXT_ROW(p_elem_pivot, (-nxt_r_osf_lu), p_mat_lu);

            }
        }
    }

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * function_example - Function example
 *
 * http://home.cc.umanitoba.ca/~farhadi/Math2120/Inverse%20Using%20LU%20decomposition.pdf
 *
 * @param   [in]    input       Example input.
 * @param   [out]   *p_output   Example output.
 *
 * @return  [int]   Function executing result.
 * @retval  [0]     Success.
 * @retval  [-1]    Fail.
 *
 */
static lm_rtn_t lm_lu_triu_mul_tril(lm_mat_t *p_mat_lu)
{
    lm_rtn_t result;
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_t mat_l_subm_shaped;
    lm_mat_t mat_u_subm_shaped;
    lm_mat_t mat_dot;
    lm_mat_elem_t elem_dot;

    lm_mat_elem_t *p_elem_lu = NULL;
    const lm_mat_dim_size_t r_size_lu = LM_MAT_GET_R_SIZE(p_mat_lu);
    const lm_mat_dim_size_t c_size_lu = LM_MAT_GET_C_SIZE(p_mat_lu);
    const lm_mat_elem_size_t nxt_r_osf_lu = LM_MAT_GET_NXT_OFS(p_mat_lu);

    if (r_size_lu == 0 || c_size_lu == 0) {
        return LM_ERR_CODE(LM_ERR_MAT_DIM_IS_ZERO);
    }

    if (r_size_lu != c_size_lu) {
        return LM_ERR_CODE(LM_ERR_MAT_IS_NOT_SQUARE);
    }

    if (r_size_lu == 1) {
        return LM_ERR_CODE(LM_SUCCESS);
    }

    /*
     *
     * -                                     -       -                                     -
     * | U11     U12     U13     U14     U15 |       |  1       0       0       0       0  |
     * |  0      U22     U23     U24     U25 |       | L21      1       0       0       0  |
     * |  0       0      U33     U34     U35 |   x   | L31     L32      1       0       0  |
     * |  0       0       0      U44     U45 |       | L41     L42     L43      1       0  |
     * |  0       0       0       0      U55 |       | L51     L52     L53     L54      1  |
     * -                                     -       -                                     -
     *
     *   -                                                                                                         -
     *   | U11 + {U12~15}{L21~51}    U12 + {U13~15}{L32~42}    U13 + {U14~15}{L43~53}    U14 + {U15}{L54}    {U15} |
     *   |                                                                                                         |
     *   |       {U22~25}{L21~51}    U22 + {U23~25}{L32~42}    U23 + {U24~25}{L43~53}    U24 + {U25}{L54}    {U25} |
     *   |                                                                                                         |
     * = |       {U32~25}{L31~51}          {U33~25}{L32~52}    U33 + {U34~35}{L43~53}    U34 + {U35}{L54}    {U35} |
     *   |                                                                                                         |
     *   |       {U44~45}{L41~51}          {U33~45}{L42~52}          {U44~45}{L43~53}    U44 + {U45}{L54}    {U45} |
     *   |                                                                                                         |
     *   |          {U55}{L51}                {U55}{L52}                {U55}{L52}             {U55}{L54}    {U55} |
     *   -                                                                                                         -
     *
     *   -                       -
     *   | B    B    B    B    C |
     *   |                       |
     *   | A    B    B    B    C |
     *   |                       |
     * = | A    A    B    B    C |
     *   |                       |
     *   | A    A    A    B    C |
     *   |                       |
     *   | A    A    A    A    C |
     *   -                       -
     *
     */

    /* Prepare a 1 by 1 matrix for dot product calculation */
    result = lm_mat_set(&mat_dot, 1, 1, &elem_dot,
                        (sizeof(elem_dot) / sizeof(lm_mat_elem_t)));
    LM_RETURN_IF_ERR(result);

    /*
     * Start the in-place L * U calculation column by column,
     * without calculating the last column.
     */
    for (c_idx = 0; c_idx < (c_size_lu - 1); c_idx++) {

            p_elem_lu = LM_MAT_GET_COL_PTR(p_mat_lu, 1, c_idx);

        /* Calculate upper triangular part of LU matrix */
        for (r_idx = 0; r_idx < (c_idx + 1); r_idx++) {

            elem_dot = p_elem_lu[0];

            /* Row vector of sub-matrix U */
            result = lm_shape_submatrix(p_mat_lu, r_idx, (c_idx + 1),
                                        1, (lm_mat_dim_size_t)(c_size_lu - (c_idx + 1)),
                                        &mat_u_subm_shaped);
            LM_RETURN_IF_ERR(result);

            /* Column vector of sub-matrix L */
            result = lm_shape_submatrix(p_mat_lu, (c_idx + 1), c_idx ,
                                        (lm_mat_dim_size_t)(c_size_lu - (c_idx + 1)), 1,
                                        &mat_l_subm_shaped);
            LM_RETURN_IF_ERR(result);

            /* Calculate U row vector * L column vector */
            result = lm_oper_gemm(false, false,
                                  LM_MAT_ONE_VAL, &mat_u_subm_shaped, &mat_l_subm_shaped,
                                  LM_MAT_ONE_VAL, &mat_dot);
            LM_RETURN_IF_ERR(result);

            /* Store the dot product result in LU[r][c] */
            p_elem_lu[0] = elem_dot;

            LM_MAT_TO_NXT_ROW(p_elem_lu, nxt_r_osf_lu, p_mat_lu);
        }

        /* Calculate lower triangular part of LU matrix */
        for (; r_idx < r_size_lu; r_idx++) {

            /* Row vector of sub-matrix U */
            result = lm_shape_submatrix(p_mat_lu, r_idx, r_idx,
                                        1, (c_size_lu - r_idx), &mat_u_subm_shaped);
            LM_RETURN_IF_ERR(result);

            /* Column vector of sub-matrix L */
            result = lm_shape_submatrix(p_mat_lu, r_idx, c_idx,
                                        (r_size_lu - r_idx), 1, &mat_l_subm_shaped);
            LM_RETURN_IF_ERR(result);

            /* Calculate U row vector * L column vector */
            result = lm_oper_gemm(false, false,
                                  LM_MAT_ONE_VAL, &mat_u_subm_shaped, &mat_l_subm_shaped,
                                  LM_MAT_ZERO_VAL, &mat_dot);
            LM_RETURN_IF_ERR(result);

            /* Store the dot product result in LU[r][c] */
            p_elem_lu[0] = elem_dot;

            LM_MAT_TO_NXT_ROW(p_elem_lu, nxt_r_osf_lu, p_mat_lu);
        }
    }

    return LM_ERR_CODE(LM_SUCCESS);
}
