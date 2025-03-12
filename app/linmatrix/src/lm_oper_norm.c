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
 * @file    lm_oper_norm.c
 * @brief   Lin matrix matrix norm computation functions
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#include <math.h>

#include "lm_oper_norm.h"
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
 * lm_oper_norm_fro - Function to calculate the Frobenius norm of matrix.
 *
 * https://en.wikipedia.org/wiki/Matrix_norm#Frobenius_norm
 *
 * The Frobenius norm of matrix A is \f$||A||_{p} = \sqrt{(\sum_{i=1}^{m}\sum_{j=1}^{n}|a_{ij}|^2)}\f$
 *
 *
 * @param   [in]        *p_mat_a        Handle of matrix A.
 * @param   [out]       *p_norm         Frobenius norm of matrix A.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_oper_norm_fro(const lm_mat_t *p_mat_a, lm_mat_elem_t *p_norm)
{
    lm_mat_dim_size_t r_idx;
    lm_mat_dim_size_t c_idx;
    lm_mat_elem_t *p_elem_a = NULL;
    const lm_mat_dim_size_t r_size = LM_MAT_GET_R_SIZE(p_mat_a);
    const lm_mat_dim_size_t c_size = LM_MAT_GET_C_SIZE(p_mat_a);
    const lm_mat_elem_size_t nxt_r_osf_a = LM_MAT_GET_NXT_OFS(p_mat_a);

    *p_norm = LM_MAT_ZERO_VAL;

    for (r_idx = 0; r_idx < r_size; r_idx++) {

        p_elem_a = LM_MAT_GET_ROW_PTR(p_mat_a, nxt_r_osf_a, r_idx);

        for (c_idx = 0; c_idx < c_size; c_idx++) {

            /* norm += A[i][j] * A[i][j] */
            *p_norm += (p_elem_a[0] * p_elem_a[0]);

            LM_MAT_TO_NXT_ELEM(p_elem_a, p_mat_a);
        }

    }

    *p_norm = (lm_mat_elem_t)sqrt(*p_norm);

    return LM_ERR_CODE(LM_SUCCESS);
}


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */

