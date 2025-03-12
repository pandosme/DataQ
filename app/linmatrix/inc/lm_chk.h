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
 * @file    lm_chk.h
 * @brief   Lin matrix auxiliary check functions
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#ifndef __LM_CHK_H__
#define __LM_CHK_H__

#include "lm_global.h"
#include "lm_mat.h"


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
 * Macros
 *******************************************************************************
 */

/**
  @def LM_CHK_VAL_ALMOST_EQ_ZERO(__val)
  Check if given value is approximately equal to zero.
*/
#define LM_CHK_VAL_ALMOST_EQ_ZERO(__val) \
    (LM_IS_ERR(lm_chk_elem_almost_equal((__val), LM_MAT_ZERO_VAL)) ? false : true)

/**
  @def LM_CHK_VAL_ALMOST_EQ_ONE(__p_perm, __stats)
  Check if given value is approximately equal to one.
*/
#define LM_CHK_VAL_ALMOST_EQ_ONE(__val) \
    (LM_IS_ERR(lm_chk_elem_almost_equal((__val), LM_MAT_ONE_VAL)) ? false : true)


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

lm_rtn_t lm_chk_machine_eps(lm_mat_elem_t *p_value);

lm_rtn_t lm_chk_elem_almost_equal(lm_mat_elem_t elem_a, lm_mat_elem_t elem_b);

lm_rtn_t lm_chk_mat_almost_equal(const lm_mat_t *p_mat_a, const lm_mat_t *p_mat_b);

lm_rtn_t lm_chk_square_mat(const lm_mat_t *p_mat_a);

lm_rtn_t lm_chk_triu_mat(const lm_mat_t *p_mat_a);

lm_rtn_t lm_chk_tril_mat(const lm_mat_t *p_mat_a);

lm_rtn_t lm_chk_diagonal_mat(const lm_mat_t *p_mat_a);

lm_rtn_t lm_chk_identity_mat(const lm_mat_t *p_mat_a);

lm_rtn_t lm_chk_orthogonal_mat(const lm_mat_t *p_mat_q);

lm_rtn_t lm_chk_banded_mat(const lm_mat_t *p_mat_a,
                           lm_mat_dim_size_t lower,
                           lm_mat_dim_size_t upper);


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */


#endif // __LM_CHK_H__
