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
 * @file    lm_lu.h
 * @brief   Lin matrix LU decomposition functions
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#ifndef __LM_LU_H__
#define __LM_LU_H__

#include "lm_mat.h"
#include "lm_permute.h"


/*
 *******************************************************************************
 * Constant value definition
 *******************************************************************************
 */


/*
 *******************************************************************************
 * Macros
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

lm_rtn_t lm_lu_decomp(lm_mat_t *p_mat_lu,
                      lm_permute_list_t *p_perm_p,
                      int32_t *p_invert_sgn_det);

lm_rtn_t lm_lu_det(const lm_mat_t *p_mat_lu,
                   int32_t inv_sgn_det,
                   lm_mat_elem_t *p_det);

lm_rtn_t lm_lu_rank(const lm_mat_t *p_mat_lu,
                    lm_mat_elem_size_t *p_rank);

lm_rtn_t lm_lu_invert(const lm_mat_t *p_mat_lu,
                      const lm_permute_list_t *p_perm_p,
                      lm_mat_t *p_mat_inv);


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */


#endif // __LM_LU_H__
