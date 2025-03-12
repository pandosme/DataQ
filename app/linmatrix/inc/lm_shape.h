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
 * @file    lm_shape.h
 * @brief   Lin matrix shaping functions
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#ifndef __LM_SHAPE_H__
#define __LM_SHAPE_H__

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

lm_rtn_t lm_shape_row_vect(const lm_mat_t *p_mat_a,
                           lm_mat_dim_size_t row_idx,
                           lm_mat_t *p_mat_shaped);

lm_rtn_t lm_shape_col_vect(const lm_mat_t *p_mat_a,
                           lm_mat_dim_size_t col_idx,
                           lm_mat_t *p_mat_shaped);

lm_rtn_t lm_shape_submatrix(const lm_mat_t *p_mat_a,
                            lm_mat_dim_size_t row_idx,
                            lm_mat_dim_size_t col_idx,
                            lm_mat_dim_size_t row_num,
                            lm_mat_dim_size_t col_num,
                            lm_mat_t *p_mat_shaped);

lm_rtn_t lm_shape_diag(const lm_mat_t *p_mat_a,
                       lm_mat_dim_offset_t diag_osf,
                       lm_mat_t *p_mat_shaped);


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */


#endif // __LM_SHAPE_H__
