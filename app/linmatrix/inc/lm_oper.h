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
 * @file    lm_oper.h
 * @brief   Lin matrix operating functions
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#ifndef __LM_OPER_H__
#define __LM_OPER_H__

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

lm_rtn_t lm_oper_zeros(lm_mat_t *p_mat_a);

lm_rtn_t lm_oper_zeros_diagonal(const lm_mat_t *p_mat_a,
                                lm_mat_dim_offset_t diag_osf);

lm_rtn_t lm_oper_zeros_triu(lm_mat_t *p_mat_a,
                            lm_mat_dim_offset_t diag_osf);

lm_rtn_t lm_oper_zeros_tril(lm_mat_t *p_mat_a,
                            lm_mat_dim_offset_t diag_osf);

lm_rtn_t lm_oper_identity(lm_mat_t *p_mat_a);

lm_rtn_t lm_oper_abs(lm_mat_t *p_mat_a);

lm_rtn_t lm_oper_max(const lm_mat_t *p_mat_a,
                     lm_mat_dim_size_t *p_r_idx,
                     lm_mat_dim_size_t *p_c_idx,
                     lm_mat_elem_t *p_max);

lm_rtn_t lm_oper_max_abs(const lm_mat_t *p_mat_a,
                         lm_mat_dim_size_t *p_r_idx,
                         lm_mat_dim_size_t *p_c_idx,
                         lm_mat_elem_t *p_max);

lm_rtn_t lm_oper_min(const lm_mat_t *p_mat_a,
                     lm_mat_dim_size_t *p_r_idx,
                     lm_mat_dim_size_t *p_c_idx,
                     lm_mat_elem_t *p_min);

lm_rtn_t lm_oper_min_abs(const lm_mat_t *p_mat_a,
                         lm_mat_dim_size_t *p_r_idx,
                         lm_mat_dim_size_t *p_c_idx,
                         lm_mat_elem_t *p_min);

lm_rtn_t lm_oper_swap_row(lm_mat_t *p_mat_a,
                          lm_mat_dim_size_t src_r_idx,
                          lm_mat_dim_size_t dst_r_idx);

lm_rtn_t lm_oper_swap_col(lm_mat_t *p_mat_a,
                          lm_mat_dim_size_t src_c_idx,
                          lm_mat_dim_size_t dst_c_idx);

lm_rtn_t lm_oper_permute_row(lm_mat_t *p_mat_a,
                             const lm_permute_list_t *p_list);

lm_rtn_t lm_oper_permute_row_inverse(lm_mat_t *p_mat_a,
                                     const lm_permute_list_t *p_list);

lm_rtn_t lm_oper_permute_col(lm_mat_t *p_mat_a,
                             const lm_permute_list_t *p_list);

lm_rtn_t lm_oper_permute_col_inverse(lm_mat_t *p_mat_a,
                                     const lm_permute_list_t *p_list);

lm_rtn_t lm_oper_copy(const lm_mat_t *p_mat_src,
                      lm_mat_t *p_mat_dst);

lm_rtn_t lm_oper_copy_diagonal(const lm_mat_t *p_mat_src,
                               lm_mat_t *p_mat_dst,
                               lm_mat_dim_offset_t diag_osf);

lm_rtn_t lm_oper_copy_triu(const lm_mat_t *p_mat_src,
                           lm_mat_t *p_mat_dst,
                           lm_mat_dim_offset_t diag_osf);

lm_rtn_t lm_oper_copy_tril(const lm_mat_t *p_mat_src,
                           lm_mat_t *p_mat_dst,
                           lm_mat_dim_offset_t diag_osf);

lm_rtn_t lm_oper_copy_transpose(const lm_mat_t *p_mat_a,
                                lm_mat_t *p_mat_trans);

lm_rtn_t lm_oper_transpose(lm_mat_t *p_mat_a);

lm_rtn_t lm_oper_scalar(lm_mat_t *p_mat_a, lm_mat_elem_t scalar);

lm_rtn_t lm_oper_bandwidth(const lm_mat_t *p_mat_a,
                           lm_mat_dim_size_t *p_lower,
                           lm_mat_dim_size_t *p_upper);

lm_rtn_t lm_oper_givens(const lm_mat_elem_t elem_a,
                        const lm_mat_elem_t elem_b,
                        lm_mat_elem_t *p_sin,
                        lm_mat_elem_t *p_cos);

lm_rtn_t lm_oper_trace(const lm_mat_t *p_mat_a,
                       lm_mat_elem_t *p_trace);


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */


#endif // __LM_OPER_H__
