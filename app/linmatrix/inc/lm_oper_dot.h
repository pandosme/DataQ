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
 * @file    lm_oper_dot.h
 * @brief   Lin matrix arithmetic dot product functions
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#ifndef __LM_OPER_DOT_H__
#define __LM_OPER_DOT_H__

#include "lm_mat.h"


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

lm_rtn_t lm_oper_dot(const lm_mat_t *p_mat_a,
                     const lm_mat_t *p_mat_b,
                     lm_mat_t *p_mat_out);
lm_rtn_t lm_oper_dot_gemm11(const lm_mat_t *p_mat_a,
                            const lm_mat_t *p_mat_b,
                            lm_mat_t *p_mat_out);
lm_rtn_t lm_oper_dot_gemm14(const lm_mat_t *p_mat_a,
                            const lm_mat_t *p_mat_b,
                            lm_mat_t *p_mat_out);
lm_rtn_t lm_oper_dot_gemm41(const lm_mat_t *p_mat_a,
                            const lm_mat_t *p_mat_b,
                            lm_mat_t *p_mat_out);
lm_rtn_t lm_oper_dot_gemm44(const lm_mat_t *p_mat_a,
                            const lm_mat_t *p_mat_b,
                            lm_mat_t *p_mat_out);

/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */


#endif // __LM_OPER_DOT_H__
