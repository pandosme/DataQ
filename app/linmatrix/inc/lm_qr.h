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
 * @file    lm_qr.h
 * @brief   Lin matrix QR decomposition functions
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#ifndef __LM_QR_H__
#define __LM_QR_H__

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

lm_rtn_t lm_qr_housh_v(lm_mat_t *p_mat_v,
                       lm_mat_elem_t *p_alpha,
                       lm_mat_elem_t *p_beta);

lm_rtn_t lm_qr_housh_refl(lm_mat_t *p_mat_a,
                          lm_mat_elem_t beta,
                          lm_mat_t *p_mat_houshv,
                          lm_mat_t *p_mat_work);

lm_rtn_t lm_qr_decomp(lm_mat_t *p_mat_qr,
                      lm_mat_t *p_mat_beta,
                      lm_mat_t *p_mat_work);

lm_rtn_t lm_qr_explicit(lm_mat_t *p_mat_qr,
                        const lm_mat_t *p_mat_beta,
                        lm_mat_t *p_mat_q,
                        lm_mat_t *p_mat_work);


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */


#endif // __LM_QR_H__
