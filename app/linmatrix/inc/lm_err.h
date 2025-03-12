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
 * @file    lm_err.h
 * @brief   Lin matrix error code definition
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#ifndef __LM_ERR_H__
#define __LM_ERR_H__

#include <stdbool.h>


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

#define LM_IS_ERR(_postive_err_code) \
    ((_postive_err_code < 0) ? true : false)
#define LM_ERR_CODE(_postive_err_code) (-(_postive_err_code))
#define LM_ERR_MSG(_negative_err_code) err_msg_get(_negative_err_code)


/*
 *******************************************************************************
 * Data type definition
 *******************************************************************************
 */

/** Matrix error code definition */
typedef enum lm_err_code {
    /* Success */
    LM_SUCCESS                                      = 0,

    /* Generic */
    LM_ERR_UNKNOWN                                  = 1,
    LM_ERR_NULL_PTR                                 = 2,
    LM_ERR_NO_MEM                                   = 3,
    LM_ERR_NEED_MORE_MEM                            = 4,

    /* Matrix */
    LM_ERR_MAT_DIM_IS_ZERO                          = 5,
    LM_ERR_MAT_DIM_LIMIT_EXCEEDED                   = 6,
    LM_ERR_MAT_DIM_MISMATCH                         = 7,
    LM_ERR_MAT_IS_NOT_VECTOR                        = 8,
    LM_ERR_MAT_IS_NOT_SQUARE                        = 9,
    LM_ERR_MAT_IS_NOT_UPPER_TRIANGULAR              = 10,
    LM_ERR_MAT_IS_NOT_LOWER_TRIANGULAR              = 11,
    LM_ERR_MAT_IS_NOT_DIAGONAL                      = 12,
    LM_ERR_MAT_IS_NOT_IDENTITY                      = 13,
    LM_ERR_MAT_IS_NOT_ORTHOGONAL                    = 14,
    LM_ERR_MAT_IS_NOT_BANDED                        = 15,
    LM_ERR_MAT_IS_NOT_INVERTIBLE                    = 16,
    LM_ERR_MAT_ELEM_VALUE_NOT_EQUAL                 = 17,
    LM_ERR_MAT_ELEM_VALUE_NEGATIVE                  = 18,
    LM_ERR_MAT_NEED_DIFFERENT_MAT_TO_STORE_OUTPUT   = 19,
    LM_ERR_MAT_DOT_PRODUCT_DIM_MISMATCHED           = 20,
    LM_ERR_MAT_PERMUTATION_DIM_MISMATCHED           = 21,
    LM_ERR_MAT_ROW_IDX_OUT_OF_RANGE                 = 22,
    LM_ERR_MAT_COL_IDX_OUT_OF_RANGE                 = 23,
    LM_ERR_MAT_CANNOT_DEFLATE_ANYMORE               = 24,
    LM_ERR_MAT_EXCEEDED_MAX_ITERATION               = 25,

    LM_ERR_MAT_INVALID_INVERT_SIGN_DETERMINANT      = 26,

    /* Permutation */
    LM_ERR_PM_MEM_ELEM_TOTAL_IS_ZERO                = 1024,
    LM_ERR_PM_ELEM_NUM_EXCEED_MEM_LIMIT             = 1025,
    LM_ERR_PM_ELEM_VALUE_OUT_OF_RANGE               = 1026,
    LM_ERR_PM_CYCLE_GROUP_ELEM_NUM_IS_ZERO          = 1027,
    LM_ERR_PM_CYCLE_GROUP_ELEM_NUM_OUT_OF_RANGE     = 1028,
    LM_ERR_PM_CYCLE_GROUP_NUM_IS_ZERO               = 1029,
    LM_ERR_PM_CYCLE_GROUP_NUM_MISMATCHED            = 1030,
    LM_ERR_PM_CYCLE_GROUP_NUM_OUT_OF_RANGE          = 1031,
    LM_ERR_PM_CYCLE_GROUP_CANNOT_COMPLETE           = 1032,
    LM_ERR_PM_ONE_LINE_BUFF_TOO_SMALL               = 1033,
    LM_ERR_PM_ONE_LINE_NOTATION_NUM_MISMATCHED      = 1034,

    /* Reserved, please keep this declaration at the end */
    LM_ERR_MAX_RESERVED,

} lm_err_code_t;


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

/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */


#endif // __LM_ERR_H__
