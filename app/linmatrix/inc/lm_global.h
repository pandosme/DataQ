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
 * @file    lm_global.h
 * @brief   Lin matrix global data types, definitions and configurations
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#ifndef __LM_GLOBAL_H__
#define __LM_GLOBAL_H__

#include <stdint.h>


/*
 *******************************************************************************
 * Constant value definition
 *******************************************************************************
 */

/* Data type selection */
#define LM_GLOBAL_MAT_ELEM_TYPE_SINGLE

/* Element data type: Single */
#if defined(LM_GLOBAL_MAT_ELEM_TYPE_SINGLE)
    #include "float.h"
    typedef int32_t lm_rtn_t;
    typedef float lm_mat_elem_t;
    typedef int32_t lm_mat_elem_type_punning_t;
    typedef int32_t lm_mat_elem_ulp_diff_t;
    typedef union {
        lm_mat_elem_t flt;
        lm_mat_elem_type_punning_t uint;
    } lm_mat_elem_union_t;
    #define LM_MAT_NAME_ENABLED (true)
    #define LM_MAT_SIZEOF_ELEM (sizeof(lm_mat_elem_t))
    #define LM_MAT_DIM_LIMIT (4096)
    #define LM_MAT_ZERO_VAL ((lm_mat_elem_t)(0.0))
    #define LM_MAT_ONE_VAL ((lm_mat_elem_t)(1.0))
    #define LM_MAT_ELEM_PRINT_FMT "%.7e"
    #define LM_MAT_MACHINE_EPS FLT_EPSILON
    #define LM_MAT_EPSILON_MAX ((lm_mat_elem_t)(1.5e-5))
    #define LM_MAT_ULP_MAX (4550)
    #define LM_MAT_EIGEN_TOLERANCE (1.0e-09)
    #define LM_MAT_EIGEN_MAX_ITER_PER_CYC (16)

/* Element data type: Double */
#elif defined(LM_GLOBAL_MAT_ELEM_TYPE_DOUBLE)
    #include "float.h"
    typedef int32_t lm_rtn_t;
    typedef double lm_mat_elem_t;
    typedef int64_t lm_mat_elem_type_punning_t;
    typedef int64_t lm_mat_elem_ulp_diff_t;
    typedef union {
        lm_mat_elem_t flt;
        lm_mat_elem_type_punning_t uint;
    } lm_mat_elem_union_t;
    #define LM_MAT_NAME_ENABLED (true)
    #define LM_MAT_SIZEOF_ELEM (sizeof(lm_mat_elem_t))
    #define LM_MAT_DIM_LIMIT (4096)
    #define LM_MAT_ZERO_VAL ((lm_mat_elem_t)(0.0))
    #define LM_MAT_ONE_VAL ((lm_mat_elem_t)(1.0))
    #define LM_MAT_ELEM_PRINT_FMT "%.15e"
    #define LM_MAT_MACHINE_EPS DBL_EPSILON
    #define LM_MAT_EPSILON_MAX ((lm_mat_elem_t)(1.0e-11))
    #define LM_MAT_ULP_MAX (8500)
    #define LM_MAT_EIGEN_TOLERANCE (1.0e-09)
    #define LM_MAT_EIGEN_MAX_ITER_PER_CYC (16)

/* Element data type: Undefined */
#else
    #error "Unsupported matrix element type"
#endif


/*
 *******************************************************************************
 * Macros
 *******************************************************************************
 */

#define LM_MIN(__a, __b) (((__a) < (__b)) ? (__a) : (__b))
#define LM_MAX(__a, __b) (((__a) > (__b)) ? (__a) : (__b))
#define LM_SIGN(__a) (((__a) < (LM_MAT_ZERO_VAL))               \
                      ? (-LM_MAT_ONE_VAL) : (LM_MAT_ONE_VAL))


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


#endif // __LM_GLOBAL_H__
