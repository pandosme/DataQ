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
 * @file    lm_assert.h
 * @brief   Lin matrix assertion
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#ifndef __LM_ASSERT_H__
#define __LM_ASSERT_H__

#include "lm_log.h"


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

#define LM_ASSERT_DBG(__cond, __msg)                                            \
    do {                                                                        \
        if ((__cond) == false) {                                                \
            LM_LOG_ERR("\n");                                                   \
            LM_LOG_ERR("@ Assert cond: %s\n", #__cond);                         \
            LM_LOG_ERR("@ Message = %s\n", __msg);                              \
            LM_LOG_ERR("@ %s:%d (%s) \n", __FILE__, __LINE__, __FUNCTION__);    \
            abort(); \
        } \
    } while (0);

#define LM_ASSERT_REL(__cond, __msg)                                            \
    do {                                                                        \
        if ((__cond) == false) {                                                \
            LM_LOG_ERR("\n");                                                   \
            LM_LOG_ERR("@ Assert cond: %s\n", #__cond);                         \
            LM_LOG_ERR("@ Message = %s\n", __msg);                              \
            LM_LOG_ERR("@ %s:%d (%s) \n", __FILE__, __LINE__, __FUNCTION__);    \
            abort(); \
        } \
    } while (0);


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


#endif // __LM_ASSERT_H__
