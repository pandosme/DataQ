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
 * @file    lm_permute.h
 * @brief   Lin matrix permutation functions
 *          based on the set theory and combinatorics
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#ifndef __LM_PERMUTE_H__
#define __LM_PERMUTE_H__

#include <stdint.h>
#include <stdbool.h>

#include "lm_global.h"
#include "lm_assert.h"
#include "lm_mat.h"


/*
 *******************************************************************************
 * Constant value definition
 *******************************************************************************
 */

#define LM_PERMUTE_STATS_INIT           (1 << 0)
#define LM_PERMUTE_STATS_BUILTIN_MEM    (1 << 1)
#define LM_PERMUTE_ONE_LINE_NOTATION    (1 << 2)
#define LM_PERMUTE_CYCLE_NOTATION       (1 << 2)

#define LM_PERMUTE_MATCHED_FLAG         (1 << 15)

#define LM_PERMUTE_CYCLE_END_SYM        0xFFFF


/*
 *******************************************************************************
 * Macros
 *******************************************************************************
 */

/**
  @def LM_PERMUTE_SET_STATS(__p_perm, __stats)
  To setup the stats of permutation handle.
*/
#define LM_PERMUTE_SET_STATS(__p_perm, __stats)                             \
    do {                                                                    \
        ((lm_permute_list_t *)(__p_perm))->stats |= (uint8_t)(__stats);     \
    } while(0)

/**
  @def LM_PERMUTE_CLR_STATS(__p_perm, __stats)
  To clear the stats of permutation handle.
*/
#define LM_PERMUTE_CLR_STATS(__p_perm, __stats)                             \
    do {                                                                    \
        ((lm_permute_list_t *)(__p_perm))->stats &= ~((uint8_t)(__stats));  \
    } while(0)

/**
  @def LM_PERMUTE_SWAP_OLINE_ELEM(__elem_1, __elem_2, __swap_buf)
  To sawp 2 elements of permutation list.
*/
#define LM_PERMUTE_SWAP_OLINE_ELEM(__elem_1, __elem_2, __swap_buf)          \
    do {                                                                    \
        (__swap_buf) = (__elem_1);                                          \
        (__elem_1) = (__elem_2);                                            \
        (__elem_2) = (__swap_buf);                                          \
    } while(0)


/*
 *******************************************************************************
 * Data type definition
 *******************************************************************************
 */

/** @brief Data type of permutation element */
typedef lm_mat_dim_size_t lm_permute_elem_t;

/** @brief Data type of permutation list size */
typedef lm_mat_dim_size_t lm_permute_size_t;

/** @brief Data structure of permutation list handle */
typedef struct lm_permute_list {

    /** @brief Stats of this permutation list */
    uint8_t stats;

    /** @brief Permutation element description */
    struct {
        /** @brief Point to the first element of permutation list */
        lm_permute_elem_t *ptr;

        /**
         * @brief Number of elements stored in this permutation list.
         *        If the data stored in this list is in cycle notation
         *        format, the total number of elements stored should
         *        equal to "number of elements" + "number of cycle groups"
         *        E.g. if the cycle notation data is (0, 1) (2),
         *             then this number should be set to (3 + 2) = 5,
         *             3 for 3 elements, 2 for 2 cycle groups.
         */
        lm_permute_size_t num;

        /**
         * @brief Total number of cycle notation group stored in this list
         *        (this counter is created for cycle notation only)
         */
        lm_permute_size_t cyc_grp_num;

    } elem;

    /** @brief Permutation list memory block description */
    struct {

        /** @brief Point to start address of assigned memory block */
        lm_permute_elem_t *ptr;

        /** @brief Total number of elements that can be stored in this memory block */
        lm_mat_elem_size_t elem_tot;

    } mem;

} lm_permute_list_t;


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

lm_rtn_t lm_permute_set(lm_permute_list_t *p_list,
                        lm_permute_size_t elem_num,
                        lm_permute_elem_t *p_men,
                        lm_permute_size_t mem_elem_tot);

lm_rtn_t lm_permute_clr(lm_permute_list_t *p_list);

lm_rtn_t lm_permute_dump(const lm_permute_list_t *p_list);

lm_rtn_t lm_permute_dump_cycle_notation(const lm_permute_list_t *p_list);

lm_rtn_t lm_permute_dump_oline_notation(const lm_permute_list_t *p_list);

lm_rtn_t lm_permute_oline_to_cycle(lm_permute_list_t *p_list);

lm_rtn_t lm_permute_cycle_to_oline(const lm_permute_list_t *p_cycle_list,
                                   lm_permute_list_t *p_oline_list);


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */


#endif // __LM_PERMUTE_H__
