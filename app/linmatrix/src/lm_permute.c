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
 * @file    lm_permute.c
 * @brief   Lin matrix permutation functions
 *          based on the set theory and combinatorics
 * @note
 *
 * Reference:
 *      - https://www.youtube.com/watch?v=nBHCOMGOyKw
 *      - https://mathworld.wolfram.com/PermutationCycle.html
 *      - https://www.wolframalpha.com/examples/mathematics/discrete-mathematics/combinatorics/permutations/
 *      - https://www.wolframalpha.com/input/?i=permutations+of+%7B0%2C+1%2C+2%2C+3%2C+4%7D
 *
 * @author  Y.H.Kuo
 *******************************************************************************
 */

#include <string.h>

#include "lm_permute.h"
#include "lm_mat.h"
#include "lm_err.h"
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
 * lm_permute_set - Function to initialize a permutation list handle.
 *
 *      Please note that if the total number of members in a permutation
 *      of a set is N, the caller needs to allocate 2 * N memory buffer
 *      elements to the permutation list handle.
 *
 *      The double size memory buffer is required for performing one line
 *      notation and cycle notation conversion.
 *
 *      For Example:
 *           there are 5 members 0, 1, 2, 3, 4 in a permutation of a set,
 *           its corresponding one line notation of these 5 members is:
 *           (0 1 2 3 4)
 *           which requires only 5 elements memory space.
 *
 *           and its cycle notation is:
 *           (0) (1) (2) (3) (4)
 *           which requires 2 * 5 elements because not only the data
 *           members need to be store in permutation list, the cycle group
 *           information should also be store in permutation list.
 *
 *           members store in permutation list in one line notation format:
 *               P[10] = {0, 1, 2, 3, 4, X, X, X, X, X}
 *
 *               The X above represents unused.
 *
 *           members store in permutation list in cycle notation format:
 *               P[10] = {0, 0xFFFF, 1, 0xFFFF, 2, 0xFFFF, 3, 0xFFFF, 4, 0xFFFF}
 *
 *               The 0xFFFF above represents end of a cycle group.
 *
 *      https://en.wikipedia.org/wiki/Permutation
 *
 * @param   [in,out]    *p_list         Address of new permutation list handle.
 * @param   [in]        elem_num        Number of elements stored in this list.
 * @param   [in]        *p_men          Address of external memory buffer.
 * @param   [in]        mem_elem_tot    Number of total permutation element which
 *                                      can be stored in external memory buffer.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_permute_set(lm_permute_list_t *p_list,
                        lm_permute_size_t elem_num,
                        lm_permute_elem_t *p_men,
                        lm_permute_size_t mem_elem_tot)
{
    if (p_list == NULL || p_men == NULL) {
        return LM_ERR_CODE(LM_ERR_NULL_PTR);
    }

    if (mem_elem_tot == 0) {
        return LM_ERR_CODE(LM_ERR_PM_MEM_ELEM_TOTAL_IS_ZERO);
    }

    if (elem_num > mem_elem_tot) {
        return LM_ERR_CODE(LM_ERR_PM_ELEM_NUM_EXCEED_MEM_LIMIT);
    }

    p_list->stats = 0;
    p_list->elem.ptr = p_men;
    p_list->elem.num = elem_num;
    p_list->elem.cyc_grp_num = 0;
    p_list->mem.ptr = p_men;
    p_list->mem.elem_tot = mem_elem_tot;

    LM_PERMUTE_SET_STATS(p_list, (LM_PERMUTE_STATS_INIT));

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_permute_clr - Function to clear a permutation list handle.
 *
 * Please note that the externally allocated memory buffer should be
 * released by the caller if the memory buffer is dynamic allocated.
 *
 * @param   [in,out]    *p_list     Address of permutation list
 *                                  handle needs to be clear.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_permute_clr(lm_permute_list_t *p_list)
{
    if (p_list == NULL) {
        return LM_ERR_CODE(LM_ERR_NULL_PTR);
    }

    p_list->stats = 0;
    p_list->elem.ptr = NULL;
    p_list->elem.num = 0;
    p_list->elem.cyc_grp_num = 0;
    p_list->mem.ptr = NULL;
    p_list->mem.elem_tot = 0;

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_permute_dump - Function to dump a permutation list in raw format.
 *
 * @param   [in]        *p_list     Address of permutation list handle.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_permute_dump(const lm_permute_list_t *p_list)
{
    lm_permute_size_t idx;
    const lm_permute_size_t num = p_list->mem.elem_tot;

    if (p_list->elem.ptr == NULL) {
        return LM_ERR_CODE(LM_ERR_NULL_PTR);
    }

    LM_LOG_INFO("PERMUTE (raw) = [");

    for (idx = 0; idx < num; idx++) {

        if (idx % 8 == 0) {

            if (p_list->elem.ptr[idx] == LM_PERMUTE_CYCLE_END_SYM) {
                LM_LOG_INFO("\n\t[%02u](#####) ", idx);
            }
            else {
                LM_LOG_INFO("\n\t[%02u](%04Xh) ", idx, p_list->elem.ptr[idx]);
            }

        }
        else {
            if (p_list->elem.ptr[idx] == LM_PERMUTE_CYCLE_END_SYM) {
                LM_LOG_INFO("[%02u](#####) ", idx);
            }
            else {
                LM_LOG_INFO("[%02u](%04Xh) ", idx, p_list->elem.ptr[idx]);
            }

        }

    }

    LM_LOG_INFO("\n]\n");

    LM_LOG_INFO("\n");

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_permute_dump_cycle_notation - Function to dump a permutation
 *                                  list in cycle notation format.
 *
 * @param   [in]        *p_list     Address of permutation list handle.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_permute_dump_cycle_notation(const lm_permute_list_t *p_list)
{
    lm_permute_size_t idx;
    const lm_permute_size_t num = p_list->elem.num;

    if (p_list->elem.ptr == NULL) {
        return LM_ERR_CODE(LM_ERR_NULL_PTR);
    }

    if (num > p_list->mem.elem_tot) {
        return LM_ERR_CODE(LM_ERR_PM_ELEM_NUM_EXCEED_MEM_LIMIT);
    }

    LM_LOG_INFO("PERMUTE (cycle) = [\n");

    for (idx = 0; idx < num; idx++) {

        if (idx == 0) {
            LM_LOG_INFO("\t");
        }

        if (p_list->elem.ptr[idx] == LM_PERMUTE_CYCLE_END_SYM) {
            LM_LOG_INFO(", ");
        }
        else {
            LM_LOG_INFO("%u ", p_list->elem.ptr[idx]);
        }
    }

    LM_LOG_INFO("\n]\n");

    LM_LOG_INFO("\n");

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_permute_dump_cycle_notation - Function to dump a permutation
 *                                  list in one line notation format.
 *
 * @param   [in]        *p_list     Address of permutation list handle.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_permute_dump_oline_notation(const lm_permute_list_t *p_list)
{
    lm_permute_size_t idx;
    const lm_permute_size_t num = p_list->elem.num;

    if (p_list->elem.ptr == NULL) {
        return LM_ERR_CODE(LM_ERR_NULL_PTR);
    }

    if (num > p_list->mem.elem_tot) {
        return LM_ERR_CODE(LM_ERR_PM_ELEM_NUM_EXCEED_MEM_LIMIT);
    }

    LM_LOG_INFO("PERMUTE = (one line) [");

    for (idx = 0; idx < num; idx++) {

        if (idx % 8 == 0) {
            LM_LOG_INFO("\n\t%hu ", p_list->elem.ptr[idx]);
        }
        else {
            LM_LOG_INFO("%hu ", p_list->elem.ptr[idx]);
        }

    }

    LM_LOG_INFO("\n]\n");

    LM_LOG_INFO("\n");

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_permute_oline_to_cycle - Function to in-place convert the one line
 *                             notation stored in permutation list to
 *                             cycle notation.
 *
 * @param   [in,out]    *p_list     Address of permutation list handle.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_permute_oline_to_cycle(lm_permute_list_t *p_list)
{
    lm_permute_size_t root_idx;
    lm_permute_size_t next_idx;
    lm_permute_size_t search_idx;
    lm_permute_size_t elem_num = p_list->elem.num;
    lm_permute_size_t group_elem_num;
    lm_permute_elem_t *p_group_start = p_list->elem.ptr;
    lm_permute_elem_t *p_group_end = p_list->elem.ptr;
    lm_permute_elem_t *p_line_notation = (p_list->elem.ptr + elem_num);

    if (p_list->elem.ptr == NULL) {
        return LM_ERR_CODE(LM_ERR_NULL_PTR);
    }

    if (elem_num > p_list->mem.elem_tot) {
        return LM_ERR_CODE(LM_ERR_PM_ELEM_NUM_EXCEED_MEM_LIMIT);
    }

    if ((elem_num * 2) > p_list->mem.elem_tot) {
        return LM_ERR_CODE(LM_ERR_NEED_MORE_MEM);
    }

    /*
     * Copy the original one-line notation
     * to the second half of the buffer
     */
    memcpy((void *)(p_line_notation),
           (void *)(p_group_start),
           elem_num * sizeof(lm_permute_elem_t));

    /* Reset group counter and the total occupied element counter */
    p_list->elem.num = 0;
    p_list->elem.cyc_grp_num = 0;

    for (root_idx = 0; root_idx < elem_num; root_idx++) {

        group_elem_num = 0;

        /* Find out the new start element of cyclic group */
        if ((p_line_notation[root_idx] & LM_PERMUTE_MATCHED_FLAG) == 0) {

            p_group_start = p_group_end;
            p_group_end[0] = root_idx;
            p_group_end++;
            p_list->elem.num++;
            group_elem_num++;
            next_idx = p_line_notation[root_idx];

            if (next_idx >= elem_num) {
                return LM_ERR_CODE(LM_ERR_PM_ELEM_VALUE_OUT_OF_RANGE);
            }

            /* Make this element as resolved */
            p_line_notation[root_idx] |= LM_PERMUTE_MATCHED_FLAG;

            /* Found a single element group */
            if (next_idx == root_idx) {
                p_group_end[0] = LM_PERMUTE_CYCLE_END_SYM;
                p_group_end++;
                p_list->elem.num++;
                p_list->elem.cyc_grp_num++;
            }
            /* Something goes wrong if the next element has already been marked as resolve? */
            else if ((p_line_notation[next_idx] & LM_PERMUTE_MATCHED_FLAG) != 0) {
                return LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_CANNOT_COMPLETE);
            }
            else {
                /* Find out all the elements which are belong to this group */
                for (search_idx = 0; search_idx < elem_num; search_idx++) {

                    /* Reached end of cycle */
                    if (p_line_notation[next_idx] == p_group_start[0]) {

                        group_elem_num++;

                        if (group_elem_num > elem_num) {
                            return LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_ELEM_NUM_OUT_OF_RANGE);
                        }

                        p_group_end[0] = next_idx;
                        p_group_end[1] = LM_PERMUTE_CYCLE_END_SYM;
                        p_group_end += 2;
                        p_list->elem.num += 2;

                        p_line_notation[next_idx] |= LM_PERMUTE_MATCHED_FLAG;
                        p_list->elem.cyc_grp_num++;
                        break;
                    }
                    else {
                        group_elem_num++;

                        if (group_elem_num > elem_num) {
                            return LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_ELEM_NUM_OUT_OF_RANGE);
                        }

                        p_group_end[0] = next_idx;
                        p_group_end++;
                        p_list->elem.num++;
                        p_line_notation[next_idx] |= LM_PERMUTE_MATCHED_FLAG;
                        next_idx = (lm_permute_size_t)(p_line_notation[next_idx] & ~(LM_PERMUTE_MATCHED_FLAG));
                    }
                }
            }
        }
    }

    LM_ASSERT_DBG(((elem_num + p_list->elem.cyc_grp_num) == p_list->elem.num),
                  "The number of total occupied element should equal to "
                  "cycle group number + number of element stored in one-line notation")

    return LM_ERR_CODE(LM_SUCCESS);
}

/**
 * lm_permute_cycle_to_oline - Function to convert the cycle notation stored
 *                             in permutation list to one line notation and
 *                             store it to another permutation list.
 *
 * This function requires another permutation list handle to store
 * the one line notation data. The allocated element size of another
 * permutation list handle should be the same to the allocated
 * element size of original permutation list handle.
 *
 * @param   [in]        *p_cycle_list   Address of permutation list handle
 *                                      contains cycle notation data.
 * @param   [out]       *p_oline_list   Address of permutation list handle.
 *                                      for storing one line notation data.
 *
 * @return  [lm_rtn_t]  Function executing result.
 * @retval  [0]         Success.
 * @retval  [-N]        Failure (error code).
 *
 */
lm_rtn_t lm_permute_cycle_to_oline(const lm_permute_list_t *p_cycle_list,
                                   lm_permute_list_t *p_oline_list)
{
    lm_permute_size_t cycle_idx;
    lm_permute_size_t total_elem_num = (p_cycle_list->elem.num);
    lm_permute_size_t actual_elem_num;
    lm_permute_size_t found_elem_cnt = 0;
    lm_permute_size_t found_group_cnt = 0;
    const lm_permute_elem_t *p_group_start = p_cycle_list->elem.ptr;
    const lm_permute_elem_t *p_group_end = p_cycle_list->elem.ptr;
    lm_permute_elem_t *p_oline_notation = p_oline_list->elem.ptr;

    if (total_elem_num > p_cycle_list->mem.elem_tot) {
        return LM_ERR_CODE(LM_ERR_PM_ELEM_NUM_EXCEED_MEM_LIMIT);
    }

    if (p_cycle_list->elem.cyc_grp_num == 0) {
        return LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_NUM_IS_ZERO);
    }

    if (p_cycle_list->elem.num == 0) {
        return LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_ELEM_NUM_IS_ZERO);
    }

    if (p_cycle_list->elem.num <= p_cycle_list->elem.cyc_grp_num) {
        return LM_ERR_CODE(LM_ERR_PM_CYCLE_GROUP_NUM_MISMATCHED);
    }

    actual_elem_num = p_cycle_list->elem.num - p_cycle_list->elem.cyc_grp_num;

    if (p_oline_list->mem.elem_tot < actual_elem_num) {
        return LM_ERR_CODE(LM_ERR_PM_ONE_LINE_BUFF_TOO_SMALL);
    }

    for (cycle_idx = 0; cycle_idx < total_elem_num; cycle_idx++) {

        /* Ignore the cycle notation group end symbol */
        if (p_group_end[cycle_idx] == LM_PERMUTE_CYCLE_END_SYM) {
            continue;
        }
        else if (p_group_end[cycle_idx] >= actual_elem_num) {
            return LM_ERR_CODE(LM_ERR_PM_ELEM_VALUE_OUT_OF_RANGE);
        }

        if (p_group_end[cycle_idx + 1] == LM_PERMUTE_CYCLE_END_SYM) {
            p_oline_notation[p_group_end[cycle_idx]] = p_group_start[0];
            found_elem_cnt += 1;
            found_group_cnt += 1;
            p_group_start = p_group_end + cycle_idx + 2;
        }
        else {
            p_oline_notation[p_group_end[cycle_idx]] = p_group_end[cycle_idx + 1];
            found_elem_cnt += 1;
        }
    }

    if (found_elem_cnt != actual_elem_num) {
        return LM_ERR_CODE(LM_ERR_PM_ONE_LINE_NOTATION_NUM_MISMATCHED);
    }
    else if (found_group_cnt != p_cycle_list->elem.cyc_grp_num) {
        return LM_ERR_CODE(LM_ERR_PM_ONE_LINE_NOTATION_NUM_MISMATCHED);
    }
    else {

        p_oline_list->elem.num = found_elem_cnt;
        p_oline_list->elem.cyc_grp_num = 0;

        return LM_ERR_CODE(LM_SUCCESS);
    }
}


/*
 *******************************************************************************
 * Private functions
 *******************************************************************************
 */

