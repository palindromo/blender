/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contributor(s): Joseph Eagar, Geoffrey Bantle, Campbell Barton
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/bmesh/intern/bmesh_inline.h
 *  \ingroup bmesh
 *
 * BM Inline functions.
 */

#ifndef __BMESH_INLINE_H__
#define __BMESH_INLINE_H__

/* stuff for dealing with header flags */
#define BM_elem_flag_test(     ele, hflag)      _bm_elem_flag_test     (&(ele)->head, hflag)
#define BM_elem_flag_test_bool(ele, hflag)      _bm_elem_flag_test_bool(&(ele)->head, hflag)

#define BM_elem_flag_enable(   bm, ele, hflag)      _bm_elem_flag_enable   (bm, &(ele)->head, hflag)
#define BM_elem_flag_disable(  bm, ele, hflag)      _bm_elem_flag_disable  (bm, &(ele)->head, hflag)
#define BM_elem_flag_set(      bm, ele, hflag, val) _bm_elem_flag_set      (bm, &(ele)->head, hflag, val)
#define BM_elem_flag_toggle(   bm, ele, hflag)      _bm_elem_flag_toggle   (bm, &(ele)->head, hflag)
#define BM_elem_flag_merge(    bm, ele_a, ele_b)    _bm_elem_flag_merge    (bm, &(ele_a)->head, &(ele_b)->head)

BLI_INLINE char _bm_elem_flag_test(const BMHeader *head, const char hflag)
{
	return head->hflag & hflag;
}

BLI_INLINE short _bm_elem_flag_test_bool(const BMHeader *head, const char hflag)
{
	return (head->hflag & hflag) != 0;
}

BLI_INLINE void _bm_elem_flag_enable(BMesh *UNUSED(bm), BMHeader *head, const char hflag)
{
	head->hflag |= hflag;
}

BLI_INLINE void _bm_elem_flag_disable(BMesh *UNUSED(bm), BMHeader *head, const char hflag)
{
	head->hflag &= ~hflag;
}

BLI_INLINE void _bm_elem_flag_set(BMesh *bm, BMHeader *head, const char hflag, const int val)
{
	if (val)  _bm_elem_flag_enable(bm, head,  hflag);
	else      _bm_elem_flag_disable(bm, head, hflag);
}

BLI_INLINE void _bm_elem_flag_toggle(BMesh *UNUSED(bm), BMHeader *head, const char hflag)
{
	head->hflag ^= hflag;
}

BLI_INLINE void _bm_elem_flag_merge(BMesh *UNUSED(bm), BMHeader *head_a, BMHeader *head_b)
{
	head_a->hflag = head_b->hflag = head_a->hflag | head_b->hflag;
}

/**
 * notes on #BM_elem_index_set(...) usage,
 * Set index is sometimes abused as temp storage, other times we cant be
 * sure if the index values are valid because certain operations have modified
 * the mesh structure.
 *
 * To set the elements to valid indices 'BM_mesh_elem_index_ensure' should be used
 * rather then adding inline loops, however there are cases where we still
 * set the index directly
 *
 * In an attempt to manage this,
 * here are 5 tags I'm adding to uses of #BM_elem_index_set
 *
 * - 'set_inline'  -- since the data is already being looped over set to a
 *                    valid value inline.
 *
 * - 'set_dirty!'  -- intentionally sets the index to an invalid value,
 *                    flagging 'bm->elem_index_dirty' so we don't use it.
 *
 * - 'set_ok'      -- this is valid use since the part of the code is low level.
 *
 * - 'set_ok_invalid'  -- set to -1 on purpose since this should not be
 *                    used without a full array re-index, do this on
 *                    adding new vert/edge/faces since they may be added at
 *                    the end of the array.
 *
 * - 'set_loop'    -- currently loop index values are not used used much so
 *                    assume each case they are dirty.
 *
 * - campbell */

#define BM_elem_index_get(ele)           _bm_elem_index_get(&(ele)->head)
#define BM_elem_index_set(ele, index)    _bm_elem_index_set(&(ele)->head, index)

BLI_INLINE void _bm_elem_index_set(BMHeader *head, const int index)
{
	head->index = index;
}

BLI_INLINE int _bm_elem_index_get(const BMHeader *head)
{
	return head->index;
}

/************************* Vertex coordinates *************************/

/* A small set of functions for changing vertex coordinates. All
   changes should go through this API rather than directly setting
   BMVert.co (needed for undo/redo) */

/* TODO: could reduce this to just BM_vert_copy_v3() and let callers
   do the math, not sure which is better. */

BLI_INLINE void BM_vert_copy_v3(BMesh *UNUSED(bm), BMVert *v, const float co[3])
{
	copy_v3_v3(v->co, co);
}

BLI_INLINE void BM_vert_add_v3(BMesh *UNUSED(bm), BMVert *v, const float a[3])
{
	add_v3_v3(v->co, a);
}

BLI_INLINE void BM_vert_add_v3v3(BMesh *UNUSED(bm), BMVert *v,
								 const float a[3], const float b[3])
{
	add_v3_v3v3(v->co, a, b);
}

BLI_INLINE void BM_vert_madd_v3fl(BMesh *UNUSED(bm), BMVert *v,
								  const float a[3], float f)
{
	madd_v3_v3fl(v->co, a, f);
}

BLI_INLINE void BM_vert_madd_v3v3fl(BMesh *UNUSED(bm), BMVert *v,
									const float a[3], const float b[3], float f)
{
	madd_v3_v3v3fl(v->co, a, b, f);
}

BLI_INLINE void BM_vert_sub_v3(BMesh *UNUSED(bm), BMVert *v, const float a[3])
{
	sub_v3_v3(v->co, a);
}

BLI_INLINE void BM_vert_sub_v3v3(BMesh *UNUSED(bm), BMVert *v,
								 const float a[3], const float b[3])
{
	sub_v3_v3v3(v->co, a, b);
}

BLI_INLINE void BM_vert_mul_m4(BMesh *UNUSED(bm), BMVert *v,
							   float mat[4][4])
{
	mul_v3_m4v3(v->co, mat, v->co);
}

BLI_INLINE void BM_vert_mul_m4v3(BMesh *UNUSED(bm), BMVert *v,
								 float mat[4][4], const float a[3])
{
	mul_v3_m4v3(v->co, mat, a);
}

BLI_INLINE void BM_vert_interp_v3v3(BMesh *UNUSED(bm), BMVert *v,
									const float a[3], const float b[3],
									float factor)
{
	interp_v3_v3v3(v->co, a, b, factor);
}

#endif /* __BMESH_INLINE_H__ */
