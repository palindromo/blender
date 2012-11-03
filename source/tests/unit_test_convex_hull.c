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
 * Contributor(s): Nicholas Bishop
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#ifndef NDEBUG

#include "BLI_utildefines.h"

#include <assert.h>

#include "bmesh.h"
#include "unit_test_intern.h"

static BMesh *bm_create(void)
{
	return BM_mesh_create(&bm_mesh_allocsize_default);
}

static BMVert *bm_vert_create_co(BMesh *bm, float x, float y, float z)
{
	const float co[3] = {x, y, z};
	return BM_vert_create(bm, co, NULL, 0);
}

static BMEdge *bm_edge_create(BMesh *bm, BMVert *v1, BMVert *v2)
{
	return BM_edge_create(bm, v1, v2, NULL, FALSE);
}

static BMFace *bm_triangle_create(BMesh *bm,
								  BMVert *v1,
								  BMVert *v2,
								  BMVert *v3)
{
	BMVert *verts[3] = {v1, v2, v3};
	BMEdge *edges[3] = {
		BM_edge_exists(v1, v2),
		BM_edge_exists(v2, v3),
		BM_edge_exists(v3, v1)
	};
	return BM_face_create(bm, verts, edges, 3, FALSE);
}

/* Note: triangle count is used rather than face count to protect
 * against differences between implementations that merge adjacent
 * flat triangles into an ngon and those that don't. The edge count is
 * similarly inflated to add fan-triangle edges. */
static void bm_count_elems(BMesh *bm,
						   int *totvert,
						   int *totedge,
						   int *tottriangle)
{
	BMIter bm_iter;
	BMFace *f;

	(*totvert) = bm->totvert;
	(*totedge) = bm->totedge;
	(*tottriangle) = 0;

	BM_ITER_MESH (f, &bm_iter, bm, BM_FACES_OF_MESH) {
		assert(f->len >= 3);

		(*tottriangle) += f->len - 2;
		if (f->len > 3)
			(*totedge) += f->len - 3;
	}
}

/* Note: macro used here so that assert gives correct line number */
#define CHECK_NUM_ELEMS(bm_,										\
						expected_totvert_,							\
						expected_totedge_,							\
						expected_tottriangle_)						\
	do {															\
		int totvert, totedge, tottriangle;							\
		bm_count_elems(bm_, &totvert, &totedge, &tottriangle);		\
																	\
		if (totvert != expected_totvert_) {							\
			printf("   %s: bm->totvert=%d, expected %d\n",			\
				   __func__, totvert, expected_totvert_);			\
		}															\
		assert(totvert == expected_totvert_);						\
																	\
		if (totedge != expected_totedge_) {							\
			printf("   %s: bm->totedge=%d, expected %d\n",			\
				   __func__, totedge, expected_totedge_);			\
		}															\
		assert(totedge == expected_totedge_);						\
																	\
		if (tottriangle != expected_tottriangle_) {					\
			printf("   %s: bm triangle count=%d, expected %d\n",	\
				   __func__, tottriangle, expected_tottriangle_);	\
		}															\
		assert(tottriangle == expected_tottriangle_);				\
	} while(0)

/* Note: macro used here so that assert gives correct line number */
#define CHECK_SLOT_ELEM_COUNTS(bm_, op_, slot_name_,					\
							   expected_totvert_,						\
							   expected_totedge_,						\
							   expected_tottriangle_)					\
	do {																\
		BMOIter oiter;													\
		BMElemF *ele;													\
		int totvert = 0, totedge = 0, tottriangle = 0;					\
																		\
		assert(BMO_slot_exists((op_)->slots_out, slot_name_));			\
																		\
		BMO_ITER (ele, &oiter, (op_)->slots_out, slot_name_, BM_ALL) {	\
			if (ele->head.htype == BM_VERT)								\
				totvert++;												\
			if (ele->head.htype == BM_EDGE)								\
				totedge++;												\
			if (ele->head.htype == BM_FACE) {							\
				BMFace *f = (BMFace*)ele;								\
				assert(f->len >= 3);									\
				tottriangle += f->len - 2;								\
				if (f->len > 3)											\
					totedge += f->len - 3;								\
			}															\
		}																\
																		\
		if (totvert != expected_totvert_) {								\
			printf("   slot %s has %d vertices, expected %d\n",			\
				   slot_name_, totvert, expected_totvert_);				\
		}																\
		assert(totvert == expected_totvert_);							\
																		\
		if (totedge != expected_totedge_) {								\
			printf("   slot %s has %d edges, expected %d\n",			\
				   slot_name_, totedge, expected_totedge_);				\
		}																\
		assert(totedge == expected_totedge_);							\
																		\
		if (tottriangle != expected_tottriangle_) {						\
			printf("   slot %s has %d triangles, expected %d\n",		\
				   slot_name_, tottriangle, expected_tottriangle_);		\
		}																\
		assert(tottriangle == expected_tottriangle_);					\
	} while(0)

/* Check if a buffer slot contains 'key' */
static int bmo_slot_buffer_contains(BMOperator *op,
									const char *slot_name, void *key)
{
	BMOIter oiter;
	BMElemF *ele;

	assert(BMO_slot_exists(op->slots_out, slot_name));
	BMO_ITER (ele, &oiter, op->slots_out, slot_name, BM_ALL) {
		if (ele == key)
			return TRUE;
	}
	return FALSE;
}

/* Run convex hull operator on the vertices of a tetrahedron */
static void test_convex_hull_tetrahedron(void)
{
	BMesh *bm = bm_create();
	BMOperator op;

	TEST_HEADER();

	bm_vert_create_co(bm, 0, 0, 0);
	bm_vert_create_co(bm, 1, 0, 0);
	bm_vert_create_co(bm, 0, 1, 0);
	bm_vert_create_co(bm, 0, 0, 1);

	assert(BMO_op_initf(bm, &op, BMO_FLAG_DEFAULTS,
						"convex_hull input=%av "
						"use_existing_faces=%b",
						FALSE));
	BMO_op_exec(bm, &op);
	assert(!BMO_error_occurred(bm));
	CHECK_NUM_ELEMS(bm, 4, 6, 4);

	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom.out", 4, 6, 4);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_interior.out", 0, 0, 0);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_unused.out", 0, 0, 0);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_holes.out", 0, 0, 0);

	BMO_op_finish(bm, &op);
	BM_mesh_free(bm);
}

/* Run convex hull operator on the vertices of a tetrahedron and an
 * interior triangle. The interior triangles face, vertices, and edges
 * should all be marked as interior and unused. */
static void test_convex_hull_interior_elems(void)
{
	BMesh *bm = bm_create();
	BMOperator op;
	BMVert *v[7] = {
		bm_vert_create_co(bm, 0, 0, 0),
		bm_vert_create_co(bm, 1, 0, 0),
		bm_vert_create_co(bm, 0, 1, 0),
		bm_vert_create_co(bm, 0, 0, 1),

		bm_vert_create_co(bm, 0.3, 0.3, 0.3),
		bm_vert_create_co(bm, 0.32, 0.3, 0.3),
		bm_vert_create_co(bm, 0.3, 0.32, 0.3)
	};
	BMEdge *e[3] = {
		bm_edge_create(bm, v[4], v[5]),
		bm_edge_create(bm, v[5], v[6]),
		bm_edge_create(bm, v[6], v[4])
	};
	BMFace *f[1] = {
		bm_triangle_create(bm, v[4], v[5], v[6])
	};
	int i;

	TEST_HEADER();

	assert(BMO_op_initf(bm, &op, BMO_FLAG_DEFAULTS,
						"convex_hull input=%avef "
						"use_existing_faces=%b",
						FALSE));
	BMO_op_exec(bm, &op);
	assert(!BMO_error_occurred(bm));
	CHECK_NUM_ELEMS(bm, 7, 3 + 6, 1 + 4);

	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom.out", 4, 6, 4);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_interior.out", 3, 3, 1);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_unused.out", 3, 3, 1);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_holes.out", 0, 0, 0);

	for (i = 0; i < 3; i++) {
		assert(bmo_slot_buffer_contains(&op, "geom_interior.out", v[4 + i]));
		assert(bmo_slot_buffer_contains(&op, "geom_interior.out", e[i]));
		assert(bmo_slot_buffer_contains(&op, "geom_unused.out", v[4 + i]));
		assert(bmo_slot_buffer_contains(&op, "geom_unused.out", e[i]));
	}
	assert(bmo_slot_buffer_contains(&op, "geom_interior.out", f[0]));
	assert(bmo_slot_buffer_contains(&op, "geom_unused.out", f[0]));

	BMO_op_finish(bm, &op);
	BM_mesh_free(bm);
}

/* Run convex hull operator on the vertices of a triangular
 * bi-pyramid. The central edges and a face bisecting the bi-pyramid
 * are created. An additional edge is created between the poles of the
 * pyramid. The convex hull should mark the interior edge and the
 * interior face for removal. */
static void test_convex_hull_unused_edge_and_face(void)
{
	BMesh *bm = bm_create();
	BMOperator op;
	BMVert *v[5] = {
		bm_vert_create_co(bm, -1, -1, 0),
		bm_vert_create_co(bm, 1, 0, 0),
		bm_vert_create_co(bm, 0, 1, 0),
		bm_vert_create_co(bm, 0, 0, 1),
		bm_vert_create_co(bm, 0, 0, -1)
	};
	BMEdge *e[4] = {
		bm_edge_create(bm, v[0], v[1]),
		bm_edge_create(bm, v[1], v[2]),
		bm_edge_create(bm, v[2], v[0]),
		bm_edge_create(bm, v[3], v[4])
	};
	BMFace *f[1] = {
		bm_triangle_create(bm, v[0], v[1], v[2])
	};

	TEST_HEADER();
	(void)e;

	assert(BMO_op_initf(bm, &op, BMO_FLAG_DEFAULTS,
						"convex_hull input=%avef "
						"use_existing_faces=%b",
						FALSE));
	BMO_op_exec(bm, &op);
	assert(!BMO_error_occurred(bm));
	CHECK_NUM_ELEMS(bm, 5, 10, 1 + 6);

	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom.out", 5, 9, 6);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_interior.out", 0, 1, 1);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_unused.out", 0, 1, 1);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_holes.out", 0, 0, 0);

	assert(bmo_slot_buffer_contains(&op, "geom_interior.out", f[0]));
	assert(bmo_slot_buffer_contains(&op, "geom_unused.out", f[0]));

	BMO_op_finish(bm, &op);
	BM_mesh_free(bm);
}

/* Run convex hull operator on the vertices of a tetrahedron. Two
 * interior vertices and an edge connecting them are added. The extra
 * edge is not added to the input, but the vertices are.
 *
 * The vertices should be marked as interior, but not marked as
 * unused. */
static void test_convex_hull_interior_but_used(void)
{
	BMesh *bm = bm_create();
	BMOperator op;
	BMVert *v[6] = {
		bm_vert_create_co(bm, -1, -1, 0),
		bm_vert_create_co(bm, 1, 0, 0),
		bm_vert_create_co(bm, 0, 1, 0),
		bm_vert_create_co(bm, 0, 0, 1),
		bm_vert_create_co(bm, 0.3, 0.3, 0.3),
		bm_vert_create_co(bm, 0.32, 0.3, 0.3)
	};
	BMEdge *e[1] = {
		bm_edge_create(bm, v[4], v[5]),
	};

	TEST_HEADER();
	(void)e;

	assert(BMO_op_initf(bm, &op, BMO_FLAG_DEFAULTS,
						"convex_hull input=%av "
						"use_existing_faces=%b",
						FALSE));
	BMO_op_exec(bm, &op);
	assert(!BMO_error_occurred(bm));
	CHECK_NUM_ELEMS(bm, 6, 1 + 6, 4);

	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom.out", 4, 6, 4);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_interior.out", 2, 0, 0);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_unused.out", 0, 0, 0);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_holes.out", 0, 0, 0);

	assert(bmo_slot_buffer_contains(&op, "geom_interior.out", v[4]));
	assert(bmo_slot_buffer_contains(&op, "geom_interior.out", v[5]));

	BMO_op_finish(bm, &op);
	BM_mesh_free(bm);
}

/* A tetrahedron with all vertices, edges, and faces is created. One
 * of the tetrahedron's faces is duplicated a short distance away.
 *
 * The duplicate and original face are given as input to the convex
 * hull (including the vertices and edges of those faces.) The convex
 * hull should bridge the faces and mark the original face as a
 * hole. */
static void test_convex_hull_single_hole(void)
{
	BMesh *bm = bm_create();
	BMOperator op;
	BMVert *v[7] = {
		bm_vert_create_co(bm, -1, -1, 0),
		bm_vert_create_co(bm, 1, 0, 0),
		bm_vert_create_co(bm, 0, 1, 0),
		bm_vert_create_co(bm, 0, 0, 1),

		bm_vert_create_co(bm, 1.5, 0.5, 0),
		bm_vert_create_co(bm, 0.5, 1.5, 0),
		bm_vert_create_co(bm, 0.5, 0.5, 1)
	};
	BMEdge *e[9] = {
		bm_edge_create(bm, v[0], v[1]),
		bm_edge_create(bm, v[1], v[2]),
		bm_edge_create(bm, v[2], v[0]),

		bm_edge_create(bm, v[3], v[0]),
		bm_edge_create(bm, v[3], v[1]),
		bm_edge_create(bm, v[3], v[2]),

		bm_edge_create(bm, v[4], v[5]),
		bm_edge_create(bm, v[5], v[6]),
		bm_edge_create(bm, v[6], v[4])
	};
	BMFace *f[5] = {
		bm_triangle_create(bm, v[0], v[1], v[2]),
		bm_triangle_create(bm, v[0], v[1], v[3]),
		bm_triangle_create(bm, v[1], v[2], v[3]),
		bm_triangle_create(bm, v[2], v[0], v[3]),

		bm_triangle_create(bm, v[4], v[5], v[6])
	};

	TEST_HEADER();

	BM_elem_flag_enable(v[1], BM_ELEM_TAG);
	BM_elem_flag_enable(v[2], BM_ELEM_TAG);
	BM_elem_flag_enable(v[3], BM_ELEM_TAG);

	BM_elem_flag_enable(v[4], BM_ELEM_TAG);
	BM_elem_flag_enable(v[5], BM_ELEM_TAG);
	BM_elem_flag_enable(v[6], BM_ELEM_TAG);

	BM_elem_flag_enable(e[1], BM_ELEM_TAG);
	BM_elem_flag_enable(e[4], BM_ELEM_TAG);
	BM_elem_flag_enable(e[5], BM_ELEM_TAG);

	BM_elem_flag_enable(e[6], BM_ELEM_TAG);
	BM_elem_flag_enable(e[7], BM_ELEM_TAG);
	BM_elem_flag_enable(e[8], BM_ELEM_TAG);

	BM_elem_flag_enable(f[2], BM_ELEM_TAG);
	BM_elem_flag_enable(f[4], BM_ELEM_TAG);

	assert(BMO_op_initf(bm, &op, BMO_FLAG_DEFAULTS,
						"convex_hull input=%hvef "
						"use_existing_faces=%b",
						BM_ELEM_TAG, TRUE));
	BMO_op_exec(bm, &op);
	assert(!BMO_error_occurred(bm));

	CHECK_NUM_ELEMS(bm, 7, 9 + 3 + 3, 5 + 6);

	/* Note that the two cap faces are part of the hull but since they
	 * were already created, not in geom.out. Is this good behavior? */
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom.out", 6, 3 + 3 + 3 + 3, 2 * 3);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_interior.out", 0, 0, 0);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_unused.out", 0, 0, 0);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_holes.out", 0, 0, 1);

	//assert(geom_holes.out_not_in_other_slots_check(bm, &op));

	assert(bmo_slot_buffer_contains(&op, "geom_holes.out", f[2]));

	BMO_op_finish(bm, &op);
	BM_mesh_free(bm);
}

/* Added for bug #32960
 *
 * Test convex hull on the vertices of a non-degenerate triangle with
 * only one edge (which is part of the input) and no faces */
static void test_convex_hull_one_edge(void)
{
	BMesh *bm = bm_create();
	BMOperator op;
	BMVert *v[3] = {
		bm_vert_create_co(bm, 0, 0, 0),
		bm_vert_create_co(bm, 1, 0, 0),
		bm_vert_create_co(bm, 0, 1, 0)
	};

	bm_edge_create(bm, v[0], v[1]);

	TEST_HEADER();

	assert(BMO_op_initf(bm, &op, BMO_FLAG_DEFAULTS,
						"convex_hull input=%ave "
						"use_existing_faces=%b",
						TRUE));
	BMO_op_exec(bm, &op);
	assert(!BMO_error_occurred(bm));

	CHECK_NUM_ELEMS(bm, 3, 3, 1);

	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom.out", 3, 3, 1);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_interior.out", 0, 0, 0);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_unused.out", 0, 0, 0);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_holes.out", 0, 0, 0);

	BMO_op_finish(bm, &op);
	BM_mesh_free(bm);
}

/* Test convex hull on a non-degenerate triangle with three vertices,
 * three edges, and a face as input. The use_existing_faces parameter
 * is disabled. */
static void test_convex_hull_one_triangle(void)
{
	BMesh *bm = bm_create();
	BMOperator op;
	BMVert *v[3] = {
		bm_vert_create_co(bm, 0, 0, 0),
		bm_vert_create_co(bm, 1, 0, 0),
		bm_vert_create_co(bm, 0, 1, 0)
	};

	bm_edge_create(bm, v[0], v[1]);
	bm_edge_create(bm, v[1], v[2]);
	bm_edge_create(bm, v[2], v[0]);

	bm_triangle_create(bm, v[0], v[1], v[2]);

	TEST_HEADER();

	assert(BMO_op_initf(bm, &op, BMO_FLAG_DEFAULTS,
						"convex_hull input=%avef "
						"use_existing_faces=%b",
						FALSE));
	BMO_op_exec(bm, &op);
	assert(!BMO_error_occurred(bm));

	CHECK_NUM_ELEMS(bm, 3, 3, 1);

	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom.out", 3, 3, 1);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_interior.out", 0, 0, 0);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_unused.out", 0, 0, 0);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_holes.out", 0, 0, 0);

	BMO_op_finish(bm, &op);
	BM_mesh_free(bm);
}

/* Test convex hull on a non-degenerate triangle with three vertices,
 * three edges, and a face as input, and the use_existing_faces
 * parameter enabled. */
static void test_convex_hull_one_triangle_use_existing(void)
{
	BMesh *bm = bm_create();
	BMOperator op;
	BMVert *v[3] = {
		bm_vert_create_co(bm, 0, 0, 0),
		bm_vert_create_co(bm, 1, 0, 0),
		bm_vert_create_co(bm, 0, 1, 0)
	};

	bm_edge_create(bm, v[0], v[1]);
	bm_edge_create(bm, v[1], v[2]);
	bm_edge_create(bm, v[2], v[0]);

	bm_triangle_create(bm, v[0], v[1], v[2]);

	TEST_HEADER();

	assert(BMO_op_initf(bm, &op, BMO_FLAG_DEFAULTS,
						"convex_hull input=%avef "
						"use_existing_faces=%b",
						TRUE));
	BMO_op_exec(bm, &op);
	assert(!BMO_error_occurred(bm));

	CHECK_NUM_ELEMS(bm, 3, 3, 1);

	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom.out", 3, 3, 0);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_interior.out", 0, 0, 0);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_unused.out", 0, 0, 0);
	CHECK_SLOT_ELEM_COUNTS(bm, &op, "geom_holes.out", 0, 0, 0);

	BMO_op_finish(bm, &op);
	BM_mesh_free(bm);
}

void unit_test_convex_hull(void)
{
	GROUP_HEADER();
	test_convex_hull_tetrahedron();
	test_convex_hull_interior_elems();
	test_convex_hull_unused_edge_and_face();
	test_convex_hull_interior_but_used();
	test_convex_hull_single_hole();
	test_convex_hull_one_edge();
	test_convex_hull_one_triangle();
	test_convex_hull_one_triangle_use_existing();
}

#endif
