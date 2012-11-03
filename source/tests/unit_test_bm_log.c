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

#include "MEM_guardedalloc.h"

#include "BLI_math.h"
#include "BLI_utildefines.h"

#include <assert.h>
#include <limits.h>

#include "bmesh.h"
#include "range_tree_c_api.h"
#include "unit_test_intern.h"

typedef struct {
	BMesh *bm;
	BMLog *log;
	RangeTreeUInt *unused_ids;
	BMLogEntry *entry;
} BMTestData;

static BMesh *bm_create(void)
{
	return BM_mesh_create(&bm_mesh_allocsize_default);
}

/* Create new BMTestData and initialize
 *
 * The result contains a new BMesh and BMLog. The BMLog is initialized
 * with a new BMLogEntry. */
static BMTestData *bm_test_data_create(void)
{
	BMTestData *td;

	td = MEM_callocN(sizeof(*td), AT);
	td->bm = bm_create();
	td->log = BM_log_create(td->bm);
	td->unused_ids = BM_log_unused_ids(td->log);
	td->entry = BM_log_entry_add(td->log);

	assert(td->bm);
	assert(td->log);
	assert(td->unused_ids);
	assert(td->entry);

	return td;
}

/* Free a BMTestData and its contents */
static void bm_test_data_free(BMTestData *td)
{
	BM_log_free(td->log);
	BM_mesh_free(td->bm);

	MEM_freeN(td);
}

static BMVert *bm_vert_from_co_create(BMTestData *td,
									  float x,
									  float y,
									  float z)
{
	const float co[3] = {x, y, z};
	return BM_vert_create(td->bm, co, NULL, 0);
}

static BMFace *bm_triangle_create(BMTestData *td,
								  BMVert *v1,
								  BMVert *v2,
								  BMVert *v3)
{
	return BM_face_create_quad_tri(td->bm, v1, v2, v3, NULL, NULL, FALSE);
}

#if 0
/* Test function naming convention: the name is a condensed list of
 * actions.
 *
 * 'E' means new entry
 * 'Nav' means add vertex N times
 * 'Ndv' means delete vertex N times
 * 'Nmv' means move vertex N times
 * 'Naf' means add face N times
 * 'Ndf' means delete face N times
 */

static void test_bm_log_E_1av_1dv(void)
{
	BMTestData *td = bm_test_data_create();
	BMVert *v;

	TEST_HEADER();

	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 0, UINT_MAX));
	
	v = bm_vert_create_co(td->bm, 0, 0, 0);
	BM_log_vert_added(td->log, v);

	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 1, UINT_MAX));

	BM_vert_kill(td->bm, v);
	BM_log_vert_removed(td->log, v);

	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 0, UINT_MAX));

	bm_test_data_free(td);
}

static void test_bm_log_E_1av_E_1dv(void)
{
	BMTestData *td = bm_test_data_create();
	BMVert *v;

	TEST_HEADER();

	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 0, UINT_MAX));
	
	v = bm_vert_create_co(td->bm, 0, 0, 0);
	BM_log_vert_added(td->log, v);

	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 1, UINT_MAX));

	BM_log_entry_add(td->log);

	BM_vert_kill(td->bm, v);
	BM_log_vert_removed(td->log, v);

	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 1, UINT_MAX));

	bm_test_data_free(td);
}

static void test_bm_log_E_3av_1af_1df(void)
{
	BMTestData *td = bm_test_data_create();
	BMVert *v;

	TEST_HEADER();

	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 0, UINT_MAX));
	
	v = bm_vert_create_co(td->bm, 0, 0, 0);
	BM_log_vert_added(td->log, v);

	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 1, UINT_MAX));

	BM_vert_kill(td->bm, v);
	BM_log_vert_removed(td->log, v);

	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 0, UINT_MAX));

	bm_test_data_free(td);
}
#endif

/* Probably a violation of good unit testing, but is a long test that
 * covers a lot of BMLog operations.
 *
 * Doesn't do anything with moving vertices, that goes in a separate
 * test. */
static void test_bm_log_narrative(void)
{
	BMTestData *td = bm_test_data_create();
	BMVert *v0, *v1, *v2;
	BMFace *f0;

	TEST_HEADER();

	/* Check that all possible IDs are free */
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 0, UINT_MAX));

	/* Create three new vertices and log them */
	v0 = bm_vert_from_co_create(td, 1, 0, 0);
	v1 = bm_vert_from_co_create(td, 0, 1, 0);
	v2 = bm_vert_from_co_create(td, 0, 0, 1);
	BM_log_vert_added(td->bm, td->log, v0);
	BM_log_vert_added(td->bm, td->log, v1);
	BM_log_vert_added(td->bm, td->log, v2);

	/* Check unused IDs */
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 3, UINT_MAX));

	/* Delete the second vertex */
	BM_vert_kill(td->bm, v1);
	BM_log_vert_removed(td->bm, td->log, v1);
	assert(range_tree_uint_size(td->unused_ids) == 2);
	assert(range_tree_uint_has_range(td->unused_ids, 1, 1));
	assert(range_tree_uint_has_range(td->unused_ids, 3, UINT_MAX));

	/* Re-create the deleted vertex */
	v1 = bm_vert_from_co_create(td, 0, 1, 0);
	BM_log_vert_added(td->bm, td->log, v1);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 3, UINT_MAX));

	/* Create a new face */
	f0 = bm_triangle_create(td, v0, v1, v2);
	BM_log_face_added(td->log, f0);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));

	/* Delete the new face */
	BM_face_kill(td->bm, f0);
	BM_log_face_removed(td->log, f0);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 3, UINT_MAX));

	/* Re-create the deleted face */
	f0 = bm_triangle_create(td, v0, v1, v2);
	BM_log_face_added(td->log, f0);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));

	/* Start a new log entry */
	BM_log_entry_add(td->log);

	/* Delete the face */
	BM_face_kill(td->bm, f0);
	BM_log_face_removed(td->log, f0);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));

	/* Delete all the vertices */
	BM_vert_kill(td->bm, v0);
	BM_vert_kill(td->bm, v1);
	BM_vert_kill(td->bm, v2);
	BM_log_vert_removed(td->bm, td->log, v0);
	BM_log_vert_removed(td->bm, td->log, v1);
	BM_log_vert_removed(td->bm, td->log, v2);
	assert(td->bm->totvert == 0);
	assert(td->bm->totface == 0);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));

	/* Undo and verify that all the deleted elements come back */
	BM_log_undo(td->bm, td->log);
	assert(td->bm->totvert == 3);
	assert(td->bm->totface == 1);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));

	/* Undo again and verify the mesh is empty */
	BM_log_undo(td->bm, td->log);
	assert(td->bm->totvert == 0);
	assert(td->bm->totface == 0);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));

	/* Undo again and verify that nothing changes */
	BM_log_undo(td->bm, td->log);
	assert(td->bm->totvert == 0);
	assert(td->bm->totface == 0);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));

	/* Redo and verify that everything gets re-added */
	BM_log_redo(td->bm, td->log);
	assert(td->bm->totvert == 3);
	assert(td->bm->totface == 1);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));

	/* Redo and verify that everything gets re-deleted */
	BM_log_redo(td->bm, td->log);
	assert(td->bm->totvert == 0);
	assert(td->bm->totface == 0);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));

	/* Undo and add a new entry */
	BM_log_undo(td->bm, td->log);
	BM_log_entry_add(td->log);
	assert(td->bm->totvert == 3);
	assert(td->bm->totface == 1);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));

	/* Verify that redoing doesn't do anything */
	BM_log_redo(td->bm, td->log);
	assert(td->bm->totvert == 3);
	assert(td->bm->totface == 1);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));

	bm_test_data_free(td);
}

static BMVert *bm_first_vert(BMesh *bm)
{
	BMIter bm_iter;
	BMVert *v;
	BM_ITER_MESH (v, &bm_iter, bm, BM_VERTS_OF_MESH)
		return v;
	return NULL;
}

/* Test various vertex move scenarios */
static void test_bm_log_vert_move(void)
{
	BMTestData *td = bm_test_data_create();
	BMVert *v;
	const float a[3] = {1, 2, 3};
	const float b[3] = {4, 5, 6};
	const float c[3] = {7, 8, 9};

	TEST_HEADER();

	/* Create a vertex and start new log entry */
	v = bm_vert_from_co_create(td, a[0], a[1], a[2]);
	BM_log_vert_added(td->bm, td->log, v);
	BM_log_entry_add(td->log);

	/* Move the vertex */
	BM_log_vert_before_modified(td->bm, td->log, v);
	copy_v3_v3(v->co, b);

	/* In a new entry, log another move then delete the vertex */
	BM_log_entry_add(td->log);
	BM_log_vert_before_modified(td->bm, td->log, v);
	copy_v3_v3(v->co, c);
	BM_vert_kill(td->bm, v);
	BM_log_vert_removed(td->bm, td->log, v);
	assert(bm_first_vert(td->bm) == NULL);

	/* Undo and verify the coordinates */
	BM_log_undo(td->bm, td->log);
	v = bm_first_vert(td->bm);
	assert(equals_v3v3(v->co, b));
	BM_log_undo(td->bm, td->log);
	v = bm_first_vert(td->bm);
	assert(equals_v3v3(v->co, a));

	/* Redo and verify the coordinates */
	BM_log_redo(td->bm, td->log);
	assert(equals_v3v3(v->co, b));
	BM_log_redo(td->bm, td->log);

	bm_test_data_free(td);
}

/* Test changing a vertex coordinate twice in a log entry */
static void test_bm_log_vert_multi_move(void)
{
	BMTestData *td = bm_test_data_create();
	BMVert *v;
	const float a[3] = {1, 2, 3};
	const float b[3] = {4, 5, 6};
	const float c[3] = {7, 8, 9};

	TEST_HEADER();

	/* Create a vertex and start new log entry */
	v = bm_vert_from_co_create(td, a[0], a[1], a[2]);
	BM_log_vert_added(td->bm, td->log, v);
	BM_log_entry_add(td->log);

	/* Move the vertex */
	BM_log_vert_before_modified(td->bm, td->log, v);
	copy_v3_v3(v->co, b);

	BM_log_vert_before_modified(td->bm, td->log, v);
	copy_v3_v3(v->co, c);

	/* Undo and verify the coordinates */
	BM_log_undo(td->bm, td->log);
	assert(equals_v3v3(v->co, a));

	/* Redo and verify the coordinates */
	BM_log_redo(td->bm, td->log);
	assert(equals_v3v3(v->co, c));

	bm_test_data_free(td);
}

/* Test changing a vertex coordinate in the same entry it was added in */
static void test_bm_log_vert_add_move(void)
{
	BMTestData *td = bm_test_data_create();
	BMVert *v;
	const float a[3] = {1, 2, 3};
	const float b[3] = {4, 5, 6};

	TEST_HEADER();

	/* Create a vertex */
	v = bm_vert_from_co_create(td, a[0], a[1], a[2]);
	BM_log_vert_added(td->bm, td->log, v);

	/* Move the vertex */
	BM_log_vert_before_modified(td->bm, td->log, v);
	copy_v3_v3(v->co, b);

	/* Undo */
	BM_log_undo(td->bm, td->log);
	assert(bm_first_vert(td->bm) == NULL);

	/* Redo and verify the coordinates */
	BM_log_redo(td->bm, td->log);
	v = bm_first_vert(td->bm);
	assert(equals_v3v3(v->co, b));

	bm_test_data_free(td);
}

/* Test getting a moved vertex's original location */
static void test_bm_log_vert_move_get_orig(void)
{
	BMTestData *td = bm_test_data_create();
	BMVert *v;
	const float a[3] = {1, 2, 3};
	const float b[3] = {4, 5, 6};

	TEST_HEADER();

	/* Create a vertex and start new log entry */
	v = bm_vert_from_co_create(td, a[0], a[1], a[2]);
	BM_log_vert_added(td->bm, td->log, v);
	BM_log_entry_add(td->log);

	/* Move the vertex */
	BM_log_vert_before_modified(td->bm, td->log, v);
	copy_v3_v3(v->co, b);

	/* Get original location */
	assert(equals_v3v3(BM_log_original_vert_co(td->log, v), a));

	bm_test_data_free(td);
}

/* Test creating a BMesh with elements, then adding a log to it */
static void test_bm_log_existing_elems(void)
{
	BMTestData *td = MEM_callocN(sizeof(*td), AT);
	BMVert *v0, *v1, *v2;

	TEST_HEADER();

	td->bm = bm_create();

	/* Create a triangle */
	v0 = bm_vert_from_co_create(td, 1, 0, 0);
	v1 = bm_vert_from_co_create(td, 0, 1, 0);
	v2 = bm_vert_from_co_create(td, 0, 0, 1);
	bm_triangle_create(td, v0, v1, v2);

	/* Create log */
	td->log = BM_log_create(td->bm);
	td->unused_ids = BM_log_unused_ids(td->log);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));

	bm_test_data_free(td);
}

/* Test functions for logging full replacement of BMesh elements */
static void test_bm_log_all_added_and_removed(void)
{
	BMTestData *td = bm_test_data_create();
	BMVert *v0, *v1, *v2;
	BMFace *f0;

	TEST_HEADER();

	/* Create triangle */
	v0 = bm_vert_from_co_create(td, 1, 0, 0);
	v1 = bm_vert_from_co_create(td, 0, 1, 0);
	v2 = bm_vert_from_co_create(td, 0, 0, 1);
	f0 = bm_triangle_create(td, v0, v1, v2);

	BM_log_all_added(td->bm, td->log);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));

	/* New entry, delete everything */
	BM_log_entry_add(td->log);
	BM_log_before_all_removed(td->bm, td->log);
	BM_face_kill(td->bm, f0);
	BM_vert_kill(td->bm, v0);
	BM_vert_kill(td->bm, v1);
	BM_vert_kill(td->bm, v2);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));
	assert(td->bm->totvert == 0);
	assert(td->bm->totface == 0);

	/* Undo and verify everything comes back */
	BM_log_undo(td->bm, td->log);
	assert(td->bm->totvert == 3);
	assert(td->bm->totface == 1);

	/* Redo and verify everything's gone again */
	BM_log_redo(td->bm, td->log);
	assert(td->bm->totvert == 0);
	assert(td->bm->totface == 0);

	bm_test_data_free(td);
}

/* Test dropping entries */
static void test_bm_log_entry_drop_first(void)
{
	BMTestData *td = bm_test_data_create();
	BMLogEntry *first_entry;

	TEST_HEADER();

	/* Drop the initial entry */
	BM_log_entry_drop(td->entry);
	assert(BM_log_current_entry(td->log) == NULL);
	assert(BM_log_length(td->log) == 0);

	/* Add new log entries */
	first_entry = BM_log_entry_add(td->log);
	assert(BM_log_current_entry(td->log) == first_entry);
	assert(BM_log_length(td->log) == 1);
	td->entry = BM_log_entry_add(td->log);
	assert(BM_log_current_entry(td->log) == td->entry);
	assert(BM_log_length(td->log) == 2);

	/* Drop the first */
	BM_log_entry_drop(first_entry);
	assert(BM_log_current_entry(td->log) == td->entry);
	assert(BM_log_length(td->log) == 1);

	bm_test_data_free(td);
}

/* Test that dropping entries frees up IDs */
static void test_bm_log_entry_drop_release_ids(void)
{
	BMTestData *td = bm_test_data_create();
	BMVert *v0, *v1, *v2;
	BMFace *f;
	BMLogEntry *entry1;

	TEST_HEADER();

	/* Create triangle and log all elements added */
	v0 = bm_vert_from_co_create(td, 1, 0, 0);
	v1 = bm_vert_from_co_create(td, 0, 1, 0);
	v2 = bm_vert_from_co_create(td, 0, 0, 1);
	f = bm_triangle_create(td, v0, v1, v2);
	BM_log_all_added(td->bm, td->log);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));

	/* In a new entry, delete all elements */
	entry1 = BM_log_entry_add(td->log);
	BM_log_before_all_removed(td->bm, td->log);
	BM_face_kill(td->bm, f);
	BM_vert_kill(td->bm, v0);
	BM_vert_kill(td->bm, v1);
	BM_vert_kill(td->bm, v2);

	/* Drop the first entry -- not much should change */
	assert(BM_log_length(td->log) == 2);
	BM_log_entry_drop(td->entry);
	assert(BM_log_current_entry(td->log) == entry1);
	assert(BM_log_length(td->log) == 1);
	assert(td->bm->totvert == 0);
	assert(td->bm->totface == 0);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 4, UINT_MAX));

	/* Drop the second entry, IDs should be released */
	BM_log_entry_drop(entry1);
	assert(BM_log_current_entry(td->log) == NULL);
	assert(BM_log_length(td->log) == 0);
	assert(td->bm->totvert == 0);
	assert(td->bm->totface == 0);
	assert(range_tree_uint_size(td->unused_ids) == 1);
	assert(range_tree_uint_has_range(td->unused_ids, 0, UINT_MAX));

	bm_test_data_free(td);
}

/* Test reordering mesh elements */
static void test_bm_log_vert_reorder(void)
{
	BMTestData *td = bm_test_data_create();
	BMVert *v, *v0, *v1, *v2;
	float co[3][3] = {
		{1, 0, 0},
		{2, 0, 0},
		{3, 0, 0}};

	TEST_HEADER();

	/* Log entry adding a single vertex */
	v0 = bm_vert_from_co_create(td, co[0][0], co[0][1], co[0][2]);
	BM_log_vert_added(td->bm, td->log, v0);

	/* Second log entry adds a single vertex */
	v1 = bm_vert_from_co_create(td, co[1][0], co[1][1], co[1][2]);
	BM_log_vert_added(td->bm, td->log, v1);
	BM_log_entry_add(td->log);

	/* Delete first vertex's log entry */
	BM_log_entry_drop(td->entry);
 
	/* Add another vertex, it's ID should take the first vertex's
	 * unused ID */
	v2 = bm_vert_from_co_create(td, co[2][0], co[2][1], co[2][2]);
	BM_log_vert_added(td->bm, td->log, v2);

	BM_vert_kill(td->bm, v0);

	/* Check current ordering */
	v = BM_iter_at_index(td->bm, BM_VERTS_OF_MESH, NULL, 0);
	assert(equals_v3v3(v->co, co[1]));
	v = BM_iter_at_index(td->bm, BM_VERTS_OF_MESH, NULL, 1);
	assert(equals_v3v3(v->co, co[2]));

	/* Check ID re-ordering */
	BM_log_mesh_elems_reorder(td->bm, td->log);
	v = BM_iter_at_index(td->bm, BM_VERTS_OF_MESH, NULL, 0);
	assert(equals_v3v3(v->co, co[2]));
	v = BM_iter_at_index(td->bm, BM_VERTS_OF_MESH, NULL, 1);
	assert(equals_v3v3(v->co, co[1 ]));
}

void unit_test_bm_log(void)
{
	GROUP_HEADER();
	test_bm_log_narrative();
	test_bm_log_vert_move();
	test_bm_log_vert_multi_move();
	test_bm_log_vert_add_move();
	test_bm_log_vert_move_get_orig();
	test_bm_log_existing_elems();
	test_bm_log_all_added_and_removed();
	test_bm_log_entry_drop_first();
	test_bm_log_entry_drop_release_ids();
	test_bm_log_vert_reorder();
}

/* NDEBUG */
#endif
