#include "BLI_listbase.h"

#include "bmesh.h"

#include <assert.h>

typedef enum {
	BM_TEST_UNEQUAL_TOTVERT,
	BM_TEST_UNEQUAL_TOTEDGE,
	BM_TEST_UNEQUAL_TOTFACE,

	BM_TEST_MISSING_VERT,
	BM_TEST_MISSING_EDGE,
	BM_TEST_MISSING_FACE,

	BM_TEST_UNEQUAL_VERT_HFLAG,
	BM_TEST_UNEQUAL_EDGE_HFLAG,
	BM_TEST_UNEQUAL_FACE_HFLAG,

	BM_TEST_UNEQUAL_VERT_CO,

	BM_TEST_UNEQUAL_EDGE_V1,
	BM_TEST_UNEQUAL_EDGE_V2,

	BM_TEST_UNEQUAL_FACE_LEN,
	BM_TEST_UNEQUAL_FACE_VERT,
	BM_TEST_UNEQUAL_FACE_EDGE,

	BM_TEST_EQUAL
} BMTestEqualResult;

static const char *bm_test_equal_result_str(BMTestEqualResult error)
{
	switch(error) {
	#define CASE(val_) \
		case val_: \
			return #val_

		CASE(BM_TEST_UNEQUAL_TOTVERT);
		CASE(BM_TEST_UNEQUAL_TOTEDGE);
		CASE(BM_TEST_UNEQUAL_TOTFACE);

		CASE(BM_TEST_MISSING_VERT);
		CASE(BM_TEST_MISSING_EDGE);
		CASE(BM_TEST_MISSING_FACE);

		CASE(BM_TEST_UNEQUAL_VERT_HFLAG);
		CASE(BM_TEST_UNEQUAL_EDGE_HFLAG);
		CASE(BM_TEST_UNEQUAL_FACE_HFLAG);

		CASE(BM_TEST_UNEQUAL_VERT_CO);

		CASE(BM_TEST_UNEQUAL_EDGE_V1);
		CASE(BM_TEST_UNEQUAL_EDGE_V2);

		CASE(BM_TEST_UNEQUAL_FACE_LEN);
		CASE(BM_TEST_UNEQUAL_FACE_VERT);
		CASE(BM_TEST_UNEQUAL_FACE_EDGE);

		CASE(BM_TEST_EQUAL);

	#undef CASE
	}

	return "UNKNOWN";
}

static void bmesh_print(BMesh *bm)
{
	BMIter iter, siter;
	BMVert *v;
	BMEdge *e;
	BMFace *f;

	fprintf(stderr, "\nbm=%p, totvert=%d, totedge=%d, "
			"totloop=%d, totface=%d\n",
			bm, bm->totvert, bm->totedge,
			bm->totloop, bm->totface);

	fprintf(stderr, "vertices:\n");
	BM_ITER_MESH(v, &iter, bm, BM_VERTS_OF_MESH) {
		fprintf(stderr, "  %p id=%d co=(%.3f %.3f %.3f) hflag=%d\n",
				v, v->head.id,
				v->co[0], v->co[1], v->co[2],
				v->head.hflag);
	}

	fprintf(stderr, "edges:\n");
	BM_ITER_MESH(e, &iter, bm, BM_EDGES_OF_MESH) {
		fprintf(stderr, "  %p v1=%p, v2=%p, id=%d\n",
				e, e->v1, e->v2,
				e->head.id);
	}

	fprintf(stderr, "faces:\n");
	BM_ITER_MESH(f, &iter, bm, BM_FACES_OF_MESH) {
		fprintf(stderr, "  %p len=%d, id=%d\n",
				f, f->len,
				f->head.id);

		fprintf(stderr, "    v: ");
		BM_ITER_ELEM(v, &siter, f, BM_VERTS_OF_FACE) {
			fprintf(stderr, "%p ", v);
		}
		fprintf(stderr, "\n");

		fprintf(stderr, "    e: ");
		BM_ITER_ELEM(e, &siter, f, BM_EDGES_OF_FACE) {
			fprintf(stderr, "%p ", e);
		}
		fprintf(stderr, "\n");
	}
}

/* Separate function to make adding breakpoints easy */
static BMTestEqualResult catch_error(BMTestEqualResult error)
{
	if (error != BM_TEST_EQUAL) {
		fprintf(stderr, "\nERROR: bm_test_equal failed (%s)\n\n",
				bm_test_equal_result_str(error));
	}
	return error;
}

#define MAX_FACE_LEN 64

/* Test if the elements' hflags are equal, ignoing for now the
 * internal flag */
static int bm_test_hflag_equal(const BMHeader *a, const BMHeader *b)
{
	return ((a->hflag & ~BM_ELEM_INTERNAL_TAG) ==
			(b->hflag & ~BM_ELEM_INTERNAL_TAG));
}

static BMTestEqualResult bm_test_equal(BMesh *a, BMesh *b)
{
	BMIter bm_iter;
	BMVert *av, *bv;
	BMEdge *ae, *be;
	BMFace *af, *bf;

	/* Compare number of elements */
	if (a->totvert != b->totvert)
		return catch_error(BM_TEST_UNEQUAL_TOTVERT);
	if (a->totedge != b->totedge)
		return catch_error(BM_TEST_UNEQUAL_TOTEDGE);
	if (a->totface != b->totface)
		return catch_error(BM_TEST_UNEQUAL_TOTFACE);

	/* Compare vertices */
	BM_ITER_MESH(av, &bm_iter, a, BM_VERTS_OF_MESH) {
		bv = bm_id_lookup(b, av->head.id);
		assert(av != bv);
		assert(bv->head.htype == BM_VERT);

		if (!bv)
			return catch_error(BM_TEST_MISSING_VERT);
		if (!bm_test_hflag_equal(&av->head, &bv->head))
			return catch_error(BM_TEST_UNEQUAL_VERT_HFLAG);

		if (!equals_v3v3(av->co, bv->co))
			return catch_error(BM_TEST_UNEQUAL_VERT_CO);
	}

	/* Compare edges */
	BM_ITER_MESH(ae, &bm_iter, a, BM_EDGES_OF_MESH) {
		be = bm_id_lookup(b, ae->head.id);
		assert(ae != be);
		assert(be->head.htype == BM_EDGE);

		if (!be)
			return catch_error(BM_TEST_MISSING_EDGE);
		if (ae->head.hflag != be->head.hflag)
			return catch_error(BM_TEST_UNEQUAL_EDGE_HFLAG);

		if (ae->v1->head.id != be->v1->head.id)
			return catch_error(BM_TEST_UNEQUAL_EDGE_V1);
		if (ae->v2->head.id != be->v2->head.id)
			return catch_error(BM_TEST_UNEQUAL_EDGE_V2);
	}
	
	/* Compare faces */
	BM_ITER_MESH(af, &bm_iter, a, BM_FACES_OF_MESH) {
		BMVert *afv[MAX_FACE_LEN], *bfv[MAX_FACE_LEN];
		BMEdge *afe[MAX_FACE_LEN], *bfe[MAX_FACE_LEN];
		int i;

		bf = bm_id_lookup(b, af->head.id);
		assert(af != bf);
		assert(bf->head.htype == BM_FACE);

		if (!bf)
			return catch_error(BM_TEST_MISSING_FACE);
		if (af->head.hflag != bf->head.hflag)
			return catch_error(BM_TEST_UNEQUAL_FACE_HFLAG);
		if (af->len != bf->len)
			return catch_error(BM_TEST_UNEQUAL_FACE_LEN);

		/* Hardcoded maximum so we don't have to malloc here */
		assert(af->len < MAX_FACE_LEN);

		BM_iter_as_array(NULL, BM_VERTS_OF_FACE, af, (void*)afv, MAX_FACE_LEN);
		BM_iter_as_array(NULL, BM_VERTS_OF_FACE, bf, (void*)bfv, MAX_FACE_LEN);
		BM_iter_as_array(NULL, BM_EDGES_OF_FACE, af, (void*)afe, MAX_FACE_LEN);
		BM_iter_as_array(NULL, BM_EDGES_OF_FACE, bf, (void*)bfe, MAX_FACE_LEN);

		for (i = 0; i < af->len; i++) {
			assert(afv[i] != bfv[i]);
			if (afv[i]->head.id != bfv[i]->head.id)
				return catch_error(BM_TEST_UNEQUAL_FACE_VERT);
			if (afe[i]->head.id != bfe[i]->head.id)
				return catch_error(BM_TEST_UNEQUAL_FACE_EDGE);
		}
	}

	return BM_TEST_EQUAL;
}

static BMesh *create_test_bmesh(void)
{
	BMesh *bm = BM_mesh_create(&bm_mesh_allocsize_default);
	BM_mesh_enable_logging(bm);
	return bm;
}

/* Undo back to the the original mesh, checking at each undo step that
   the result matches the corresponding mesh copy. Then redo back to
   the final mesh, again checking that each step matches the stored
   mesh copy. */
static void check_history(BMesh *bm, ListBase *copies)
{
	LinkData *link;

	for (link = copies->last; link; link = link->prev) {
		/* Check that the original matches the current log step, then undo */
		assert(bm_test_equal(bm, link->data) == BM_TEST_EQUAL);
		bm_undo(bm);
	}

	/* Reverse course, redo and check each step */
	for (link = copies->first; link; link = link->next) {
		bm_redo(bm);
		assert(bm_test_equal(bm, link->data) == BM_TEST_EQUAL);
	}
}

static void free_copies(ListBase *copies)
{
	LinkData *link;
	for (link = copies->first; link; link = link->next) {
		BM_mesh_free(link->data);
	}
}

#define BM_LOG_TEST_INIT() \
	ListBase lcopies = {NULL, NULL}; \
	ListBase *copies = &lcopies; \
	printf("Starting test: %s\n", __func__)

/* Start a new log group, run the operation, then store a copy of the
   mesh */
#define BM_LOG_TEST_STEP(op_) \
	{ \
	bm_log_group_create(bm->log, "test"); \
	op_; \
	BLI_addtail(copies, BLI_genericNodeN(BM_mesh_copy(bm))); \
	}

#define BM_LOG_TEST_FINISH() \
	check_history(bm, copies); \
	free_copies(copies); \
	BLI_freelistN(copies); \
	BM_mesh_free(bm)

#define BM_LOG_TEST_TRIANGLE(v1, v2, v3, e1, e2, e3) \
	fv[0] = v[v1]; \
	fv[1] = v[v2]; \
	fv[2] = v[v3]; \
	fe[0] = e[e1]; \
	fe[1] = e[e2]; \
	fe[2] = e[e3];

/* Create a diamond shape with six vertices, eleven edges, and six
   faces */
static BMesh *create_double_diamond(ListBase *copies)
{
	BMesh *bm = create_test_bmesh();
	const float co[6][3] = {
		{0, 1, 0},
		{2, 0, 0},
		{0, -1, 0},
		{-2, 0, 0},
		{-1, 0, 0},
		{1, 0, 0}};
	BMVert *v[6], *fv[3];
	BMEdge *e[11], *fe[3];
	int i;

	/* Vertex IDs [0, 5] */
	for (i = 0; i < 6; i++)
		BM_LOG_TEST_STEP(v[i] = BM_vert_create(bm, co[0], NULL));

	/* Edge IDs [6, 16] */
	BM_LOG_TEST_STEP(e[0] = BM_edge_create(bm, v[0], v[1], NULL, FALSE));
	BM_LOG_TEST_STEP(e[1] = BM_edge_create(bm, v[1], v[2], NULL, FALSE));
	BM_LOG_TEST_STEP(e[2] = BM_edge_create(bm, v[2], v[3], NULL, FALSE));
	BM_LOG_TEST_STEP(e[3] = BM_edge_create(bm, v[3], v[0], NULL, FALSE));
	BM_LOG_TEST_STEP(e[4] = BM_edge_create(bm, v[0], v[5], NULL, FALSE));
	BM_LOG_TEST_STEP(e[5] = BM_edge_create(bm, v[2], v[5], NULL, FALSE));
	BM_LOG_TEST_STEP(e[6] = BM_edge_create(bm, v[2], v[4], NULL, FALSE));
	BM_LOG_TEST_STEP(e[7] = BM_edge_create(bm, v[0], v[4], NULL, FALSE));
	BM_LOG_TEST_STEP(e[8] = BM_edge_create(bm, v[3], v[4], NULL, FALSE));
	BM_LOG_TEST_STEP(e[9] = BM_edge_create(bm, v[4], v[5], NULL, FALSE));
	BM_LOG_TEST_STEP(e[10] = BM_edge_create(bm, v[5], v[1], NULL, FALSE));

	/* Face IDs [17, 22] */
	BM_LOG_TEST_TRIANGLE(0, 4, 3, 7, 8, 3);
	BM_LOG_TEST_STEP(BM_face_create(bm, fv, fe, 3, FALSE));
	BM_LOG_TEST_TRIANGLE(0, 5, 4, 4, 9, 7);
	BM_LOG_TEST_STEP(BM_face_create(bm, fv, fe, 3, FALSE));
	BM_LOG_TEST_TRIANGLE(0, 1, 5, 0, 10, 4);
	BM_LOG_TEST_STEP(BM_face_create(bm, fv, fe, 3, FALSE));
	BM_LOG_TEST_TRIANGLE(2, 3, 4, 2, 8, 6);
	BM_LOG_TEST_STEP(BM_face_create(bm, fv, fe, 3, FALSE));
	BM_LOG_TEST_TRIANGLE(2, 4, 5, 6, 9, 5);
	BM_LOG_TEST_STEP(BM_face_create(bm, fv, fe, 3, FALSE));
	BM_LOG_TEST_TRIANGLE(2, 5, 1, 5, 10, 1);
	BM_LOG_TEST_STEP(BM_face_create(bm, fv, fe, 3, FALSE));

	return bm;
}

static void bm_log_test_create_0(void)
{
	BMesh *bm;

	BM_LOG_TEST_INIT();

	bm = create_double_diamond(copies);

	BM_LOG_TEST_FINISH();
}

#if 0
static void bm_log_test_splice(void)
{
	BMesh *bm;
	BMVert *v1, *v2;
	BMEdge *e;

	BM_LOG_TEST_INIT();

	bm = create_double_diamond(copies);
	v1 = bm_id_lookup(bm, 4);
	v2 = bm_id_lookup(bm, 5);
	e = BM_edge_exists(v1, v2);
	BM_LOG_TEST_STEP((BM_vert_splice(bm, v1, v2),
					  bmesh_print(bm),
					  BM_edge_kill(bm, e)));

	BM_LOG_TEST_FINISH();
}
#endif

/***************************** SEMV / JEKV ****************************/

static void bm_log_test_semv_simple(void)
{
	BMesh *bm = create_test_bmesh();
	BMVert *v[2], *nv;
	BMEdge *e, *re1, *re2, *re3;
	float co[2][3] = {{-1, 0, 0},
					  { 1, 0, 0}};

	BM_LOG_TEST_INIT();

	/* Create a single edge and split it */
	BM_LOG_TEST_STEP(v[0] = BM_vert_create(bm, co[0], NULL));
	BM_LOG_TEST_STEP(v[1] = BM_vert_create(bm, co[1], NULL));
	BM_LOG_TEST_STEP(e = BM_edge_create(bm, v[0], v[1], NULL, FALSE));
	BM_LOG_TEST_STEP(nv = bmesh_semv(bm, v[1], e, &re1));

	/* Split both output edges */
	BM_LOG_TEST_STEP(bmesh_semv(bm, nv, e, &re2));
	BM_LOG_TEST_STEP(bmesh_semv(bm, v[1], re1, &re3));

	BM_LOG_TEST_FINISH();
}

static void bm_log_test_semv_with_faces(void)
{
	BMesh *bm = create_test_bmesh();
	BMVert *ev[2], *fv[3], *nv;
	BMEdge *e, *re1, *re2, *re3;
	float co[5][3] = {{-1,      0,  0},
					  { 1,      0,  0},
					  { 0.5,    1,  0},
					  { 0.5, -0.5, -1},
					  { 0.5, -0.5,  1}};

	BM_LOG_TEST_INIT();

	/* Create an edge */
	BM_LOG_TEST_STEP(ev[0] = BM_vert_create(bm, co[0], NULL));
	BM_LOG_TEST_STEP(ev[1] = BM_vert_create(bm, co[1], NULL));
	BM_LOG_TEST_STEP(e = BM_edge_create(bm, ev[0], ev[1], NULL, FALSE));

	/* Create three faces around the edge (ala shuttle Tydirium) */
	BM_LOG_TEST_STEP(fv[0] = BM_vert_create(bm, co[2], NULL));
	BM_LOG_TEST_STEP(fv[1] = BM_vert_create(bm, co[3], NULL));
	BM_LOG_TEST_STEP(fv[2] = BM_vert_create(bm, co[4], NULL));
	BM_LOG_TEST_STEP(BM_face_create_quad_tri(bm, ev[0], ev[1], fv[0], NULL, NULL, FALSE));
	BM_LOG_TEST_STEP(BM_face_create_quad_tri(bm, ev[0], ev[1], fv[1], NULL, NULL, FALSE));
	BM_LOG_TEST_STEP(BM_face_create_quad_tri(bm, ev[0], ev[1], fv[2], NULL, NULL, FALSE));

	/* Split the original edge */
	BM_LOG_TEST_STEP(nv = bmesh_semv(bm, ev[1], e, &re1));
	
	/* Split both output edges */
	BM_LOG_TEST_STEP(bmesh_semv(bm, nv, e, &re2));
	BM_LOG_TEST_STEP(bmesh_semv(bm, ev[1], re1, &re3));

	BM_LOG_TEST_FINISH();
}

/**************************** Vertex Splice ***************************/

#if 0
static void bm_log_test_vert_splice_simple(void)
{
	BMesh *bm = create_test_bmesh();
	BMVert *v[2];
	float co[2][3] = {{-1, 0, 0},
					  { 1, 0, 0}};

	BM_LOG_TEST_INIT();

	/* Create two isolated verts and splice */
	BM_LOG_TEST_STEP(v[0] = BM_vert_create(bm, co[0], NULL));
	BM_LOG_TEST_STEP(v[1] = BM_vert_create(bm, co[1], NULL));
	BM_LOG_TEST_STEP(BM_vert_splice(bm, v[0], v[1]));

	/* Create an edge and splice its endpoints */
	/*BM_LOG_TEST_STEP(v[0] = BM_vert_create(bm, co[0], NULL));
	BM_LOG_TEST_STEP(v[1] = BM_vert_create(bm, co[1], NULL));
	BM_LOG_TEST_STEP(BM_edge_create(bm, v[0], v[1], NULL, FALSE));*/
	//BM_LOG_TEST_STEP(BM_vert_splice(bm, v[0], v[1]));

	BM_LOG_TEST_FINISH();
}
#endif

/******************************** SF/JF *******************************/

/* - Create a quad and a separate edge that connects one of the quads
 *   diagonals
 * 
 * - Split the quad along that edge
 *
 * - Rejoin the resulting triangles into a quad
 */
static void bm_log_test_sf_jf_0(void)
{
	BMesh *bm = create_test_bmesh();
	/* Create four vertices, coordinates don't matter */
	float co[3] = {0, 0, 0};
	BMVert *qv[4];
	BMFace *f, *f_new, *f_orig;
	BMEdge *e_diag;
	BMIter bm_iter;
	int i;

	BM_LOG_TEST_INIT();

	for (i = 0; i < 4; i++)
		BM_LOG_TEST_STEP(qv[i] = BM_vert_create(bm, co, NULL));

	/* Create a quad */
	BM_LOG_TEST_STEP(f_orig = BM_face_create_quad_tri_v(bm, qv, 4, NULL, FALSE));

	/* Create an edge across one diagonal */
	BM_LOG_TEST_STEP(e_diag = BM_edge_create(bm, qv[0], qv[2], NULL, FALSE));

	/* Split the quad */
	BM_LOG_TEST_STEP(f_new = bmesh_sf(bm, f_orig, e_diag, NULL));
	assert(f_new);

	/* Basic checks, could do more */
	assert(bm->totvert == 4);
	assert(bm->totedge == 5);
	assert(bm->totface == 2);
	BM_ITER_MESH (f, &bm_iter, bm, BM_FACES_OF_MESH) {
		assert(f->len == 3);
	}

	/* Rejoin the triangles into a quad */
	BM_LOG_TEST_STEP(bmesh_jf(bm, f_orig, f_new, e_diag));

	/* Basic checks, could do more */
	assert(bm->totvert == 4);
	assert(bm->totedge == 5);
	assert(bm->totface == 1);
	BM_ITER_MESH (f, &bm_iter, bm, BM_FACES_OF_MESH) {
		assert(f->len == 4);
	}
	assert(BM_edge_face_count(e_diag) == 0);

	BM_LOG_TEST_FINISH();
}

/* - Create two quads that share three vertices (and the edges between
 *   them)
 *
 * - Create a single new edge along the diagonal using vertices shared
 *   by both faces
 *
 * - Split both quads into triangles with the same edge
 *
 * - Rejoin the split triangles back into the original quad pattern
 */
static void bm_log_test_sf_jf_1(void)
{
	BMesh *bm = create_test_bmesh();
	/* Create five vertices, coordinates don't matter */
	float co[3] = {0, 0, 0};
	BMVert *qv1[4] = {
		BM_vert_create(bm, co, NULL),
		BM_vert_create(bm, co, NULL),
		BM_vert_create(bm, co, NULL),
		BM_vert_create(bm, co, NULL),
	};
	BMVert *qv2[4] = {
		BM_vert_create(bm, co, NULL),
		qv1[2],
		qv1[1],
		qv1[0]
	};
	BMFace *f, *f_new[2], *f_orig[2];
	BMEdge *e_diag;
	BMIter bm_iter;

	BM_LOG_TEST_INIT();

	/* Create a quad */
	f_orig[0] = BM_face_create_quad_tri_v(bm, qv1, 4, NULL, FALSE);
	f_orig[1] = BM_face_create_quad_tri_v(bm, qv2, 4, NULL, FALSE);

	/* Create an edge across the diagonal */
	e_diag = BM_edge_create(bm, qv1[0], qv1[2], NULL, FALSE);

	/* Split both quads */
	f_new[0] = bmesh_sf(bm, f_orig[0], e_diag, NULL);
	f_new[1] = bmesh_sf(bm, f_orig[1], e_diag, NULL);
	assert(f_new[0] && f_new[1]);

	/* Basic checks, could do more */
	assert(bm->totvert == 5);
	assert(bm->totedge == 7);
	assert(bm->totface == 4);
	BM_ITER_MESH (f, &bm_iter, bm, BM_FACES_OF_MESH) {
		assert(f->len == 3);
	}

	/* Rejoin the triangles into quads */
	bmesh_jf(bm, f_orig[0], f_new[0], e_diag);
	bmesh_jf(bm, f_orig[1], f_new[1], e_diag);

	/* Basic checks, could do more */
	assert(bm->totvert == 5);
	assert(bm->totedge == 7);
	assert(bm->totface == 2);
	BM_ITER_MESH (f, &bm_iter, bm, BM_FACES_OF_MESH) {
		assert(f->len == 4);
	}
	assert(BM_edge_face_count(e_diag) == 0);

	BM_LOG_TEST_FINISH();
}

/**********************************************************************/

void bm_log_run_all_tests(void)
{
	/* Create/kill for verts, edges, and faces */
	bm_log_test_create_0();

	/* SEMV/JEKV */
	bm_log_test_semv_simple();
	bm_log_test_semv_with_faces();

	/* Vertex Splice */
	//bm_log_test_vert_splice_simple();
	//bm_log_test_splice();

	/* SF/JF */
	bm_log_test_sf_jf_0();
	bm_log_test_sf_jf_1();

	printf("\nXXX: all BM log tests passed\n\n");
}
