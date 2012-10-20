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

/** \file blender/bmesh/operators/bmo_hull.c
 *  \ingroup bmesh
 */

#include "MEM_guardedalloc.h"

#include "BLI_array.h"
#include "BLI_ghash.h"
#include "BLI_listbase.h"
#include "BLI_math.h"
#include "BLI_utildefines.h"

#include "Bullet-C-Api.h"

/*XXX: This operator doesn't work well (at all?) for flat surfaces with
 * >3 sides - creating overlapping faces at times.
 * An easy workaround is to add in some noise but this is
 * weak and unreliable, ideally this would detect flat surfaces
 * (possibly making them into ngons) - see
 */

/* XXX: using 128 for totelem and pchunk of mempool, no idea what good
 * values would be though */
#include "BLI_mempool.h"

#include "bmesh.h"

#include "intern/bmesh_operators_private.h"  /* own include */

#define HULL_EPSILON_FLT 0.0001f
/* values above 0.0001 cause errors, see below for details, don't increase
 * without checking against bug [#32027] */
#define HULL_EPSILON_DOT_FLT 0.00000001f

/* Internal operator flags */
typedef enum {
	HULL_FLAG_INPUT =           (1 << 0),
	HULL_FLAG_TETRA_VERT =      (1 << 1),
	
	HULL_FLAG_INTERIOR_ELE =    (1 << 2),
	HULL_FLAG_OUTPUT_GEOM =     (1 << 3),
	
	HULL_FLAG_DEL =             (1 << 4),
	HULL_FLAG_HOLE =            (1 << 5)
} HullFlags;

/* Store hull triangles separate from BMesh faces until the end; this
 * way we don't have to worry about cleaning up extraneous edges or
 * incorrectly deleting existing geometry. */
typedef struct HullTriangle {
	BMVert *v[3];
	float no[3];
	int skip;
} HullTriangle;

/* These edges define the hole created in the hull by deleting faces
 * that can "see" a new vertex (the boundary edges then form the edge
 * of a new triangle fan that has the new vertex as its center) */
typedef struct HullBoundaryEdge {
	struct HullBoundaryEdge *next, *prev;
	BMVert *v[2];
} HullBoundaryEdge;



/*************************** Boundary Edges ***************************/

static int edge_match(BMVert *e1_v1, BMVert *e1_v2, BMVert *e2[2])
{
	return (e1_v1 == e2[0] && e1_v2 == e2[1]) ||
	       (e1_v1 == e2[1] && e1_v2 == e2[0]);
}

/* Returns true if the edge (e1, e2) is already in edges; that edge is
 * deleted here as well. if not found just returns 0 */
static int check_for_dup(ListBase *edges, BLI_mempool *pool,
                         BMVert *v1, BMVert *v2)
{
	HullBoundaryEdge *e, *e_next;

	for (e = edges->first; e; e = e_next) {
		e_next = e->next;

		if (edge_match(v1, v2, e->v)) {
			/* remove the interior edge */
			BLI_remlink(edges, e);
			BLI_mempool_free(pool, e);
			return 1;
		}
	}

	return 0;
}

static void expand_boundary_edges(ListBase *edges, BLI_mempool *edge_pool,
                                  const HullTriangle *t)
{
	HullBoundaryEdge *e_new;
	int i;

	/* Insert each triangle edge into the boundary list; if any of
	 * its edges are already in there, remove the edge entirely */
	for (i = 0; i < 3; i++) {
		if (!check_for_dup(edges, edge_pool, t->v[i], t->v[(i + 1) % 3])) {
			e_new = BLI_mempool_calloc(edge_pool);
			e_new->v[0] = t->v[i];
			e_new->v[1] = t->v[(i + 1) % 3];
			BLI_addtail(edges, e_new);
		}
	}
}



/*************************** Hull Triangles ***************************/

static void hull_add_triangle(BMesh *bm, GHash *hull_triangles, BLI_mempool *pool,
                              BMVert *v1, BMVert *v2, BMVert *v3)
{
	HullTriangle *t;
	int i;

	t = BLI_mempool_calloc(pool);
	t->v[0] = v1;
	t->v[1] = v2;
	t->v[2] = v3;

	/* Mark triangles vertices as not interior */
	for (i = 0; i < 3; i++)
		BMO_elem_flag_disable(bm, t->v[i], HULL_FLAG_INTERIOR_ELE);

	BLI_ghash_insert(hull_triangles, t, NULL);
	normal_tri_v3(t->no, v1->co, v2->co, v3->co);
}

static int hull_point_tri_side(const HullTriangle *t, const float co[3])
{
	/* Added epsilon to fix bug [#31941], improves output when some
	 * vertices are nearly coplanar. Might need further tweaking for
	 * other cases though.
	 * ...
	 * Update: epsilon of 0.0001 causes [#32027], use HULL_EPSILON_DOT_FLT
	 * and give it a much smaller value
	 * */
	float p[3], d;
	sub_v3_v3v3(p, co, t->v[0]->co);
	d = dot_v3v3(t->no, p);
	if      (d < -HULL_EPSILON_DOT_FLT) return -1;
	else if (d >  HULL_EPSILON_DOT_FLT) return  1;
	else return 0;
}

/* Get all hull triangles that vertex 'v' is outside of */
static GHash *hull_triangles_v_outside(GHash *hull_triangles, const BMVert *v)
{
	GHash *outside;
	GHashIterator iter;

	outside = BLI_ghash_ptr_new("outside");

	GHASH_ITER (iter, hull_triangles) {
		HullTriangle *t = BLI_ghashIterator_getKey(&iter);
		
		if (hull_point_tri_side(t, v->co) > 0)
			BLI_ghash_insert(outside, t, NULL);
	}

	return outside;
}

/* For vertex 'v', find which triangles must be deleted to extend the
 * hull; find the boundary edges of that hole so that it can be filled
 * with connections to the new vertex, and update the hull_triangles
 * to delete the marked triangles */
static void add_point(BMesh *bm, GHash *hull_triangles, BLI_mempool *hull_pool,
                      BLI_mempool *edge_pool, GHash *outside, BMVert *v)
{
	ListBase edges = {NULL, NULL};
	HullBoundaryEdge *e, *e_next;
	GHashIterator iter;

	GHASH_ITER (iter, outside) {
		HullTriangle *t = BLI_ghashIterator_getKey(&iter);
		int i;
		
		expand_boundary_edges(&edges, edge_pool, t);

		/* Mark triangle's vertices as interior */
		for (i = 0; i < 3; i++)
			BMO_elem_flag_enable(bm, t->v[i], HULL_FLAG_INTERIOR_ELE);
		
		/* Delete the triangle */
		BLI_ghash_remove(hull_triangles, t, NULL, NULL);
		BLI_mempool_free(hull_pool, t);
	}

	/* Fill hole boundary with triangles to new point */
	for (e = edges.first; e; e = e_next) {
		e_next = e->next;
		hull_add_triangle(bm, hull_triangles, hull_pool, e->v[0], e->v[1], v);
		BLI_mempool_free(edge_pool, e);
	}
}

static BMFace *hull_find_example_face(BMesh *bm, BMEdge *e)
{
	BMIter iter;
	BMFace *f;

	BM_ITER_ELEM (f, &iter, e, BM_FACES_OF_EDGE) {
		if (BMO_elem_flag_test(bm, f, HULL_FLAG_INPUT) ||
		    !BMO_elem_flag_test(bm, f, HULL_FLAG_OUTPUT_GEOM))
		{
			return f;
		}
	}

	return NULL;
}

static void hull_output_triangles(BMesh *bm, GHash *hull_triangles)
{
	GHashIterator iter;
	
	GHASH_ITER (iter, hull_triangles) {
		HullTriangle *t = BLI_ghashIterator_getKey(&iter);

		if (!t->skip) {
			BMEdge *edges[3] = {
				BM_edge_create(bm, t->v[0], t->v[1], NULL, TRUE),
				BM_edge_create(bm, t->v[1], t->v[2], NULL, TRUE),
				BM_edge_create(bm, t->v[2], t->v[0], NULL, TRUE)
			};
			BMFace *f, *example = NULL;
			int i;

			/* Look for an adjacent face that existed before the hull */
			for (i = 0; i < 3; i++) {
				if (!example)
					example = hull_find_example_face(bm, edges[i]);
			}

			f = BM_face_create_quad_tri_v(bm, t->v, 3, example, FALSE);
			BM_face_copy_shared(bm, f);

			/* Mark face/verts/edges for 'geomout' slot and select */
			BMO_elem_flag_enable(bm, f, HULL_FLAG_OUTPUT_GEOM);
			BM_face_select_set(bm, f, TRUE);
			for (i = 0; i < 3; i++) {
				BMO_elem_flag_enable(bm, t->v[i], HULL_FLAG_OUTPUT_GEOM);
				BMO_elem_flag_enable(bm, edges[i], HULL_FLAG_OUTPUT_GEOM);
			}
		}
	}
}



/***************************** Final Edges ****************************/

typedef struct {
	GHash *edges;
	BLI_mempool *base_pool, *link_pool;
} HullFinalEdges;

static LinkData *final_edges_find_link(ListBase *adj, BMVert *v)
{
	LinkData *link;

	for (link = adj->first; link; link = link->next) {
		if (link->data == v)
			return link;
	}

	return NULL;
}

static int hull_final_edges_lookup(HullFinalEdges *final_edges,
                                   BMVert *v1, BMVert *v2)
{
	ListBase *adj;

	/* Use lower vertex pointer for hash key */
	if (v1 > v2)
		SWAP(BMVert *, v1, v2);

	adj = BLI_ghash_lookup(final_edges->edges, v1);
	if (!adj)
		return FALSE;

	return !!final_edges_find_link(adj, v2);
}

/* Used for checking whether a pre-existing edge lies on the hull */
static HullFinalEdges *hull_final_edges(GHash *hull_triangles)
{
	HullFinalEdges *final_edges;
	GHashIterator iter;
	
	final_edges = MEM_callocN(sizeof(HullFinalEdges), "HullFinalEdges");
	final_edges->edges = BLI_ghash_ptr_new("final edges ghash");
	final_edges->base_pool = BLI_mempool_create(sizeof(ListBase), 128, 128, 0);
	final_edges->link_pool = BLI_mempool_create(sizeof(LinkData), 128, 128, 0);

	GHASH_ITER (iter, hull_triangles) {
		LinkData *link;
		int i;
		
		for (i = 0; i < 3; i++) {
			HullTriangle *t = BLI_ghashIterator_getKey(&iter);
			BMVert *v1 = t->v[i];
			BMVert *v2 = t->v[(i + 1) % 3];
			ListBase *adj;

			/* Use lower vertex pointer for hash key */
			if (v1 > v2)
				SWAP(BMVert *, v1, v2);

			adj = BLI_ghash_lookup(final_edges->edges, v1);
			if (!adj) {
				adj = BLI_mempool_calloc(final_edges->base_pool);
				BLI_ghash_insert(final_edges->edges, v1, adj);
			}

			if (!final_edges_find_link(adj, v2)) {
				link = BLI_mempool_calloc(final_edges->link_pool);
				link->data = v2;
				BLI_addtail(adj, link);
			}
		}
	}

	return final_edges;
}

static void hull_final_edges_free(HullFinalEdges *final_edges)
{
	BLI_ghash_free(final_edges->edges, NULL, NULL);
	BLI_mempool_destroy(final_edges->base_pool);
	BLI_mempool_destroy(final_edges->link_pool);
	MEM_freeN(final_edges);
}



/************************* Initial Tetrahedron ************************/

static void hull_add_tetrahedron(BMesh *bm, GHash *hull_triangles, BLI_mempool *pool,
                                 BMVert *tetra[4])
{
	float center[3];
	int i, indices[4][3] = {
		{0, 1, 2},
		{0, 2, 3},
		{1, 0, 3},
		{2, 1, 3}
	};

	/* Calculate center */
	zero_v3(center);
	for (i = 0; i < 4; i++)
		add_v3_v3(center, tetra[i]->co);
	mul_v3_fl(center, 0.25f);

	for (i = 0; i < 4; i++) {
		BMVert *v1 = tetra[indices[i][0]];
		BMVert *v2 = tetra[indices[i][1]];
		BMVert *v3 = tetra[indices[i][2]];
		float no[3], d[3];

		normal_tri_v3(no, v1->co, v2->co, v3->co);
		sub_v3_v3v3(d, center, v1->co);
		if (dot_v3v3(no, d) > 0)
			SWAP(BMVert *, v1, v3);

		hull_add_triangle(bm, hull_triangles, pool, v1, v2, v3);
	}
}

/* For each axis, get the minimum and maximum input vertices */
static void hull_get_min_max(BMesh *bm, BMOperator *op,
                             BMVert *min[3], BMVert *max[3])
{
	BMOIter oiter;
	BMVert *v;

	min[0] = min[1] = min[2] = NULL;
	max[0] = max[1] = max[2] = NULL;

	BMO_ITER (v, &oiter, bm, op, "input", BM_VERT) {
		int i;
		
		for (i = 0; i < 3; i++) {
			if (!min[i] || v->co[i] < min[i]->co[i])
				min[i] = v;
			if (!max[i] || v->co[i] > max[i]->co[i])
				max[i] = v;
		}
	}
}

/* Returns true if input is coplanar */
static int hull_find_large_tetrahedron(BMesh *bm, BMOperator *op,
                                       BMVert *tetra[4])
{
	BMVert *min[3], *max[3], *v;
	BMOIter oiter;
	float widest_axis_len, largest_dist, plane_normal[3];
	int i, j, widest_axis;
	
	tetra[0] = tetra[1] = tetra[2] = tetra[3] = NULL;
	hull_get_min_max(bm, op, min, max);

	/* Check for flat axis */
	for (i = 0; i < 3; i++) {
		if (min[i] == max[i]) {
			return TRUE;
		}
	}

	/* Find widest axis */
	widest_axis_len = 0.0f;
	widest_axis = 0; /* set here in the unlikey case this isn't set below */
	for (i = 0; i < 3; i++) {
		float len = (max[i]->co[i] - min[i]->co[i]);
		if (len >= widest_axis_len) {
			widest_axis_len = len;
			widest_axis = i;
		}
	}

	/* Use widest axis for first two points */
	tetra[0] = min[widest_axis];
	tetra[1] = max[widest_axis];
	BMO_elem_flag_enable(bm, tetra[0], HULL_FLAG_TETRA_VERT);
	BMO_elem_flag_enable(bm, tetra[1], HULL_FLAG_TETRA_VERT);

	/* Choose third vertex farthest from existing line segment */
	largest_dist = 0;
	for (i = 0; i < 3; i++) {
		BMVert *v;
		float dist;

		if (i == widest_axis)
			continue;

		v = min[i];
		for (j = 0; j < 2; j++) {
			dist = dist_to_line_segment_v3(v->co, tetra[0]->co, tetra[1]->co);
			if (dist > largest_dist) {
				largest_dist = dist;
				tetra[2] = v;
			}

			v = max[i];
		}
	}

	if (tetra[2]) {
		BMO_elem_flag_enable(bm, tetra[2], HULL_FLAG_TETRA_VERT);
	}
	else {
		return TRUE;
	}

	/* Check for colinear vertices */
	if (largest_dist < HULL_EPSILON_FLT)
		return TRUE;

	/* Choose fourth point farthest from existing plane */
	largest_dist = 0;
	normal_tri_v3(plane_normal, tetra[0]->co, tetra[1]->co, tetra[2]->co);
	BMO_ITER (v, &oiter, bm, op, "input", BM_VERT) {
		if (!BMO_elem_flag_test(bm, v, HULL_FLAG_TETRA_VERT)) {
			float dist = fabsf(dist_to_plane_v3(v->co, tetra[0]->co, plane_normal));
			if (dist > largest_dist) {
				largest_dist = dist;
				tetra[3] = v;
			}
		}
	}

	if (tetra[3]) {
		BMO_elem_flag_enable(bm, tetra[3], HULL_FLAG_TETRA_VERT);
	}
	else {
		return TRUE;
	}

	if (largest_dist < HULL_EPSILON_FLT)
		return TRUE;

	return FALSE;
}



/**************************** Final Output ****************************/

static void hull_remove_overlapping(BMesh *bm, GHash *hull_triangles)
{
	GHashIterator hull_iter;

	GHASH_ITER (hull_iter, hull_triangles) {
		HullTriangle *t = BLI_ghashIterator_getKey(&hull_iter);
		BMIter bm_iter1, bm_iter2;
		BMFace *f;
		int f_on_hull;

		BM_ITER_ELEM (f, &bm_iter1, t->v[0], BM_FACES_OF_VERT) {
			BMEdge *e;

			/* Check that all the face's edges are on the hull,
			 * otherwise can't reuse it */
			f_on_hull = TRUE;
			BM_ITER_ELEM (e, &bm_iter2, f, BM_EDGES_OF_FACE) {
				if (!BMO_elem_flag_test(bm, e, HULL_FLAG_OUTPUT_GEOM)) {
					f_on_hull = FALSE;
					break;
				}
			}
			
			/* Note: can't change ghash while iterating, so mark
			 * with 'skip' flag rather than deleting triangles */
			if (BM_vert_in_face(f, t->v[1]) &&
			    BM_vert_in_face(f, t->v[2]) && f_on_hull)
			{
				t->skip = TRUE;
				BMO_elem_flag_disable(bm, f, HULL_FLAG_INTERIOR_ELE);
				BMO_elem_flag_enable(bm, f, HULL_FLAG_HOLE);
			}
		}
	}
}

static void hull_mark_interior_elements(BMesh *bm, BMOperator *op)
{
	BMEdge *e;
	BMFace *f;
	BMOIter oiter;

	/* Check for interior edges too */
	BMO_ITER (e, &oiter, bm, op, "input", BM_EDGE) {
		if (!BMO_elem_flag_test(bm, e, HULL_FLAG_OUTPUT_GEOM))
			BMO_elem_flag_enable(bm, e, HULL_FLAG_INTERIOR_ELE);
	}

	/* Mark all input faces as interior, some may be unmarked in
	 * hull_remove_overlapping() */
	BMO_ITER (f, &oiter, bm, op, "input", BM_FACE) {
		BMO_elem_flag_enable(bm, f, HULL_FLAG_INTERIOR_ELE);
	}
}

static void hull_tag_unused(BMesh *bm, BMOperator *op)
{
	BMIter iter;
	BMOIter oiter;
	BMVert *v;
	BMEdge *e;
	BMFace *f;

	/* Mark vertices, edges, and faces that are already marked
	 * interior (i.e. were already part of the input, but not part of
	 * the hull), but that aren't also used by elements outside the
	 * input set */
	BMO_ITER (v, &oiter, bm, op, "input", BM_VERT) {
		if (BMO_elem_flag_test(bm, v, HULL_FLAG_INTERIOR_ELE)) {
			int del = TRUE;
		
			BM_ITER_ELEM (e, &iter, v, BM_EDGES_OF_VERT) {
				if (!BMO_elem_flag_test(bm, e, HULL_FLAG_INPUT)) {
					del = FALSE;
					break;
				}
			}

			BM_ITER_ELEM (f, &iter, v, BM_FACES_OF_VERT) {
				if (!BMO_elem_flag_test(bm, f, HULL_FLAG_INPUT)) {
					del = FALSE;
					break;
				}
			}

			if (del)
				BMO_elem_flag_enable(bm, v, HULL_FLAG_DEL);
		}
	}

	BMO_ITER (e, &oiter, bm, op, "input", BM_EDGE) {
		if (BMO_elem_flag_test(bm, e, HULL_FLAG_INTERIOR_ELE)) {
			int del = TRUE;

			BM_ITER_ELEM (f, &iter, e, BM_FACES_OF_EDGE) {
				if (!BMO_elem_flag_test(bm, f, HULL_FLAG_INPUT)) {
					del = FALSE;
					break;
				}
			}

			if (del)
				BMO_elem_flag_enable(bm, e, HULL_FLAG_DEL);
		}
	}

	BMO_ITER (f, &oiter, bm, op, "input", BM_FACE) {
		if (BMO_elem_flag_test(bm, f, HULL_FLAG_INTERIOR_ELE))
			BMO_elem_flag_enable(bm, f, HULL_FLAG_DEL);
	}
}

static void hull_tag_holes(BMesh *bm, BMOperator *op)
{
	BMIter iter;
	BMOIter oiter;
	BMFace *f;
	BMEdge *e;

	/* Unmark any hole faces if they are isolated or part of a
	 * border */
	BMO_ITER (f, &oiter, bm, op, "input", BM_FACE) {
		if (BMO_elem_flag_test(bm, f, HULL_FLAG_HOLE)) {
			BM_ITER_ELEM (e, &iter, f, BM_EDGES_OF_FACE) {
				if (BM_edge_face_count(e) == 1) {
					BMO_elem_flag_disable(bm, f, HULL_FLAG_HOLE);
					break;
				}
			}
		}
	}

	/* Mark edges too if all adjacent faces are holes */
	BMO_ITER (e, &oiter, bm, op, "input", BM_EDGE) {
		int hole = TRUE;
		
		BM_ITER_ELEM (f, &iter, e, BM_FACES_OF_EDGE) {
			if (!BMO_elem_flag_test(bm, f, HULL_FLAG_HOLE)) {
				hole = FALSE;
				break;
			}
		}

		if (hole)
			BMO_elem_flag_enable(bm, e, HULL_FLAG_HOLE);
	}
}



/******************************* Bullet *******************************/

static void blah(BMesh *bm, BMOperator *op)
{
	BMVert **fv = NULL;
	BLI_array_declare(fv);
	BMEdge **fe = NULL;
	BLI_array_declare(fe);
	int *fvi = NULL;
	BLI_array_declare(fvi);
	BMVert *v;
	BMVert **verts;
	BMVert **input_verts;
	plConvexHull hull;
	BMElemF *ele;
	BMOIter oiter;
	float (*coords)[3];
	int i, count = 0, num_input_verts;

	num_input_verts = 0;
	BMO_ITER (ele, &oiter, bm, op, "input", BM_ALL) {
		BMO_elem_flag_enable(bm, ele, HULL_FLAG_INPUT);
		
		if (ele->head.htype == BM_VERT) {
			/* Mark all vertices as interior to begin with */
			BMO_elem_flag_enable(bm, ele, HULL_FLAG_INTERIOR_ELE);

			/* Count number of vertices */
			num_input_verts++;
		}
	}

	coords = MEM_callocN(sizeof(*coords) * num_input_verts, AT);
	input_verts = MEM_callocN(sizeof(*input_verts) * num_input_verts, AT);
	i = 0;
	BMO_ITER (v, &oiter, bm, op, "input", BM_VERT) {
		copy_v3_v3(coords[i], v->co);
		input_verts[i] = v;
		i++;
	}

	hull = plConvexHullCompute(coords, num_input_verts);

	count = plConvexHullNumVertices(hull);
	verts = MEM_mallocN(sizeof(*verts) * count, AT);
	for (i = 0; i < count; i++) {
		float co[3];
		int original_index;
		plConvexHullGetVertex(hull, i, co, &original_index);
		if (original_index >= 0 && original_index < num_input_verts)
			verts[i] = input_verts[original_index];
		else
			verts[i] = BM_vert_create(bm, co, NULL);
	}

	count = plConvexHullNumFaces(hull);
	for (i = 0; i < count; i++) {
		int j, len = plConvexHullGetFaceSize(hull, i);

		if (len > 2) {
			BMFace *f;

			BLI_array_empty(fv);
			BLI_array_grow_items(fv, len);
			BLI_array_empty(fe);
			BLI_array_grow_items(fe, len);
			BLI_array_empty(fvi);
			BLI_array_grow_items(fvi, len);

			plConvexHullGetFaceVertices(hull, i, fvi);

			for (j = 0; j < len; j++) {
				int next = (j + 1 == len ? 0 : j + 1);
				fv[j] = verts[fvi[j]];
				fe[j] = BM_edge_create(bm,
									   verts[fvi[j]],
									   verts[fvi[next]],
									   NULL, TRUE);

				/* Mark verts/edges for 'geomout' slot */
				BMO_elem_flag_enable(bm, fv[j], HULL_FLAG_OUTPUT_GEOM);
				BMO_elem_flag_enable(bm, fe[j], HULL_FLAG_OUTPUT_GEOM);
			}

			f = BM_face_create(bm, fv, fe, len, TRUE);
			/* Mark face for 'geomout' slot and select */
			BMO_elem_flag_enable(bm, f, HULL_FLAG_OUTPUT_GEOM);
			BM_face_select_set(bm, f, TRUE);
		}
	}

	BLI_array_free(fv);
	BLI_array_free(fe);
	BLI_array_free(fvi);
	MEM_freeN(verts);
	MEM_freeN(coords);
	MEM_freeN(input_verts);
}

void bmo_convex_hull_exec(BMesh *bm, BMOperator *op)
{
	HullFinalEdges *final_edges;
	BLI_mempool *hull_pool, *edge_pool;
	BMVert *v, *tetra[4];
	BMElemF *ele;
	BMOIter oiter;
	GHash *hull_triangles;

	/* Verify that at least three verts are in the input */
	if (BMO_slot_get(op, "input")->len < 3) {
		BMO_error_raise(bm, op, BMERR_CONVEX_HULL_FAILED,
		                "Requires at least three vertices");
		return;
	}

	/* TODO, XXX, etc */
	blah(bm, op);

	hull_mark_interior_elements(bm, op);

	/* Remove hull triangles covered by an existing face */
	/*if (BMO_slot_bool_get(op, "use_existing_faces")) {
		hull_remove_overlapping(bm, hull_triangles);

		hull_tag_holes(bm, op);
		}*/

	/* Done with edges */
	//hull_final_edges_free(final_edges);

	/* Convert hull triangles to BMesh faces */
	//hull_output_triangles(bm, hull_triangles);
	//BLI_mempool_destroy(hull_pool);

	//BLI_ghash_free(hull_triangles, NULL, NULL);

	hull_tag_unused(bm, op);

	/* Output slot of input elements that ended up inside the hull
	 * rather than part of it */
	BMO_slot_buffer_from_enabled_flag(bm, op, "interior_geom", BM_ALL,
	                                  HULL_FLAG_INTERIOR_ELE);

	/* Output slot of input elements that ended up inside the hull and
	 * are are unused by other geometry. */
	BMO_slot_buffer_from_enabled_flag(bm, op, "unused_geom", BM_ALL,
	                                  HULL_FLAG_DEL);

	/* Output slot of faces and edges that were in the input and on
	 * the hull (useful for cases like bridging where you want to
	 * delete some input geometry) */
	BMO_slot_buffer_from_enabled_flag(bm, op, "holes_geom", BM_ALL,
	                                  HULL_FLAG_HOLE);

	/* Output slot of all hull vertices, faces, and edges */
	BMO_slot_buffer_from_enabled_flag(bm, op, "geomout", BM_ALL,
	                                  HULL_FLAG_OUTPUT_GEOM);
}
