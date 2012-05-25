#include "MEM_guardedalloc.h"

#include "BLI_array.h"
#include "BLI_bitmap.h"
#include "BLI_math.h"
#include "BLI_utildefines.h"

#include "bmesh.h"

#include <limits.h>
#include <string.h>

/* SPR mesh representation */

/* Note that SPR coordinates are projected into a 2D plane and scaled
   into an integer range (hopefully faster calculations and fewer
   precision errors) */

typedef int SprCoord;

typedef struct {
	/* Projected coordinates */
	SprCoord co[2];

	/* Source vertex */
	BMVert *original_bm_vert;

	unsigned totface;
	unsigned num_exterior_faces;

	unsigned on_boundary;

	unsigned finished;
} SprVert;

/*typedef struct {
	unsigned verts[2];
	//unsigned faces[2];
	unsigned totface;
	unsigned on_boundary;
	} SprEdge;*/

/* Output quadrilateral
 * 
 * Note: winding order is not important during SPR, normals will get
 * fixed afterward. */
typedef struct {
	unsigned verts[4];
} SprQuad;

/* Input polygon. These are the unfinished regions that have not yet
   been split into quadrilaterals. */
typedef struct {
	/* Array of all vertex indices in this polygon. The first
	   'num_boundary_verts' indices are the ordered vertices around
	   the polygon's perimeter, followed by 'num_loose_verts' indices
	   of vertices that lie interior to the polygon. */
	unsigned *verts;
	unsigned num_boundary_verts;
	unsigned num_loose_verts;
} SprPoly;

typedef struct {
	SprVert *verts;
	SprPoly *polys;
	SprQuad *quads;

	/* Store edge information in bitmaps, easy to lookup with vertex
	   indices */
	/* TODO: could optimize these as a triangle matrix */
	BLI_bitmap edge_exists;
	BLI_bitmap edge_on_boundary;
	BLI_bitmap edge_has_face1;
	BLI_bitmap edge_has_face2;

	unsigned num_verts;
	unsigned num_polys;
	unsigned num_quads;

	/* XXX: debugging */
	unsigned depth;

	/* Scoring */
	unsigned num_extraordinary_vertices;
	unsigned num_steiner_points;
} SprMesh;

static const float spr_maxco_f = 1000.0f;
static const SprCoord spr_maxco = 1000;

int spr_debug = 0;
unsigned long long spr_combinations = 0;
unsigned long long spr_max_combinations = 10000000;
void print_depth(const SprMesh *mesh)
{
	int i;
	for (i = 0; i < mesh->depth; i++)
		printf(">> ");
}

void print_quad(const unsigned quad[4])
{
	int i;
	printf("(");
	for (i = 0; i < 4; i++) {
		printf("%d", quad[i]);
		if (i < 3)
			printf(", ");
	}
	printf(")");
}

/* General TODO: check for small & frequent memory allocs */

static SprMesh *spr_mesh_create(unsigned num_verts,
								unsigned max_quads,
								unsigned max_polys)
{
	const unsigned edge_size = num_verts * num_verts;
	SprMesh *mesh = MEM_callocN(sizeof(SprMesh), "SprMesh");

	mesh->verts = MEM_callocN(sizeof(SprVert) * num_verts, "SprVert");
	mesh->quads = MEM_callocN(sizeof(SprQuad) * max_quads, "SprQuad");
	mesh->polys = MEM_callocN(sizeof(SprPoly) * max_polys, "SprPoly");
	
	mesh->edge_exists = BLI_BITMAP_NEW(edge_size, "edge_exists");
	mesh->edge_on_boundary = BLI_BITMAP_NEW(edge_size, "edge_on_boundary");
	mesh->edge_has_face1 = BLI_BITMAP_NEW(edge_size, "edge_has_face1");
	mesh->edge_has_face2 = BLI_BITMAP_NEW(edge_size, "edge_has_face2");

	mesh->num_verts = 0;
	mesh->num_quads = 0;
	mesh->num_polys = 0;

	mesh->depth = 0;

	return mesh;
}

static void spr_mesh_free(SprMesh *mesh)
{
	unsigned i;

	for (i = 0; i < mesh->num_polys; i++) {
		MEM_freeN(mesh->polys[i].verts);
	}

	MEM_freeN(mesh->edge_exists);
	MEM_freeN(mesh->edge_on_boundary);
	MEM_freeN(mesh->edge_has_face1);
	MEM_freeN(mesh->edge_has_face2);
	MEM_freeN(mesh->verts);
	MEM_freeN(mesh->quads);
	MEM_freeN(mesh->polys);
	MEM_freeN(mesh);
}

static SprMesh *spr_mesh_copy(const SprMesh *src)
{
	const unsigned edge_bytes = BLI_BITMAP_SIZE(src->num_verts * src->num_verts);
	unsigned i;

	/* Add space for one new quad and two new polys */
	SprMesh *dst = spr_mesh_create(src->num_verts,
								   src->num_quads + 1,
								   src->num_polys + 2);

	dst->num_verts = src->num_verts;
	dst->num_quads = src->num_quads;
	dst->num_polys = src->num_polys;

	memcpy(dst->verts, src->verts, sizeof(SprVert) * src->num_verts);
	memcpy(dst->quads, src->quads, sizeof(SprQuad) * src->num_quads);

	for (i = 0; i < src->num_polys; i++) {
		dst->polys[i] = src->polys[i];
		dst->polys[i].verts = MEM_dupallocN(dst->polys[i].verts);
	}
	
	memcpy(dst->edge_exists, src->edge_exists, edge_bytes);
	memcpy(dst->edge_on_boundary, src->edge_on_boundary, edge_bytes);
	memcpy(dst->edge_has_face1, src->edge_has_face1, edge_bytes);
	memcpy(dst->edge_has_face2, src->edge_has_face2, edge_bytes);

	dst->depth = src->depth + 1;

	dst->num_steiner_points = src->num_steiner_points;

	return dst;
}

BLI_INLINE unsigned spr_edge_bitmap_index(const SprMesh *mesh,
										  unsigned v1, unsigned v2)
{
	BLI_assert(v1 != v2);
	if (v1 < v2)
		SWAP(unsigned, v1, v2);
	return v1 * mesh->num_verts + v2;
}

BLI_INLINE unsigned spr_edge_exists(const SprMesh *mesh,
									unsigned v1, unsigned v2)
{
	return BLI_BITMAP_GET(mesh->edge_exists,
						  spr_edge_bitmap_index(mesh, v1, v2));
}

BLI_INLINE int spr_edge_on_boundary(const SprMesh *mesh,
									unsigned v1, unsigned v2)
{
	BLI_assert(spr_edge_exists(mesh, v1, v2));
	return BLI_BITMAP_GET(mesh->edge_on_boundary,
						  spr_edge_bitmap_index(mesh, v1, v2));
}

static int spr_edge_num_faces(const SprMesh *mesh,
							  unsigned v1, unsigned v2)
{
	const unsigned bi = spr_edge_bitmap_index(mesh, v1, v2);
	
	if (BLI_BITMAP_GET(mesh->edge_has_face1, bi)) {
		if (BLI_BITMAP_GET(mesh->edge_has_face2, bi))
			return 2;
		else
			return 1;
	}
	else
		return 0;
}

BLI_INLINE void spr_copy_co_co(SprCoord dst[2], const SprCoord src[2])
{
	dst[0] = src[0];
	dst[1] = src[1];
}

BLI_INLINE void spr_sub_co_co_co(SprCoord dst[2],
								 const SprCoord a[2],
								 const SprCoord b[2])
{
	dst[0] = a[0] - b[0];
	dst[1] = a[1] - b[1];
}

BLI_INLINE SprCoord spr_dot_co_co(const SprCoord a[2],
								  const SprCoord b[2])
{
	return a[0] * b[0] + a[1] * b[1];
}

BLI_INLINE void spr_perpendicular_co_co(SprCoord dst[2],
										const SprCoord src[2])
{
	dst[0] = -src[0];
	dst[1] = src[1];
}

/* Sign indicates winding, zero indicates degenerate */
BLI_INLINE SprCoord spr_tri_area_co_co_co(const SprCoord a[2],
										  const SprCoord b[2],
										  const SprCoord c[2])
{
	return ((a[0] - c[0]) * (b[1] - c[1]) -
			(a[1] - c[1]) * (b[0] - c[0]));
}

int spr_isect_seg_seg(const SprCoord a[2], const SprCoord b[2],
					  const SprCoord c[2], const SprCoord d[2])
{
	SprCoord a1 = spr_tri_area_co_co_co(a, b, d);
	SprCoord a2 = spr_tri_area_co_co_co(a, b, c);

	if (a1 && a2 && (a1 ^ a2) < 0) {
		SprCoord a3 = spr_tri_area_co_co_co(c, d, a);
		SprCoord a4 = a3 + a2 - a1;

		if (a3 && a4 && (a3 ^ a4) < 0) {
			/*t = a3 / (a3 - a4);
			  p = a + t * (b - a);*/
			return TRUE;
		}
	}
   
	return FALSE;
}


#if 0
static void shrink_v22_v2_v2(SprCoord shrink[2][2],
							 const SprCoord a[2], const SprCoord b[2])
{
	const float factor = 0.0001;
	SprCoord dir[2];

	spr_copy_co_co(shrink[0], a);
	spr_copy_co_co(shrink[1], b);

	spr_sub_co_co_co(dir, b, a);
	mul_v2_fl(dir, factor);

	add_v2_v2(shrink[0], dir);
	sub_v2_v2(shrink[1], dir);
}
#endif

#if 0
static int test_lines_overlap(const SprMesh *mesh,
							  unsigned a1, unsigned a2,
							  unsigned b1, unsigned b2)
{
	/* Ignore if input edges are identical */
	if (a1 != b1 || a1 != b2 || a2 != b1 || b2 != b2) {
		SprCoord shrink1[2][2], shrink2[2][2];
		float vi[2];

		/* Shrink the lines down a little to avoid
		   intersection match at the endpoints */
		shrink_v22_v2_v2(shrink1,
						 mesh->verts[a1].co,
						 mesh->verts[a2].co);
		shrink_v22_v2_v2(shrink2,
						 mesh->verts[b1].co,
						 mesh->verts[b2].co);
				
		if (isect_seg_seg_v2_point(shrink1[0],
								   shrink1[1],
								   shrink2[0],
								   shrink2[1], vi) == 1)
		{
			return TRUE;
		}
	}

	return FALSE;
}
#endif
#if 0
static int test_quads_overlapping(const SprMesh *mesh,
								  const unsigned q1[4],
								  const unsigned q2[4])
{
	int i, j;
	
	/* Not sure how best to do this, for now just checking each edge
	   for intersection */
	/* TODO: need to at least add a full containment test */
	
	for (i = 0; i < 4; i++) {
		unsigned e1[2] = {q1[i], q1[(i + 1) % 4]};
		for (j = 0; j < 4; j++) {
			unsigned e2[2] = {q2[j], q2[(j + 1) % 4]};

			if (test_lines_overlap(mesh, e1[0], e1[1], e2[0], e2[1]))
			{
				return TRUE;
			}
		}
	}

	return FALSE;
}
#endif

static int test_quad_line_overlap(const SprMesh *mesh,
								  const unsigned q[4],
								  unsigned v1, unsigned v2)
{
	int i;
	
	for (i = 0; i < 4; i++) {
		unsigned qe[2] = {q[i], q[(i + 1) % 4]};

		if (spr_isect_seg_seg(mesh->verts[qe[0]].co,
							  mesh->verts[qe[1]].co,
							  mesh->verts[v1].co,
							  mesh->verts[v2].co))
		//if (test_lines_overlap(mesh, qe[0], qe[1], v1, v2))
		{
			/*printf ("blah=%d\n", spr_isect_seg_seg(mesh->verts[qe[0]].co,
									  mesh->verts[qe[1]].co,
									  mesh->verts[v1].co,
									  mesh->verts[v2].co));*/
			return TRUE;
		}
	}

	return FALSE;
}

#if 0
static int spr_is_existing_edge_usable(const SprMesh *mesh,
									   unsigned v1, unsigned v2)
{
	const int e_on_boundary = spr_edge_on_boundary(mesh, v1, v2);
	
	if (e_on_boundary && (spr_edge_num_faces(mesh, v1, v2) > 0)) {
		return FALSE;
	}
	else if ((!e_on_boundary) && spr_edge_num_faces(mesh, v1, v2) != 1) {
		return FALSE;
	}
	else {
		int i;
		for (i = 0; i < mesh->num_verts; i++) {
			if (test_quad_line_overlap(mesh, mesh->faces[i].verts, v1, v2))
				return FALSE;
		}
		return TRUE;
	}
}

static int spr_is_vert_pair_usable(SprMesh *mesh, unsigned v1, unsigned v2)
{
	if (spr_edge_exists(mesh, v1, v2)) {
		/* Edge already exists, check if it's usable */
		return spr_is_existing_edge_usable(mesh, v1, v2);
	}
	else {
		int i;
		for (i = 0; i < mesh->totface; i++) {
			/*if (test_quad_line_overlap(mesh, mesh->faces[i].verts, v1, v2))
			  return FALSE;*/
		}
		
		//SprVert *verts[2] = {&mesh->verts[v1], &mesh->verts[v2]};

		/* Edge does not exist */
		//if (verts[0]->on_boundary && verts[1]->on_boundary)

		/* TODO: check if edge crosses the outer boundary */
		return TRUE;
	}
}
#endif

static int is_quad_convex(const SprCoord v1[2], const SprCoord v2[2],
						  const SprCoord v3[2], const SprCoord v4[2])
{
	return spr_isect_seg_seg(v1, v3, v2, v4);
}

static int spr_is_quad_usable(SprMesh *mesh, const unsigned quad[4])
{
	/* Check that the quad's diagonals are not connected by existing
	   edges */
	if (spr_edge_exists(mesh, quad[0], quad[2])) {
		return FALSE;
	}
	else if (spr_edge_exists(mesh, quad[1], quad[3])) {
		return FALSE;
	}
	else {
		/* TODO: check verts */

		/* Check for concavity */		
		if (!is_quad_convex(mesh->verts[quad[0]].co, mesh->verts[quad[1]].co,
							mesh->verts[quad[2]].co, mesh->verts[quad[3]].co))
		{
			return FALSE;
		}

		/* Check for overlaps */
		/*for (i = 0; i < mesh->totface; i++) {
			const unsigned *qv = mesh->faces[i].verts;

			if (test_quads_overlapping(mesh, quad, qv)) {
				return FALSE;
			}
			}*/
		
		return TRUE;
	}
}

static int spr_is_edge_finished(const SprMesh *mesh, unsigned v1, unsigned v2)
{
	
	const int num_faces = spr_edge_num_faces(mesh, v1, v2);
	const int on_boundary = spr_edge_on_boundary(mesh, v1, v2);
	return ((on_boundary && (num_faces == 1)) ||
			((!on_boundary) && (num_faces == 2)));
}

static int spr_mesh_test_bad_loops(SprMesh *mesh, unsigned v1, unsigned v2)
{
	unsigned i, j;

	for (i = 0; i < mesh->num_verts; i++) {
		if ((i == v1) || (i == v2))
			continue;

		/* Look for three-sided loops */
		if (spr_edge_exists(mesh, v1, i) &&
			spr_edge_exists(mesh, v2, i)) {
			/* XXX: Might be wrong, what about floating verts? */
			return TRUE;
		}

		/* Look for concave four-sided loops */
		for (j = 0; j < mesh->num_verts; j++) {
			if ((i != j) &&
				spr_edge_exists(mesh, v2, i) &&
				spr_edge_exists(mesh, i, j) &&
				spr_edge_exists(mesh, j, v1))
			{
				if (!is_quad_convex(mesh->verts[v1].co,
									mesh->verts[v2].co,
									mesh->verts[i].co,
									mesh->verts[j].co))
				{
					return TRUE;
				}
			}
		}
	}

	return FALSE;
}

static int spr_vert_in_quad(const unsigned quad_verts[4], unsigned v)
{
	unsigned i;
	for (i = 0; i < 4; i++) {
		if (quad_verts[i] == v)
			return i;
	}
	return -1;
}

static int spr_vert_in_poly_boundary(const SprPoly *p, unsigned v)
{
	unsigned i;

	for (i = 0; i < p->num_boundary_verts; i++) {
		if (p->verts[i] == v)
			return i;
	}
	return -1;
}

static int spr_polygon_contains_point(const SprMesh *mesh,
									  const unsigned *boundary_verts,
									  unsigned num_boundary_verts,
									  const SprCoord co[2])
{
	SprCoord end[2];
	unsigned i, num_isect = 0;

	end[0] = co[0] + spr_maxco * 2;
	end[1] = co[1];

	for (i = 0; i < num_boundary_verts; i++) {
		unsigned v = boundary_verts[i];
		unsigned next = boundary_verts[(i + 1) % num_boundary_verts];

		/* Test intersection on a "ray" (actually long line segment)
		   starting at co and extending along the +x axis (arbitrary
		   choice) */
		if (spr_isect_seg_seg(co, end,
							  mesh->verts[v].co,
							  mesh->verts[next].co))
		{
			num_isect++;
		}
	}

	/* Point is contained if number of intersections is odd */
	return (num_isect % 2);
}

static int spr_mesh_subtract_quad(SprMesh *mesh, unsigned poly_index,
								  const unsigned quad_verts[4])
{
	SprPoly *p = &mesh->polys[poly_index];

	/* Early check: if polygon matches the quad, just delete the polygon */
	if (p->num_boundary_verts == 4 &&
		(spr_vert_in_poly_boundary(p, quad_verts[0]) != -1) &&
		(spr_vert_in_poly_boundary(p, quad_verts[1]) != -1) &&
		(spr_vert_in_poly_boundary(p, quad_verts[2]) != -1) &&
		(spr_vert_in_poly_boundary(p, quad_verts[3]) != -1))
	{
		MEM_freeN(p->verts);
		/* Move last polygon into this polygon's slot */
		if (poly_index < mesh->num_polys - 1)
			(*p) = mesh->polys[mesh->num_polys - 1];
		mesh->num_polys--;
	}
	else {
		const SprPoly p_orig = *p;
		int first_new_poly = TRUE;
		unsigned i, *tmp;

		/* XXX: not sure about correct size here */
		tmp = MEM_callocN(sizeof(unsigned) *
						  (p->num_boundary_verts +
						   p->num_loose_verts + 4),
						  "spr_mesh_subtract_quad.tmp");

		/* Note: quad has same winding as polygon */

		for (i = 0; i < 4; i++) {
			unsigned v1 = quad_verts[i];
			int in_poly = spr_vert_in_poly_boundary(&p_orig, v1);
			
			if (in_poly != -1) {
				unsigned j, num_boundary_verts = 0;

				for (j = 0; j < p_orig.num_boundary_verts; j++) {
					unsigned v2 = p_orig.verts[(j + in_poly) %
											   p_orig.num_boundary_verts];
					int in_quad = spr_vert_in_quad(quad_verts, v2);

					tmp[num_boundary_verts++] = v2;
					if (in_quad != -1 && in_quad != i &&
						(in_quad == 0 || in_quad > i)) {
						unsigned num_loose_verts = 0;
						unsigned l, bytes;
						int k;

						/* Ignore two-sided poly */
						if (num_boundary_verts == 2)
							break;
						
						/* Copy any intermediate quad vertices into
						   the loop in reverse order */
						for (k = ((in_quad + 3) % 4); k != i; k = (k + 3) % 4)
							tmp[num_boundary_verts++] = quad_verts[k];

						/* Copy over loose verts that are inside the
						   new polygon */
						for (l = 0; l < p_orig.num_loose_verts; l++) {
							unsigned offset = p_orig.num_boundary_verts + l;
							unsigned v3 = p_orig.verts[offset];
							const SprCoord *co = mesh->verts[v3].co;
							int found = FALSE;
							unsigned m;

							for (m = 0; m < num_boundary_verts; m++) {
								if (tmp[m] == v3) {
									found = TRUE;
									break;
								}
							}
							
							/* TODO: check on what happens when the
							   point is right on the border */
							if ((!found) &&
								spr_polygon_contains_point(mesh,
														   tmp,
														   num_boundary_verts,
														   co)) {
								tmp[num_boundary_verts + num_loose_verts] = v3;
								num_loose_verts++;
							}
						}

						for (l = 0; l < (num_loose_verts + num_boundary_verts); l++)
							BLI_assert(tmp[l] < mesh->num_verts);

						/* Can't fill an odd-sided loop with just quads */
						if (num_boundary_verts % 2) {
							if (p_orig.verts != p->verts)
								MEM_freeN(p_orig.verts);
							MEM_freeN(tmp);
							return FALSE;
						}
						
						bytes = (sizeof(unsigned) *
								 (num_boundary_verts + num_loose_verts));
						p->verts = MEM_mallocN(bytes, "SprPoly.verts");
						memcpy(p->verts, tmp, bytes);
						p->num_boundary_verts = num_boundary_verts;
						p->num_loose_verts = num_loose_verts;

						if (first_new_poly) {
							first_new_poly = FALSE;
							p = &mesh->polys[mesh->num_polys];
						}
						else {
							mesh->num_polys++;
							p++;
						}
						
						break;
					}
				}
			}
		}

		MEM_freeN(p_orig.verts);
		MEM_freeN(tmp);
	}

	return TRUE;
}

static int spr_mesh_add_quad(SprMesh *mesh, unsigned poly_index,
							 const unsigned quad[4])
{
	SprQuad *q;
	unsigned i;

	/* Update quad's edges */
	for (i = 0; i < 4; i++) {
		unsigned v1 = quad[i];
		unsigned v2 = quad[(i + 1) % 4];
		const unsigned bi = spr_edge_bitmap_index(mesh, v1, v2);

		/* XXX: now that we have the poly stuff, don't think this is
		   needed anymore */
#if 0
		if (!BLI_BITMAP_GET(mesh->edge_exists, bi)) {
			if (spr_mesh_test_bad_loops(mesh, v1, v2)) {
				/*if (spr_debug) {
					print_quad(quad);
					printf(" gave bad loops\n");
					}*/
				return FALSE;
			}
			
		}
#endif

		/* Mark the edge as existing and increase face count */
		BLI_BITMAP_SET(mesh->edge_exists, bi);
		if (!BLI_BITMAP_GET(mesh->edge_has_face1, bi))
			BLI_BITMAP_SET(mesh->edge_has_face1, bi);
		else
			BLI_BITMAP_SET(mesh->edge_has_face2, bi);
	}

	if (!spr_mesh_subtract_quad(mesh, poly_index, quad))
		return FALSE;

	q = &mesh->quads[mesh->num_quads++];
	memcpy(q->verts, quad, sizeof(q->verts));

	/* Update quad's verts */
	for(i = 0; i < 4; i++) {
		unsigned v_prev = quad[(i + 3) % 4];
		unsigned v_next = quad[(i + 1) % 4];
		unsigned v = quad[i];
		mesh->verts[v].totface++;
		
		if (spr_edge_on_boundary(mesh, v_prev, v) &&
			spr_edge_on_boundary(mesh, v, v_next))
		{
			/* This corner (vert and it's two neighbors) lie on the
			   border */
			mesh->verts[v].finished = TRUE;
		}
		else {
			unsigned j;
			
			mesh->verts[v].finished = TRUE;
			/* Check if all the adjacent edges are done */
			for (j = 0; j < mesh->num_verts; j++) {
				if (j != v && spr_edge_exists(mesh, j, v)) {
					if (!spr_is_edge_finished(mesh, j, v))
						mesh->verts[v].finished = FALSE;
				}
			}
		}
	}

	return TRUE;
}

static int spr_mesh_is_candidate(const SprMesh *mesh)
{
	unsigned i;
	
	/* Check that boundary edges have one face and interior edges have
	   two faces */
	for (i = 0; i < mesh->num_verts; i++) {
		unsigned j;
		/* XXX: overflow */
		for (j = i + 1; j < mesh->num_verts; j++) {
			if (spr_edge_exists(mesh, i, j)) {
				if (!spr_is_edge_finished(mesh, i, j))
					return FALSE;
			}
		}
	}

	#if 0
	/* Check that all interior vertices were used */
	for (i = 0; i < mesh->num_verts; i++) {
		const SprVert *v = &mesh->verts[i];
		if (!v->on_boundary && v->totface == 0) {
			return FALSE;
		}
	}
	#endif

	return TRUE;
}

static void spr_mesh_count_extraordinary_verts(SprMesh *mesh)
{
	unsigned i;
	
	mesh->num_extraordinary_vertices = 0;
	for (i = 0; i < mesh->num_verts; i++) {
		SprVert *v = &mesh->verts[i];
		unsigned num_faces = v->num_exterior_faces + v->totface;
		if (num_faces != 0 && (num_faces > 4 || (v->finished && num_faces != 4)))
			mesh->num_extraordinary_vertices++;
	}
}

static int spr_is_mesh_better(SprMesh *mesh, SprMesh *best)
{
	if (best) {
		if (mesh->num_extraordinary_vertices < best->num_extraordinary_vertices)
			return TRUE;

		return FALSE;
	}
	else {
		return TRUE;
	}
}

int test_quad_match(const unsigned quad[4],
					unsigned v1, unsigned v2,
					unsigned v3, unsigned v4)
{
	int i, found = 0;
	for (i = 0; i < 4; i++) {
		if (quad[i] == v1)
			found++;
		if (quad[i] == v2)
			found++;
		if (quad[i] == v3)
			found++;
		if (quad[i] == v4)
			found++;
	}
	return (found == 4);
}

static int min4(unsigned a, unsigned b, unsigned c, unsigned d)
{
	if (a < b) {
		if (a < c)
			return ((a < d) ? 0 : 3);
		else
			return ((c < d) ? 2 : 3);
	}
	else {
		if (b < c)
			return ((b < d) ? 1 : 3);
		else
			return ((c < d) ? 2 : 3);
	}
}

void reorder_quad(unsigned quad[4])
{
	int l = min4(quad[0], quad[1], quad[2], quad[3]);
	int reverse = quad[(l + 3) % 4] < quad[(l + 1) % 4];

	if (l != 0 || reverse) {
		unsigned tmp[4];
		int i;
		if (reverse) {
			for (i = 0; i < 4; i++)
				tmp[i] = quad[(4 - i + l) % 4];
		}
		else {
			for (i = 0; i < 4; i++)
				tmp[i] = quad[(i + l) % 4];
		}
		memcpy(quad, tmp, sizeof(tmp));
	}
}

#if 0
int test_quad_exists(const SprMesh *mesh,
					 unsigned v1, unsigned v2,
					 unsigned v3, unsigned v4)
{
	unsigned i;
	for (i = 0; i < mesh->totface; i++) {
		if (test_quad_match(mesh->faces[i].verts, v1, v2, v3, v4))
			return TRUE;
	}
	return FALSE;
}
#endif

void spr_mesh_print(const SprMesh *mesh)
{
	unsigned i;

	print_depth(mesh);
	printf("SprMesh:\n");
	print_depth(mesh);
	printf(" quads:\n");
	for (i = 0; i < mesh->num_quads; i++) {
		unsigned v[4];
		memcpy(v, mesh->quads[i].verts, sizeof(v));
		//reorder_quad(v);
		print_depth(mesh);
		printf("  (%d, %d, %d, %d)\n",
			   v[0], v[1], v[2], v[3]);
	}
	print_depth(mesh);
	printf(" polys:\n");
	for (i = 0; i < mesh->num_polys; i++) {
		const SprPoly *p = &mesh->polys[i];
		unsigned j;
		print_depth(mesh);
		printf("  (");
		for (j = 0; j < p->num_boundary_verts; j++) {
			printf("%d", p->verts[j]);
			if (j < p->num_boundary_verts - 1)
				printf(", ");
		}
		printf("), (");
		for (j = 0; j < p->num_loose_verts; j++) {
			printf("%d", p->verts[p->num_boundary_verts + j]);
			if (j < p->num_loose_verts - 1)
				printf(", ");
		}
		printf(")\n");
	}
	printf("\n");
}

static void spr_step(SprMesh *mesh, SprMesh **candidate,
					 unsigned max_extraordinary_verts,
					 unsigned max_steiner_points);

typedef enum {
	SPR_QUAD_CONCAVE,
	SPR_BAD_LOOPS, /* better name TODO? */
	SPR_TOO_EXTRAORDINARY,
	SPR_CANDIDATE_FOUND,
	SPR_NO_CANDIDATES_FOUND,
	SPR_XXX,
} SprResult;

static int spr_quad_verts_exact_match(const unsigned quad[4],
									  unsigned v1, unsigned v2,
									  unsigned v3, unsigned v4)
{
	return quad[0] == v1 && quad[1] == v2 && quad[2] == v3 && quad[3] == v4;
}

static int spr_quad_exact_match(const SprQuad *q,
								unsigned v1, unsigned v2,
								unsigned v3, unsigned v4)
{
	return spr_quad_verts_exact_match(q->verts, v1, v2, v3, v4);
}

/* Returns true if search should continue */
static SprResult spr_try_quad(SprMesh *mesh,
							  SprMesh **candidate,
							  const unsigned quad[4],
							  unsigned poly_index,
							  unsigned max_extraordinary_verts,
							  unsigned max_steiner_points,
							  unsigned num_new_steiner_points)
{
	if (mesh->num_quads == 7 &&
		spr_quad_verts_exact_match(quad, 24, 8, 9, 10) &&
		spr_quad_exact_match(&mesh->quads[0], 11, 0, 24, 10) &&
		spr_quad_exact_match(&mesh->quads[1], 24, 0, 1, 22) &&
		spr_quad_exact_match(&mesh->quads[2], 22, 1, 2, 20) &&
		spr_quad_exact_match(&mesh->quads[3], 20, 2, 3, 4) &&
		spr_quad_exact_match(&mesh->quads[4], 20, 4, 5, 6) &&
		spr_quad_exact_match(&mesh->quads[5], 20, 6, 7, 22) &&
		spr_quad_exact_match(&mesh->quads[6], 22, 7, 8, 24))
	{
		printf("blah!\n");
	}

	if (spr_is_quad_usable(mesh, quad)) {
		SprMesh *new_mesh = spr_mesh_copy(mesh);

		if (!spr_mesh_add_quad(new_mesh, poly_index, quad)) {
			/* The new quad made it impossible to find
			   any candidates on this path */
			if (spr_debug > 1) {
				print_depth(new_mesh);
				printf("quad rejected (add_quad)\n");
			}
			spr_mesh_free(new_mesh);
			return SPR_BAD_LOOPS;
		}

		/*if (spr_mesh_is_candidate(new_mesh))
		  spr_mesh_print(new_mesh);*/

		new_mesh->num_steiner_points += num_new_steiner_points;
		if (new_mesh->num_steiner_points > max_steiner_points) {
			if (spr_debug > 1) {
				print_depth(new_mesh);
				printf("quad rejected (exceeds max steiner points)\n");
			}
			spr_mesh_free(new_mesh);
			return SPR_XXX;
		}

		spr_mesh_count_extraordinary_verts(new_mesh);
		if (new_mesh->num_extraordinary_vertices >=
			max_extraordinary_verts)
		{
			if (spr_debug > 1) {
				print_depth(new_mesh);
				printf("quad rejected (exceeds max extraordinary)\n");
			}
			spr_mesh_free(new_mesh);
			return SPR_TOO_EXTRAORDINARY;
		}

		if ((*candidate) &&
			(new_mesh->num_extraordinary_vertices >=
			 (*candidate)->num_extraordinary_vertices))
		{
			if (spr_debug > 1) {
				print_depth(new_mesh);
				printf("quad rejected (exceeds candidate extraordinary)\n");
			}
			spr_mesh_free(new_mesh);
			return SPR_TOO_EXTRAORDINARY;
		}

		/* SET LOG LEVEL HERE */
		spr_debug = 0;
		if (spr_debug) {
			print_depth(new_mesh);
			printf("add_quad ");
			print_quad(quad);
			printf("\n");

			spr_mesh_print(new_mesh);
		}

		if (new_mesh->num_polys == 0) {
		//if (spr_mesh_is_candidate(new_mesh)) {
			print_depth(mesh);
			printf("candidate found\n");
			spr_mesh_print(new_mesh);

			if (spr_is_mesh_better(new_mesh,
								   *candidate)) {
				if (*candidate)
					spr_mesh_free(*candidate);
				(*candidate) = new_mesh;
			}
			else
				spr_mesh_free(new_mesh);

			return SPR_CANDIDATE_FOUND;
		}
		else {
			spr_step(new_mesh, candidate, max_extraordinary_verts,
					 max_steiner_points);
			spr_mesh_free(new_mesh);
		}

		return SPR_XXX;
	}
	else {
		if (spr_debug > 1) {
			print_depth(mesh);
			printf("quad rejected (concave)\n");
		}
		
		return SPR_QUAD_CONCAVE;
	}
}

/* Choose edge of polygon to process next. For now (TODO?), just picks
   an arbitrary edge that has already been used (if possible). */
static unsigned spr_poly_priority_edge(const SprMesh *mesh,
									   const SprPoly *poly)
{
	unsigned i;
	
	for (i = 0; i < poly->num_boundary_verts; i++) {
		unsigned v1 = poly->verts[i];
		unsigned v2 = poly->verts[((i + 1) % poly->num_boundary_verts)];

		if (spr_edge_num_faces(mesh, v1, v2) == 1)
			return i;
	}

	return 0;
}

/* Choose polygon of mesh to process next. For now (TODO?) just picks
   arbitrarily among the polygons with lowest number of edges. */
static unsigned spr_mesh_priority_polygon(const SprMesh *mesh)
{
	unsigned best = 0;
	unsigned lowest_num = 0;
	unsigned i;

	for (i = 0; i < mesh->num_polys; i++) {
		const SprPoly *p = &mesh->polys[i];
		BLI_assert(p->num_boundary_verts >= 4 &&
				   (p->num_boundary_verts % 2) == 0);
		if ((i == 0) || (p->num_boundary_verts < lowest_num)) {
			lowest_num = p->num_boundary_verts;
			best = i;
		}
	}

	return best;
}

static void spr_step(SprMesh *mesh, SprMesh **candidate,
					 unsigned max_extraordinary_verts,
					 unsigned max_steiner_points)
{
	/* XXX: testing */
	if (spr_combinations > spr_max_combinations) {
		printf("Max combinations (%lld) exceeded, bailing out\n",
			   spr_max_combinations);
		return;
	}
	if (*candidate)
		;//return;

	if ((*candidate) && ((*candidate)->num_extraordinary_vertices == 0))
		return;

	/* Try first unfinished polygon (a successful candidate will use
	   all polygons eventually through recursion, so no need to loop
	   through all polygons here) */
	/* TODO: might still be a better way to prioritize
	   though... smallest first? */
	if (mesh->num_polys) {
		unsigned poly_index = spr_mesh_priority_polygon(mesh);
		const SprPoly *poly = &mesh->polys[poly_index];
		unsigned edge = spr_poly_priority_edge(mesh, poly);
		unsigned i, num_verts, quad[4];
		
		/* Try an unfinished edge of the polygon. As with the polygon,
		   only need to try one edge here since if it is successful
		   all the other edges will be tried through recursion. */
		quad[0] = poly->verts[edge];
		quad[1] = poly->verts[((edge + 1) % poly->num_boundary_verts)];

		/* Try an arbitrary vertex of the polygon -- this includes
		   loose vertices (aka Steiner points, these are optional), so
		   we do need to loop through all here */
		num_verts = poly->num_boundary_verts + poly->num_loose_verts;
		for (i = 0; i < num_verts; i++) {
			unsigned j;
				
			quad[2] = poly->verts[i];
				
			/* Reject duplicate vertex  */
			if ((quad[2] == quad[0]) || (quad[2] == quad[1]) /* ||
				!spr_is_vert_pair_usable(mesh, quad[1], quad[2])*/)
				continue;

			/* Take one more arbitrary vertex of the polygon to
			   complete the quad */
			for (j = 0; j < num_verts; j++) {
				unsigned num_new_steiner_points = 0;
					
				quad[3] = poly->verts[j];

				/* Reject duplicate vertex */
				if ((quad[3] == quad[0]) ||
					(quad[3] == quad[1]) ||
					(quad[3] == quad[2]))
					continue;

				if (i >= poly->num_boundary_verts)
					num_new_steiner_points++;
				if (j >= poly->num_boundary_verts)
					num_new_steiner_points++;

				if (spr_debug > 1) {
					print_depth(mesh);
					printf("trying quad ");
					print_quad(quad);
					printf("\n");
				}

				spr_combinations++;
				if (spr_combinations % 1000000 == 0) {
					printf("spr_combinations = %lld\n", spr_combinations);
				}
					
				if (spr_try_quad(mesh, candidate, quad, poly_index,
								 max_extraordinary_verts,
								 max_steiner_points,
								 num_new_steiner_points) ==
					SPR_CANDIDATE_FOUND)
				{
					return;
				}
			}
		}
	}
}

void spr_project_coord(float dst[2], const float src[3],
					   const float UNUSED(normal[3]))
{
	/* XXX: plane is hardcoded to XY for now */
	dst[0] = src[0];
	dst[1] = src[1];
}

void spr_final_coord(SprCoord dst[2], const float src[2],
					 const float normal[3],
					 const float offset[2],
					 const float scale[2])
{
	float p[2];
	spr_project_coord(p, src, normal);
	dst[0] = (p[0] + offset[0]) * (spr_maxco_f / scale[0]);
	dst[1] = (p[1] + offset[1]) * (spr_maxco_f / scale[1]);
}

/* Returns true iff the edge is manifold and one adjacent face is
   marked while the other is not */
static int spr_input_edge_on_boundary(BMEdge *e)
{
	BMFace *adj[2];
	return (BM_edge_face_pair(e, &adj[0], &adj[1]) &&
			(BM_elem_flag_test(adj[0], BM_ELEM_TAG) ^
			 BM_elem_flag_test(adj[1], BM_ELEM_TAG)));
}

/* Mark the two layers of faces around the 'v_center' vertex. Returns
   NULL if a non-quad is encountered, otherwise returns an arbitrary
   face from the second layer of faces. Also collects all marked
   face's vertices into the 'verts' set. */
static BMFace *spr_search_patch_quads(BMVert *v_center, GHash *verts)
{
	BMIter iter_f1;
	BMFace *f1, *result = NULL;

	BM_ITER_ELEM (f1, &iter_f1, v_center, BM_FACES_OF_VERT) {
		BMIter iter_v_corner;
		BMVert *v_corner;
		
		BM_ITER_ELEM (v_corner, &iter_v_corner, f1, BM_VERTS_OF_FACE) {
			BMIter iter_f2;
			BMFace *f2;
			
			if (!BLI_ghash_haskey(verts, v_corner))
				BLI_ghash_insert(verts, v_corner, NULL);
			
			if ((v_corner == v_center) || !BM_vert_is_manifold(v_corner))
				continue;

			BM_ITER_ELEM (f2, &iter_f2, v_corner, BM_FACES_OF_VERT) {
				BMIter iter_e;
				BMEdge *e;

				if (f2->len != 4)
					return NULL;

				BM_elem_flag_enable(f2, BM_ELEM_TAG);
				if (!result && !BM_vert_in_face(f2, v_center))
					result = f2;

				/* Mark edges that are completely surrounded by marked
				   faces */
				BM_ITER_ELEM (e, &iter_e, f2, BM_EDGES_OF_FACE) {
					BMFace *adj[2];
					if (BM_edge_face_pair(e, &adj[0], &adj[1]) &&
						BM_elem_flag_test(adj[0], BM_ELEM_TAG) &&
						BM_elem_flag_test(adj[1], BM_ELEM_TAG))
					{
						BM_elem_flag_enable(e, BM_ELEM_TAG);
					}
				}
			}
		}
	}

	return result;
}

static BMEdge *spr_patch_search_first_edge(BMFace *f_outer)
{
	BMIter iter;
	BMEdge *e;
	
	/* Find an edge in the outer layer that has one adjacent marked
	   face and one not marked */
	BM_ITER_ELEM (e, &iter, f_outer, BM_EDGES_OF_FACE) {
		if (spr_input_edge_on_boundary(e))
			return e;
	}

	return NULL;
}

static BMVert **spr_patch_search_boundary(BMEdge *e_first,
										  int *num_boundary_verts)
{
	BMEdge *e_prev;
	BMVert *v_prev;
	BMVert **boundary_verts = NULL;
	BLI_array_declare(boundary_verts);

	/* Iteratively search for new vertices along the boundary */
	v_prev = e_first->v2;
	e_prev = e_first;
	do {
		BMIter iter;
		BMEdge *e;
		int found = FALSE;

		BM_ITER_ELEM (e, &iter, v_prev, BM_EDGES_OF_VERT) {
			if ((e != e_prev) &&
				spr_input_edge_on_boundary(e))
			{
				BMVert *v = BM_edge_other_vert(e, v_prev);
				
				found = TRUE;

				/* TODO: failsafe */
				{
					int i;
					for (i = 0; i < BLI_array_count(boundary_verts); i++) {
						if (boundary_verts[i] == v) {
							BLI_array_free(boundary_verts);
							return NULL;
						}
					}
				}

				/* Add to boundary verts */
				BLI_array_append(boundary_verts, v);

				v_prev = v;
				e_prev = e;
				break;
			}
		}

		if (!found) {
			BLI_array_free(boundary_verts);
			return NULL;
		}
	} while (e_prev != e_first);

	(*num_boundary_verts) = BLI_array_count(boundary_verts);
	return boundary_verts;
}

static BMVert **spr_get_patch(BMVert *v_center,
							  GHash *loose_verts,
							  int *num_boundary_verts)
{
	BMVert **boundary_verts;
	BMFace *f_outer;
	BMEdge *e_first;
	int i;

	/* Tag the first two layers of faces around 'v_center'. All the
	   tagged faces' verts will be adde to 'loose_verts', from which
	   boundary verts will later be subtracted. */
	f_outer = spr_search_patch_quads(v_center, loose_verts);
	if (!f_outer)
		return NULL;

	/* Find first boundary edge */
	e_first = spr_patch_search_first_edge(f_outer);
	if (!e_first)
		return NULL;

	/* Build list boundary of boundary verts */
	boundary_verts = spr_patch_search_boundary(e_first,
											   num_boundary_verts);

	if (boundary_verts) {
		/* Remove boundary verts from the 'loose_verts' set */
		for (i = 0; i < (*num_boundary_verts); i++) {
			BLI_ghash_remove(loose_verts, boundary_verts[i],
							 NULL, NULL);
		}
	}

#if 0
	/* For testing, select the boundary verts */
	for (i = 0; i < (*num_boundary_verts); i++) {
		BM_elem_flag_enable(boundary_verts[i], BM_ELEM_SELECT);
	}
#endif
#if 0
	{
		GHashIterator gh_iter;
		/* For testing, select the loose verts */
		GHASH_ITER (gh_iter, loose_verts) {
			BMVert *v = BLI_ghashIterator_getKey(&gh_iter);
			BM_elem_flag_enable(v, BM_ELEM_SELECT);
		}
	}
#endif
	
	return boundary_verts;
}

/* XXX: Doubtless there is a better way to express this */
static int bm_vert_is_manifold_and_not_on_boundary(BMVert *v)
{
	if (BM_vert_is_manifold(v)) {
		BMIter iter;
		BMEdge *e;
		BM_ITER_ELEM (e, &iter, v, BM_EDGES_OF_VERT) {
			if (!BM_edge_is_manifold(e))
				return FALSE;
		}
	}
	return TRUE;
}

static void spr_mesh_add_bm_vert(SprMesh *spr_mesh, BMVert *v,
								 const float normal[3],
								 const float offset[2],
								 const float scale[2],
								 int on_boundary,
								 int num_boundary_verts)
{
	SprVert *spr_vert = &spr_mesh->verts[spr_mesh->num_verts];
	SprPoly *p = &spr_mesh->polys[0];
	int v_index = spr_mesh->num_verts;

	BM_elem_index_set(v, v_index); /* set_dirty! */
			
	spr_final_coord(spr_vert->co, v->co, normal,
					offset, scale);

	spr_vert->original_bm_vert = v;
	spr_vert->totface = 0;
	spr_vert->on_boundary = on_boundary;
	spr_vert->num_exterior_faces = 0;
	
	if (on_boundary) {
		BMIter iter;
		BMFace *f;
		
		/* Count number of adjacent marked faces */
		BM_ITER_ELEM (f, &iter, v, BM_FACES_OF_VERT) {
			if (!BM_elem_flag_test(f, BM_ELEM_TAG))
				spr_vert->num_exterior_faces++;
		}
		
		p->verts[p->num_boundary_verts] = v_index;
		p->num_boundary_verts++;
	}
	else {
		p->verts[num_boundary_verts + p->num_loose_verts] = v_index;
		p->num_loose_verts++;
	}

	spr_mesh->num_verts++;
}

static int spr_vert_cmp(const void *a_v, const void *b_v)
{
	const SprVert *a = a_v;
	const SprVert *b = b_v;

	if (a->co[0] < b->co[0])
		return -1;
	else if (a->co[0] > b->co[0])
		return 1;
	else if (a->co[1] < b->co[1])
		return -1;
	else
		return 1;
}

static SprMesh *spr_mesh_create_from_bm_region(BMVert **boundary_verts,
											   int num_boundary_verts,
											   GHash *loose_verts)
{
	GHashIterator gh_iter;
	SprMesh *spr_mesh;
	float normal[3];
	float bb_min[2];
	float bb_max[2];
	float offset[2];
	float scale[2];
	unsigned num_verts;
	int i;

	/* TODO */
	zero_v3(normal);

	/* Get 2D bounding box to calculate offset and scale */
	bb_min[0] = bb_min[1] = FLT_MAX;
	bb_max[0] = bb_max[1] = FLT_MIN;
	for (i = 0; i < num_boundary_verts; i++) {
		BMVert *v = boundary_verts[i];
		float p[2];
		spr_project_coord(p, v->co, normal);
		if (p[0] < bb_min[0])
			bb_min[0] = p[0];
		if (p[0] > bb_max[0])
			bb_max[0] = p[0];
		if (p[1] < bb_min[1])
			bb_min[1] = p[1];
		if (p[1] > bb_max[1])
			bb_max[1] = p[1];
	}

	add_v2_v2v2(offset, bb_max, bb_min);
	mul_v2_fl(offset, -0.5f);
	sub_v2_v2v2(scale, bb_max, bb_min);
	mul_v2_fl(scale, 0.5);
	print_v2("offset", offset);
	print_v2("scale", scale);

	/* Create SPR mesh */
	num_verts = num_boundary_verts + BLI_ghash_size(loose_verts);
	spr_mesh = spr_mesh_create(num_verts, 0, 1);
	spr_mesh->num_polys = 1;
	spr_mesh->polys[0].verts = MEM_callocN(sizeof(unsigned) * num_verts,
										   "SprPoly.verts");

	/* Copy vertices and assign indices */
	for (i = 0; i < num_boundary_verts; i++) {
		spr_mesh_add_bm_vert(spr_mesh,
							 boundary_verts[i],
							 normal, offset, scale,
							 TRUE, num_boundary_verts);
	}
	GHASH_ITER (gh_iter, loose_verts) {
		spr_mesh_add_bm_vert(spr_mesh,
							 BLI_ghashIterator_getKey(&gh_iter),
							 normal, offset, scale,
							 FALSE, num_boundary_verts);
	}
	#if 1
	/* XXX: just for debugging, sort the loose verts by coordinate so
	   that the order is always the same */
	qsort(spr_mesh->verts + num_boundary_verts,
		  BLI_ghash_size(loose_verts),
		  sizeof(SprVert),
		  spr_vert_cmp);

	for (i = 0; i < spr_mesh->num_verts; i++) {
		const SprVert *spr_vert = &spr_mesh->verts[i];
		const BMVert *v = spr_vert->original_bm_vert;
		printf("SprVert %d (%.2f, %.2f, %.2f) -> (%d, %d)\n",
			   i, v->co[0], v->co[1], v->co[2],
			   spr_vert->co[0], spr_vert->co[1]);
	}
	#endif

	/* Copy boundary edges */
	for (i = 0; i < num_boundary_verts; i++) {
		unsigned v1 = BM_elem_index_get(boundary_verts[i]);
		unsigned v2 = BM_elem_index_get(boundary_verts[(i + 1) % num_boundary_verts]);
		unsigned bi = spr_edge_bitmap_index(spr_mesh, v1, v2);

		BLI_BITMAP_SET(spr_mesh->edge_exists, bi);
		BLI_BITMAP_SET(spr_mesh->edge_on_boundary, bi);
	}

	return spr_mesh;
}

static unsigned spr_max_extraordinary_verts(BMVert **boundary_verts,
											int num_boundary_verts,
											GHash *loose_verts)
{
	GHashIterator gh_iter;
	unsigned max_extraordinary_verts = 0;
	int i;
	
	for (i = 0; i < num_boundary_verts; i++) {
		BMVert *v = boundary_verts[i];
		if (BM_vert_face_count(v) != 4)
			max_extraordinary_verts++;
	}

	GHASH_ITER (gh_iter, loose_verts) {
		BMVert *v = BLI_ghashIterator_getKey(&gh_iter);
		if (BM_vert_face_count(v) != 4)
			max_extraordinary_verts++;
	}

	return max_extraordinary_verts;
}

/* Returns true iff 'bm' is modified. */
static int spr_patch_update(BMesh *bm,
							BMVert **boundary_verts,
							int num_boundary_verts,
							GHash *loose_verts)
{
	SprMesh *input, *candidate;
	int modified = FALSE;
	unsigned max_extraordinary_verts;
	unsigned i;

	max_extraordinary_verts =
		spr_max_extraordinary_verts(boundary_verts,
									num_boundary_verts,
									loose_verts);

	printf("max_extraordinary_verts=%d\n", max_extraordinary_verts);
					
	bm->elem_index_dirty |= BM_VERT;
	input = spr_mesh_create_from_bm_region(boundary_verts,
										   num_boundary_verts,
										   loose_verts);

	candidate = NULL;

	spr_combinations = 0;

	/* XXX: trying this out: add a max number of steiner points,
	   i.e. the loose vertices. Can increase the number of points as
	   needed */
	for (i = 0; i < input->polys[0].num_loose_verts; i++) {
		printf("steiner points = %d\n", i);
		spr_step(input, &candidate, max_extraordinary_verts, i);

		if (candidate && candidate->num_extraordinary_vertices == 0)
			break;

		if (spr_combinations > spr_max_combinations)
			break;
	}
	
	printf("spr_combinations = %lld\n", spr_combinations);

	if (candidate && candidate != input) {
		int i;

		printf("%s: candidate accepted\n", __func__);
		modified = TRUE;

		/* Delete all marked faces/edges */
		BMO_op_callf(bm, "del geom=%hef context=%i",
					 BM_ELEM_TAG, DEL_ONLYTAGGED);

		/* Replace with candidate's quads */
		for (i = 0; i < candidate->num_quads; i++) {
			SprQuad *q = &candidate->quads[i];
			BMVert *verts[4];
			int j;
			for (j = 0; j < 4; j++) {
				verts[j] = candidate->verts[q->verts[j]].original_bm_vert;
			}
			BM_face_create_quad_tri_v(bm, verts, 4, NULL, FALSE);
		}

		spr_mesh_free(candidate);
	}
	else
		printf("%s: no candidate found\n", __func__);

	spr_mesh_free(input);

	return modified;
}

void bmo_spr_exec(BMesh *bm, BMOperator *UNUSED(op))
{
	int modified = FALSE;

	do {
		BMIter iter;
		BMVert *v;

		modified = FALSE;

		BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
			int face_count = BM_vert_face_count(v);
			if ((face_count != 4) && (face_count != 0) &&
				bm_vert_is_manifold_and_not_on_boundary(v))
			{
				BMIter iter2;
				GHash *loose_verts;
				BMVert **boundary_verts;
				int num_boundary_verts;
				BMFace *e;
				BMFace *f;

				printf("%s: found extraordinary vert %p (%d)\n",
					   __func__, v, face_count);
				
				loose_verts = BLI_ghash_ptr_new("loose_verts");

				/* Unmark all faces/edges */
				BM_ITER_MESH (f, &iter2, bm, BM_FACES_OF_MESH) {
					BM_elem_flag_disable(f, BM_ELEM_TAG);
				}
				BM_ITER_MESH (e, &iter2, bm, BM_EDGES_OF_MESH) {
					BM_elem_flag_disable(e, BM_ELEM_TAG);
				}

				boundary_verts = spr_get_patch(v, loose_verts,
											   &num_boundary_verts);
				if (boundary_verts) {
					printf("%s: %d boundary, %d loose\n",
						   __func__, num_boundary_verts,
						   BLI_ghash_size(loose_verts));

					if (spr_patch_update(bm, boundary_verts,
										 num_boundary_verts,
										 loose_verts))
					{
						modified = TRUE;
					}
					
					MEM_freeN(boundary_verts);
					BLI_ghash_free(loose_verts, NULL, NULL);

					break;
				}

				BLI_ghash_free(loose_verts, NULL, NULL);
			}
		}
	} while(modified);
}
