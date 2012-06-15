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
 * ***** END GPL LICENSE BLOCK *****
 */

#include "MEM_guardedalloc.h"

#include "BLI_ghash.h"
#include "BLI_listbase.h"
#include "BLI_math.h"
#include "BLI_string.h"
#include "BLI_utildefines.h"

#include "bmesh.h"

#include "range_tree_c_api.h"

// Enable for testing memory usage of bmlog vs. straight copying
#define BMLOG_DEBUG_RECORD_BMESH_SIZE 1

#if BMLOG_DEBUG_RECORD_BMESH_SIZE
static int bmesh_num_bytes(const BMesh *bm)
{
	// TODO: doesn't include customdata or listbases yet

	// TODO: verify this calculation is correct

	return (sizeof(*bm) +
			BLI_mempool_num_bytes(bm->vpool) +
			BLI_mempool_num_bytes(bm->epool) +
			BLI_mempool_num_bytes(bm->lpool) +
			BLI_mempool_num_bytes(bm->fpool) +
			BLI_mempool_num_bytes(bm->toolflagpool));
}
#endif

typedef enum {
	BM_LOG_COORDS_SET,
	BM_LOG_HFLAGS_SET,

	BM_LOG_VERT_CREATE_KILL,
	BM_LOG_EDGE_CREATE_KILL,
	BM_LOG_FACE_CREATE_KILL,

	BM_LOG_SEMV_JEKV,
	BM_LOG_SF_JF,

	BM_LOG_VERT_SPLICE_RIP
} BMLogEntryType;

struct BMLogEntry {
	struct BMLogEntry *next, *prev;
	BMLogEntryType type;
	unsigned char inverse : 1;
};

struct BMLogGroup {
	struct BMLogGroup *next, *prev;
	ListBase entries;
	const char *description;

#if BMLOG_DEBUG_RECORD_BMESH_SIZE
	int bm_size;
#endif
};

struct BMLog {
	ListBase groups;
	BMLogEntry *current_entry;
	BMLogGroup *current_group;

	/* Map from an element's head.id to the element's address */
	GHash *id_to_elem;

	/* Tree of free IDs */
	RangeTreeUInt *unused_ids;

	/* When recreating an element from the log, skip_id_assign is set to
	   TRUE before calling the create functions, then the ID can be set after */
	int skip_id_assign;

	/* During undo/redo operations, the log should not be modified */
	int lock;
};

static void bm_log_groups_free_from(BMLog *log, BMLogGroup *group_first);
static void bm_log_append(BMesh *bm, BMLogEntry *entry);
static void *bm_log_entry_alloc(BMLogEntryType type, int inverse);
static void bm_elem_id_update(BMesh *bm, BMHeader *head, unsigned int id);

void *bm_id_lookup(BMesh *bm, unsigned int id)
{
	void *key = SET_INT_IN_POINTER(id);
	BLI_assert(BLI_ghash_haskey(bm->log->id_to_elem, key));
	return BLI_ghash_lookup(bm->log->id_to_elem, key);
}

static void *bm_log_try_reusing_current(BMLog *log,
										BMLogEntryType type)
{
	BMLogEntry *current = log->current_entry;
	BMLogEntry *entry = NULL;

	if (current && current->type == type) {
		/* TODO: check this, think it's' OK but otherwise have to free
		   later entries too */
		BLI_assert(!current->next);

		entry = current;
		bm_log_groups_free_from(log, log->current_group->next);
	}

	return entry;
}

/***************************** CoordsSet ******************************/

typedef struct {
	float old_co[3];
} BMLogCoord;

typedef struct {
	BMLogEntry header;
	GHash *id_to_coords;
} BMLogCoordsSet;

void bm_log_coord_set(BMesh *bm, BMVert *v)
{
	if (bm->log && !bm->log->lock) {
		BMLogCoordsSet *entry;
		void *key;
		int append = FALSE;
		BMLogCoord *coord = NULL;

		/* If the current log entry is already coords, append to it
		   rather than creating a new log entry */
		entry = bm_log_try_reusing_current(bm->log, BM_LOG_COORDS_SET);
		if (!entry) {
			entry = bm_log_entry_alloc(BM_LOG_COORDS_SET, FALSE);
			append = TRUE;
		}

		if (!entry->id_to_coords)
			entry->id_to_coords = BLI_ghash_ptr_new(AT);

		key = SET_INT_IN_POINTER(v->head.id);

		if (BLI_ghash_haskey(entry->id_to_coords, key)) {
			coord = BLI_ghash_lookup(entry->id_to_coords, key);
		}
		else {
			/* TODO: mempool */
			coord = MEM_callocN(sizeof(*coord), AT);
			copy_v3_v3(coord->old_co, v->co);
			BLI_ghash_insert(entry->id_to_coords, key, coord);
		}

		if (append)
			bm_log_append(bm, &entry->header);
	}
}

const float *bm_log_coords_get(const BMLogEntry *entry_generic, unsigned int id)
{
	BMLogCoordsSet *entry;
	BMLogCoord *co;

	BLI_assert(entry_generic->type == BM_LOG_COORDS_SET);
	entry = (BMLogCoordsSet*)entry_generic;

	co = BLI_ghash_lookup(entry->id_to_coords, SET_INT_IN_POINTER(id));
	return co->old_co;
}

static void bm_log_coords_set_undo_redo(BMesh *bm, BMLogCoordsSet *entry)
{
	GHashIterator gh_iter;

	GHASH_ITER (gh_iter, entry->id_to_coords) {
		void *key = BLI_ghashIterator_getKey(&gh_iter);
		BMVert *v = bm_id_lookup(bm, GET_INT_FROM_POINTER(key));
		BMLogCoord *coord = BLI_ghashIterator_getValue(&gh_iter);

		swap_v3_v3(v->co, coord->old_co);
	}
}

/****************************** HFlagSet ******************************/

static void *set_2chars_in_ptr(char a, char b)
{
	int val = b;
	return SET_INT_IN_POINTER((val << 8) | a);
}

static void get_2chars_from_ptr(void *p, char *a, char *b)
{
	int val = GET_INT_FROM_POINTER(p);
	(*a) = val & 0xff;
	(*b) = (val >> 8) & 0xff;
}

typedef struct {
	BMLogEntry header;
	/* Each key is an element ID, the value is the old and new hflag
	   values for that ID (packed into a pointer) */
	GHash *id_to_hflags;
} BMLogHFlagsSet;

void bm_log_hflag_set(BMesh *bm, BMHeader *head, char new_hflag)
{
	if (bm->log && !bm->log->lock) {
		BMLogHFlagsSet *entry;
		void *key;
		int append = FALSE;
		char old_hflag = head->hflag;;

		/* If the current log entry is already coords, append to it
		   rather than creating a new log entry */
		entry = bm_log_try_reusing_current(bm->log, BM_LOG_HFLAGS_SET);
		if (!entry) {
			entry = bm_log_entry_alloc(BM_LOG_HFLAGS_SET, FALSE);
			append = TRUE;
		}

		if (!entry->id_to_hflags)
			entry->id_to_hflags = BLI_ghash_ptr_new(AT);

		key = SET_INT_IN_POINTER(head->id);

		if (BLI_ghash_haskey(entry->id_to_hflags, key)) {
			void *val = BLI_ghash_lookup(entry->id_to_hflags, key);
			char tmp_new;

			get_2chars_from_ptr(val, &old_hflag, &tmp_new);
			BLI_ghash_remove(entry->id_to_hflags, key, NULL, NULL);
		}

		BLI_ghash_insert(entry->id_to_hflags, key,
						 set_2chars_in_ptr(old_hflag, new_hflag));

		if (append)
			bm_log_append(bm, &entry->header);
	}
}

static void bm_log_hflags_set_undo_redo(BMesh *bm, BMLogHFlagsSet *entry, int undo)
{
	GHashIterator gh_iter;

	GHASH_ITER (gh_iter, entry->id_to_hflags) {
		void *key = BLI_ghashIterator_getKey(&gh_iter);
		BMHeader *head = bm_id_lookup(bm, GET_INT_FROM_POINTER(key));
		void *val = BLI_ghashIterator_getValue(&gh_iter);
		char old_hflag, new_hflag;

		get_2chars_from_ptr(val, &old_hflag, &new_hflag);

		head->hflag = (undo ? old_hflag : new_hflag);
	}
}

/************************* VertCreate/VertKill ************************/

typedef struct {
	BMLogEntry header;
	unsigned int id;
	float co[3];
	float no[3];
	char hflag;
	/* TODO: other attributes */
} BMLogVertCreateKill;

void bm_log_vert_create_kill(BMesh *bm, const BMVert *v, int inverse)
{
	if (bm->log && !bm->log->lock) {
		BMLogVertCreateKill *entry;

		entry = bm_log_entry_alloc(BM_LOG_VERT_CREATE_KILL, inverse);
		entry->id = v->head.id;
		copy_v3_v3(entry->co, v->co);
		copy_v3_v3(entry->no, v->no);
		entry->hflag = v->head.hflag;

		bm_log_append(bm, &entry->header);
	}
}

static void bm_log_vert_create_kill_undo(BMesh *bm, BMLogVertCreateKill *entry)
{
	BMVert *v = bm_id_lookup(bm, entry->id);
	BM_vert_kill(bm, v);
}

static void bm_log_vert_create_kill_redo(BMesh *bm, BMLogVertCreateKill *entry)
{
	BMVert *v;

	bm->log->skip_id_assign = TRUE;
	v = BM_vert_create(bm, entry->co, NULL);
	bm->log->skip_id_assign = FALSE;

	bm_elem_id_update(bm, &v->head, entry->id);
	v->head.id = entry->id;
	v->head.hflag = entry->hflag;
	copy_v3_v3(v->no, entry->no);
}

/***************************** EdgeCreate *****************************/

typedef struct {
	BMLogEntry header;
	unsigned int id;
	unsigned int id_v1;
	unsigned int id_v2;
	/* TODO: other attributes */
} BMLogEdgeCreateKill;

void bm_log_edge_create_kill(BMesh *bm, const BMEdge *e, int inverse)
{
	if (bm->log && !bm->log->lock) {
		BMLogEdgeCreateKill *entry;

		entry = bm_log_entry_alloc(BM_LOG_EDGE_CREATE_KILL, inverse);
		entry->id = e->head.id;
		entry->id_v1 = e->v1->head.id;
		entry->id_v2 = e->v2->head.id;

		bm_log_append(bm, &entry->header);
	}
}

static void bm_log_edge_create_kill_undo(BMesh *bm, BMLogEdgeCreateKill *entry)
{
	BMEdge *e = bm_id_lookup(bm, entry->id);
	BM_edge_kill(bm, e);
}

static void bm_log_edge_create_kill_redo(BMesh *bm, BMLogEdgeCreateKill *entry)
{
	BMEdge *e;
	BMVert *v1, *v2;

	v1 = bm_id_lookup(bm, entry->id_v1);
	v2 = bm_id_lookup(bm, entry->id_v2);

	bm->log->skip_id_assign = TRUE;
	e = BM_edge_create(bm, v1, v2, NULL, FALSE);
	bm->log->skip_id_assign = FALSE;

	bm_elem_id_update(bm, &e->head, entry->id);
}

/***************************** FaceCreate *****************************/

typedef struct {
	BMLogEntry header;
	unsigned int id;
	int len;
	unsigned int *verts;
	unsigned int *edges;
	/* TODO: other attributes */
} BMLogFaceCreateKill;

void bm_log_face_create_kill(BMesh *bm, BMFace *f, int inverse)
{
	if (bm->log && !bm->log->lock) {
		BMLogFaceCreateKill *entry;
		BMIter bm_iter;
		BMLoop *l;
		int i;

		entry = bm_log_entry_alloc(BM_LOG_FACE_CREATE_KILL, inverse);
		entry->id = f->head.id;

		entry->len = f->len;
		entry->verts = MEM_callocN(sizeof(*entry->verts) * f->len, AT);
		entry->edges = MEM_callocN(sizeof(*entry->edges) * f->len, AT);

		i = 0;
		BM_ITER_ELEM (l, &bm_iter, f, BM_LOOPS_OF_FACE) {
			entry->verts[i] = l->v->head.id;
			entry->edges[i] = l->e->head.id;
			i++;
		}

		bm_log_append(bm, &entry->header);
	}
}

static void bm_log_face_create_kill_undo(BMesh *bm, BMLogFaceCreateKill *entry)
{
	BMFace *f = bm_id_lookup(bm, entry->id);
	BM_face_kill(bm, f);
}

static void bm_log_face_create_kill_redo(BMesh *bm, BMLogFaceCreateKill *entry)
{
	BMVert **verts;
	BMEdge **edges;
	BMFace *f;
	int i;

	verts = MEM_callocN(sizeof(*verts) * entry->len, AT);
	edges = MEM_callocN(sizeof(*edges) * entry->len, AT);

	for (i = 0; i < entry->len; i++) {
		verts[i] = bm_id_lookup(bm, entry->verts[i]);
		edges[i] = bm_id_lookup(bm, entry->edges[i]);
	}

	bm->log->skip_id_assign = TRUE;
	f = BM_face_create(bm, verts, edges, entry->len, FALSE);
	bm->log->skip_id_assign = FALSE;
	MEM_freeN(verts);
	MEM_freeN(edges);

	bm_elem_id_update(bm, &f->head, entry->id);
}

/******************************** SEMV ********************************/

typedef struct {
	BMLogEntry header;
	unsigned int target_vert_id;
	unsigned int orig_edge_id;
	unsigned int new_edge_id;
	unsigned int new_vert_id;
} BMLogSEMV_JEKV;

void bm_log_semv(BMesh *bm,
				 const BMVert *target_vert,
				 const BMEdge *orig_edge,
				 const BMEdge *new_edge,
				 const BMVert *new_vert)
{
	if (bm->log && !bm->log->lock) {
		BMLogSEMV_JEKV *entry = bm_log_entry_alloc(BM_LOG_SEMV_JEKV, FALSE);

		entry->target_vert_id = target_vert->head.id;
		entry->orig_edge_id = orig_edge->head.id;
		entry->new_edge_id = new_edge->head.id;
		entry->new_vert_id = new_vert->head.id;

		bm_log_append(bm, &entry->header);
	}
}

static void bm_log_semv_jekv_undo(BMesh *bm, BMLogSEMV_JEKV *entry)
{
	BMEdge *kill_edge = bm_id_lookup(bm, entry->new_edge_id);
	BMVert *kill_vert = bm_id_lookup(bm, entry->new_vert_id);

	bmesh_jekv(bm, kill_edge, kill_vert, FALSE);
}

static void bm_log_semv_jekv_redo(BMesh *bm, BMLogSEMV_JEKV *entry)
{
	BMVert *new_vert, *target_vert;
	BMEdge *new_edge, *orig_edge;

	target_vert = bm_id_lookup(bm, entry->target_vert_id);
	orig_edge = bm_id_lookup(bm, entry->orig_edge_id);

	bm->log->skip_id_assign = TRUE;
	new_vert = bmesh_semv(bm, target_vert, orig_edge, &new_edge);
	bm->log->skip_id_assign = FALSE;

	bm_elem_id_update(bm, &new_vert->head, entry->new_vert_id);
	bm_elem_id_update(bm, &new_edge->head, entry->new_edge_id);
}

/***************************** SF / JF ****************************/

typedef struct {
	BMLogEntry header;
	unsigned int f_orig_id;
	unsigned int f_new_id;
	unsigned int e_id;
} BMLogSF_JF;

void bm_log_sf_jf(BMesh *bm, BMFace *f_orig, BMFace *f_new,
				  BMEdge *e, int inverse)
{
	if (bm->log && !bm->log->lock) {
		BMLogSF_JF *entry = bm_log_entry_alloc(BM_LOG_SF_JF, inverse);

		entry->f_orig_id = f_orig->head.id;
		entry->f_new_id = f_new->head.id;
		entry->e_id = e->head.id;

		bm_log_append(bm, &entry->header);
	}
}

/* XXX: SFME and JFKE are not proper inverses. Consider two quads that
 * share three vertices. If SFME is run on each face using the two
 * vertices adjacent to the one unshared vertex and nodoubles is true,
 * attempting to reverse that operation with JFKE will fail because
 * the edge is not manifold -- it is used by four faces.
 *
 * This case can be seen by running triangulate on Suzanne -- the area
 * above the nose gives issues.
 */

static void bm_log_sf_jf_undo(BMesh *bm, BMLogSF_JF *entry)
{
	BMFace *f_orig, *f_new, *f_out;
	BMEdge *e;

	f_orig = bm_id_lookup(bm, entry->f_orig_id);
	f_new = bm_id_lookup(bm, entry->f_new_id);
	e = bm_id_lookup(bm, entry->e_id);

	f_out = bmesh_jf(bm, f_orig, f_new, e);
	BLI_assert(f_out == f_orig);
	BLI_assert(f_orig->head.id == entry->f_orig_id);
}

static void bm_log_sf_jf_redo(BMesh *bm, BMLogSF_JF *entry)
{
	BMFace *f_orig, *f_new;
	BMEdge *e;

	f_orig = bm_id_lookup(bm, entry->f_orig_id);
	e = bm_id_lookup(bm, entry->e_id);
	bm->log->skip_id_assign = TRUE;
	f_new = bmesh_sf(bm, f_orig, e, NULL);
	bm->log->skip_id_assign = FALSE;
	BLI_assert(f_new);

	bm_elem_id_update(bm, &f_new->head, entry->f_new_id);
}

/**************************** Vertex Splice ***************************/

typedef struct {
	BMLogEntry header;
	unsigned int vert_id;
	unsigned int target_vert_id;
	unsigned int *edge_ids;
	int totedge;
	float vert_co[3];
} BMLogVertSpliceRip;

void bm_log_splice(BMesh *bm, BMVert *v, BMVert *vtarget)
{
	if (bm->log && !bm->log->lock) {
		BMLogVertSpliceRip *entry = bm_log_entry_alloc(BM_LOG_VERT_SPLICE_RIP, FALSE);
		BMIter bm_iter;
		BMEdge *e;

		entry->target_vert_id = vtarget->head.id;
		entry->vert_id = v->head.id;

		entry->edge_ids = MEM_callocN(sizeof(*entry->edge_ids), AT);
		BM_ITER_ELEM (e, &bm_iter, v, BM_EDGES_OF_VERT) {
			entry->edge_ids[entry->totedge++] = e->head.id;
		}

		copy_v3_v3(entry->vert_co, v->co);

		bm_log_append(bm, &entry->header);
	}
}

static void bm_log_vert_splice_rip_undo(BMesh *bm, BMLogVertSpliceRip *entry)
{
	BMVert **vout;
	BMEdge **edges;
	int i, vout_len;

	#if 0
	/* Re-create vert */
	bm->log->skip_id_assign = TRUE;
	v = BM_vert_create(bm, entry->vert_co, NULL);
	bm->log->skip_id_assign = FALSE;
	bm_elem_id_update(bm, &v->head, entry->vert_id);

	#endif

	edges = MEM_callocN(sizeof(*edges) * entry->totedge, AT);
	for (i = 0; i < entry->totedge; i++)
		edges[i] = bm_id_lookup(bm, entry->edge_ids[i]);

	BM_vert_separate(bm, bm_id_lookup(bm, entry->target_vert_id),
					 &vout, &vout_len,
					 edges, entry->totedge);

	MEM_freeN(edges);
}

static void bm_log_vert_splice_rip_redo(BMesh *bm, BMLogVertSpliceRip *entry)
{
	BM_vert_splice(bm,
				   bm_id_lookup(bm, entry->vert_id),
				   bm_id_lookup(bm, entry->target_vert_id));
}

/**********************************************************************/

/* Allocate a log entry of the specified type and set its type field
   (void* return to avoid casting) */
static void *bm_log_entry_alloc(BMLogEntryType type, int inverse)
{
	BMLogEntry *entry = NULL;

	BLI_assert(inverse == !!inverse);

	/* TODO: could use mempools for these */

	switch(type) {
		case BM_LOG_COORDS_SET:
			entry = MEM_callocN(sizeof(BMLogCoordsSet), AT);
			break;
		case BM_LOG_HFLAGS_SET:
			entry = MEM_callocN(sizeof(BMLogHFlagsSet), AT);
			break;
		case BM_LOG_VERT_CREATE_KILL:
			entry = MEM_callocN(sizeof(BMLogVertCreateKill), AT);
			break;
		case BM_LOG_EDGE_CREATE_KILL:
			entry = MEM_callocN(sizeof(BMLogEdgeCreateKill), AT);
			break;
		case BM_LOG_FACE_CREATE_KILL:
			entry = MEM_callocN(sizeof(BMLogFaceCreateKill), AT);
			break;
		case BM_LOG_SEMV_JEKV:
			entry = MEM_callocN(sizeof(BMLogSEMV_JEKV), AT);
			break;
		case BM_LOG_SF_JF:
			entry = MEM_callocN(sizeof(BMLogSF_JF), AT);
			break;
		case BM_LOG_VERT_SPLICE_RIP:
			entry = MEM_callocN(sizeof(BMLogVertSpliceRip), AT);
			break;
	}

	entry->type = type;
	entry->inverse = inverse;

	return entry;
}

/**********************************************************************/

/* Run the undo action for 'entry'
 * 
 * Always runs undo regardless of the BMLogEntry.inverse field.
 */
static void bm_log_entry_undo(BMesh *bm, BMLogEntry *entry)
{
	switch(entry->type) {
		case BM_LOG_COORDS_SET:
			bm_log_coords_set_undo_redo(bm, (BMLogCoordsSet*)entry);
			break;
		case BM_LOG_HFLAGS_SET:
			bm_log_hflags_set_undo_redo(bm, (BMLogHFlagsSet*)entry, TRUE);
			break;
		case BM_LOG_VERT_CREATE_KILL:
			bm_log_vert_create_kill_undo(bm, (BMLogVertCreateKill*)entry);
			break;
		case BM_LOG_EDGE_CREATE_KILL:
			bm_log_edge_create_kill_undo(bm, (BMLogEdgeCreateKill*)entry);
			break;
		case BM_LOG_FACE_CREATE_KILL:
			bm_log_face_create_kill_undo(bm, (BMLogFaceCreateKill*)entry);
			break;
		case BM_LOG_SEMV_JEKV:
			bm_log_semv_jekv_undo(bm, (BMLogSEMV_JEKV*)entry);
			break;
		case BM_LOG_SF_JF:
			bm_log_sf_jf_undo(bm, (BMLogSF_JF*)entry);
			break;
		case BM_LOG_VERT_SPLICE_RIP:
			bm_log_vert_splice_rip_undo(bm, (BMLogVertSpliceRip*)entry);
			break;
	}
}

/* Run the redo action for 'entry'
 * 
 * Always runs redo regardless of the BMLogEntry.inverse field.
 */
static void bm_log_entry_redo(BMesh *bm, BMLogEntry *entry)
{
	switch(entry->type) {
		case BM_LOG_COORDS_SET:
			bm_log_coords_set_undo_redo(bm, (BMLogCoordsSet*)entry);
			break;
		case BM_LOG_HFLAGS_SET:
			bm_log_hflags_set_undo_redo(bm, (BMLogHFlagsSet*)entry, FALSE);
			break;
		case BM_LOG_VERT_CREATE_KILL:
			bm_log_vert_create_kill_redo(bm, (BMLogVertCreateKill*)entry);
			break;
		case BM_LOG_EDGE_CREATE_KILL:
			bm_log_edge_create_kill_redo(bm, (BMLogEdgeCreateKill*)entry);
			break;
		case BM_LOG_FACE_CREATE_KILL:
			bm_log_face_create_kill_redo(bm, (BMLogFaceCreateKill*)entry);
			break;
		case BM_LOG_SEMV_JEKV:
			bm_log_semv_jekv_redo(bm, (BMLogSEMV_JEKV*)entry);
			break;
		case BM_LOG_SF_JF:
			bm_log_sf_jf_redo(bm, (BMLogSF_JF*)entry);
			break;
		case BM_LOG_VERT_SPLICE_RIP:
			bm_log_vert_splice_rip_redo(bm, (BMLogVertSpliceRip*)entry);
			break;
	}
}

/* Undo each entry in the current group */
void bm_undo(BMesh *bm)
{
	BMLogGroup *group = bm->log->current_group;

	bm_log_lock(bm);

	if (group) {
		BMLogEntry *entry;

		/* Undo each entry in the group from last to first */
		for (entry = group->entries.last; entry; entry = entry->prev) {
			if (!entry->inverse)
				bm_log_entry_undo(bm, entry);
			else
				bm_log_entry_redo(bm, entry);
		}

		/* Move to previous group */
		bm->log->current_group = group->prev;

		/* Normals are not stored in the log, recalculate them */
		BM_mesh_normals_update(bm, FALSE);
	}

	bm_log_unlock(bm);
}

void bm_redo(BMesh *bm)
{
	BMLogGroup *group = bm->log->current_group;

	bm_log_lock(bm);

	if (group)
		group = group->next;
	else
		group = bm->log->groups.first;

	if (group) {
		BMLogEntry *entry;

		/* Redo each entry in the group from first to last */
		for (entry = group->entries.first; entry; entry = entry->next) {
			if (!entry->inverse)
				bm_log_entry_redo(bm, entry);
			else
				bm_log_entry_undo(bm, entry);
		}

		/* Move to next group */
		bm->log->current_group = group;

		/* Normals are not stored in the log, recalculate them */
		BM_mesh_normals_update(bm, FALSE);
	}

	bm_log_unlock(bm);
}

/***************************** Element ID *****************************/

BMLog *bm_log_create(BMesh *bm)
{
	BMLog *log = MEM_callocN(sizeof(*log), AT);
	BMIter bm_iter;
	BMVert *v;
	BMEdge *e;
	BMFace *f;
	unsigned int id;

	log->id_to_elem = BLI_ghash_ptr_new(AT);

	/* If the BMesh already has mesh elements, give them IDs */
	id = 0;
	BM_ITER_MESH (v, &bm_iter, bm, BM_VERTS_OF_MESH) {
		v->head.id = id;
		BLI_ghash_insert(log->id_to_elem, SET_INT_IN_POINTER(id), v);
		id++;
	}
	BM_ITER_MESH (e, &bm_iter, bm, BM_EDGES_OF_MESH) {
		e->head.id = id;
		BLI_ghash_insert(log->id_to_elem, SET_INT_IN_POINTER(id), e);
		id++;
	}
	BM_ITER_MESH (f, &bm_iter, bm, BM_FACES_OF_MESH) {
		f->head.id = id;
		BLI_ghash_insert(log->id_to_elem, SET_INT_IN_POINTER(id), f);
		id++;
	}

	/* Initialize unused IDs */
	log->unused_ids = range_tree_uint_alloc(id, (unsigned)-1);

	bm_log_group_create(log, "(initial)");

	return log;
}

/* TODO: move to blenlib */
typedef Link *(*BLI_listbase_copy_func)(Link *l);
void BLI_listbase_copy(ListBase *dst, ListBase *src,
					   BLI_listbase_copy_func copy)
{
	Link *l;

	memset(dst, 0, sizeof(*dst));

	for (l = src->first; l; l = l->next) {
		BLI_addtail(dst, copy(l));
	}
}

/* TODO: move to blenlib */
/* XXX: delete this? not needed anymore */
typedef void *(*GHashCopyFP)(void *src);
GHash *BLI_ghash_copy(GHash *src, GHashCopyFP copy_key, GHashCopyFP copy_val)
{
	GHash *dst = BLI_ghash_new(src->hashfp, src->cmpfp, AT);
	GHashIterator iter;

	GHASH_ITER (iter, src) {
		void *key = BLI_ghashIterator_getKey(&iter);
		void *val = BLI_ghashIterator_getValue(&iter);

		if (copy_key)
			key = copy_key(key);
		if (copy_val)
			val = copy_val(val);

		BLI_ghash_insert(dst, key, val);
	}

	return dst;
}

BMLogEntry *bm_log_entry_copy(BMLogEntry *src)
{
	BMLogEntry *dst = MEM_dupallocN(src);

	switch (dst->type) {
		case BM_LOG_FACE_CREATE_KILL:
			((BMLogFaceCreateKill*)dst)->verts =
				MEM_dupallocN(((BMLogFaceCreateKill*)src)->verts);
			((BMLogFaceCreateKill*)dst)->edges =
				MEM_dupallocN(((BMLogFaceCreateKill*)src)->edges);
			break;

		default:
			break;
	}

	return dst;
}

BMLogGroup *bm_log_group_copy(BMLogGroup *src)
{
	BMLogGroup *dst = MEM_dupallocN(src);

	BLI_listbase_copy(&dst->entries, &src->entries,
					  (BLI_listbase_copy_func)bm_log_entry_copy);

	return dst;
}

/* Return a deep copy of 'log' */
BMLog *bm_log_copy(BMesh *dst_bm, BMLog *src_log)
{
	BMLog *dst = MEM_dupallocN(src_log);
	BMIter bm_iter;
	BMVert *v;
	BMEdge *e;
	BMFace *f;

	BLI_listbase_copy(&dst->groups, &src_log->groups,
					  (BLI_listbase_copy_func)bm_log_group_copy);

	/* TODO: update current */

	/* Update id_to_elem mapping */
	dst->id_to_elem = BLI_ghash_ptr_new(AT);
	BM_ITER_MESH (v, &bm_iter, dst_bm, BM_VERTS_OF_MESH)
		BLI_ghash_insert(dst->id_to_elem, SET_INT_IN_POINTER(v->head.id), v);
	BM_ITER_MESH (e, &bm_iter, dst_bm, BM_EDGES_OF_MESH)
		BLI_ghash_insert(dst->id_to_elem, SET_INT_IN_POINTER(e->head.id), e);
	BM_ITER_MESH (f, &bm_iter, dst_bm, BM_FACES_OF_MESH)
		BLI_ghash_insert(dst->id_to_elem, SET_INT_IN_POINTER(f->head.id), f);
	
	dst->unused_ids = range_tree_uint_copy(src_log->unused_ids);

	return dst;
}

void bm_log_lock(BMesh *bm)
{
	if (bm->log) {
		BLI_assert(bm->log->lock >= 0);
		bm->log->lock++;
	}
}
void bm_log_unlock(BMesh *bm)
{
	if (bm->log) {
		bm->log->lock--;
		BLI_assert(bm->log->lock >= 0);
	}
}

/* Does not free the log itself */
void bm_log_free(BMLog *log)
{
	bm_log_groups_free_from(log, log->groups.first);

	BLI_ghash_free(log->id_to_elem, NULL, NULL);

	range_tree_uint_free(log->unused_ids);
}

static void bm_elem_id_update(BMesh *bm, BMHeader *head, unsigned int id)
{
	void *key;

	range_tree_uint_take(bm->log->unused_ids, id);
	head->id = id;

	/* Add mapping from ID to element */
	key = SET_INT_IN_POINTER(head->id);
	BLI_assert(!BLI_ghash_haskey(bm->log->id_to_elem, key));
	BLI_ghash_insert(bm->log->id_to_elem, key, head);
}

void bm_elem_id_assign(BMesh *bm, BMHeader *head)
{
	/* Assign next ID */
	if (bm->log && !bm->log->skip_id_assign) {
		void *key;

		head->id = range_tree_uint_take_any(bm->log->unused_ids);

		/* Add mapping from ID to element */
		key = SET_INT_IN_POINTER(head->id);
		BLI_assert(!BLI_ghash_haskey(bm->log->id_to_elem, key));
		BLI_ghash_insert(bm->log->id_to_elem, key, head);
	}
}

void bm_elem_id_delete(BMesh *bm, BMHeader *head)
{
	if (bm->log) {
		void *key = SET_INT_IN_POINTER(head->id);
		range_tree_uint_release(bm->log->unused_ids, head->id);
		BLI_assert(BLI_ghash_haskey(bm->log->id_to_elem, key));
		BLI_ghash_remove(bm->log->id_to_elem, key, NULL, NULL);
	}
}

/***************************** Log Entries ****************************/

/* Does not free 'entry' itself */
static void bm_log_entry_free(BMLogEntry *entry)
{
	switch(entry->type) {
		case BM_LOG_COORDS_SET:
			BLI_ghash_free(((BMLogCoordsSet*)entry)->id_to_coords,
						   NULL, (void*)MEM_freeN);
			break;
		case BM_LOG_HFLAGS_SET:
			BLI_ghash_free(((BMLogHFlagsSet*)entry)->id_to_hflags, NULL, NULL);
			break;
		case BM_LOG_FACE_CREATE_KILL:
			MEM_freeN(((BMLogFaceCreateKill*)entry)->verts);
			MEM_freeN(((BMLogFaceCreateKill*)entry)->edges);
			break;
		default:
			break;
	}
}

/* Does not free 'group' itself */
static void bm_log_group_free(BMLogGroup *group)
{
	BMLogEntry *entry;

	for (entry = group->entries.first; entry; entry = entry->next)
		bm_log_entry_free(entry);

	BLI_freelistN(&group->entries);
}

/* Free all the log groups starting at (and including) 'group_first'
   until the end of the list */
static void bm_log_groups_free_from(BMLog *log, BMLogGroup *group_first)
{
	BMLogGroup *group, *group_next;

	for (group = group_first; group; group = group_next) {
		group_next = group->next;
		bm_log_group_free(group);
		BLI_freelinkN(&log->groups, group);
	}
}

static void bm_log_append(BMesh *bm, BMLogEntry *entry)
{
	BMLogGroup *group = bm->log->current_group;

	/* Delete any groups after the current one */
	if (group) {
		bm_log_groups_free_from(bm->log, group->next);
	}
	else {
		bm_log_groups_free_from(bm->log, bm->log->groups.first);
		/* Create a new group */
		bm_log_group_create(bm->log, "Group");
		group = bm->log->groups.first;
		bm->log->current_group = group;
	}

	/* Append the new entry */
	BLI_addtail(&group->entries, entry);
	bm->log->current_entry = entry;

#if BMLOG_DEBUG_RECORD_BMESH_SIZE
	group->bm_size = bmesh_num_bytes(bm);
#endif
}

void bm_log_group_create(BMLog *log, const char description[])
{
	BMLogGroup *group = log->current_group;

	/* Delete any groups after the current one */
	/* TODO: de-duplicate */
	if (group)
		bm_log_groups_free_from(log, group->next);
	else {
		bm_log_groups_free_from(log, log->groups.first);
	}

	if (group && !group->entries.first) {
		/* The previous group was never used, reuse it */
		group->description = description;
	}
	else {
		/* Create a new group */
		group = MEM_callocN(sizeof(BMLogGroup), AT);
		group->description = description;
		BLI_addtail(&log->groups, group);
		log->current_group = group;
	}

	log->current_entry = NULL;
}

BMLogGroup *bm_log_group_current(BMLog *log)
{
	return log->current_group;
}

const char *bm_log_group_description(const BMLogGroup *group)
{
	return group->description;
}

void bm_log_group_description_set(BMLogGroup *group, const char *description)
{
	group->description = description;
}

const BMLogEntry *bm_log_group_last_entry(const BMLogGroup *group)
{
	return group->entries.last;
}

/******************************** Debug *******************************/

static int bm_log_entry_num_bytes(const BMLogEntry *entry_g)
{
	switch (entry_g->type) {
		case BM_LOG_COORDS_SET: {
			BMLogCoordsSet *entry = (BMLogCoordsSet*)entry_g;
			return (sizeof(*entry) +
					BLI_ghash_num_bytes(entry->id_to_coords) +
					(BLI_ghash_size(entry->id_to_coords) *
					 sizeof(BMLogCoord)));
		}

		case BM_LOG_HFLAGS_SET: {
			BMLogHFlagsSet *entry = (BMLogHFlagsSet*)entry_g;
			return (sizeof(*entry) +
					BLI_ghash_num_bytes(entry->id_to_hflags));
		}

		case BM_LOG_VERT_CREATE_KILL: {
			BMLogVertCreateKill *entry = (BMLogVertCreateKill*)entry_g;
			return sizeof(*entry);
		}

		case BM_LOG_EDGE_CREATE_KILL: {
			BMLogEdgeCreateKill *entry = (BMLogEdgeCreateKill*)entry_g;
			return sizeof(*entry);
		}

		case BM_LOG_FACE_CREATE_KILL: {
			BMLogFaceCreateKill *entry = (BMLogFaceCreateKill*)entry_g;
			return sizeof(*entry);
		}

		case BM_LOG_SEMV_JEKV: {
			BMLogSEMV_JEKV *entry = (BMLogSEMV_JEKV*)entry_g;
			return sizeof(*entry);
		}

		case BM_LOG_SF_JF: {
			BMLogSF_JF *entry = (BMLogSF_JF*)entry_g;
			return sizeof(*entry);
		}

		case BM_LOG_VERT_SPLICE_RIP: {
			BMLogVertSpliceRip *entry = (BMLogVertSpliceRip*)entry_g;
			return sizeof(*entry);
		}
	}

	return 0;
}

// Includes the size of the bmesh itself
int bm_log_num_bytes(BMesh *bm)
{
	if (bm && bm->log) {
		BMLogGroup *group;
		int num_bytes = sizeof(*bm->log);

		num_bytes += range_tree_uint_allocation_lower_bound(bm->log->unused_ids);

		for (group = bm->log->groups.first; group; group = group->next) {
			BMLogEntry *entry;

			num_bytes += sizeof(*group);
			for (entry = group->entries.first; entry; entry = entry->next)
				num_bytes += bm_log_entry_num_bytes(entry);
		}

		// Add bmesh size
		num_bytes += bmesh_num_bytes(bm);

		return num_bytes;
	}
	else
		return 0;
}

int bm_log_full_copy_num_bytes(BMesh *bm)
{
	int num_bytes = 0;

	if (bm && bm->log) {

#if BMLOG_DEBUG_RECORD_BMESH_SIZE
		BMLogGroup *group;

		for (group = bm->log->groups.first; group; group = group->next)
			num_bytes += group->bm_size;
#endif
	}

	return num_bytes;
}

static const char *bm_log_entry_type_name(BMLogEntryType type)
{
	switch(type) {
#define S_(type_) \
		case type_: \
			return #type_

		S_(BM_LOG_COORDS_SET);
		S_(BM_LOG_HFLAGS_SET);
		S_(BM_LOG_VERT_CREATE_KILL);
		S_(BM_LOG_EDGE_CREATE_KILL);
		S_(BM_LOG_FACE_CREATE_KILL);
		S_(BM_LOG_SEMV_JEKV);
		S_(BM_LOG_SF_JF);
		S_(BM_LOG_VERT_SPLICE_RIP);

#undef S_
	}

	return NULL;
}

char *bm_log_entry_description(BMLog *log, BMLogEntry *entry)
{
	char *special = NULL, *desc;

	switch(entry->type) {
		GHash *idset;
		BMLogVertCreateKill *vert;

		case BM_LOG_COORDS_SET:
			idset = ((BMLogCoordsSet*)entry)->id_to_coords;
			special = BLI_sprintfN("totvert=%d", BLI_ghash_size(idset));
			break;
		case BM_LOG_HFLAGS_SET:
			idset = ((BMLogHFlagsSet*)entry)->id_to_hflags;
			special = BLI_sprintfN("num=%d", BLI_ghash_size(idset));
			break;
		case BM_LOG_VERT_CREATE_KILL:
			vert = (BMLogVertCreateKill*)entry;
			special = BLI_sprintfN("id=%d, co={%0.2f, %0.2f, %0.2f}",
								   vert->id, vert->co[0],
								   vert->co[1], vert->co[2]);
			break;
		case BM_LOG_EDGE_CREATE_KILL:
			special = BLI_sprintfN("id=%d", ((BMLogEdgeCreateKill*)entry)->id);
			break;
		case BM_LOG_FACE_CREATE_KILL:
			special = BLI_sprintfN("id=%d, len=%d",
								   ((BMLogFaceCreateKill*)entry)->id,
								   ((BMLogFaceCreateKill*)entry)->len);
			break;

		case BM_LOG_SEMV_JEKV:
			special = BLI_sprintfN("e=%d, tv=%d, ne=%d, nv=%d",
								   ((BMLogSEMV_JEKV*)entry)->orig_edge_id,
								   ((BMLogSEMV_JEKV*)entry)->target_vert_id,
								   ((BMLogSEMV_JEKV*)entry)->new_edge_id,
								   ((BMLogSEMV_JEKV*)entry)->new_vert_id);
			break;
		case BM_LOG_SF_JF:
			//special = BLI_sprintfN("");
			break;
		default:
			break;
	}

	desc = BLI_sprintfN("%s(%s)%s%s",
						bm_log_entry_type_name(entry->type),
						special ? special : "",
						entry->inverse ? " inverse" : "",
						(log->current_entry == entry) ? " current" : "");

	if (special)
		MEM_freeN(special);
	return desc;
}

/****************************** Iteration *****************************/

void bm_log_groups_iter(BMesh *bm, BMLogGroupsIterFunc func, void *data)
{
	if (bm->log) {
		BMLogGroup *group;

		for (group = bm->log->groups.first; group; group = group->next)
			func(bm->log, group, data);
	}
}

void bm_log_group_entries_iter(BMLog *log, BMLogGroup *group,
							   BMLogGroupEntriesIterFunc func, void *data)
{
	BMLogEntry *entry;

	for (entry = group->entries.first; entry; entry = entry->next)
		func(log, entry, data);
}
