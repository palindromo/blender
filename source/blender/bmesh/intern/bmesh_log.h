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

#ifndef __BMESH_UNDO_H__
#define __BMESH_UNDO_H__

/** \file blender/bmesh/intern/bmesh_undo.h
 *  \ingroup bmesh
 */

typedef struct BMLog BMLog;
typedef struct BMLogEntry BMLogEntry;
typedef struct BMLogGroup BMLogGroup;

/* Alloc/free BMLog */
BMLog *bm_log_create(BMesh *bm);
BMLog *bm_log_copy(BMesh *dst_bm, BMLog *src_log);
void bm_log_free(BMLog *log);
int bm_log_num_bytes(BMesh *bm);
int bm_log_full_copy_num_bytes(BMesh *bm);

/* Locking */
void bm_log_lock(BMesh *bm);
void bm_log_unlock(BMesh *bm);

/* Undo or redo an entire group of log entries */
void bm_undo(BMesh *bm);
void bm_redo(BMesh *bm);

/* Log groups (high-level operations) */
void bm_log_group_create(BMLog *log, const char description[]);
BMLogGroup *bm_log_group_current(BMLog *log);
const char *bm_log_group_description(const BMLogGroup *group);
void bm_log_group_description_set(BMLogGroup *group, const char *description);
const BMLogEntry *bm_log_group_last_entry(const BMLogGroup *group);

/* Log entries (low-level operations) */
char *bm_log_entry_description(BMLog *log, BMLogEntry *group);

/* Attributes */
void bm_log_coord_set(BMesh *bm, BMVert *v);
const float *bm_log_coords_get(const BMLogEntry *entry, unsigned int id);
void bm_log_hflag_set(BMesh *bm, BMHeader *head, char new_hflag);

/* Euler operators */
void bm_log_vert_create_kill(BMesh *bm, const BMVert *v, int inverse);
void bm_log_edge_create_kill(BMesh *bm, const BMEdge *e, int inverse);
void bm_log_face_create_kill(BMesh *bm, BMFace *f, int inverse);

void bm_log_semv(BMesh *bm,
				 const BMVert *target_vert,
				 const BMEdge *orig_edge,
				 const BMEdge *new_edge,
				 const BMVert *new_vert);

void bm_log_sf_jf(BMesh *bm, BMFace *f_orig, BMFace *f_new, BMEdge *e,
				  int inverse);

void bm_log_splice(BMesh *bm, BMVert *v, BMVert *vtarget);

/* Element IDs */
void bm_elem_id_assign(BMesh *bm, BMHeader *head);
void bm_elem_id_delete(BMesh *bm, BMHeader *head);
void *bm_id_lookup(BMesh *bm, unsigned int id);

/* Iteration */
typedef void (*BMLogGroupsIterFunc)(BMLog *log, BMLogGroup *group, void *data);
typedef void (*BMLogGroupEntriesIterFunc)(BMLog *log, BMLogEntry *entry,
										  void *data);

void bm_log_groups_iter(BMesh *bm, BMLogGroupsIterFunc func, void *data);
void bm_log_group_entries_iter(BMLog *log, BMLogGroup *group,
							   BMLogGroupEntriesIterFunc func, void *data);

#endif /* __BMESH_UNDO_H__ */
