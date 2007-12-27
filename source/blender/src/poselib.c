/**
 * $Id$
 *
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
 * Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * The Original Code is Copyright (C) 2007, Blender Foundation
 * This is a new part of Blender
 *
 * Contributor(s): Joshua Leung
 *
 * ***** END GPL LICENSE BLOCK *****
 */
 
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "MEM_guardedalloc.h"

#include "BLI_arithb.h"
#include "BLI_blenlib.h"
#include "BLI_dynstr.h"

#include "DNA_listBase.h"
#include "DNA_action_types.h"
#include "DNA_armature_types.h"
#include "DNA_curve_types.h"
#include "DNA_ipo_types.h"
#include "DNA_object_types.h"
#include "DNA_object_force.h"
#include "DNA_scene_types.h"

#include "BKE_action.h"
#include "BKE_armature.h"
#include "BKE_depsgraph.h"
#include "BKE_ipo.h"
#include "BKE_modifier.h"
#include "BKE_object.h"

#include "BKE_global.h"
#include "BKE_utildefines.h"

//#include "BIF_keyframing.h"
#include "BSE_editipo.h"

#include "BDR_drawaction.h"

#include "BIF_poselib.h"
#include "BIF_interface.h"
#include "BIF_editaction.h"
#include "BIF_space.h"
#include "BIF_screen.h"
#include "BIF_toets.h"
#include "BIF_toolbox.h"

#include "blendef.h"

#include "PIL_time.h"			/* sleep				*/
#include "mydevice.h"

/* ************************************************************* */
/* == POSE-LIBRARY TOOL FOR BLENDER == 
 *	
 * Overview: 
 * 	This tool allows animators to store a set of frequently used poses to dump into
 * 	the active action to help in "budget" productions to quickly block out new actions.
 * 	It acts as a kind of "glorified clipboard for poses", allowing for naming of poses.
 *
 * Features:
 *	- PoseLibs are simply normal Actions, but with a poselib
 *	- Each "pose" is simply a set of keyframes that occur on a particular frame
 *		-> a bPoseLibRef struct is used to identify and label poses in the Action
 *		-> all bPoseLibRefs are stored in the order they were added
 *		-> keys for poses should occur on each positively numbered frame (starting from frame 1)
 *	- The Scrollwheel or PageUp/Down buttons when used in a special mode or after pressing/holding
 *	  [a modifier] key, cycles through the poses available for the active pose's poselib, allowing the
 *	  animator to preview what action best suits that pose
 */
/* ************************************************************* */

/* gets list of poses in poselib as a string */
char *poselib_build_poses_menu (bPoseLib *pl, char title[])
{
	DynStr *pupds= BLI_dynstr_new();
	bPoseLibRef *plr;
	char *str;
	char buf[64];
	int i;
	
	/* add title first */
	sprintf(buf, "%s%%t|", title);
	BLI_dynstr_append(pupds, buf);
	
	/* loop through keyingsets, adding them */
	for (plr=pl->poses.first, i=1; plr; plr=plr->next, i++) {
		BLI_dynstr_append(pupds, plr->name);
		
		sprintf(buf, "%%x%d", i);
		BLI_dynstr_append(pupds, buf);
		
		if (plr->next)
			BLI_dynstr_append(pupds, "|");
	}
	
	/* convert to normal MEM_malloc'd string */
	str= BLI_dynstr_get_cstring(pupds);
	BLI_dynstr_free(pupds);
	
	return str;
}

/* finds an unique name for pose to be added to poselib */
void poselib_unique_pose_name (bPoseLib *pl, char name[])
{
	bPoseLibRef *plr;
	char tempname[32];
	int	number = 1, exists = 0;
	char *dot;
	
	/* See if we are given an empty string */
	if (name[0] == '\0') {
		/* give it default name first */
		strcpy(name, "Pose");
	}
	
	/* See if we even need to do this */	
	for (plr= pl->poses.first; plr; plr= plr->next) {
		if (!strcmp(name, plr->name)) {
			exists = 1;
			break;
		}
	}
	
	if (exists == 0)
		return;

	/*	Strip off the suffix */
	dot = strchr(name, '.');
	if (dot)
		*dot=0;
	
	for (number = 1; number <= 999; number++) {
		sprintf(tempname, "%s.%03d", name, number);
		
		exists = 0;
		for (plr= pl->poses.first; plr; plr= plr->next) {
			if (strcmp(name, tempname)==0) {
				exists = 1;
				break;
			}
		}
		if (exists == 0) {
			BLI_strncpy(name, tempname, 32);
			return;
		}
	}
}

/* gets the first available frame in poselib to store a pose on 
 *	- frames start from 1, and a pose should occur on every frame... 0 is error!
 */
int poselib_get_free_index (bPoseLib *pl)
{
	bPoseLibRef *plr;
	int low=0, high=0;
	
	/* sanity checks */
	if (ELEM(NULL, pl, pl->poses.first)) return 1;
	
	/* loop over poses finding various values (poses are not stored in chronological order) */
	for (plr= pl->poses.first; plr; plr= plr->next) {
		/* only increase low if value is 1 greater than low, to find "gaps" where
		 * poses were removed from the poselib
		 */
		if (plr->frame == (low + 1)) 
			low++;
		
		/* value replaces high if it is the highest value encountered yet */
		if (plr->frame > high) 
			high= plr->frame;
	}
	
	/* - if low is not equal to high, then low+1 is a gap 
	 * - if low is equal to high, then high+1 is the next index (add at end) 
	 */
	if (low < high) 
		return (low + 1);
	else 
		return (high + 1);
}

/* ************************************************************* */

/* Initialise a new poselib (whether it is needed or not) */
bPoseLib *poselib_init_new (Object *ob)
{
	bPose *pose= (ob) ? ob->pose : NULL;
	bAction *act;
	bPoseLib *pl;
	
	if (ELEM(NULL, ob, pose))
		return NULL;
	
	/* init pose's poselib action (unlink old one if there) */
	if (pose->poselib)
		pose->poselib->id.us--;
	pose->poselib= add_empty_action("PoseLib");
	act= pose->poselib;
	
	/* init actions's poselib data */
	if (act->poselib == NULL)
		act->poselib= MEM_callocN(sizeof(bPoseLib), "bPoseLib");
	pl= act->poselib;
	
	return pl;
}

/* Initialise a new poselib (checks if that needs to happen) */
bPoseLib *poselib_validate (Object *ob)
{
	bPose *pose= (ob) ? ob->pose : NULL;
	bAction *act= (pose) ? pose->poselib : NULL;
	bPoseLib *pl= (act) ? act->poselib : NULL;
	
	if (ELEM(NULL, ob, pose))
		return NULL;
		
	if (ELEM(NULL, act, pl))
		return poselib_init_new(ob);
	else
		return pl;
}


/* This tool automagically generates/validates poselib data so that it corresponds to the data 
 * in the action. This is for use in making existing actions usable as poselibs.
 */
void poselib_validate_act (bAction *act)
{
	ListBase keys = {NULL, NULL};
	ActKeyColumn *ak;
	bPoseLib *pl;
	bPoseLibRef *plr, *plrn;
	
	/* validate action and poselib */
	if (act == NULL)  {
		error("No Action to validate");
		return;
	}
	
	if (act->poselib == NULL)
		act->poselib= MEM_callocN(sizeof(bPoseLib), "bPoseLib");
	pl= act->poselib;
	
	/* determine which frames have keys */
	action_to_keylist(act, &keys, NULL);
	
	/* for each key, make sure there is a correspnding pose */
	for (ak= keys.first; ak; ak= ak->next) {
		/* check if any pose matches this */
		for (plr= pl->poses.first; plr; plr= plr->next) {
			if (IS_EQ(plr->frame, ak->cfra)) {
				plr->flag = 1;
				break;
			}
		}
		
		/* add new if none found */
		if (plr == NULL) {
			char name[32];
			
			/* add pose to poselib */
			plr= MEM_callocN(sizeof(bPoseLibRef), "bPoseLibRef");
			
			strcpy(name, "Pose");
			poselib_unique_pose_name(pl, name);
			BLI_strncpy(plr->name, name, sizeof(plr->name));
			
			plr->frame= (int)ak->cfra;
			plr->flag= 1;
			
			BLI_addtail(&pl->poses, plr);
		}
	}
	
	/* remove all untagged poses (unused), and remove all tags */
	for (plr= pl->poses.first; plr; plr= plrn) {
		plrn= plr->next;
		
		if (plr->flag == 0)
			BLI_freelinkN(&pl->poses, plr);
		else
			plr->flag = 0;
	}
	
	/* free temp memory */
	BLI_freelistN(&keys);
	
	BIF_undo_push("PoseLib Validate Action");
}

/* ************************************************************* */

/* This function adds an ipo-curve of the right type where it's needed */
static IpoCurve *poselib_verify_icu (Ipo *ipo, int adrcode)
{
	IpoCurve *icu;
	
	for (icu= ipo->curve.first; icu; icu= icu->next) {
		if (icu->adrcode==adrcode) break;
	}
	if (icu==NULL) {
		icu= MEM_callocN(sizeof(IpoCurve), "ipocurve");
		
		icu->flag |= IPO_VISIBLE|IPO_AUTO_HORIZ;
		if (ipo->curve.first==NULL) icu->flag |= IPO_ACTIVE;	/* first one added active */
		
		icu->blocktype= ID_PO;
		icu->adrcode= adrcode;
		
		set_icu_vars(icu);
		
		BLI_addtail(&ipo->curve, icu);
	}
	
	return icu;
}

/* This tool adds the current pose to the poselib 
 *	Note: Standard insertkey cannot be used for this due to its limitations
 */
void poselib_add_current_pose (Object *ob, int val)
{
	bArmature *arm= (ob) ? ob->data : NULL;
	bPose *pose= (ob) ? ob->pose : NULL;
	bPoseChannel *pchan;
	bPoseLib *pl;
	bPoseLibRef *plr;
	bAction *act;
	bActionChannel *achan;
	IpoCurve *icu;
	int frame;
	char name[32];
	
	/* sanity check */
	if (ELEM3(NULL, ob, arm, pose)) 
		return;
	
	/* mode - add new or replace existing */
	if (val == 0) {
		if (pose->poselib && pose->poselib->poselib->poses.first) {
			val= pupmenu("PoseLib Add Current Pose%t|Add New%x1|Replace Existing%x2");
			if (val <= 0) return;
		}
		else 
			val= 1;
	}
	
	if ((pose->poselib) && (val == 2)) {
		char *menustr;
		
		/* get poselib */
		act= pose->poselib;
		pl= act->poselib;
		
		/* get the pose to replace */
		menustr= poselib_build_poses_menu(pl, "Replace PoseLib Pose");
		val= pupmenu(menustr);
		if (menustr) MEM_freeN(menustr);
		
		if (val <= 0) return;
		plr= BLI_findlink(&pl->poses, val-1);
		if (plr == NULL) return;
		
		/* get the frame from the poselib */
		frame= plr->frame;
	}
	else {
		/* get name of pose */
		sprintf(name, "Pose");
		if (sbutton(name, 0, sizeof(name)-1, "Name: ") == 0)
			return;
			
		/* get/initialise poselib */
		pl= poselib_validate(ob);
		act= pose->poselib;	
		
		/* validate name and get frame */
		poselib_unique_pose_name(pl, name);
		frame= poselib_get_free_index(pl);
		
		/* add pose to poselib */
		plr= MEM_callocN(sizeof(bPoseLibRef), "bPoseLibRef");
		BLI_strncpy(plr->name, name, sizeof(plr->name));
		plr->frame= frame;
		BLI_addtail(&pl->poses, plr);
	}	
	
	/* loop through selected posechannels, keying their pose to the action */
	for (pchan= pose->chanbase.first; pchan; pchan= pchan->next) {
		/* check if available */
		if ((pchan->bone) && (arm->layer & pchan->bone->layer)) {
			if (pchan->bone->flag & (BONE_SELECTED|BONE_ACTIVE)) {
				/* make action-channel if needed */
				achan= verify_action_channel(act, pchan->name);
				
				/* make ipo if needed... */
				if (achan->ipo == NULL)
					achan->ipo= add_ipo(achan->name, ID_PO);
					
				/* add missing ipo-curves and insert keys */
				#define INSERT_KEY_ICU(adrcode, data) {\
						icu= poselib_verify_icu(achan->ipo, adrcode); \
						insert_vert_icu(icu, frame, data, 1); \
					}
					
				INSERT_KEY_ICU(AC_LOC_X, pchan->loc[0])
				INSERT_KEY_ICU(AC_LOC_Y, pchan->loc[1])
				INSERT_KEY_ICU(AC_LOC_Z, pchan->loc[2])
				INSERT_KEY_ICU(AC_SIZE_X, pchan->size[0])
				INSERT_KEY_ICU(AC_SIZE_Y, pchan->size[1])
				INSERT_KEY_ICU(AC_SIZE_Z, pchan->size[2])
				INSERT_KEY_ICU(AC_QUAT_W, pchan->quat[0])
				INSERT_KEY_ICU(AC_QUAT_X, pchan->quat[1])
				INSERT_KEY_ICU(AC_QUAT_Y, pchan->quat[2])
				INSERT_KEY_ICU(AC_QUAT_Z, pchan->quat[3])
			}
		}
	}
	
	/* store new 'active' pose number */
	pl->active_nr= BLI_countlist(&pl->poses);
	
	BIF_undo_push("PoseLib Add Pose");
	allqueue(REDRAWBUTSEDIT, 0);
}


/* This tool removes the pose that the user selected from the poselib (or the provided pose) */
void poselib_remove_pose (Object *ob, bPoseLibRef *plr)
{
	bPose *pose= (ob) ? ob->pose : NULL;
	bAction *act= (pose) ? pose->poselib : NULL;
	bActionChannel *achan;
	bPoseLib *pl= (act) ? act->poselib : NULL;
	char *menustr;
	int val;
	
	/* check if valid poselib */
	if (ELEM(NULL, ob, pose)) {
		error("PoseLib is only for Armatures in PoseMode");
		return;
	}
	if (ELEM(NULL, act, pl)) {
		error("Pose doesn't have a valid PoseLib");
		return;
	}
	
	/* get index (and pointer) of pose to remove */
	if (plr == NULL) {
		menustr= poselib_build_poses_menu(pl, "Remove PoseLib Pose");
		val= pupmenu(menustr);
		if (menustr) MEM_freeN(menustr);
		
		if (val <= 0) return;
		plr= BLI_findlink(&pl->poses, val-1);
		if (plr == NULL) return;
	}
	else {
		/* only continue if pose belongs to poselib */
		if (BLI_findindex(&pl->poses, plr) == -1) 
			return;
	}
	
	/* remove relevant keyframes */
	for (achan= act->chanbase.first; achan; achan= achan->next) {
		Ipo *ipo= achan->ipo;
		IpoCurve *icu;
		BezTriple *bezt;
		int i;
		
		for (icu= ipo->curve.first; icu; icu= icu->next) {
			for (i=0, bezt=icu->bezt; i < icu->totvert; i++, bezt++) {
				/* check if remove... */
				if (IS_EQ(bezt->vec[1][0], plr->frame)) {
					delete_icu_key(icu, i);
					break;
				}
			}	
		}
	}
	
	/* remove poselib from list */
	BLI_freelinkN(&pl->poses, plr);
	
	/* fix active pose number */
	pl->active_nr= 0;
	
	/* undo + redraw */
	BIF_undo_push("PoseLib Remove Pose");
	allqueue(REDRAWBUTSEDIT, 0);
}


/* This tool renames the pose that the user selected from the poselib */
void poselib_rename_pose (Object *ob)
{
	bPose *pose= (ob) ? ob->pose : NULL;
	bAction *act= (pose) ? pose->poselib : NULL;
	bPoseLib *pl= (act) ? act->poselib : NULL;
	bPoseLibRef *plr;
	char *menustr, name[32];
	int val;
	
	/* check if valid poselib */
	if (ELEM(NULL, ob, pose)) {
		error("PoseLib is only for Armatures in PoseMode");
		return;
	}
	if (ELEM(NULL, act, pl)) {
		error("Pose doesn't have a valid PoseLib");
		return;
	}
	
	/* get index of pose to remove */
	menustr= poselib_build_poses_menu(pl, "Rename PoseLib Pose");
	val= pupmenu(menustr);
	if (menustr) MEM_freeN(menustr);
	
	if (val <= 0) return;
	plr= BLI_findlink(&pl->poses, val-1);
	if (plr == NULL) return;
	
	/* get name of pose */
	sprintf(name, plr->name);
	if (sbutton(name, 0, sizeof(name)-1, "Name: ") == 0)
		return;
		
	/* validate name */
	poselib_unique_pose_name(pl, name); // hmm what happens with the old pose's name...
	
	/* copy name */
	BLI_strncpy(plr->name, name, sizeof(plr->name));
	
	/* undo and update */
	BIF_undo_push("PoseLib Rename Pose");
	allqueue(REDRAWBUTSEDIT, 0);
}


/* ************************************************************* */

/* simple struct for storing backup info */
typedef struct tPoseLib_Backup {
	struct tPoseLib_Backup *next, *prev;
	
	bPoseChannel *pchan;
	
	float oldloc[3];
	float oldsize[3];
	float oldquat[4];
	
	int oldflag;
} tPoseLib_Backup;

/* Makes a copy of the current pose for restoration purposes - doesn't do constraints currently */
static void poselib_backup_posecopy (ListBase *backups, bPose *pose)
{
	bAction *poselib= pose->poselib;
	bActionChannel *achan;
	bPoseChannel *pchan;
	
	/* for each posechannel that has an actionchannel in */
	for (achan= poselib->chanbase.first; achan; achan= achan->next) {
		/* try to find posechannel */
		pchan= get_pose_channel(pose, achan->name);
		
		/* backup data if available */
		if (pchan) {
			tPoseLib_Backup *plb;
			
			plb= MEM_callocN(sizeof(tPoseLib_Backup), "tPoseLib_Backup");
			
			VECCOPY(plb->oldloc, pchan->loc);
			VECCOPY(plb->oldsize, pchan->size);
			QUATCOPY(plb->oldquat, pchan->quat);
			
			plb->pchan= pchan;
			
			BLI_addtail(backups, plb);
		}
	}
}

/* Restores original pose - doesn't do constraints currently */
static void poselib_backup_restore (ListBase *backups)
{
	tPoseLib_Backup *plb;
	
	for (plb= backups->first; plb; plb= plb->next) {
		VECCOPY(plb->pchan->loc, plb->oldloc);
		VECCOPY(plb->pchan->size, plb->oldsize);
		VECCOPY(plb->pchan->quat, plb->oldquat);
		
		plb->pchan->flag = plb->oldflag;
	}
}

/* ---------------------------- */

/* Applies the appropriate stored pose from the pose-library to the current pose
 *	- assumes that a valid object, with a poselib has been supplied
 *	- gets the string to print in the header
 * 	- this code is based on the code for extract_pose_from_action in blenkernel/action.c
 */
static void poselib_apply_pose (Object *ob, bPoseLibRef *plr, char headerstr[])
{
	bPose *pose= ob->pose;
	bPoseChannel *pchan;
	bAction *act= pose->poselib;
	bActionChannel *achan;
	IpoCurve *icu;
	int frame= plr->frame;
	
	/* start applying - only those channels which have a key at this point in time! */
	for (achan= act->chanbase.first; achan; achan= achan->next) {
		short found= 0;
		
		/* apply this achan? */
		if (achan->ipo) {
			/* find a keyframe at this frame - users may not have defined the pose on every channel, so this is necessary */
			for (icu= achan->ipo->curve.first; icu; icu= icu->next) {
				BezTriple *bezt;
				int i;
				
				for (i=0, bezt=icu->bezt; i < icu->totvert; i++, bezt++) {
					if (IN_RANGE(bezt->vec[1][0], (frame-0.5f), (frame+0.5f))) {
						found= 1;
						break;
					}
				}
				
				if (found) break;
			}
			
			/* apply pose */
			if (found) {
				pchan= get_pose_channel(pose, achan->name);
				
				if (pchan) {
					/* Evaluates and sets the internal ipo values	*/
					calc_ipo(achan->ipo, frame);
					/* This call also sets the pchan flags */
					execute_action_ipo(achan, pchan);
				}
			}
		}
		
		/* tag achan as having been used or not... */
		if (found)
			achan->flag |= ACHAN_SELECTED;
		else
			achan->flag &= ~ACHAN_SELECTED;
	}
}

/* Auto-keys/tags bones affected by the pose used from the poselib */
static void poselib_keytag_pose (Object *ob)
{
	bPose *pose= ob->pose;
	bPoseChannel *pchan;
	bAction *act= pose->poselib;
	bActionChannel *achan;
	
	/* start tagging/keying */
	for (achan= act->chanbase.first; achan; achan= achan->next) {
		/* only for selected action channels */
		if (achan->flag & ACHAN_SELECTED) {
			pchan= get_pose_channel(ob->pose, achan->name);
			
			if (pchan) {
				if (G.flags & G_RECORDKEYS) {
					ID *id= &ob->id;
					
					/* Set keys on pose */
					if (pchan->flag & POSE_ROT) {
						insertkey(id, ID_PO, pchan->name, NULL, AC_QUAT_X, 0);
						insertkey(id, ID_PO, pchan->name, NULL, AC_QUAT_Y, 0);
						insertkey(id, ID_PO, pchan->name, NULL, AC_QUAT_Z, 0);
						insertkey(id, ID_PO, pchan->name, NULL, AC_QUAT_W, 0);
					}
					if (pchan->flag & POSE_SIZE) {
						insertkey(id, ID_PO, pchan->name, NULL, AC_SIZE_X, 0);
						insertkey(id, ID_PO, pchan->name, NULL, AC_SIZE_Y, 0);
						insertkey(id, ID_PO, pchan->name, NULL, AC_SIZE_Z, 0);
					}
					if (pchan->flag & POSE_LOC) {
						insertkey(id, ID_PO, pchan->name, NULL, AC_LOC_X, 0);
						insertkey(id, ID_PO, pchan->name, NULL, AC_LOC_Y, 0);
						insertkey(id, ID_PO, pchan->name, NULL, AC_LOC_Z, 0);
					}
					
					/* clear any unkeyed tags */
					if (pchan->bone)
						pchan->bone->flag &= ~BONE_UNKEYED;
				}
				else {
					/* add unkeyed tags */
					if (pchan->bone)
						pchan->bone->flag |= BONE_UNKEYED;
				}
			}
		}
	}
}

/* ---------------------------- */

/* defines for poselib_preview_poses --> ret_val values */
enum {
	PL_PREVIEW_RUNNING = 0,
	PL_PREVIEW_CONFIRM,
	PL_PREVIEW_CANCEL,
	PL_PREVIEW_RUNONCE 
};

/* defines for poselib_preview_poses --> redraw values */
enum {
	PL_PREVIEW_NOREDRAW = 0,
	PL_PREVIEW_REDRAWALL,
	PL_PREVIEW_REDRAWHEADER,
};

/* This tool allows users to preview the pose from the pose-lib using the mouse-scrollwheel/pageupdown
 * It is also used to apply the active poselib pose only
 */
void poselib_preview_poses (Object *ob, short apply_active)
{
	ListBase backups = {NULL, NULL};
	
	bPose *pose= (ob) ? (ob->pose) : NULL;
	bArmature *arm= (ob) ? (ob->data) : NULL;
	bAction *act= (pose) ? (pose->poselib) : NULL; 
	bPoseLib *pl= (act) ? (act->poselib) : NULL;
	bPoseLibRef *plr= (pl == NULL) ? NULL : (pl->active_nr) ? BLI_findlink(&pl->poses, pl->active_nr-1) : pl->poses.first;
	Base *base;
	
	short ret_val= (apply_active) ? PL_PREVIEW_RUNONCE : PL_PREVIEW_RUNNING;
	short val=0, redraw=1, firsttime=1;
	unsigned short event;
	char headerstr[200];
	
	/* check if valid poselib */
	if (ELEM(NULL, ob, pose)) {
		error("PoseLib is only for Armatures in PoseMode");
		return;
	}
	if (ELEM(NULL, act, pl)) {
		error("Pose doesn't have a valid PoseLib");
		return;
	}
	if (plr == NULL) {
		error("PoseLib has no poses to preview");
		return;
	}
	
	/* make backup of current pose for restoring pose */
	poselib_backup_posecopy(&backups, pose);
	
	/* set depsgraph flags */
		/* make sure the lock is set OK, unlock can be accidentally saved? */
	pose->flag |= POSE_LOCKED;
	pose->flag &= ~POSE_DO_UNLOCK;

	
	/* start preview loop */
	while (ELEM(ret_val, PL_PREVIEW_RUNNING, PL_PREVIEW_RUNONCE)) {
		/* preview a pose */
		if (redraw) {
			/* only recalc pose (and its dependencies) if pose has changed */
			if (redraw == PL_PREVIEW_REDRAWALL) {
				/* don't clear pose if firsttime */
				if (firsttime == 0)
					poselib_backup_restore(&backups);
				else
					firsttime = 0;
					
				/* pose should be the right one to draw */
				poselib_apply_pose(ob, plr, headerstr);
				
				/* old optimize trick... this enforces to bypass the depgraph 
				 *	- note: code copied from transform_generics.c -> recalcData()
				 */
				if ((arm->flag & ARM_DELAYDEFORM)==0) {
					DAG_object_flush_update(G.scene, ob, OB_RECALC_DATA);  /* sets recalc flags */
					
					/* bah, softbody exception... recalcdata doesnt reset */
					for (base= FIRSTBASE; base; base= base->next) {
						if (base->object->recalc & OB_RECALC_DATA)
							if (modifiers_isSoftbodyEnabled(base->object)) {
								base->object->softflag |= OB_SB_REDO;
						}
					}
				}
				else
					where_is_pose(ob);
			}
			
			/* do header print - if interactively previewing */
			if (ret_val == PL_PREVIEW_RUNNING) {
				sprintf(headerstr, "PoseLib Previewing Pose: \"%s\"  | Use ScrollWheel or PageUp/Down to change", plr->name);
				headerprint(headerstr);
			}
			
			/* force drawing of view + clear redraw flag */
			force_draw(0);
			redraw= PL_PREVIEW_NOREDRAW;
		}
		
		/* stop now if only running once */
		if (ret_val == PL_PREVIEW_RUNONCE) {
			ret_val = PL_PREVIEW_CONFIRM;
			break;
		}
		
		/* essential for idling subloop */
		if (qtest() == 0) 
			PIL_sleep_ms(2);
		
		/* emptying queue and reading events */
		while ( qtest() ) {
			event= extern_qread(&val);
			
			/* event processing */
			if (val) {
				switch (event) {
					/* exit - cancel */
					case ESCKEY:
					case RIGHTMOUSE:
						ret_val= PL_PREVIEW_CANCEL;
						break;
						
					/* exit - confirm */
					case LEFTMOUSE:
					case RETKEY:
					case SPACEKEY:
						ret_val= PL_PREVIEW_CONFIRM;
						break;
					
					/* change to previous pose - go back to end of list if no previous (cyclic) */
					case PAGEUPKEY:
					case WHEELUPMOUSE:
						plr= (plr->prev) ? plr->prev : pl->poses.last;
						redraw= PL_PREVIEW_REDRAWALL;
						break;
						
					/* change to next pose - go back to start of list if no next (cyclic) */
					case PAGEDOWNKEY:
					case WHEELDOWNMOUSE:
						plr= (plr->next) ? plr->next : pl->poses.first;
						redraw= PL_PREVIEW_REDRAWALL;
						break;
						
					/* view manipulation */
					case MIDDLEMOUSE:
						// there's a little bug here that causes the normal header to get drawn while view is manipulated 
						handle_view_middlemouse();
						redraw= PL_PREVIEW_REDRAWHEADER;
						break;
						
					case PAD0: case PAD1: case PAD2: case PAD3: case PAD4:
					case PAD5: case PAD6: case PAD7: case PAD8: case PAD9:
					case PADPLUSKEY:
					case PADMINUS:
					case PADENTER:
						persptoetsen(event);
						redraw= PL_PREVIEW_REDRAWHEADER;
						break;
				}
			}
		}
	}
	
	/* this signal does one recalc on pose, then unlocks, so ESC or edit will work */
	pose->flag |= POSE_DO_UNLOCK;
	
	/* clear pose if cancelled */
	if (ret_val == PL_PREVIEW_CANCEL) {
		poselib_backup_restore(&backups);
		
		/* old optimize trick... this enforces to bypass the depgraph 
		 *	- note: code copied from transform_generics.c -> recalcData()
		 */
		if ((arm->flag & ARM_DELAYDEFORM)==0) {
			DAG_object_flush_update(G.scene, ob, OB_RECALC_DATA);  /* sets recalc flags */
			
			/* bah, softbody exception... recalcdata doesnt reset */
			for (base= FIRSTBASE; base; base= base->next) {
				if (base->object->recalc & OB_RECALC_DATA)
					if (modifiers_isSoftbodyEnabled(base->object)) {
						base->object->softflag |= OB_SB_REDO;
				}
			}
		}
		else
			where_is_pose(ob);
		
		allqueue(REDRAWVIEW3D, 0);
	}
	else if (ret_val == PL_PREVIEW_CONFIRM) {
		/* tag poses as appropriate */
		poselib_keytag_pose(ob);
		
		/* change active pose setting */
		pl->active_nr= BLI_findindex(&pl->poses, plr) + 1;
		
		/* Update event for pose and deformation children */
		DAG_object_flush_update(G.scene, ob, OB_RECALC_DATA);
		
		/* updates */
		if (G.flags & G_RECORDKEYS) {
			remake_action_ipos(ob->action);
			
			allqueue(REDRAWIPO, 0);
			allqueue(REDRAWVIEW3D, 0);
			allqueue(REDRAWBUTSEDIT, 0);
			allqueue(REDRAWACTION, 0);		
			allqueue(REDRAWNLA, 0);
		}
		else {
			/* need to trick depgraph, action is not allowed to execute on pose */
			where_is_pose(ob);
			ob->recalc= 0;
			
			allqueue(REDRAWVIEW3D, 0);
			allqueue(REDRAWBUTSEDIT, 0);
		}
	}
	/* free memory used for backups */
	BLI_freelistN(&backups);
	
	BIF_undo_push("PoseLib Apply Pose");
}
