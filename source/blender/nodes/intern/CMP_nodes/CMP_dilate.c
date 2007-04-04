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
 * The Original Code is Copyright (C) 2006 Blender Foundation.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): none yet.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include "../CMP_util.h"


/* **************** Dilate/Erode ******************** */

static bNodeSocketType cmp_node_dilateerode_in[]= {
	{	SOCK_VALUE, 1, "Mask",		0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f},
	{	-1, 0, ""	}
};
static bNodeSocketType cmp_node_dilateerode_out[]= {
	{	SOCK_VALUE, 0, "Mask",		0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f},
	{	-1, 0, ""	}
};

static void morpho_dilate(CompBuf *cbuf)
{
	int x, y;
	float *p, *rectf = cbuf->rect;
	
	for (y=0; y < cbuf->y; y++) {
		for (x=0; x < cbuf->x-1; x++) {
			p = rectf + cbuf->x*y + x;
			*p = MAX2(*p, *(p + 1));
		}
	}

	for (y=0; y < cbuf->y; y++) {
		for (x=cbuf->x-1; x >= 1; x--) {
			p = rectf + cbuf->x*y + x;
			*p = MAX2(*p, *(p - 1));
		}
	}

	for (x=0; x < cbuf->x; x++) {
		for (y=0; y < cbuf->y-1; y++) {
			p = rectf + cbuf->x*y + x;
			*p = MAX2(*p, *(p + cbuf->x));
		}
	}

	for (x=0; x < cbuf->x; x++) {
		for (y=cbuf->y-1; y >= 1; y--) {
			p = rectf + cbuf->x*y + x;
			*p = MAX2(*p, *(p - cbuf->x));
		}
	}
}

static void morpho_erode(CompBuf *cbuf)
{
	int x, y;
	float *p, *rectf = cbuf->rect;
	
	for (y=0; y < cbuf->y; y++) {
		for (x=0; x < cbuf->x-1; x++) {
			p = rectf + cbuf->x*y + x;
			*p = MIN2(*p, *(p + 1));
		}
	}

	for (y=0; y < cbuf->y; y++) {
		for (x=cbuf->x-1; x >= 1; x--) {
			p = rectf + cbuf->x*y + x;
			*p = MIN2(*p, *(p - 1));
		}
	}

	for (x=0; x < cbuf->x; x++) {
		for (y=0; y < cbuf->y-1; y++) {
			p = rectf + cbuf->x*y + x;
			*p = MIN2(*p, *(p + cbuf->x));
		}
	}

	for (x=0; x < cbuf->x; x++) {
		for (y=cbuf->y-1; y >= 1; y--) {
			p = rectf + cbuf->x*y + x;
			*p = MIN2(*p, *(p - cbuf->x));
		}
	}
	
}

static void node_composit_exec_dilateerode(void *data, bNode *node, bNodeStack **in, bNodeStack **out)
{
	/* stack order in: mask */
	/* stack order out: mask */
	if(out[0]->hasoutput==0) 
		return;
	
	/* input no image? then only color operation */
	if(in[0]->data==NULL) {
		out[0]->vec[0] = out[0]->vec[1] = out[0]->vec[2] = 0.0f;
		out[0]->vec[3] = 0.0f;
	}
	else {
		/* make output size of input image */
		CompBuf *cbuf= typecheck_compbuf(in[0]->data, CB_VAL);
		CompBuf *stackbuf= dupalloc_compbuf(cbuf);
		short i;
		
		if (node->custom2 > 0) { // positive, dilate
			for (i = 0; i < node->custom2; i++)
				morpho_dilate(stackbuf);
		} else if (node->custom2 < 0) { // negative, erode
			for (i = 0; i > node->custom2; i--)
				morpho_erode(stackbuf);
		}
		
		if(cbuf!=in[0]->data)
			free_compbuf(cbuf);
		
		out[0]->data= stackbuf;
	}
}

bNodeType cmp_node_dilateerode= {
	/* *next,*prev */	NULL, NULL,
	/* type code   */	CMP_NODE_DILATEERODE,
	/* name        */	"Dilate/Erode",
	/* width+range */	130, 100, 320,
	/* class+opts  */	NODE_CLASS_OP_FILTER, NODE_OPTIONS,
	/* input sock  */	cmp_node_dilateerode_in,
	/* output sock */	cmp_node_dilateerode_out,
	/* storage     */	"",
	/* execfunc    */	node_composit_exec_dilateerode,
	/* butfunc     */	NULL,
	/* initfunc    */	NULL,
	/* freestoragefunc    */	NULL,
	/* copystoragefunc    */	NULL,
	/* id          */	NULL
};


