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

#ifndef __UNIT_TEST_H__
#define __UNIT_TEST_H__

/* Run all unit tests
 *
 * In future, could do something more advanced here like filtering
 * based on test name. Could also look at using an existing
 * unit-testing library. */
void run_unit_tests(void);

/* To get code coverage with CMake, gcc, and gcov:
 *
 * - Set the coverage compile flag in CMakeCache.txt:
 *   CMAKE_C_FLAGS:STRING=--coverage
 *   CMAKE_EXE_LINKER_FLAGS:STRING=--coverage
 *
 *   Essentially this is enabling "-fprofile-arcs" and
 *   "-ftest-coverage" for compiling, and linking with libgcov.
 *
 * - Compile, then run "bin/blender --unit-test"
 *
 * - Run "gcov <source-file>.c -o
 *             <cmake-build-dir>/<funky-cmake-path>/<source-file>.c.gcno"
 *
 *   Example:
 *   "gcov source/blender/bmesh/operators/bmo_hull.c -o
 *    cmake-debug/source/blender/bmesh/CMakeFiles/
 *    bf_bmesh.dir/operators/bmo_hull.c.gcno"
 *
 * - I'm not sure why a full path has to be specified to the gcno
 *   file. If only the directory is passed in, gcov fails to find the
 *   gcno file because it forgets the ".c" portion of the filename.
 *
 * - Gcov will print some useful statistics about line coverage and
 *   put an annotated version of the source file in
 *   ./<source-file>.c.gcov
 */

#endif
