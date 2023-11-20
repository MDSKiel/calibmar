# Copyright (c) 2023, ETH Zurich and UNC Chapel Hill.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
#       its contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


set(FOLDER_NAME "tools")

# COLMAP_ADD_EXECUTABLE(example example.cc)
COLMAP_ADD_EXECUTABLE(
    NAME colmap_testbed
    SRCS 
        testbed.cc
    LINK_LIBS
        colmap_scene
        colmap_estimators
)

COLMAP_ADD_EXECUTABLE(
    NAME colmap_sel_imgs_by_pos
    SRCS 
        sel_imgs_by_pos.cc
    LINK_LIBS
        colmap_scene
        colmap_controllers
        colmap_util_cuda
)

COLMAP_ADD_EXECUTABLE(
    NAME colmap_eval_refrac_abs_pose
    SRCS 
        eval_refrac_abs_pose.cc
    LINK_LIBS
        colmap_scene
        colmap_estimators
)

COLMAP_ADD_EXECUTABLE(
    NAME colmap_eval_refrac_rel_pose
    SRCS 
        eval_refrac_rel_pose.cc
    LINK_LIBS
        colmap_scene
        colmap_estimators
)

set_target_properties(colmap_testbed PROPERTIES OUTPUT_NAME testbed) 
set_target_properties(colmap_sel_imgs_by_pos PROPERTIES OUTPUT_NAME sel_imgs_by_pos)
set_target_properties(colmap_eval_refrac_abs_pose PROPERTIES OUTPUT_NAME eval_refrac_abs_pose)
set_target_properties(colmap_eval_refrac_rel_pose PROPERTIES OUTPUT_NAME eval_refrac_rel_pose) 