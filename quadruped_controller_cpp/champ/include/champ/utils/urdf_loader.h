/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef URDF_LOADER_H
#define URDF_LOADER_H

#include <urdf/model.h>
// #include <utils/xmlrpc_helpers.h>

#include <quadruped_base/quadruped_base.h>
#include <quadruped_base/quadruped_leg.h>
#include <quadruped_base/quadruped_joint.h>

namespace champ
{
    namespace URDF
    {
        void getPose(urdf::Pose *pose, std::string ref_link, std::string end_link, urdf::Model &model)
        {
            urdf::LinkConstSharedPtr ref_link_ptr = model.getLink(ref_link);

            std::string current_parent_name = end_link;
            urdf::LinkConstSharedPtr prev_link = model.getLink(current_parent_name);

            while(ref_link_ptr->name != current_parent_name)
            {   
                urdf::LinkConstSharedPtr current_link = model.getLink(current_parent_name);
                urdf::Pose current_pose = current_link->parent_joint->parent_to_joint_origin_transform;
              
                current_parent_name = current_link->getParent()->name;
                prev_link = model.getLink(current_parent_name);
                pose->position.x += current_pose.position.x;
                pose->position.y += current_pose.position.y;
                pose->position.z += current_pose.position.z;
            }
        }

        void fillLeg(champ::QuadrupedLeg *leg, urdf::Model &model, const std::vector<std::string>& links_param)
        {
            // 링크 설정
            for (int i = 3; i > -1; i--) {
                std::string ref_link, end_link;

                if (i > 0) {
                    ref_link = links_param[i - 1];
                } else {
                    ref_link = model.getRoot()->name;
                }

                end_link = links_param[i];

                urdf::Pose pose;
                getPose(&pose, ref_link, end_link, model);

                double x = pose.position.x;
                double y = pose.position.y;
                double z = pose.position.z;
                leg->joint_chain[i]->setTranslation(x, y, z);
            }
        }

        void loadFromString(champ::QuadrupedBase &base, std::vector<std::vector<std::string>> links_map, const std::string& urdf_string)
        {
            urdf::Model model;
            if (!model.initString(urdf_string)){
                std::cout << "Failed to parse urdf string" << std::endl;
            }
            else{
                for (int i = 0; i < 4; i++) {
                    fillLeg(base.legs[i], model, links_map[i]);
                }
            }
        }
    }
}

#endif