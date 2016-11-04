/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Paul Mathieu. */

#include "play_motion/controller_updater.h"

#include <boost/foreach.hpp>
#include <controller_manager_msgs/ListControllers.h>

#define foreach BOOST_FOREACH

namespace play_motion
{

  static ros::ServiceClient initCmClient(ros::NodeHandle nh)
  {
    return nh.serviceClient<controller_manager_msgs::ListControllers>
        ("controller_manager/list_controllers", true);
  }

  ControllerUpdater::ControllerUpdater(ros::NodeHandle nh) : nh_(nh)
  {
    cm_client_ = initCmClient(nh_);
    main_thread_ = boost::thread(&ControllerUpdater::mainLoop, this);
  }

  ControllerUpdater::~ControllerUpdater()
  {}

  static bool isJointTrajectoryController(const std::string& name)
  {
    std::string tofind = "JointTrajectoryController"; //XXX: magic value
    size_t pos = name.find(tofind);
    if (pos == std::string::npos)
      return false;
    if (pos != name.length() - tofind.length())
      return false;

    return true;
  }

  void ControllerUpdater::mainLoop()
  {
    ros::Rate r(1.0); //XXX: magic value
    for (; ros::ok(); r.sleep())
    {
      if (!update_cb_)
        continue;

      /* controller_manager_msgs::ListControllers srv;

      if (!cm_client_.isValid())
        cm_client_ = initCmClient(nh_);
      if(!cm_client_.call(srv))
      {
        ROS_WARN_THROTTLE(5.0, "Could not get list of controllers from controller manager.");
        continue;
      }

      ControllerStates states;
      ControllerJoints joints;
      typedef controller_manager_msgs::ControllerState cstate_t;
      foreach (const cstate_t& cs, srv.response.controller)
      {
        if (!isJointTrajectoryController(cs.type))
          continue;
        states[cs.name] = (cs.state == "running" ? RUNNING : STOPPED);
        joints[cs.name] = cs.resources;
      }

      if (states == last_cstates_)
        continue; */

      ControllerStates states;
      ControllerJoints joints;
      states["torso_controller"] = RUNNING;
      JointNames joints1;
      joints1.push_back("torso_lift_joint");
      joints["torso_controller"] = joints1;
      states["head_traj_controller"] = RUNNING;
      JointNames joints2;
      joints2.push_back("head_pan_joint");
      joints2.push_back("head_tilt_joint");
      joints["head_traj_controller"] = joints2;
      JointNames joints3;
      joints3.push_back("r_shoulder_pan_joint");
      joints3.push_back("r_shoulder_lift_joint");
      joints3.push_back("r_upper_arm_roll_joint");
      joints3.push_back("r_elbow_flex_joint");
      joints3.push_back("r_forearm_roll_joint");
      joints3.push_back("r_wrist_flex_joint");
      joints3.push_back("r_wrist_roll_joint");
      states["r_arm_controller"] = RUNNING;
      joints["r_arm_controller"] = joints3;
      JointNames joints4;
      joints4.push_back("l_shoulder_pan_joint");
      joints4.push_back("l_shoulder_lift_joint");
      joints4.push_back("l_upper_arm_roll_joint");
      joints4.push_back("l_elbow_flex_joint");
      joints4.push_back("l_forearm_roll_joint");
      joints4.push_back("l_wrist_flex_joint");
      joints4.push_back("l_wrist_roll_joint");
      states["l_arm_controller"] = RUNNING;
      joints["l_arm_controller"] = joints4;


      ROS_INFO("The set of running joint trajectory controllers has changed, updating it.");
      update_timer_ = nh_.createTimer(ros::Duration(0), boost::bind(update_cb_, states, joints), true);
      last_cstates_ = states;
      /**/
    }
  }

}
