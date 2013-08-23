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

#include "play_motion/play_motion.h"

#include <cassert>
#include <iostream>
#include <sstream>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

#include "play_motion/move_joint_group.h"

#define foreach BOOST_FOREACH

namespace play_motion
{
  PlayMotion::PlayMotion(ros::NodeHandle& nh) :
    nh_(nh), joint_states_sub_(nh_.subscribe("joint_states", 10, &PlayMotion::jointStateCb, this))
  {}

  void PlayMotion::setControllerList(const ControllerList& controller_list)
  {
    move_joint_groups_.clear();
    foreach (const std::string& controller_name, controller_list)
      move_joint_groups_.push_back(MoveJointGroupPtr(new MoveJointGroup(controller_name)));
  }

  void PlayMotion::controllerCb(bool success)
  {
    ROS_DEBUG("return from joint group, %d active controllers", current_active_controllers_-1);
    current_success_ &= success;
    if (--current_active_controllers_ < 1)
      client_cb_(current_success_);
  };

  void PlayMotion::jointStateCb(const sensor_msgs::JointStatePtr& msg)
  {
    joint_states_.clear();
    for (uint32_t i=0; i < msg->name.size(); ++i)
      joint_states_[msg->name[i]] = msg->position[i];
  }

  bool PlayMotion::getGroupTraj(MoveJointGroupPtr move_joint_group,
      const std::vector<std::string>& motion_joints,
      const Trajectory& motion_points, Trajectory& traj_group)
  {
    std::vector<std::string>   group_joint_names = move_joint_group->getJointNames();
    std::vector<double>        joint_states;
    std::map<std::string, int> joint_index;

    traj_group.clear();
    traj_group.reserve(motion_points.size());

    foreach (const std::string& jn, group_joint_names)
    {
      // store the index of this joint in the given motion
      int index = std::find(motion_joints.begin(), motion_joints.end(), jn) - motion_joints.begin();
      joint_index[jn] = index;

      // retrieve joint state,  we should have it from the joint_states subscriber
      if (joint_states_.find(jn) == joint_states_.end())
      {
        ROS_ERROR_STREAM("Could not get current position of joint \'" << jn << "\'.");
        return false;
      }
      joint_states.push_back(joint_states_[jn]);
    }

    foreach (const TrajPoint& p, motion_points)
    {
      bool has_velocities = !p.velocities.empty();
      TrajPoint point;
      point.positions.resize(group_joint_names.size());
      if (has_velocities)
        point.velocities.resize(group_joint_names.size(), 0);
      point.time_from_start = p.time_from_start;

      for (std::size_t i = 0; i < group_joint_names.size(); ++i)
      {
        // first assignment, overriden by given motion if joint specified
        point.positions[i] = joint_states[i];

        size_t index = joint_index[group_joint_names[i]];
        if (index < motion_joints.size())
        {
          point.positions[i] = p.positions[index];
          if (has_velocities)
            point.velocities[i] = p.velocities[index];
        }
      }
      traj_group.push_back(point);
    }
    return true;
  }

#define RETHROW(x) \
  do { if( !(x) ) return false; } while(0)

  namespace xr
  {
    bool checkParamType(const XmlRpc::XmlRpcValue& val, const XmlRpc::XmlRpcValue::Type& expected_type,
        const std::string& param_name = "")
    {
      if (val.getType() != expected_type)
      {
        if (param_name.empty())
          ROS_ERROR("wrong parameter type");
        else
          ROS_ERROR("wrong parameter type for param '%s'", param_name.c_str());
        return false;
      }
      return true;
    }

    template <class T>
      bool fetchParam(ros::NodeHandle nh, const std::string& param_name,
          const XmlRpc::XmlRpcValue::Type& expected_type, T& output)
      {
        XmlRpc::XmlRpcValue val;
        if (nh.getParam(param_name, val))
        {
          ROS_ERROR("could not load parameter '%s'. (namespace: %s)",
                  param_name.c_str(), nh.getNamespace().c_str());
          return false;
        }
        RETHROW(checkParamType(val, expected_type, param_name));
        output = static_cast<T>(val);
        return true;
      }

    bool checkSubItem(const XmlRpc::XmlRpcValue& col, int index)
    {
      RETHROW(checkParamType(col, XmlRpc::XmlRpcValue::TypeArray));
      if(index < col.size())
        return true;
      ROS_ERROR("index '%d' is over array capacity", index);
      return false;
    }
    bool checkSubItem(const XmlRpc::XmlRpcValue& col, const std::string& member)
    {
      RETHROW(checkParamType(col, XmlRpc::XmlRpcValue::TypeStruct));
      if (col.hasMember(member))
        return true;
      ROS_ERROR("could not find member '%s'", member.c_str());
      return false;
    }

    template <class T, class U>
      bool getSubItem(XmlRpc::XmlRpcValue& col, U member,
          const XmlRpc::XmlRpcValue::Type& expected_type, T& output)
      {
        RETHROW(checkSubItem(col, member));
        RETHROW(checkParamType(col[member], expected_type));
        output = static_cast<T>(col[member]);
        return true;
      }

#define getArrayItem getSubItem
#define getStructMember getSubItem
  }

  bool PlayMotion::getMotionJoints(const std::string& motion_name, std::vector<std::string>& motion_joints)
  {
    using namespace XmlRpc;
    ros::NodeHandle nh("~");
    XmlRpcValue joint_names;

    RETHROW(xr::fetchParam(nh, "motions/" + motion_name + "/joints", XmlRpcValue::TypeArray, joint_names));

    motion_joints.clear();
    motion_joints.resize(joint_names.size());
    for (int i = 0; i < joint_names.size(); ++i)
      RETHROW(xr::getArrayItem(joint_names, i, XmlRpcValue::TypeString, motion_joints[i]));

    return true;
  }

  bool PlayMotion::getMotionPoints(const std::string& motion_name, Trajectory& motion_points)
  {
    using namespace XmlRpc;
    ros::NodeHandle nh("~");
    XmlRpcValue traj_points;

    RETHROW(xr::fetchParam(nh, "motions/" + motion_name + "/points", XmlRpcValue::TypeArray, traj_points));

    motion_points.clear();
    motion_points.reserve(traj_points.size());
    for (int i = 0; i < traj_points.size(); ++i)
    {
      XmlRpcValue &name_value = traj_points[i];
      TrajPoint point;
      RETHROW(xr::getStructMember(name_value, "time_from_start",
            XmlRpcValue::TypeDouble, point.time_from_start));

      XmlRpcValue positions;
      RETHROW(xr::getStructMember(name_value, "positions", XmlRpcValue::TypeArray, positions));
      point.positions.resize(positions.size());
      for (int j = 0; j < positions.size(); ++j)
        RETHROW(xr::getArrayItem(positions, j, XmlRpcValue::TypeDouble, point.positions[i]));
      if (name_value.hasMember("velocities"))
      {
        XmlRpcValue velocities;
        RETHROW(xr::getStructMember(name_value, "velocities", XmlRpcValue::TypeArray, velocities));
        point.velocities.resize(velocities.size());
        for (int j = 0; j < velocities.size(); ++j)
          RETHROW(xr::getArrayItem(velocities, j, XmlRpcValue::TypeDouble, point.velocities[i]));
      }
      motion_points.push_back(point);
    }
    return true;
  }

  bool PlayMotion::checkControllers(const std::vector<std::string>& motion_joints)
  {
    foreach (const std::string& jn, motion_joints)
    {
      foreach (MoveJointGroupPtr ctrlr, move_joint_groups_)
        if (ctrlr->isControllingJoint(jn))
          break;

      ROS_ERROR_STREAM("no controller was found for joint '" << jn << "'");
      return false;
    }
    return true;
  }

  template <class T>
  bool hasNonNullIntersection(const std::vector<T>& v1, const std::vector<T>& v2)
  {
    foreach (const T& e1, v1)
      foreach (const T& e2, v2)
        if (e1 == e2)
          return true;
    return false;
  }

  bool PlayMotion::run(const std::string& motion_name, const ros::Duration& duration)
  {
    std::vector<Trajectory>  joint_group_traj; // Desired motion split into joint groups
    std::vector<std::string> motion_joints;
    Trajectory               motion_points;

    RETHROW(getMotionJoints(motion_name, motion_joints));
    RETHROW(checkControllers(motion_joints));
    RETHROW(getMotionPoints(motion_name, motion_points));

    // Seed target pose with current joint state
    joint_group_traj.reserve(move_joint_groups_.size());
    foreach (MoveJointGroupPtr move_joint_group, move_joint_groups_)
    {
      Trajectory traj;
      if (hasNonNullIntersection(motion_joints, move_joint_group->getJointNames()))
        RETHROW(getGroupTraj(move_joint_group, motion_joints, motion_points, traj));
      joint_group_traj.push_back(traj);
    }

    current_active_controllers_ = 0;
    current_success_= true;

    // Send pose commands
    for (std::size_t i = 0; i < move_joint_groups_.size(); ++i)
    {
      if (joint_group_traj[i].empty()) // nothing for this controller
        continue;

      move_joint_groups_[i]->setCallback(boost::bind(&PlayMotion::controllerCb, this, _1));
      RETHROW(move_joint_groups_[i]->sendGoal(joint_group_traj[i], duration));
      current_active_controllers_++;
    }

    if (current_active_controllers_ < 1)
      return false;

    return true;
  }
}