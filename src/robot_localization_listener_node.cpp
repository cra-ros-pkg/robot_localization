/*
 * Copyright (c) 2016, TNO IVS Helmond.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot_localization/ros_robot_localization_listener.h"
#include "robot_localization/GetState.h"

#include <string>

namespace RobotLocalization
{

class RobotLocalizationListenerNode
{
public:
  RobotLocalizationListenerNode()
  {
    service_ = n_.advertiseService("get_state", &RobotLocalizationListenerNode::getStateCallback, this);
  }

  std::string getService()
  {
    return service_.getService();
  }

private:
  RosRobotLocalizationListener rll_;
  ros::NodeHandle n_;
  ros::ServiceServer service_;

  bool getStateCallback(robot_localization::GetState::Request  &req,
                        robot_localization::GetState::Response &res)
  {
    Eigen::VectorXd state(STATE_SIZE);
    Eigen::MatrixXd covariance(STATE_SIZE, STATE_SIZE);

    if ( !rll_.getState(req.time_stamp, req.frame_id, state, covariance) )
    {
      ROS_ERROR("Robot Localization Listener Node: Listener instance returned false at getState call.");
      return false;
    }

    for (size_t i = 0; i < STATE_SIZE; i++)
    {
      res.state[i] = (*(state.data() + i));
    }

    for (size_t i = 0; i < STATE_SIZE * STATE_SIZE; i++)
    {
      res.covariance[i] = (*(covariance.data() + i));
    }

    ROS_DEBUG("Robot Localization Listener Node: Listener responded with state and covariance at the requested time.");
    return true;
  }
};

}  // namespace RobotLocalization

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_localization_listener_node");

  RobotLocalization::RobotLocalizationListenerNode rlln;
  ROS_INFO_STREAM("Robot Localization Listener Node: Ready to handle GetState requests at " << rlln.getService());

  ros::spin();

  return 0;
}
