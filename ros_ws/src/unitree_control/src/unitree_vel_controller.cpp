/*
 * Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <mutex>
#include <csignal>
#include <atomic>

#define NETWORK_INTERFACE "eth0"
using namespace unitree::common;

class Custom
{
public:
  Custom(ros::NodeHandle &nh) : should_exit(false)
  {
    // 初始化Unitree运动客户端
    sport_client.SetTimeout(10.0f);
    sport_client.Init();
    sport_client.StandUp();
    sleep(1);
    sport_client.BalanceStand();
    sleep(1);
    // if (!sport_client.ClassicWalk(true)) {
    //   std::cerr << "Failed to switch to Classic Walk mode." << std::endl;
    // }

    if (sport_client.FreeWalk() == 0) {
      std::cout << "Switched to AI Walk mode." << std::endl;
    } else {
      std::cerr << "Failed to switch to AI Walk mode." << std::endl;
    }
    if (sport_client.FreeAvoid(false) == 0) {
      std::cout << "Disabled obstacle avoidance." << std::endl;
    } else {
      std::cerr << "Failed to disable obstacle avoidance." << std::endl;
    }

    // service change walk mode
    change_walk_mode_srv = nh.advertiseService("unitree/classic_walk_mode", &Custom::ChangeWalkMode, this);

    cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &Custom::CmdVelCallback, this);
  }

  ~Custom()
  {
    SafeShutdown();
  }
  void SafeShutdown()
  {
    if (!should_exit.exchange(true)) 
    {
      std::cout << "Initiating safe shutdown..." << std::endl;
      sport_client.StopMove();
      sport_client.StandDown();
      std::cout << "Safety procedures completed." << std::endl;
    }
  }

private:
  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    sport_client.Move(msg->linear.x, msg->linear.y, msg->angular.z);
  }
  bool ChangeWalkMode(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res) {
    if (req.data) {
      std::cout << "Switching to ClassicWalk mode" << std::endl;
    } else {
      std::cout << "Switching to AI walk mode" << std::endl;
    }
    int result = sport_client.ClassicWalk(req.data);
    res.success = result == 0;
    if (!res.success) {
      std::cerr << "Failed to switch walk mode with error code " << result << std::endl;
    }
    res.message = std::string("Switch walk mode ") + 
      (res.success ? "succeeded." : "failed with error code " + std::to_string(result));
    return true;
  }
  unitree::robot::go2::SportClient sport_client;

  ros::ServiceServer change_walk_mode_srv;
  ros::Subscriber cmd_vel_sub;

  std::atomic<bool> should_exit;
};

Custom* global_custom = nullptr;

void signalHandler(int signum)
{
  std::cout << "\nInterrupt signal (" << signum << ") received.\n";
  if (global_custom != nullptr) {
    global_custom->SafeShutdown();
  }
  ros::shutdown();
}

int main(int argc, char **argv)
{
  unitree::robot::ChannelFactory::Instance()->Init(0, NETWORK_INTERFACE);
  ros::init(argc, argv, "unitree_cmd_vel_controller");
  ros::NodeHandle nh;

  Custom custom(nh);
  global_custom = &custom;
 
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);
  ros::spin();
  global_custom = nullptr;
  return 0;
}
