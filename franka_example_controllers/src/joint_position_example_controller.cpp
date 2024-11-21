// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_position_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

// init 메서드 //
bool JointPositionExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  // 로봇 하드웨어로부터 PositionJointInterface를 가져옴, 이를 통해 joint position 알 수 있음
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionExampleController: Error getting position joint interface from hardware!");   //PositionJointInterface 가져오지 못할 경우 에러 메시지 출력
    return false;
  }

  // franka_example_controller.yaml에 정의된 joint_names를 불러옴
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");  //joint 7개가 아닐 경우 에러 메시지 출력
    return false;
  }

  // joint handle 가져와 position_joint_handles_에 저장
    // handle 기능 1. joint의 위치, 속도, 토크값을 읽어옴 : getPosition(), getVelocity(), getEffort() 
    // handle 기능 2. joint의 목표 위치 설정 : setCommand(double position)

  position_joint_handles_.resize(7);  // 크기가 7인 벡터 생성
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);  // 각 joint의 handle을 가져옴
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Exception getting joint handles: " << e.what()); // handle 가져오지 못할 경우 에러 메시지 출력
      return false;
    }
  }

  // joint가 초기 위치인 q_start와 다르면 작동 X
  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}}; // 초기 위치를 q_start 배열로 선언 (0, -45, 0, -135, 0, 90, 45)
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {  // joint 위치가 q_start와 0.1 이상 차이날 경우 에러 메시지 출력
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }

  return true;
}


// starting 메서드 //
void JointPositionExampleController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ = ros::Duration(0.0);
}


// update 메서드 //
void JointPositionExampleController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

// 변위 계산 : 조화 진동 식 x(t) = A(1-cos(wt))
// A : pi/16, w : pi/5, 0.2 scaling하여 진동폭을 작게 함
  double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.2;    // 변위 계산 수식
  for (size_t i = 0; i < 7; ++i) {
    if (i == 4) {
      position_joint_handles_[i].setCommand(initial_pose_[i] - delta_angle);  // 5번 관절은 반대 방향으로 변위 적용
    } else {
      position_joint_handles_[i].setCommand(initial_pose_[i] + delta_angle);
    }
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerBase)
