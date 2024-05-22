// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
// This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run dynamixel_sdk_examples read_write_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 1000}"
// $ ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition ""
//
// Original author: Will Son
// Modified by: Fernando Moreno
*******************************************************************************/

#include <cstdio>
#include <memory>
#include <string>
#include <chrono>
#include <functional>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/tf_angles.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "read_write_node.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PROFILE_VELOCITY 112

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID                         1                   // Dynamixel ID: 1
#define DXL2_ID                         2                   // Dynamixel ID: 2
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

// Angle constraints
#define DXL1_ANGLE_LOW  -20
#define DXL1_ANGLE_HIGH  20
#define DXL2_ANGLE_LOW  -50
#define DXL2_ANGLE_HIGH  50


using namespace std::chrono_literals;

int angleToPos(int x, int in_max = 360, int out_max = 4096, int offset = 2048)
{
  return round(x * (float)(out_max) / (in_max) + offset);
}

int PosToAngle(int x, int in_max = 4096, int out_max = 360, int offset = -180)
{
  return round(x * (float)(out_max) / (in_max) + offset);
}

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;



ReadWriteNode::ReadWriteNode()
: Node("read_write_node")
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  set_position_subscriber_ =
    this->create_subscription<SetPosition>(
    "set_position",
    QOS_RKL10V,
    [this](const SetPosition::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

      // Angle Constraints
      if(msg->angle_1 < DXL1_ANGLE_LOW) msg->angle_1 = DXL1_ANGLE_LOW;
      else if(msg->angle_1 > DXL1_ANGLE_HIGH) msg->angle_1 = DXL1_ANGLE_HIGH;

      if(msg->angle_2 < DXL2_ANGLE_LOW) msg->angle_2 = DXL2_ANGLE_LOW;
      else if(msg->angle_2 > DXL2_ANGLE_HIGH) msg->angle_2 = DXL2_ANGLE_HIGH;

      // Position Value of X series is 4 byte data.
      uint32_t goal_position_1 = (unsigned int)angleToPos(msg->angle_1);  // Convert int32 -> uint32
      uint32_t goal_position_2 = (unsigned int)angleToPos(msg->angle_2);  // Convert int32 -> uint32

      

      // Write Goal Position 1 (length : 4 bytes)
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) DXL1_ID,
        ADDR_GOAL_POSITION,
        goal_position_1,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Angle: %d]", DXL1_ID, msg->angle_1);
      }

      // Write Goal Position 2 (length : 4 bytes)
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) DXL2_ID,
        ADDR_GOAL_POSITION,
        goal_position_2,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Angle: %d]", DXL2_ID, msg->angle_2);
      }

    }
    );

  //publisher
  tf_angles_publisher_ = this->create_publisher<TfAngles>("tf_angles", QOS_RKL10V);
  timer_ = this->create_wall_timer( 20ms, std::bind(&ReadWriteNode::timer_callback, this));

  
  auto get_present_position =
    [this](
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response) -> void
    {
      // Read Present Position 1 (length : 4 bytes) and Convert uint32 -> int32
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) DXL1_ID,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position_1),
        &dxl_error
      );
      // Read Present Position 2 (length : 4 bytes) and Convert uint32 -> int32
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) DXL2_ID,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position_2),
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Angle: %d]",
        DXL1_ID,
        PosToAngle(present_position_1)
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Angle: %d]",
        DXL2_ID,
        PosToAngle(present_position_2)
      );

      response->angle_1 = PosToAngle(present_position_1);
      response->angle_2 = PosToAngle(present_position_2);

    };

  get_position_server_ = create_service<GetPosition>("get_position", get_present_position);
  

}

ReadWriteNode::~ReadWriteNode()
{
}

void ReadWriteNode::timer_callback()
{
    dynamixel_sdk_custom_interfaces::msg::TfAngles msg;
    // Read Present Position 1 (length : 4 bytes) and Convert uint32 -> int32
    // uint32_t present_position_1, present_position_2;
    dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler,
      (uint8_t) DXL1_ID,
      ADDR_PRESENT_POSITION,
      reinterpret_cast<uint32_t *>(&present_position_1),
      &dxl_error
    );
    // Read Present Position 2 (length : 4 bytes) and Convert uint32 -> int32
    dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler,
      (uint8_t) DXL2_ID,
      ADDR_PRESENT_POSITION,
      reinterpret_cast<uint32_t *>(&present_position_2),
      &dxl_error
    );

    msg.tf_angle_1 = PosToAngle(present_position_1);
    msg.tf_angle_2 = PosToAngle(present_position_2);

    tf_angles_publisher_->publish(msg);
}


void setupDynamixel(uint8_t dxl_id)
{
  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    3,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Position Control Mode.");
  }

  // Change Dynamixel Profile Velocity
  packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, 33);

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<ReadWriteNode>();
  rclcpp::spin(readwritenode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}