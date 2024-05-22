/*
 * read_write.cpp
 *
 *  Created on: 2016. 2. 21.
 *      Author: leon
 */

//
// *********     Read and Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is designed for using a Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
// To use another Dynamixel model, such as X series, see their details in E-Manual(support.robotis.com) and edit below "#define"d variables yourself.
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 3 (Baudrate : 1000000 [1M])
//

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <cmath>

#include "dynamixel_sdk/dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_PROFILE_VELOCITY       112


// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel ID: 1
#define DXL2_ID                         2                   // Dynamixel ID: 2
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define VELOCITY_SCAN                   33
#define VELOCITY_FAST                   100
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MIN_POSITION_LIMIT          0                   // Value defined in control table
#define DXL_MAX_POSITION_LIMIT          4095                // Value defined in control table 
#define DXL_MIN_ANGLE_LIMIT             0                  
#define DXL_MAX_ANGLE_LIMIT             359               

#define DXL_MOVING_STATUS_THRESHOLD     1                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

int angleToPos(int x, int in_max = 360, int out_max = 4096, int offset = 2048)
{
  return round(x * (float)(out_max) / (in_max) + offset);
}

class Dxl
{
public:
  int id;
  int comm_result = COMM_TX_FAIL;             // Communication result
  int minimum_angle_value;
  int origin_angle_value;
  int maximum_angle_value;
  uint8_t error = 0;                          // Dynamixel error
  int32_t present_position = 0;               // Present position
  int positions[3];
  Dxl(int i, int min, int orig, int max) : id(i), minimum_angle_value(min), origin_angle_value(orig), maximum_angle_value(max) {
    positions[0] = angleToPos(minimum_angle_value);
    positions[1] = angleToPos(origin_angle_value);
    positions[2] = angleToPos(maximum_angle_value);
  }
};

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

void toMove(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, Dxl dxl, int velocity_value, int dxl_goal_position)
{

    // Change Dynamixel Profile Velocity
    packetHandler->write4ByteTxRx(portHandler, dxl.id, ADDR_PRO_PROFILE_VELOCITY, velocity_value);

    // Write goal position for Dynamixel
    dxl.comm_result = packetHandler->write4ByteTxRx(portHandler, dxl.id, ADDR_PRO_GOAL_POSITION, dxl_goal_position, &dxl.error);
    if (dxl.comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl.comm_result));
    }
    else if (dxl.error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl.error));
    }

    do
    {
      // Read present position for Dynamixel #1
      dxl.comm_result = packetHandler->read4ByteTxRx(portHandler, dxl.id, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl.present_position, &dxl.error);
      if (dxl.comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl.comm_result));
      }
      else if (dxl.error != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(dxl.error));
      }

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", dxl.id , dxl_goal_position, dxl.present_position);

    }while((abs(dxl_goal_position - dxl.present_position) > DXL_MOVING_STATUS_THRESHOLD));

}

void toMoveSynced(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, Dxl dxl1, Dxl dxl2, int velocity1_value, int velocity2_value, int dxl1_goal_position, int dxl2_goal_position)
{
    // Change Dynamixel #1 Profile Velocity
    packetHandler->write4ByteTxRx(portHandler, dxl1.id, ADDR_PRO_PROFILE_VELOCITY, velocity1_value);
    // Change Dynamixel #2 Profile Velocity
    packetHandler->write4ByteTxRx(portHandler, dxl2.id, ADDR_PRO_PROFILE_VELOCITY, velocity2_value);

    // Write goal position for Dynamixel #1
    dxl1.comm_result = packetHandler->write4ByteTxRx(portHandler, dxl1.id, ADDR_PRO_GOAL_POSITION, dxl1_goal_position, &dxl1.error);
    if (dxl1.comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl1.comm_result));
    }
    else if (dxl1.error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl1.error));
    }

    // Write goal position for Dynamixel #2
    dxl2.comm_result = packetHandler->write4ByteTxRx(portHandler, dxl2.id, ADDR_PRO_GOAL_POSITION, dxl2_goal_position, &dxl2.error);
    if (dxl2.comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl2.comm_result));
    }
    else if (dxl2.error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl2.error));
    }

    do
    {
      // Read present position for Dynamixel #1
      dxl1.comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl1.present_position, &dxl1.error);
      if (dxl1.comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl1.comm_result));
      }
      else if (dxl1.error != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(dxl1.error));
      }

      // Read present position for Dynamixel #2
      dxl2.comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl2.present_position, &dxl2.error);
      if (dxl2.comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl2.comm_result));
      }
      else if (dxl2.error != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(dxl2.error));
      }

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", dxl1.id, dxl1_goal_position, dxl1.present_position, dxl2.id, dxl2_goal_position, dxl2.present_position);

     }while((abs(dxl1_goal_position - dxl1.present_position) > DXL_MOVING_STATUS_THRESHOLD) 
         || (abs(dxl2_goal_position - dxl2.present_position) > DXL_MOVING_STATUS_THRESHOLD));

}


void moveToOrigin(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, Dxl dxl1, Dxl dxl2)
{
  toMoveSynced(portHandler, packetHandler, dxl1, dxl2, VELOCITY_FAST, VELOCITY_FAST, dxl1.positions[1], dxl2.positions[1]);
}


void moveScanning(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, Dxl dxl1, Dxl dxl2)
{
  int dxl1_sequence[] = {0,2,1};
  moveToOrigin(portHandler, packetHandler, dxl1, dxl2);
  for(int i = 0; i < 3; i++){
      toMove(portHandler, packetHandler, dxl2, VELOCITY_SCAN, dxl2.positions[2-i]);
      for(int j = 0; j < 3; j++){
          toMove(portHandler, packetHandler, dxl1, VELOCITY_SCAN, dxl1.positions[dxl1_sequence[j]]);
      }
  }
  moveToOrigin(portHandler, packetHandler, dxl1, dxl2);
}

void moveCamCalibration(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, Dxl dxl1, Dxl dxl2)
{
  int nsamples = 20;
  int dxl1_goal_position[nsamples];
  int dxl2_goal_position[nsamples];
  double senX;
  double cosX;

  for(int i = 0; i < nsamples; i++){
    senX = sin(2*M_PI*i/nsamples);
    cosX = cos(2*M_PI*i/nsamples);
    dxl1_goal_position[i] = dxl1.positions[1] + senX*(dxl1.positions[2]-dxl1.positions[0])/2;
    dxl2_goal_position[i] = dxl2.positions[1] + cosX*(dxl2.positions[2]-dxl2.positions[1])/5;
    toMoveSynced(portHandler, packetHandler, dxl1, dxl2, VELOCITY_SCAN, VELOCITY_SCAN, dxl1_goal_position[i], dxl2_goal_position[i]);
    printf("POS: %d/%d \n", i+1, nsamples);
    // printf("sen = %f, cos = %f \n", senX, cosX);
    getch();
  }
}


int main()
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // 
  const int dxl1_minimum_angle_value = -20;
  const int dxl1_origin_angle_value = 0;
  const int dxl1_maximum_angle_value = 20;

  const int dxl2_minimum_angle_value = -50;
  const int dxl2_origin_angle_value = 0;
  const int dxl2_maximum_angle_value = 50;

  Dxl dxl1(DXL1_ID, dxl1_minimum_angle_value, dxl1_origin_angle_value, dxl1_maximum_angle_value);
  Dxl dxl2(DXL2_ID, dxl2_minimum_angle_value, dxl2_origin_angle_value, dxl2_maximum_angle_value);

  /*int index = 0;
  int dxl1_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl2_comm_result = COMM_TX_FAIL;             // Communication result

  const int dxl1_minimum_angle_value = 160;
  const int dxl1_maximum_angle_value = 200;
  const int dxl1_origin_angle_value = 180;
  const int dxl2_minimum_angle_value = 130;
  const int dxl2_maximum_angle_value = 230;
  const int dxl2_origin_angle_value = 180;

  // const int dxl1_minimum_position_value = 1820;
  // const int dxl1_maximum_position_value = 2276;
  // const int dxl1_origin_position_value = 2048;
  // const int dxl2_minimum_position_value = 1479;
  // const int dxl2_maximum_position_value = 2617;
  // const int dxl2_origin_position_value = 2048;

  int nSecuence = 5;
  int dxl1_positions[3] = {map(dxl1_minimum_angle_value), map(dxl1_origin_angle_value), map(dxl1_maximum_angle_value)};
  int dxl2_positions[3] = {map(dxl2_minimum_angle_value), map(dxl2_origin_angle_value), map(dxl2_maximum_angle_value)};
  int dxl1_sequence[] = {1, 2, 0, 2, 1};
  int dxl2_sequence[] = {1, 2, 0, 2, 1};

  uint8_t dxl1_error = 0;                          // Dynamixel error
  int32_t dxl1_present_position = 0;               // Present position
  uint8_t dxl2_error = 0;                          // Dynamixel error
  int32_t dxl2_present_position = 0;               // Present position
  */

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Change Dynamixel#1 Profile Velocity
  packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PROFILE_VELOCITY, VELOCITY_SCAN);

  // Change Dynamixel#2 Profile VelocitY
  packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PROFILE_VELOCITY, VELOCITY_SCAN);

  // Enable Dynamixel #1 Torque
  dxl1.comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl1.error);
  if (dxl1.comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl1.comm_result));
  }
  else if (dxl1.error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl1.error));
  }
  else
  {
    printf("Dynamixel #1 has been successfully connected \n");
  }

  // Enable Dynamixel #2 Torque
  dxl2.comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl2.error);
  if (dxl2.comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl2.comm_result));
  }
  else if (dxl2.error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl2.error));
  }
  else
  {
    printf("Dynamixel #2 has been successfully connected \n");
  }

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    /*

    // Write goal position for Dynamixel #1
    dxl1_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl1_goal_position[index], &dxl1_error);
    if (dxl1_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl1_comm_result));
    }
    else if (dxl1_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl1_error));
    }

    do
    {
      // Read present position for Dynamixel #1
      dxl1_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl1_present_position, &dxl1_error);
      if (dxl1_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl1_comm_result));
      }
      else if (dxl1_error != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(dxl1_error));
      }

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL1_ID, dxl1_goal_position[index], dxl1_present_position);
      //printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL1_ID, dxl1_goal_position[index], dxl1_present_position, DXL2_ID, dxl2_goal_position[index], dxl2_present_position);

    // }while((abs(dxl1_goal_position[index] - dxl1_present_position) > DXL1_MOVING_STATUS_THRESHOLD) 
    //     || (abs(dxl2_goal_position[index] - dxl2_present_position) > DXL2_MOVING_STATUS_THRESHOLD));

    }while((abs(dxl1_goal_position[index] - dxl1_present_position) > DXL1_MOVING_STATUS_THRESHOLD));

    // Write goal position for Dynamixel #2
    dxl2_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_goal_position[index], &dxl2_error);
    if (dxl2_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl2_comm_result));
    }
    else if (dxl2_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl2_error));
    }
    
    do
    {
      // Read present position for Dynamixel #2
      dxl2_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl2_present_position, &dxl2_error);
      if (dxl2_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl2_comm_result));
      }
      else if (dxl2_error != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(dxl2_error));
      }

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL2_ID, dxl2_goal_position[index], dxl2_present_position);
      //printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL1_ID, dxl1_goal_position[index], dxl1_present_position, DXL2_ID, dxl2_goal_position[index], dxl2_present_position);
    
    }while((abs(dxl2_goal_position[index] - dxl2_present_position) > DXL2_MOVING_STATUS_THRESHOLD));

    */

    /*
    for(int i = 0; i < nSecuence; i++)
    {
      toMove(portHandler, packetHandler, DXL1_ID, dxl1_comm_result, dxl1_error, dxl1_present_position, dxl1.positions[dxl1_sequence[i]]);
      toMove(portHandler, packetHandler, DXL2_ID, dxl2_comm_result, dxl2_error, dxl2_present_position, dxl2.positions[dxl2_sequence[i]]);
    }
    */

    //moveScanning(portHandler, packetHandler, dxl1, dxl2);
    //moveCamCalibration(portHandler, packetHandler, dxl1, dxl2);
    printf("positions: %d, %d, %d \n", dxl1.positions[0], dxl1.positions[1], dxl1.positions[2]);
    printf("positions: %d, %d, %d \n", dxl2.positions[0], dxl2.positions[1], dxl2.positions[2]);
  }

  // Disable Dynamixel Torque
  dxl1.comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl1.error);
  if (dxl1.comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl1.comm_result));
  }
  else if (dxl1.error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl1.error));
  }

  // Close port
  portHandler->closePort();

  return 0;
}
