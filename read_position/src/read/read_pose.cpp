
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk/port_handler.h"
#include "dynamixel_sdk/port_handler_linux.h"

#define X_SERIES // X330, X430, X540, 2X430
#define ADDR_TORQUE_ENABLE          64
#define ADDR_GOAL_POSITION          116
#define ADDR_PRESENT_POSITION       132
#define MINIMUM_POSITION_LIMIT      2060  // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT      2800  // Refer to the Maximum Position Limit of product eManual
#define BAUDRATE                    57600
#define PROTOCOL_VERSION  2.0

#define NUMB_OF_DYNAMIXELS              4

#define DEVICENAME  "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MOVING_STATUS_THRESHOLD     20  // DYNAMIXEL moving status threshold
#define ESC_ASCII_VALUE                 0x1b

int getch() {
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int main() {
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_position[2] = {MINIMUM_POSITION_LIMIT, MAXIMUM_POSITION_LIMIT};         // Goal position

  uint8_t dxl_error = 0;                          // DYNAMIXEL error
  int32_t dxl_present_position = 0;  // Read 4 byte Position data

  // Open port
  if (portHandler->openPort()) {
    printf("Succeeded to open the port!\n");
  }
  else {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n");
  }
  else {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  while(1) {
    printf("Press any key to continue. (Press [ESC] to exit)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    do {
      // use this block of code for only reading one at
      // Read the Present Position
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, 1, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      }

      printf("[ID:%03d] Present Position:%03d\n", 1, dxl_present_position);
      printf("\n\n\n\n");
      for (int j = 0; j < 1000000; j++){}

      // // use this block of code for reading all of them
      // for (int i = 1; i <= NUMB_OF_DYNAMIXELS; i++){
      //       // Read the Present Position
      //       dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      //       if (dxl_comm_result != COMM_SUCCESS) {
      //       printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      //       }
      //       else if (dxl_error != 0) {
      //       printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      //       }

      //       printf("[ID:%03d] Present Position:%03d\n", i, dxl_present_position);
      //       }
      //   printf("\n\n\n\n");
      //   for (int j = 0; j < 1000000; j++){}
    } while(1);

  }

  // Disable DYNAMIXEL Torque
  for (int i = 1; i <= NUMB_OF_DYNAMIXELS; i++){
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else {
        printf("Succeeded disabling DYNAMIXEL Torque.\n");
    }
  }

  // Close port
  portHandler->closePort();
  return 0;
}