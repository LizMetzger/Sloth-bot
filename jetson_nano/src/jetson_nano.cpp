#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <vector>
#include <map>

#include "control/control_dyn.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk/port_handler.h"
#include "dynamixel_sdk/port_handler_linux.h"

#define STDIN_FILENO 0

// define dynamixel information
#define X_SERIES
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define MINIMUM_POSITION_LIMIT -1000 // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT 4095  // Refer to the Maximum Position Limit of product eManual
#define MAXIMUM_VELOCITY_LIMIT 5     // 0-1023
#define BAUDRATE 57600
#define ADDE_PRESENT_CURRENT 126
#define ADDER_ACCEL 108
#define ADDER_VEL 112
#define PROFILE_ACCEL 10
#define PROFILE_VEL 15
#define ADDER_P 80
#define ADDER_I 82
#define ADDER_D 84
#define P_VAL 1600
#define I_VAL 50
#define D_VAL 100

#define PROTOCOL_VERSION 2.0

#define DEVICENAME "/dev/ttyUSB0"

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0
#define DXL_MOVING_STATUS_THRESHOLD .1 // DYNAMIXEL moving status threshold
#define ENTER_ASCII_VALUE 0x0d
#define ADDR_MAX_VELOCITY 44

#define NUMB_OF_DYNAMIXELS 4

// initialize the amount to increment the position by in degrees
float position_increment = 1.0;

// initialize a list of goal positions for each servo
// std::vector<float> servo_positions = {2027.0, 0.0, 2027.0, 0.0};
std::vector<float> increment_positions(4);
std::vector<float> initial_positions(4);

using namespace climbing; 

// For non-blocking keyboard inputs
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}


int main()
{
    // Initialize PortHandler instance
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    uint8_t dxl_error = 0;              // DYNAMIXEL error
    int32_t dxl_present_position = 0;   // Read 4 byte Position data
    // int32_t last_pos = 0;                           // Get the last Position

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

    
    for (int i = 1; i <= NUMB_OF_DYNAMIXELS; i++)
    {
        // Enable DYNAMIXEL Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        // Set Profile Velcotiy
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDER_VEL, PROFILE_VEL, &dxl_error);
        // Set Profile Acceleration
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDER_ACCEL, PROFILE_ACCEL, &dxl_error);
        // Set P Value
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDER_P, P_VAL, &dxl_error);
        // Set I Value
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDER_I, I_VAL, &dxl_error);
        // Set D Value
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDER_D, D_VAL, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Succeeded enabling DYNAMIXEL Torque and setting Velocity Profile, and Acceleration Profile for servo ID %d \n", i);
        }
    }

    while (true)
    {
        printf("Press ENTER to continue... \n");

        if (getch() == ENTER_ASCII_VALUE)
        {
            printf("One");
            right_climb(dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
            // printf("Two");
            // left_climb(dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
            // printf("One");
            // right_climb(dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
            // left_climb(dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
            // left_down(dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
            // right_down(dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
            // left_down(dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
            // right_down(dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }

        // if the key press is enter then break
        if (getch() == ENTER_ASCII_VALUE)
        {
            break;
        }
    }

    // Disable DYNAMIXEL Torque
    for (int j = 1; j <= NUMB_OF_DYNAMIXELS; j++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, j, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Succeeded disabling DYNAMIXEL Torque for servo id %d.\n", j);
        }
    }

    // Close port
    portHandler->closePort();
    return 0;
}