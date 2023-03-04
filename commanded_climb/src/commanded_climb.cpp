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
#define PROFILE_ACCEL 15
#define PROFILE_VEL 30

#define PROTOCOL_VERSION 2.0

#define DEVICENAME "/dev/ttyUSB0"

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0
#define DXL_MOVING_STATUS_THRESHOLD .15 // DYNAMIXEL moving status threshold
#define ENTER_ASCII_VALUE 0x0d
#define ADDR_MAX_VELOCITY 44

#define NUMB_OF_DYNAMIXELS 4

// initialize the amount to increment the position by in degrees
float position_increment = 1.0;

// initialize a list of goal positions for each servo
// std::vector<float> servo_positions = {2027.0, 0.0, 2027.0, 0.0};
std::vector<float> increment_positions(4);
std::vector<float> initial_positions(4);

int dxl_id = 0;
float goal_pose = 0.0;

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

    // Enable DYNAMIXEL Torque
    for (int i = 1; i <= NUMB_OF_DYNAMIXELS; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_MAX_VELOCITY, MAXIMUM_VELOCITY_LIMIT, &dxl_error);
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
            printf("Succeeded enabling DYNAMIXEL Torque for servo ID %d \n", i);
        }
        // dxl_comm_result = packetHandlex
    }

    initial_positions = set_current_pose(dxl_comm_result, initial_positions, packetHandler, portHandler, dxl_present_position, dxl_error);
    std::vector<float> wake_up_poses(4);




    while (true)
    {
        printf("Press any key to continue. (Press [ESC] to exit)\n");
        // press once to move to the start position
        // std::vector<float> set_positions = {184.0, 175.0, 143.0, 176.2};
        // THIS RUNS ONCE WHEN YOU WAKE UP THE ROBOT
        if (getch() == ENTER_ASCII_VALUE)
        {
        // wake up poses move each servo a little before moving to their set positions to get rid of jumpiness
        for (int i = 1; i <= (int)size(initial_positions); i ++){
        wake_up_poses.at(i - 1) = initial_positions.at(i - 1) + .15;
        move_servos(i, wake_up_poses.at(i - 1), initial_positions.at(i - 1), dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }

        //!!//!!// LIFT THE LEFT HAND //!!///!!//
        //////// release the left hand first //////////
        // get the left hand loose
        if (getch() == ENTER_ASCII_VALUE)
        {
        move_servos(3, 218, wake_up_poses[2], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(4, 175, wake_up_poses[3], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(1, 224, wake_up_poses[0], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(2, 180, wake_up_poses[1], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // slightly raise the left hand off the bar
        move_servos(3, 200, 218, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // press the left hand in to lock the right one
        move_servos(2, 196, 180, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //////// raise the left hand /////////
        // lift the left arm with the right one until its clear
        printf("HELLO???");
        move_servos(3, 165, 210, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // pull the left arm back
        move_servos(2, 170, 198, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // rotate the left arm (partially)
        move_servos(1, 150, 224, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // roatate the body out 
        move_servos(4, 182, 175, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // lift the left arm to the bar
        move_servos(3, 110, 165, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // finish rotating the arm
        move_servos(1, 138, 150, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //////// grab the next bar /////////
        // move the hand forward
        move_servos(2, 186, 170, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move the left arm down with the right arm
        move_servos(3, 130, 110, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("YO??");
        ///////// lift the right arm //////////
        // roll the right arm back 
        move_servos(4, 185.6, 182, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("no???");
        // move both shoulders down 
        move_servos(3, 138, 130, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("here??");
        move_servos(1, 164, 138, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("???");
        // push in with right arm
        move_servos(4, 179.5, 185.6, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("push???");
        // move hand up 
        move_servos(1, 180, 164, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("???");
        // move hand back 
        move_servos(4, 188, 179.5, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("push???");
        // rotate hand 
        move_servos(3, 185, 138, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("go??");
        // move arm all the way up
        move_servos(1, 245, 180, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("?!");
        // rotate hand 
        move_servos(3, 213, 185, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("ah??");
        // move hand forward
        move_servos(4, 178, 188, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("end??");
        // move hand down to bar
        move_servos(1, 235, 245, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("?!?!??");
        }
        }

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