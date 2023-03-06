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

void set_current_pose(int dxl_comm_result, dynamixel::PacketHandler *&packetHandler, dynamixel::PortHandler *&portHandler, uint32_t dxl_present_position, uint8_t &dxl_error)
{
    // For each servo, add its current position to the servo pose
    for (int i = 1; i <= NUMB_OF_DYNAMIXELS; i++)
    {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        printf("[ID:%03d] Present Position:%03f\n", i, climbing::tics2deg(dxl_present_position));
        initial_positions[i - 1] = climbing::tics2deg(dxl_present_position);
    }
}

void move_servos(int dxl_id, double servo_poses, double increment_poses, int dxl_comm_result, dynamixel::PacketHandler *&packetHandler, dynamixel::PortHandler *&portHandler, uint32_t dxl_present_position, uint8_t &dxl_error)
{
    // get the current position to start incrementing from it 
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
    // set goal position to be where we are currently so it can be slowly incremented in the right direction
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_poses), &dxl_error);
            do
            {
                // Read the Present Position
                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
                double diff = servo_poses - climbing::tics2deg(dxl_present_position);
                // if the present position isn't the set position then increase the increment position
                if (diff > 0)
                {
                    // add value to the increment
                    increment_poses += .15;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_poses), &dxl_error);
                }
                else if (diff < 0)
                {
                    // add value to the increment
                    increment_poses -= .15;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_poses), &dxl_error);
                }
                printf("Servo %d \n", dxl_id);
                printf("present pose: %f", climbing::tics2deg(dxl_present_position));
                printf("goal pose: %f", increment_poses);
            } while ((abs(servo_poses - climbing::tics2deg(dxl_present_position)) > (DXL_MOVING_STATUS_THRESHOLD)));
            printf("ENTER PRESSED Eight\n");
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

    set_current_pose(dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
    std::vector<float> wake_up_poses(4);




    while (true)
    {
        printf("Press any key to continue. (Press [ESC] to exit)\n");
        // press once to move to the start position
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
        ////// MOVE LEFT ARM DOWN ///////
        move_servos(3, 131, wake_up_poses[2], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(4, 178, wake_up_poses[3], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(1, 141, wake_up_poses[0], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(2, 183, wake_up_poses[1], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // secure the right hand 
        move_servos(4, 170, 178, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // slightly release the left hand
        move_servos(3, 118, 131, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // wiggle the servos to let go
        move_servos(1, 130, 141, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(3, 115, 118, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(2, 180, 183, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(1, 127, 130, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(3, 108, 115, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // pull the left hand out of the ladder 
        move_servos(2, 165, 181, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // rotate the body (for safety)
        move_servos(4, 182, 170, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // rotate the arm around
        move_servos(1, 210, 127, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // drop the left arm down
        move_servos(3, 190, 106, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move left arm in
        move_servos(2, 183, 165, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // drop the left down to bar
        move_servos(3, 210, 190, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // spin arm all the way arond
        move_servos(1, 217, 210, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // slightly raise the left hand off the bar

        set_current_pose(dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // wake up poses move each servo a little before moving to their set positions to get rid of jumpiness
        for (int i = 1; i <= (int)size(initial_positions); i ++){
        wake_up_poses.at(i - 1) = initial_positions.at(i - 1) + .15;
        move_servos(i, wake_up_poses.at(i - 1), initial_positions.at(i - 1), dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }
        ////// MOVE RIGHT ARM DOWN ///////
        move_servos(3, 211, wake_up_poses[2], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(4, 181, wake_up_poses[3], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(1, 217, wake_up_poses[0], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(2, 183, wake_up_poses[1], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // lift right arm a little
        move_servos(1, 239, 217, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // wiggle!
        move_servos(3, 220, 211, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(1, 245, 239, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(3, 226, 220, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move_servos(1, 248, 241, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move the arm out of the ladder
        move_servos(4, 202, 181, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move body out
        move_servos(2, 178, 188, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //move arm over
        move_servos(3, 145, 226, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move the arm down 
        move_servos(1, 175, 252, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move arm in 
        move_servos(4, 181, 202, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // drop arm to the bar
        move_servos(1, 145, 175, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // fit hand onto bar
        move_servos(3, 140, 145, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
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