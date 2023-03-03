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
#define PROFILE_ACCEL 500
#define PROFILE_VEL 1000

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
    }

    set_current_pose(dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
    std::vector<float> wake_up_poses(4);




    while (true)
    {
        printf("Press any key to continue. (Press [ESC] to exit)\n");
        // press once to move to the start position
        // std::vector<float> set_positions = {184.0, 175.0, 143.0, 176.2};
        if (getch() == ENTER_ASCII_VALUE)
        {
        // wake up poses move each servo a little before moving to their set positions to get rid of jumpiness
        for (int i = 1; i <= size(initial_positions); i ++){
        wake_up_poses.at(i - 1) = initial_positions.at(i - 1) + .15;
        move_servos(i, wake_up_poses.at(i - 1), initial_positions.at(i - 1), dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }

        //////// release the left hand first //////////
        // get the left hand loose
        if (getch() == ENTER_ASCII_VALUE)
        {
        move_servos(3, 217.5, wake_up_poses[2], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(4, 175, wake_up_poses[3], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(1, 224, wake_up_poses[0], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(2, 179.5, wake_up_poses[1], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }
        // slightly raise the left hand off the bar
        if (getch() == ENTER_ASCII_VALUE)
        {
        move_servos(3, 205, 217.5, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }

        // press the left hand in to lock the right one
        if (getch() == ENTER_ASCII_VALUE)
        {
        move_servos(2, 190, 179.5, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }

        //////// raise the left hand /////////
        // lift the left arm with the right one until its clear
        if (getch() == ENTER_ASCII_VALUE)
        {
        move_servos(3, 165, 210, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }

        // pull the left arm back
        if (getch() == ENTER_ASCII_VALUE)
        {
        move_servos(2, 170, 194, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }
        // rotate the left arm (partially)

        if (getch() == ENTER_ASCII_VALUE)
        {
        move_servos(1, 150, 224, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }
        // roatate the body out 
        if (getch() == ENTER_ASCII_VALUE)
        {
        move_servos(4, 182, 177.5, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }
        // lift the left arm to the bar
        if (getch() == ENTER_ASCII_VALUE)
        {
        move_servos(3, 110, 165, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }
        // finish rotating the arm
        if (getch() == ENTER_ASCII_VALUE)
        {
        move_servos(1, 135, 150, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }

        //////// grab the next bar /////////
        // move the hand forward
        if (getch() == ENTER_ASCII_VALUE)
        {
        move_servos(2, 185, 170, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }
        // move the left arm down with the right arm
        if (getch() == ENTER_ASCII_VALUE)
        {
        move_servos(3, 130, 110, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }
        // adjust the left arm to grab the bar

        ///////// left the right arm //////////
        // 
        //     //    set the increment positions to be the start position of the servos
        //     increment_positions = wake_up_poses;
        //     // for each servo, if its current position is the goal (set_position) do nothing
        //     // if the position isn't at the goal then increment itincrement_positions = set_positions;
        //     move_servos(2, 186.5, increment_positions[1], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("first go \n");
        //     if (getch() == ENTER_ASCII_VALUE)
        // {
        //     move_servos(4, 178.2, increment_positions[3], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("Second go \n");
        // }
        // if (getch() == ENTER_ASCII_VALUE)
        // {
        //     move_servos(3, 149.5, increment_positions[2], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("third go \n");
        // }
        // if (getch() == ENTER_ASCII_VALUE)
        // {
        //     move_servos(1, set_positions[0], increment_positions[0], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("fourth go \n");
        // }
        // // if (getch() == ENTER_ASCII_VALUE)
        // // {
        // //     move_servos(2, 180, 185.6, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // //     printf("fifth go \n");
        // // }
        // if (getch() == ENTER_ASCII_VALUE)
        // {
        //     move_servos(3, 143, 149.5, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("sixth go \n");
        // }
        // if (getch() == ENTER_ASCII_VALUE)
        // {
        //     move_servos(2, 175, 180, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("seventh go \n");
        // }
        //     printf("ENTER PRESSED ONCE\n");
        //     // {184, 180, 143, 170}
        // }

        // // // press to go to lifting the arm
        // std::vector<float> lift_arm_positions = {135.0, 182.0, 110.0, 190.0};
        // if (getch() == ENTER_ASCII_VALUE)
        // {  
        //     increment_positions = set_positions;
        //     move_servos(3, lift_arm_positions[2], increment_positions[2], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("ENTER PRESSED WHAT\n");
        // }
        // if (getch() == ENTER_ASCII_VALUE)
        // {
        //     move_servos(1, lift_arm_positions[0], increment_positions[0], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("ENTER PRESSED Four\n");
        // }
        // if (getch() == ENTER_ASCII_VALUE){
        //     move_servos(2, lift_arm_positions[1], increment_positions[1], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("ENTER PRESSED Five\n");
        // }
        // // // press to go to lower arm to bar
        // std::vector<float> lower_arm_positions = {145.0, 185.0, 120.0, 190.0};
        // if (getch() == ENTER_ASCII_VALUE)
        // {   increment_positions = lift_arm_positions;
        //     move_servos(3, lower_arm_positions[2], increment_positions[2], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("ENTER PRESSED ho\n");
        // }
        // // if (getch() == ENTER_ASCII_VALUE)
        // // {
        // //     move_servos(2, 182, increment_positions[2], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // //     printf("ENTER PRESSED ho\n");
        // // }
        // // press to go to a second set position (release second hand)
        // std::vector<float> second_set_positions = {165.0, 187.0, 120.0, 185.0};
        // if (getch() == ENTER_ASCII_VALUE)
        // {   increment_positions = lower_arm_positions;
        //     move_servos(1, second_set_positions[0], increment_positions[0], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("ENTER PRESSED bar\n");
        // }
        // if (getch() == ENTER_ASCII_VALUE){
        //     move_servos(4, second_set_positions[3], increment_positions[3], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("ENTER PRESSED nor\n");
        // }
        // // if (getch() == ENTER_ASCII_VALUE)
        // // {
        // //     move_servos(1, 155.0, 158.0, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // //     printf("ENTER PRESSED bar\n");
        // // }
        // // if (getch() == ENTER_ASCII_VALUE){
        // //     move_servos(4, second_set_positions[3], increment_positions[3], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // //     printf("ENTER PRESSED nor\n");
        // // }
        // // // press to go to lift the second arm
        // std::vector<float> lift_second_arm_positions = {215.0, 187.0, 170.0, 175.0};
        // if (getch() == ENTER_ASCII_VALUE)
        // {   increment_positions = second_set_positions;
        //     move_servos(1, lift_second_arm_positions[0], increment_positions[0], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("ENTER PRESSED right\n");
        // }
        // if (getch() == ENTER_ASCII_VALUE)
        // {
        //     move_servos(3, lift_second_arm_positions[2], increment_positions[2], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("ENTER PRESSED here\n");
        // }
        // if (getch() == ENTER_ASCII_VALUE){
        //     move_servos(4, lift_second_arm_positions[3], increment_positions[3], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("ENTER PRESSED eight\n");
        // }
        // // position arm
        // if (getch() == ENTER_ASCII_VALUE)
        // {
        //     move_servos(3, 168, 170, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("ENTER PRESSED op\n");
        // }
        // std::vector<float> lower_second_arm_positions = {185.0, 187.0, 170.0, 175.0};
        // if (getch() == ENTER_ASCII_VALUE) 
        // {   increment_positions = lift_second_arm_positions;
        //     move_servos(1, lower_second_arm_positions[0], increment_positions[0], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //     printf("ENTER PRESSED SEVEN\n");
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