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

#define PROTOCOL_VERSION 2.0

#define DEVICENAME "/dev/ttyUSB0"

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0
#define DXL_MOVING_STATUS_THRESHOLD 1 // DYNAMIXEL moving status threshold
#define ENTER_ASCII_VALUE 0x0d
#define ADDR_MAX_VELOCITY 44

#define NUMB_OF_DYNAMIXELS 4

// initialize the amount to increment the position by in degrees
float position_increment = 1.0;

// initialize a list of goal positions for each servo
// std::vector<float> servo_positions = {2027.0, 0.0, 2027.0, 0.0};
std::vector<float> increment_positions(4);
std::vector<float> initial_positions(4);
std::vector<float> set_positions = {184.0, 187.0, 155.0, 170.0};
std::vector<float> hand_up_positions = {187.0, 183.0, 145.0, 190.0};
std::vector<float> lift_arm_positions = {145.0, 185.0, 110.0, 190.0};
std::vector<float> lower_arm_positions = {145.0, 185.0, 120.0, 190.0};
std::vector<float> second_set_positions = {155.0, 187.0, 120.0, 180.0};
std::vector<float> lift_second_arm_positions = {215.0, 187.0, 175.0, 175.0};

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

// void move_servos(int dxl_id, std::vector<float> servo_poses){
//     // for each servo, if its current position is the goal (set_position) do nothing
//     // if the position isn't at the goal then increment it
//     // write goal position
//     // Initialize PortHandler instance
//     dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

//     // Initialize PacketHandler instance
//     dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

//     uint8_t dxl_error = 0;                          // DYNAMIXEL error
//     int32_t dxl_present_position = 0;

//     int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[dxl_id - 1]), &dxl_error);
//     do{
//     // Read the Present Position
//     dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
//     double diff = servo_poses[dxl_id - 1] - climbing::tics2deg(dxl_present_position);
//     // if the present position isn't the set position then increase the increment position
//     if (diff > 0){
//         // add value to the increment
//         increment_positions[dxl_id - 1] += 1;
//         // rewrite goal position
//         dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[dxl_id - 1]), &dxl_error);
//     }
//     else if (diff < 0){
//         // add value to the increment
//         increment_positions[dxl_id - 1] -= 1;
//         // rewrite goal position
//         dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[dxl_id - 1]), &dxl_error);
//     }
//     printf("Servo %d \n", dxl_id);
//     printf( "present pose: %f", climbing::tics2deg(dxl_present_position));
//     printf( "goal pose: %f", increment_positions[dxl_id - 1]);
//     } while((abs(servo_poses[dxl_id - 1] - climbing::tics2deg(dxl_present_position)) > (DXL_MOVING_STATUS_THRESHOLD)));
//     // printf("ENTER PRESSED ONCE\n");
// }

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

    while (true)
    {
        printf("Press any key to continue. (Press [ESC] to exit)\n");
        // press once to move to the start position
        if (getch() == ENTER_ASCII_VALUE)
        {
            // set the increment positions to be the start position of the servos
            increment_positions = initial_positions;
            // for each servo, if its current position is the goal (set_position) do nothing
            // if the position isn't at the goal then increment it
            for (int i = 0; i < NUMB_OF_DYNAMIXELS; i++)
            {
                // set dynmamixel ID
                dxl_id = i + 1;
                // write goal position
                dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[i]), &dxl_error);
                do
                {
                    // Read the Present Position
                    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
                    double diff = set_positions[i] - climbing::tics2deg(dxl_present_position);
                    // if the present position isn't the set position then increase the increment position
                    if (diff > 0)
                    {
                        // add value to the increment
                        increment_positions[i] += 1;
                        // rewrite goal position
                        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[i]), &dxl_error);
                    }
                    else if (diff < 0)
                    {
                        // add value to the increment
                        increment_positions[i] -= 1;
                        // rewrite goal position
                        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[i]), &dxl_error);
                    }
                    // printf("Servo %d \n", dxl_id);
                    // printf( "present pose: %f", climbing::tics2deg(dxl_present_position));
                    // printf( "goal pose: %f", increment_positions[i]);
                } while ((abs(set_positions[i] - climbing::tics2deg(dxl_present_position)) > (DXL_MOVING_STATUS_THRESHOLD)));
                // printf("ENTER PRESSED ONCE\n");
            }
        }

        // press again to go from start to lifting one hand
        if (getch() == ENTER_ASCII_VALUE)
        {
            // set the increment positions to be the start position of the servos
            increment_positions = set_positions;
            // for each servo, if its current position is the goal (hand_up_positions) do nothing
            // if the position isn't at the goal then increment it
            // move servo 3 first
            // move_servos(3 , hand_up_positions);
            dxl_id = 3;
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[2]), &dxl_error);
            do
            {
                // Read the Present Position
                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
                double diff = hand_up_positions[2] - climbing::tics2deg(dxl_present_position);
                // if the present position isn't the set position then increase the increment position
                if (diff > 0)
                {
                    // add value to the increment
                    increment_positions[2] += 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[2]), &dxl_error);
                }
                else if (diff < 0)
                {
                    // add value to the increment
                    increment_positions[2] -= 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[2]), &dxl_error);
                }
                printf("Servo %d \n", dxl_id);
                printf("present pose: %f", climbing::tics2deg(dxl_present_position));
                printf("goal pose: %f", increment_positions[2]);
            } while ((abs(hand_up_positions[2] - climbing::tics2deg(dxl_present_position)) > (DXL_MOVING_STATUS_THRESHOLD)));
            printf("ENTER PRESSED Once\n");
        }

        // press to go to moving an arm back
        if (getch() == ENTER_ASCII_VALUE)
        {   increment_positions = set_positions;
            // then servo 2
            dxl_id = 2;
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[1]), &dxl_error);
            do
            {
                // Read the Present Position
                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
                double diff = hand_up_positions[1] - climbing::tics2deg(dxl_present_position);
                // if the present position isn't the set position then increase the increment position
                if (diff > 0)
                {
                    // add value to the increment
                    increment_positions[1] += 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[1]), &dxl_error);
                }
                else if (diff < 0)
                {
                    // add value to the increment
                    increment_positions[1] -= 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[1]), &dxl_error);
                }
                printf("Servo %d \n", dxl_id);
                printf("present pose: %f", climbing::tics2deg(dxl_present_position));
                printf("goal pose: %f", increment_positions[1]);
            } while ((abs(hand_up_positions[1] - climbing::tics2deg(dxl_present_position)) > (DXL_MOVING_STATUS_THRESHOLD)));
            printf("ENTER PRESSED Twice\n");
        }

        // press to go to lifting the arm
        if (getch() == ENTER_ASCII_VALUE)
        {   increment_positions = hand_up_positions;
            // then servo 2
            dxl_id = 3;
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[2]), &dxl_error);
            do
            {
                // Read the Present Position
                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
                double diff = lift_arm_positions[2] - climbing::tics2deg(dxl_present_position);
                // if the present position isn't the set position then increase the increment position
                if (diff > 0)
                {
                    // add value to the increment
                    increment_positions[2] += 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[2]), &dxl_error);
                }
                else if (diff < 0)
                {
                    // add value to the increment
                    increment_positions[2] -= 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[2]), &dxl_error);
                }
                printf("Servo %d \n", dxl_id);
                printf("present pose: %f", climbing::tics2deg(dxl_present_position));
                printf("goal pose: %f", increment_positions[2]);
            } while ((abs(lift_arm_positions[2] - climbing::tics2deg(dxl_present_position)) > (DXL_MOVING_STATUS_THRESHOLD)));
            printf("ENTER PRESSED Three\n");
        }
        if (getch() == ENTER_ASCII_VALUE){
        // {   increment_positions = hand_up_positions;
            // then servo 2
            dxl_id = 1;
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[0]), &dxl_error);
            do
            {
                // Read the Present Position
                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
                double diff = lift_arm_positions[0] - climbing::tics2deg(dxl_present_position);
                // if the present position isn't the set position then increase the increment position
                if (diff > 0)
                {
                    // add value to the increment
                    increment_positions[0] += 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[0]), &dxl_error);
                }
                else if (diff < 0)
                {
                    // add value to the increment
                    increment_positions[0] -= 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[0]), &dxl_error);
                }
                printf("Servo %d \n", dxl_id);
                printf("present pose: %f", climbing::tics2deg(dxl_present_position));
                printf("goal pose: %f", increment_positions[0]);
            } while ((abs(lift_arm_positions[0] - climbing::tics2deg(dxl_present_position)) > (DXL_MOVING_STATUS_THRESHOLD)));
            printf("ENTER PRESSED Four\n");
        }
        if (getch() == ENTER_ASCII_VALUE){
            // then servo 2
            dxl_id = 2;
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[1]), &dxl_error);
            do
            {
                // Read the Present Position
                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
                double diff = lift_arm_positions[1] - climbing::tics2deg(dxl_present_position);
                // if the present position isn't the set position then increase the increment position
                if (diff > 0)
                {
                    // add value to the increment
                    increment_positions[1] += 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[1]), &dxl_error);
                }
                else if (diff < 0)
                {
                    // add value to the increment
                    increment_positions[1] -= 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[1]), &dxl_error);
                }
                printf("Servo %d \n", dxl_id);
                printf("present pose: %f", climbing::tics2deg(dxl_present_position));
                printf("goal pose: %f", increment_positions[1]);
            } while ((abs(lift_arm_positions[1] - climbing::tics2deg(dxl_present_position)) > (DXL_MOVING_STATUS_THRESHOLD)));
            printf("ENTER PRESSED Five\n");
        }
        // press to go to lower arm to bar
        if (getch() == ENTER_ASCII_VALUE)
        {   increment_positions = lift_arm_positions;
            // then servo 2
            dxl_id = 3;
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[2]), &dxl_error);
            do
            {
                // Read the Present Position
                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
                double diff = lower_arm_positions[2] - climbing::tics2deg(dxl_present_position);
                // if the present position isn't the set position then increase the increment position
                if (diff > 0)
                {
                    // add value to the increment
                    increment_positions[2] += 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[2]), &dxl_error);
                }
                else if (diff < 0)
                {
                    // add value to the increment
                    increment_positions[2] -= 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[2]), &dxl_error);
                }
                printf("Servo %d \n", dxl_id);
                printf("present pose: %f", climbing::tics2deg(dxl_present_position));
                printf("goal pose: %f", increment_positions[2]);
            } while ((abs(lower_arm_positions[2] - climbing::tics2deg(dxl_present_position)) > (DXL_MOVING_STATUS_THRESHOLD)));
            printf("ENTER PRESSED Six\n");
        }
        // press to go to a second set position
        if (getch() == ENTER_ASCII_VALUE)
        {   increment_positions = lower_arm_positions;
            // then servo 2
            dxl_id = 1;
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[0]), &dxl_error);
            do
            {
                // Read the Present Position
                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
                double diff = second_set_positions[0] - climbing::tics2deg(dxl_present_position);
                // if the present position isn't the set position then increase the increment position
                if (diff > 0)
                {
                    // add value to the increment
                    increment_positions[0] += 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[0]), &dxl_error);
                }
                else if (diff < 0)
                {
                    // add value to the increment
                    increment_positions[0] -= 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[0]), &dxl_error);
                }
                printf("Servo %d \n", dxl_id);
                printf("present pose: %f", climbing::tics2deg(dxl_present_position));
                printf("goal pose: %f", increment_positions[0]);
            } while ((abs(second_set_positions[0] - climbing::tics2deg(dxl_present_position)) > (DXL_MOVING_STATUS_THRESHOLD)));
            printf("ENTER PRESSED Seven\n");
        }
        if (getch() == ENTER_ASCII_VALUE){
            // then servo 2
            dxl_id = 4;
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[3]), &dxl_error);
            do
            {
                // Read the Present Position
                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
                double diff = second_set_positions[3] - climbing::tics2deg(dxl_present_position);
                // if the present position isn't the set position then increase the increment position
                if (diff > 0)
                {
                    // add value to the increment
                    increment_positions[3] += 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[3]), &dxl_error);
                }
                else if (diff < 0)
                {
                    // add value to the increment
                    increment_positions[3] -= 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[3]), &dxl_error);
                }
                printf("Servo %d \n", dxl_id);
                printf("present pose: %f", climbing::tics2deg(dxl_present_position));
                printf("goal pose: %f", increment_positions[3]);
            } while ((abs(second_set_positions[3] - climbing::tics2deg(dxl_present_position)) > (DXL_MOVING_STATUS_THRESHOLD)));
            printf("ENTER PRESSED Eight\n");
        }
        // press to go to lift the second arm
        if (getch() == ENTER_ASCII_VALUE)
        {   increment_positions = second_set_positions;
            // then servo 2
            dxl_id = 1;
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[0]), &dxl_error);
            do
            {
                // Read the Present Position
                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
                double diff = lift_second_arm_positions[0] - climbing::tics2deg(dxl_present_position);
                // if the present position isn't the set position then increase the increment position
                if (diff > 0)
                {
                    // add value to the increment
                    increment_positions[0] += 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[0]), &dxl_error);
                }
                else if (diff < 0)
                {
                    // add value to the increment
                    increment_positions[0] -= 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[0]), &dxl_error);
                }
                printf("Servo %d \n", dxl_id);
                printf("present pose: %f", climbing::tics2deg(dxl_present_position));
                printf("goal pose: %f", increment_positions[0]);
            } while ((abs(lift_second_arm_positions[0] - climbing::tics2deg(dxl_present_position)) > (DXL_MOVING_STATUS_THRESHOLD)));
            printf("ENTER PRESSED Seven\n");
        }
        if (getch() == ENTER_ASCII_VALUE)
        {
            dxl_id = 3;
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[2]), &dxl_error);
            do
            {
                // Read the Present Position
                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
                double diff = lift_second_arm_positions[2] - climbing::tics2deg(dxl_present_position);
                // if the present position isn't the set position then increase the increment position
                if (diff > 0)
                {
                    // add value to the increment
                    increment_positions[2] += 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[2]), &dxl_error);
                }
                else if (diff < 0)
                {
                    // add value to the increment
                    increment_positions[2] -= 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[2]), &dxl_error);
                }
                printf("Servo %d \n", dxl_id);
                printf("present pose: %f", climbing::tics2deg(dxl_present_position));
                printf("goal pose: %f", increment_positions[2]);
            } while ((abs(lift_second_arm_positions[2] - climbing::tics2deg(dxl_present_position)) > (DXL_MOVING_STATUS_THRESHOLD)));
            printf("ENTER PRESSED Six\n");
        }
        if (getch() == ENTER_ASCII_VALUE){
            // then servo 2
            dxl_id = 4;
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[3]), &dxl_error);
            do
            {
                // Read the Present Position
                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
                double diff = lift_second_arm_positions[3] - climbing::tics2deg(dxl_present_position);
                // if the present position isn't the set position then increase the increment position
                if (diff > 0)
                {
                    // add value to the increment
                    increment_positions[3] += 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[3]), &dxl_error);
                }
                else if (diff < 0)
                {
                    // add value to the increment
                    increment_positions[3] -= 1;
                    // rewrite goal position
                    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(increment_positions[3]), &dxl_error);
                }
                printf("Servo %d \n", dxl_id);
                printf("present pose: %f", climbing::tics2deg(dxl_present_position));
                printf("goal pose: %f", increment_positions[3]);
            } while ((abs(lift_second_arm_positions[3] - climbing::tics2deg(dxl_present_position)) > (DXL_MOVING_STATUS_THRESHOLD)));
            printf("ENTER PRESSED Eight\n");
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