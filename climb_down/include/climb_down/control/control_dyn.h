#ifndef CONTROL_DYN_GAURD_HPP
#define CONTROL_DYN_GAURD_HPP
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define DXL_MOVING_STATUS_THRESHOLD .15 // DYNAMIXEL moving status threshold
#define NUMB_OF_DYNAMIXELS 4
#define ADDR_TORQUE_ENABLE 64
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

#include <cmath>
#include <iostream>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk/port_handler.h"
#include "dynamixel_sdk/port_handler_linux.h"

namespace climbing
{
    /// @brief - change degrees into command tics
    /// @param deg - number of degrees to convert
    /// @return - the equivilant number of tics
    constexpr uint32_t deg2tics(double deg){
        return static_cast<uint32_t>(deg*(4095/360));
    }

    /// @brief - convert command tics into degrees
    /// @param tics - number of tics to convert
    /// @return - the equivilant number of degrees
    double tics2deg(uint32_t tics){
        return static_cast<double>(tics)*(360.0/4095.0);
    }

    /// @brief - function to move a servo from one position to another
    /// @param dxl_id - the id of the servo to move
    /// @param servo_poses - the goal position of the servo
    /// @param increment_poses - the starting position of te servo
    /// @param dxl_comm_result - current comm result
    /// @param packetHandler - the dxl packetHandler
    /// @param portHandler - the dxl portHandler
    /// @param dxl_present_position - the current position of the servo
    /// @param dxl_error - the error of the servo
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
            printf("Move done!\n");
    }

    std::vector<float> set_current_pose(int dxl_comm_result, std::vector<float> initial_positions, dynamixel::PacketHandler *&packetHandler, dynamixel::PortHandler *&portHandler, uint32_t dxl_present_position, uint8_t &dxl_error)
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
    return initial_positions;
    }

    /// @brief call to disable the torque of a dynamixel
    /// @param j - the id of the servo (usually done in a for loop)
    /// @param dxl_comm_result - current comm result
    /// @param packetHandler - the dxl packetHandler
    /// @param portHandler - the dxl portHandler
    /// @param dxl_error - the dxl error
    void disable_torque(int j ,int dxl_comm_result, dynamixel::PacketHandler *&packetHandler, dynamixel::PortHandler *&portHandler,  uint8_t &dxl_error){
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

    /// @brief call to enable the ttoque of a dynamixel
    /// @param i - the id of the servo (usually done in a for loop)
    /// @param dxl_comm_result - current comm result
    /// @param packetHandler - the dxl packetHandler
    /// @param portHandler - the dxl portHandler
    /// @param dxl_error - the dxl error
    void enable_torque(int i ,int dxl_comm_result, dynamixel::PacketHandler *&packetHandler, dynamixel::PortHandler *&portHandler,  uint8_t &dxl_error){
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

    /// @brief call to make the left leg climb up a run
    /// @param dxl_comm_result - current comm result
    /// @param packetHandler - the dxl packetHandler
    /// @param portHandler - the dxl portHandler
    /// @param dxl_present_position - the current position of the servo
    /// @param dxl_error - the dxl error
    void left_climb(int dxl_comm_result, dynamixel::PacketHandler *&packetHandler, dynamixel::PortHandler *&portHandler, uint32_t dxl_present_position, uint8_t &dxl_error){
        std::vector<float> initial_positions(4); 
        initial_positions = set_current_pose(dxl_comm_result, initial_positions, packetHandler, portHandler, dxl_present_position, dxl_error);
        std::vector<float> wake_up_poses(4);
        // get all the servos to a close enough position
        for (int i = 1; i <= (int)size(initial_positions); i ++){
            wake_up_poses.at(i - 1) = initial_positions.at(i - 1) + .15;
            move_servos(i, wake_up_poses.at(i - 1), initial_positions.at(i - 1), dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }
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
        // Disable DYNAMIXEL Torque
        for (int j = 1; j <= NUMB_OF_DYNAMIXELS; j++)
        {
            disable_torque(j ,dxl_comm_result, packetHandler, portHandler,  dxl_error);
        }
        // Enable DYNAMIXEL Torque
        for (int i = 1; i <= NUMB_OF_DYNAMIXELS; i++)
        {
            enable_torque(i ,dxl_comm_result, packetHandler, portHandler,  dxl_error);
        }
    }

    /// @brief call to make the right arm climb up a run
    /// @param dxl_comm_result - current comm result
    /// @param packetHandler - the dxl packetHandler
    /// @param portHandler - the dxl portHandler
    /// @param dxl_present_position - the current position of the servo
    /// @param dxl_error - the dxl error
    void right_climb(int dxl_comm_result, dynamixel::PacketHandler *&packetHandler, dynamixel::PortHandler *&portHandler, uint32_t dxl_present_position, uint8_t &dxl_error){
        std::vector<float> initial_positions(4);
        initial_positions = set_current_pose(dxl_comm_result, initial_positions, packetHandler, portHandler, dxl_present_position, dxl_error);
        std::vector<float> wake_up_poses(4);
        // get all the servos to a close enough position
        for (int i = 1; i <= (int)size(initial_positions); i ++){
            wake_up_poses.at(i - 1) = initial_positions.at(i - 1) + .15;
            move_servos(i, wake_up_poses.at(i - 1), initial_positions.at(i - 1), dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        }
        /////// lift the right arm //////////
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
        move_servos(4, 193, 179.5, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("push???");
        // rotate hand 
        move_servos(3, 185, 138, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("go??");
        // move arm all the way up
        move_servos(1, 255, 183, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("?!");
        // rotate hand 
        move_servos(3, 213, 185, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("ah??");
        // move hand forward
        move_servos(4, 178, 193, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("end??");
        // move hand down to bar
        move_servos(1, 235, 256, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("?!?!??");
        // Disable DYNAMIXEL Torque
        for (int j = 1; j <= NUMB_OF_DYNAMIXELS; j++)
        {
            disable_torque(j ,dxl_comm_result, packetHandler, portHandler,  dxl_error);
        }
        // Enable DYNAMIXEL Torque
        for (int i = 1; i <= NUMB_OF_DYNAMIXELS; i++)
        {
            enable_torque(i ,dxl_comm_result, packetHandler, portHandler,  dxl_error);
        }
    }
}

#endif