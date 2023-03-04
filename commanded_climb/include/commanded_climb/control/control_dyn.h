#ifndef CONTROL_DYN_GAURD_HPP
#define CONTROL_DYN_GAURD_HPP
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define DXL_MOVING_STATUS_THRESHOLD .15 // DYNAMIXEL moving status threshold
#define NUMB_OF_DYNAMIXELS 4

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
        printf("HEY ITS HERE");
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
}

#endif