#ifndef CONTROL_DYN_GAURD_HPP
#define CONTROL_DYN_GAURD_HPP
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define DXL_MOVING_STATUS_THRESHOLD .5 // DYNAMIXEL moving status threshold
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
    void move_servos(int dxl_id, double servo_poses, int dxl_comm_result, dynamixel::PacketHandler *&packetHandler, dynamixel::PortHandler *&portHandler, uint32_t dxl_present_position, uint8_t &dxl_error)
    {
    // set increment poses to be servo poses
    double increment_poses = servo_poses;
    // set goal position to be where we are currently so it can be slowly incremented in the right direction
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, climbing::deg2tics(servo_poses), &dxl_error);
    // Read the Present Position
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
    // set last position to be the current position
    double last_pose = dxl_present_position;
    do
    {
        // Read the Present Position
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
        if (dxl_present_position == last_pose){
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
        }
        // printf("[ID:%03d] Servo Pose:%03f\n", dxl_id, servo_poses);
        // printf("[ID:%03d] Present Position:%03f\n", dxl_id, climbing::tics2deg(dxl_present_position));
        last_pose = dxl_present_position;
    } while ((abs(servo_poses - climbing::tics2deg(dxl_present_position)) > (DXL_MOVING_STATUS_THRESHOLD)));
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
        printf("you");
    }
    printf("swedrfgyh");
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
        // std::vector<float> initial_positions(4); 
        // initial_positions = set_current_pose(dxl_comm_result, initial_positions, packetHandler, portHandler, dxl_present_position, dxl_error);
        // std::vector<float> wake_up_poses(4);
        // // get all the servos to a close enough position
        // for (int i = 1; i <= (int)size(initial_positions); i ++){
        //     wake_up_poses.at(i - 1) = initial_positions.at(i - 1) + .15;
        //     move_servos(i, wake_up_poses.at(i - 1), initial_positions.at(i - 1), dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // }
        // move_servos(3, 218, wake_up_poses[2], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move_servos(4, 178, wake_up_poses[3], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move_servos(1, 224, wake_up_poses[0], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move_servos(2, 180, wake_up_poses[1], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // slightly raise the left hand off the bar
        move_servos(3, 200, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // press the left hand in to lock the right one
        move_servos(2, 196, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //////// raise the left hand /////////
        // lift the left arm with the right one until its clear
        printf("HELLO???");
        move_servos(3, 165, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // pull the left arm back
        move_servos(2, 170, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // rotate the left arm (partially)
        move_servos(1, 150, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // roatate the body out 
        move_servos(4, 182, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // lift the left arm to the bar
        move_servos(3, 110, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // finish rotating the arm
        move_servos(1, 138, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //////// grab the next bar /////////
        // move the hand forward
        move_servos(2, 186, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move the left arm down with the right arm
        move_servos(3, 130, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
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
        /////// lift the right arm //////////
        printf("one");
        // roll the right arm back 
        move_servos(4, 186, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("two");
        // move both shoulders down 
        move_servos(3, 138, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("three");
        move_servos(2, 185, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("four");
        // pull arm up a little
        move_servos(1, 140, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("five");
        move_servos(4, 184, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        printf("six");
        // release
        move_servos(1, 164, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // printf("seven");
        // // push in with right arm
        // move_servos(4, 180, 184, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // printf("eight");
        // // move body out
        // move_servos(2, 182, 195, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // printf("nine");
        // // move hand up 
        // move_servos(1, 180, 164, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // printf("ten");
        // // move hand back move_servos(2, 195, 182, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // printf("eleven");
        // move_servos(4, 193, 179.5, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // printf("twelve");
        // // rotate hand 
        // move_servos(3, 185, 138, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // printf("thirteen");
        // // move arm all the way up
        // move_servos(1, 248, 183, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // printf("fourteen");
        // // move the body out
        // // move_servos(2, 182, 178, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // // printf("?!");
        // // rotate hand 
        // move_servos(3, 220, 185, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // printf("fifteen");
        // // move hand forward
        // move_servos(4, 179, 193, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // printf("sixteen");
        // // move hand down to bar
        // move_servos(1, 225, 249, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // printf("seventeen");
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

    void right_down(int dxl_comm_result, dynamixel::PacketHandler *&packetHandler, dynamixel::PortHandler *&portHandler, uint32_t dxl_present_position, uint8_t &dxl_error){
        // std::vector<float> initial_positions(4);
        // initial_positions = set_current_pose(dxl_comm_result, initial_positions, packetHandler, portHandler, dxl_present_position, dxl_error);
        // // wake up poses move each servo a little before moving to their set positions to get rid of jumpiness
        // std::vector<float> wake_up_poses(4);
        // for (int i = 1; i <= (int)size(initial_positions); i ++){
        // wake_up_poses.at(i - 1) = initial_positions.at(i - 1) + .15;
        // move_servos(i, wake_up_poses.at(i - 1), initial_positions.at(i - 1), dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // }
        // ////// MOVE RIGHT ARM DOWN ///////
        // move_servos(3, 211, wake_up_poses[2], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move_servos(4, 181, wake_up_poses[3], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move_servos(1, 217, wake_up_poses[0], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move_servos(2, 183, wake_up_poses[1], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // lift right arm a little
        move_servos(1, 239, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // wiggle!
        move_servos(3, 220, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(1, 245, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(3, 226, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move_servos(1, 248, 241, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move the arm out of the ladder
        move_servos(4, 202, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move body out
        move_servos(2, 178, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        //move arm over
        move_servos(3, 145, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move the arm down 
        move_servos(1, 175, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move arm in 
        move_servos(4, 181, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // drop arm to the bar
        move_servos(1, 145, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // fit hand onto bar
        move_servos(3, 140, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
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

    void left_down(int dxl_comm_result, dynamixel::PacketHandler *&packetHandler, dynamixel::PortHandler *&portHandler, uint32_t dxl_present_position, uint8_t &dxl_error){
        // std::vector<float> initial_positions(4);
        // initial_positions = set_current_pose(dxl_comm_result, initial_positions, packetHandler, portHandler, dxl_present_position, dxl_error);
        // // wake up poses move each servo a little before moving to their set positions to get rid of jumpiness
        // std::vector<float> wake_up_poses(4);
        // for (int i = 1; i <= (int)size(initial_positions); i ++){
        // wake_up_poses.at(i - 1) = initial_positions.at(i - 1) + .15;
        // move_servos(i, wake_up_poses.at(i - 1), initial_positions.at(i - 1), dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // }
        // ////// MOVE LEFT ARM DOWN ///////
        // move_servos(3, 131, wake_up_poses[2], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move_servos(4, 178, wake_up_poses[3], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move_servos(1, 141, wake_up_poses[0], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move_servos(2, 183, wake_up_poses[1], dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // secure the right hand 
        move_servos(4, 170, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // slightly release the left hand
        move_servos(3, 120, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // wiggle the servos to let go
        move_servos(1, 127, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(3, 116.5, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(2, 170,  dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(1, 123, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(2, 178, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        move_servos(3, 107, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // pull the left hand out of the ladder 
        move_servos(2, 165, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // rotate the body (for safety)
        move_servos(4, 182, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // rotate the arm around
        move_servos(1, 210, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // drop the left arm down
        move_servos(3, 190, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // move left arm in
        move_servos(2, 178, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // drop the left down to bar
        move_servos(3, 210, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
        // spin arm all the way arond
        move_servos(1, 220, dxl_comm_result, packetHandler, portHandler, dxl_present_position, dxl_error);
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