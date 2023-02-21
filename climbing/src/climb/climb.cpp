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
#define ADDR_TORQUE_ENABLE          64
#define ADDR_GOAL_POSITION          116
#define ADDR_PRESENT_POSITION       132
#define MINIMUM_POSITION_LIMIT      -1000  // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT      4095  // Refer to the Maximum Position Limit of product eManual
#define BAUDRATE                    57600
#define ADDE_PRESENT_CURRENT        126

#define PROTOCOL_VERSION  2.0

#define DEVICENAME  "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MOVING_STATUS_THRESHOLD     50  // DYNAMIXEL moving status threshold
#define ESC_ASCII_VALUE                 0x1b

#define NUMB_OF_DYNAMIXELS              4

// initialize the amount to increment the position by in degrees
float position_increment = 1.0;

// initialize a list of goal positions for each servo
// std::vector<float> servo_positions = {2027.0, 0.0, 2027.0, 0.0};
std::vector<float> servo_positions(4);

// Map for movement keys
std::map<char, std::vector<float>> moveBindings
{
  {'w', {1}},
  {'a', {-1}},
  {'s', {-1}},
  {'d', {1}},
  {'i', {-1}},
  {'j', {-1}},
  {'k', {1}},
  {'l', {1}},
};

// Map for servo controls, position of servo in servo_positions vector (add 1 for the actual servo id)
std::map<char, std::vector<float>> servoBindings
{
  {'w', {1}},
  {'a', {0}},
  {'s', {1}},
  {'d', {0}},
  {'i', {3}},
  {'j', {2}},
  {'k', {3}},
  {'l', {2}},
};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'q', {1.0}},
  {'z', {-1.0}},
};

// Reminder message
const char* msg = R"(
To move the left arm use these bindings:
---------------------------
Moving around:
        w     
   a    d    s
              
To move the right arm use these bindings:
---------------------------
        i     
   j    k    l
              

q/z : increase/decrease max speeds of all servos by 10%

CTRL-C to quit
)";

char key(' ');
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

int main(){
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // DYNAMIXEL err

    int32_t dxl_present_position = 0;  // Read 4 byte Position data
    int32_t dxl_present_current = 0;  // Read 4 byte Position data
    int32_t last_pos = 0;

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

    // Enable DYNAMIXEL Torque
    for (int i = 1; i <= NUMB_OF_DYNAMIXELS; i++){
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else {
            printf("Succeeded enabling DYNAMIXEL Torque for servo ID %d \n", i);
        }
    }

    // For each servo, add its current position to the servo pose 
    for (int i = 1; i <= NUMB_OF_DYNAMIXELS; i++){
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        printf("[ID:%03d] Present Position:%03f\n", i, climbing::tics2deg(dxl_present_position));
        servo_positions[i-1] = climbing::tics2deg(dxl_present_position);
    }

    std::cout << "waiting for key inputs... \n";
    while(true){
        key = getch();
        // check if the pressed key is one of my bindings
        if (moveBindings.count(key)==1){
            // set the last position to be the current position of the servo
            last_pos = servo_positions.at(servoBindings[key][0]);
            // get the id of the dynamixel being commanded
            dxl_id = servoBindings[key][0] + 1;
            // update the servo position to be incremented by the value of the increment * the movebinding
            servo_positions.at(servoBindings[key][0]) += position_increment * moveBindings[key][0];
            // set the goal pose to be the new value
            goal_pose = climbing::deg2tics(servo_positions.at(servoBindings[key][0]));
            printf("servo number: %f, servo position: %f \n", servoBindings[key][0]+1.0, servo_positions.at(servoBindings[key][0]));
        }
        // check if a speed binding was pressed
        else if (speedBindings.count(key) == 1){
            position_increment += .5 * speedBindings[key][0];
            printf("speed binding: %f \n", position_increment);
        }
        // do nothing if an unbound key was pressed
        else{
            key = ' ';
        }
        // quit if ctll-C is pressed
        if (key == '\x03')
        {
            printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
            break;
        }

        // Write goal position
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, goal_pose, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        int give_up = 0;


        do {
        // Read the Present Position
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        printf("[ID:%03d] Goal Position:%03f  Present Position:%03f\n", dxl_id, climbing::tics2deg(goal_pose), climbing::tics2deg(dxl_present_position));

        // if the current present position is the same as it was last time then increment give up
        // printf("last_pos: %d current pose: %f   " , last_pos, climbing::tics2deg(dxl_present_position));
        if (last_pos == static_cast<int>(climbing::tics2deg(dxl_present_position))){
            give_up++;
        }
        // set last pose to be the current position
        last_pos = climbing::tics2deg(dxl_present_position);
        }  while((abs(goal_pose - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD) & (give_up < 3));

        printf("Present position of each servo:\n");
        // For each servo print its present position
        for (int i = 1; i <= NUMB_OF_DYNAMIXELS; i++){
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0) {
                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }
            printf("[ID:%03d] Present Position:%03f\n", i, climbing::tics2deg(dxl_present_position));
        }
    }

    // Disable DYNAMIXEL Torque
    for (int j = 1; j <= NUMB_OF_DYNAMIXELS; j++){
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, j, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else {
            printf("Succeeded disabling DYNAMIXEL Torque for servo id %d.\n", j);
        }
    }

    // Close port
    portHandler->closePort();
    return 0;
}