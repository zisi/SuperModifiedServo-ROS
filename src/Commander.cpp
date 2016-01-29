/*
  ZeroOne Supermodified Controller API for ROS.
  Copyright (c) 2015 Agisilaos Zisimatos.   All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "ros/ros.h"
extern "C"{
    #include "ZerooneSupermodified.h"
}
#include <dynamic_reconfigure/server.h>
#include <super_modified_servo/commanderConfig.h>
#include "sensor_msgs/JointState.h"

#define Max_Absolute_Position 32768
#define MaxNumSMS 10

double ticks2deg(signed int);
signed int deg2ticks(double);

int fd;
int motor_id;
int new_motor_id;
int baudRate;
uint16_t kp;
uint16_t ki;
uint16_t kd;
double set_point;
bool start_flag = false;

void CommanderCallBack(super_modified_servo::commanderConfig &config, uint32_t level)
{
    if (config.command == "search")
    {
        for (motor_id = 4; motor_id < MaxNumSMS; motor_id++)
        {
            start(fd, motor_id);
            sleep(1);
            printf("Start Node ID %d\n", motor_id);
            if (getCommunicationSuccess())
                printf("ID is %d\n", motor_id);
            resetErrors(fd, motor_id);
            sleep(1);
            stop(fd, motor_id);
            printf("Stop Node ID %d\n", motor_id);
        }
    }
    if (config.command == "start")
    {
        printf("[INFO] Start\n");
        start(fd, motor_id);
        start_flag = true;
    }
    if (config.command == "stop")
    {
        printf("[INFO] Stop\n");
        stop(fd, motor_id);
        start_flag = false;
    }
    if (config.command == "reset")
    {
        printf("[INFO] Reset\n");
        stop(fd, motor_id);
        sleep(1);
        start(fd, motor_id);
    }
    if (config.command == "setID")
    {
        printf("[INFO] Set ID: %d\n", new_motor_id);
        setNodeId(fd, motor_id, new_motor_id);
        printf("[INFO] Disconnect the servo\n");
    }
    if (config.command == "setBaud")
    {
        printf("[INFO] Set baud rate: %d\n", baudRate);
        setBaudRate(fd, motor_id, baudRate);
        printf("[INFO] Disconnect the servo\n");
        printf("[INFO] Change the bad rate in serialPortOpen function");
    }
    if (config.command == "error_reaction")
    {
        printf("[INFO] Error reaction\n");
        uint8_t resp[20];
        uint8_t errorReaction[20] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02,
             0x02, 0x01};
        setErrorReaction(fd, motor_id, errorReaction);
        sleep(3);
        if (getCommunicationSuccess() == false)
        {
            resetErrors(fd, motor_id);;
            printf("[ERROR]: %d \n", getWarning());
        }
        getErrorReaction(fd, motor_id, resp);
        for (int i=0; i<20; i++)
            printf("D[%d]: %d\n", i, resp[i]);
    }
    if (config.command == "setGain")
    {
        printf("[INFO] Set Gain: \n");
        setPGain(fd, motor_id, kp);
        setIGain(fd, motor_id, ki);
        setDGain(fd, motor_id, kd);
        printf("[INFO] kp %d \n", getPGain(fd, motor_id));
        printf("[INFO] ki %d \n", getIGain(fd, motor_id));
        printf("[INFO] kd %d \n", getDGain(fd, motor_id));
    }
    if (config.command == "getGain")
    {
        printf("[INFO] kp %d \n", getPGain(fd, motor_id));
        printf("[INFO] ki %d \n", getIGain(fd, motor_id));
        printf("[INFO] kd %d \n", getDGain(fd, motor_id));
    }

    motor_id = config.motor_id;
    new_motor_id = config.new_motor_id;
    baudRate = config.baudRate;
    kp = config.p_gain;
    ki = config.i_gain;
    kd = config.d_gain;
    set_point = config.set_point;
}

double ticks2deg(signed int ticks)
{
    return (double)360*ticks/(double)Max_Absolute_Position;
}

signed int deg2ticks(double deg)
{
    return (signed int)deg*Max_Absolute_Position/360;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "commander");
    dynamic_reconfigure::Server<super_modified_servo::commanderConfig> srv;
    dynamic_reconfigure::Server<super_modified_servo::commanderConfig>::CallbackType f;
    f = boost::bind(&CommanderCallBack, _1, _2);
    srv.setCallback(f);

    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/JointsState", 1000);
    ros::Rate loop_rate(200);

    fd = serialPortOpen("/dev/ttyUSB0", B115200);
    sensor_msgs::JointState joint_state_msg;

    char motor_name[] = "Motor";
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name.push_back(motor_name);
    joint_state_msg.position.push_back(0.0);
    joint_state_msg.velocity.push_back(0.0);
    joint_state_msg.effort.push_back(0.0);

    while(ros::ok())
    {
        if (start_flag == true)
        {
            if (getCommunicationSuccess() == false)
            {
                resetErrors(fd, motor_id);;
                printf("[ERROR]: %d \n", getWarning());
            }
            joint_state_msg.header.stamp = ros::Time::now();
            sprintf(motor_name, "ID:%d", motor_id);
            joint_state_msg.name[0] = motor_name;
            joint_state_msg.position[0] = ticks2deg(getPosition(fd, motor_id));
            joint_state_msg.velocity[0] = ticks2deg(getVelocity(fd, motor_id));
            joint_state_msg.effort[0] = getCurrent(fd, motor_id);
            joint_pub.publish(joint_state_msg);
            // profiledMoveToAbsolutePosition(fd, motor_id, deg2ticks(set_point));
            // moveToAbsolutePosition(fd, motor_id, deg2ticks(set_point));
            moveWithVelocity(fd, motor_id, deg2ticks(set_point));
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    serialPortClose(fd);
    return 0;
}
