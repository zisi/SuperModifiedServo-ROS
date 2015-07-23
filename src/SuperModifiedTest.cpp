#include "ros/ros.h"
extern "C"{
	#include "ZerooneSupermodified.h" 
}
#include "super_modified_servo/Command.h"

void smsCallBack(super_modified_servo::Command req)
{
	/* Test 1 */
	if (req.cmd == "b1")
	{
		int fd = serialPortOpen("/dev/ttyUSB0");
		if( !getCommunicationSuccess() )
    		printf("Communication Warning :%d\n", getWarning());
		sleep(1);
		printf("Start\n");
		start(fd, 0x04);
		if( !getCommunicationSuccess() )
    		printf("Communication Warning :%d\n", getWarning());
		sleep(1);
		printf("Reset incremental position\n");
		resetIncrementalPosition(fd, 0x04);
		sleep(1);
		printf("Get position: %ld\n", getPosition(fd, 0x04));
		sleep(1);
		printf("Move to absolute position: 1000 \n");
		moveToAbsolutePosition(fd, 0x04, 1000);
		sleep(1);
		if( !getCommunicationSuccess() )
    		printf("Communication Warning :%d\n", getWarning());
		printf("Get position: %ld\n", getPosition(fd, 0x04));
		sleep(1);
		printf("Move position: -3000\n");
		moveToAbsolutePosition(fd, 0x04, -3000);
		sleep(1);
		printf("Get position: %ld\n", getPosition(fd, 0x04));
		sleep(1);
		printf("Move position: 10000\n");
		moveToAbsolutePosition(fd, 0x04, 10000);
		sleep(1);
		printf("Get position: %ld\n", getPosition(fd, 0x04));
		sleep(1);
		printf("Stop\n");
		stop(fd, 0x04);
		serialPortClose(fd);
	}
	/* Test 2 */
	else if(req.cmd == "b2")
	{
		int fd = serialPortOpen("/dev/ttyUSB0");
		if( !getCommunicationSuccess() )
    		printf("Communication Warning :%d\n", getWarning());
		sleep(1);
		printf("Start\n");
		start(fd, 0x04);
		if( !getCommunicationSuccess() )
    		printf("Communication Warning :%d\n", getWarning());
		sleep(1);
		printf("Move with velocity: 40000\n");
		moveWithVelocity(fd, 0x04, 40000);
		sleep(5);
		printf("Get current: %d\n",getCurrent(fd, 0x04));
		sleep(1);
		printf("Get velocity: %d\n", getVelocity(fd, 0x04));
		sleep(1);
		printf("Move with velocity: -40000\n");
		moveWithVelocity(fd, 0x04, -40000);
		sleep(5);
		printf("Get current: %d\n",getCurrent(fd, 0x04));
		sleep(1);
		printf("Get velocity: %d\n", getVelocity(fd, 0x04));
		sleep(1);
		printf("Stop\n");
		stop(fd, 0x04);
		sleep(1);
		serialPortClose(fd);
	}
	/* Test 3 */
	else if(req.cmd == "b3")
	{
		int fd = serialPortOpen("/dev/ttyUSB0");
		if( !getCommunicationSuccess() )
    		printf("Communication Warning :%d\n", getWarning());
		sleep(1);
		printf("Start\n");
		start(fd, 0x04);
		if( !getCommunicationSuccess() )
    		printf("Communication Warning :%d\n", getWarning());
		sleep(1);
		printf("Get position: %ld\n", getPosition(fd, 0x04));
		sleep(5);
		printf("Get position: %ld\n", getPosition(fd, 0x04));
		sleep(5);
		printf("Get position: %ld\n", getPosition(fd, 0x04));
		sleep(5);
		printf("Stop\n");
		stop(fd, 0x04);
		sleep(1);
		serialPortClose(fd);
		;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "super_modified_servo");
  	ros::NodeHandle n;
  	ros::Subscriber sub = n.subscribe("Command", 1000, smsCallBack);
  	ros::spin();
  	return 0;
}