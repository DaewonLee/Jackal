#include <sched.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include "ros/ros.h"
#include <termios.h>
//#include <mavros_msgs/Mavlink.h>
//#include <mavros_msgs/Altitude.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int8.h"
#include <ros/callback_queue.h>
//#include <mavros_msgs/RCIn.h>
#include <fstream>
#include <sstream>
#include <geometry_msgs/Twist.h>
#define NUM_THREADS 2
#define PI 3.14159265359
#define STOP 0
#define WP 1
#define LOOK 2
#define MANUAL 3

#define lc_stop 0
#define lc_go 1
#define lc_heading 2
int lc_mode = lc_stop;

struct timeval t_init, t_now;
float  del_t, t, t_prev;
bool SYSTEM_RUN = true;
bool isFirst = true;
bool isStop = false;
float cmd_angular_from_joystick;
bool CTRL_MODE_MANUAL = true;
bool CTRL_MODE_WAYPOINT = false;
int cnt = 1000;	
int cnt_init = 0;
float psi_integral = 0.;
float psi_bias = 0.;
pthread_mutex_t data_acq_mutex;
pthread_cond_t data_acq_cv;
pthread_mutex_t mutex;

ros::Publisher chatter_pub;
ros::Subscriber px4_imu_subscriber;
ros::Subscriber encoder_subscriber;
ros::Subscriber GPS_subscriber;
ros::Subscriber GPS_WP_subscriber;
ros::Subscriber look_subscriber;
ros::Subscriber mode_subscriber;
int motorDir1 = 1;
int motorDir2 = -1;

float pos_encoder = 0.;
float vel_encoder = 0.;

float ugv_orientation[3];

double lonWP, latWP, lon, lat, lonlook, latlook;
int mode = STOP;

float heading_bias = 0.;
float px_orientation[3], px_gyro[3], initial_heading;
struct SState {
	float phi;
	float theta;
	float psi;
	
	float phi_dot;
	float theta_dot;
	float psi_dot;
	
	double cur_x;
	double cur_y;
	double cur_heading;
	
	double des_x;
	double des_y;
	double des_heading;
	double des_dist;
} ugv_state;

struct SCmd {
	float linear;
	float angular;
	float heading;
} cmd_vel;

struct SError {
	float p;
	float d;
	float i;
	SError()
	{
		p = 0.;
		d = 0.;
		i = 0.;
	}
};

struct STemp {
	float motorVel[2];
} control;

struct SError e_psi;
struct SError e_dist;


struct SGain {
	float p;
	float i;
	float d;
	SGain(float kp, float ki, float kd)
	{
		p = kp;
		i = ki;
		d = kd;
	}
}; 

struct SGain  k_psi(0.02, 0., 0.002);
struct SGain  k_dist(1.0, 0., 0.01);

void llToMeters(double lonn, double latt, double* x, double* y)
{
// Converts given lat/lon in WGS84 Datum to XY in Spherical Mercator EPSG:900913"
    double originShift = 2. * PI * 6378137. / 2.0; // 20037508.342789244
    *x = lonn * originShift / 180.;
    *y = log(tan((90. + latt) * PI / 360. )) / (PI / 180.);
    *y = *y * originShift / 180.;

}


int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );

    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}

void SatVal(float satval, float* val){

	if (*val > satval)
	    *val = satval;
	else if (*val < -satval)
	    *val = -satval;

}

void contrlPID(SError error, SGain K, float* output)
{
	*output = K.p * error.p + K.i * error.i + K.d * error.d;
}

void encoderCallback(const nav_msgs::Odometry& msg)
{
	float quat[4];
	vel_encoder = msg.twist.twist.linear.x;
	pos_encoder = msg.pose.pose.position.x;
	
	quat[0] = msg.pose.pose.orientation.w;
	quat[1] = msg.pose.pose.orientation.x;
	quat[2] = msg.pose.pose.orientation.y;
	quat[3] = msg.pose.pose.orientation.z;
	
	ugv_orientation[0] = atan2(2*(quat[0]*quat[1]+quat[2]*quat[3]), -quat[1]*quat[1] -quat[2]*quat[2] + quat[3]*quat[3] +quat[0]*quat[0]);
        ugv_orientation[1] = asin((2*quat[0]*quat[2])-(2*quat[3]*quat[1]));
        ugv_orientation[2] = atan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),(1-2*(quat[2]*quat[2]+quat[3]*quat[3])));
        
        ugv_state.psi = ugv_orientation[2]*180./PI - heading_bias;
}

void GPSCallback(const sensor_msgs::NavSatFix& msg)
{
	lon = msg.longitude;
	lat = msg.latitude;
}

void GPSWPCallback(const std_msgs::Float64MultiArray& msg)
{
	lonWP = msg.data[0];
	latWP = msg.data[1];

}

void lookCallback(const std_msgs::Float64MultiArray& msg)
{
	lonlook = msg.data[1];
	latlook = msg.data[0];

}

void modeCallback(const std_msgs::Int8& msg)
{
	mode = msg.data;
}



void *command_input(void *thread_id)
{
    unsigned char command_key;
    int stick_input = 10;
    while(SYSTEM_RUN == true) {
        command_key = getch();

	if (isspace(command_key))
	{
		printf("STOP!\n");
        	control.motorVel[0] = 0.;
        	control.motorVel[1] = 0.;
        	cmd_vel.linear = 0.;
        	cmd_vel.heading = ugv_state.psi;
        	isStop = true;
        	usleep(200000);
        	SYSTEM_RUN = false;
		
	}
	else if (command_key == 'r' || command_key == 'R')
        {
	    std::ifstream pathFile;
	    pathFile.open("/home/leegroup/Documents/groundrobot/onboard_computer/src/ugv_ctrl/src/path.txt");
	    int n = 0; //n is the number of the integers in the file ==> 12
	    int num;
	    double arr[100];
	    
	    //for(int i=0; i < 5 ; i++)
	    while (pathFile >> arr[n])
	    {
	    	//File >> arr[n];
        	n++;
		//pathFile >> arr[i];
		
	    }
	    pathFile.close();
	    for(int i=0; i < n ; i++)
		{
		   // std::cout << arr[i] << " ";
		}
	    
        	printf("[%d]Reading way-point file! \n",n);
	    double waypoints[n/2][2];
	    for(int k=0; k<n/2 ; k++)
	    {
	    	waypoints[k][0] = arr[2*k];
	    	waypoints[k][1] = arr[2*k+1];
	    //	printf("%e, %e\n",waypoints[k][0],waypoints[k][1]);
	    }
        	
        	
        }
	else if (command_key == 'm' || command_key == 'M')
        {

        	CTRL_MODE_MANUAL = true;
		CTRL_MODE_WAYPOINT = false;
        	printf("Manual Control Mode On!\n");
        }
        else if (command_key == 'n' || command_key == 'N')
        {
        	CTRL_MODE_MANUAL = false;
		CTRL_MODE_WAYPOINT = true;
        	printf("Waypoint Control Mode On!\n");
        }	
        else if (command_key == 'w' || command_key == 'W')
        {
        	if(CTRL_MODE_MANUAL == true)
        	{
			isStop = false;
			cmd_vel.linear += 0.05;
			printf("Increase Vel_d: %e\n", cmd_vel.linear);
        	}
        	else if (CTRL_MODE_WAYPOINT == true)
        	{
        		ugv_state.des_x = 1.0;
        		ugv_state.des_y = 0.0;
        		//convertXY2ANGLE();
        		printf("waypoint(%e, %e)\n",ugv_state.des_x, ugv_state.des_y);
        		printf("desired heading: %e, dist: %e\n\n",ugv_state.des_heading, ugv_state.des_dist);
        		
        	}


        }

        else if (command_key == 's' || command_key == 'S')
        {

        	if(CTRL_MODE_MANUAL == true)
        	{
			isStop = false;
			cmd_vel.linear -= 0.05;
			printf("Decrease Vel_d: %e\n", cmd_vel.linear);
		}
		else if (CTRL_MODE_WAYPOINT == true)
        	{
        		ugv_state.des_x = -1.0;
        		ugv_state.des_y = 0.0;
        		//convertXY2ANGLE();
        		printf("waypoint(%e, %e)\n",ugv_state.des_x, ugv_state.des_y);
        		printf("desired heading: %e, dist: %e\n\n",ugv_state.des_heading, ugv_state.des_dist);
        	}

        }
        else if (command_key == 'a' || command_key == 'A')
        {
        	if(CTRL_MODE_MANUAL == true)
        	{
			isStop = false;
			cmd_vel.heading += 5.;
			printf("Decrease Heading_d: %e\n",cmd_vel.heading);
		}
		else if (CTRL_MODE_WAYPOINT == true)
        	{
        		ugv_state.des_x = 0.0;
        		ugv_state.des_y = -1.0;
        		//convertXY2ANGLE();
        		printf("waypoint(%e, %e)\n",ugv_state.des_x, ugv_state.des_y);
        		printf("desired heading: %e, dist: %e\n\n",ugv_state.des_heading, ugv_state.des_dist);
        	}
        }
        else if (command_key == 'd' || command_key == 'D')
        {
        	if(CTRL_MODE_MANUAL == true)
        	{
			isStop = false;
			cmd_vel.heading -= 5.;
			printf("Decrease Heading_d: %e\n",cmd_vel.heading);
		}
		else if (CTRL_MODE_WAYPOINT == true)
        	{
        		ugv_state.des_x = 0.0;
        		ugv_state.des_y = 1.0;
        		//convertXY2ANGLE();
        		printf("waypoint(%e, %e)\n",ugv_state.des_x, ugv_state.des_y);
        		printf("desired heading: %e, dist: %e\n\n",ugv_state.des_heading, ugv_state.des_dist);
        	}
	}
        else if (command_key == 'e' || command_key == 'E')
        {
        	if(CTRL_MODE_MANUAL == true)
        	{
			printf("STOP!\n");
			control.motorVel[0] = 0.;
			control.motorVel[1] = 0.;
			cmd_vel.linear = 0.;
			cmd_vel.heading = ugv_state.psi;
			isStop = true;
		}
		else if (CTRL_MODE_WAYPOINT == true)
        	{
        		ugv_state.des_x = 1.0;
        		ugv_state.des_y = 1.0;
        		//convertXY2ANGLE();
        		cmd_vel.heading = 10.;
        		printf("waypoint(%e, %e)\n",ugv_state.des_x, ugv_state.des_y);
        		printf("desired heading: %e, dist: %e\n\n",ugv_state.des_heading, ugv_state.des_dist);
        	}
	}
        else if (command_key == 'q' || command_key == 'Q')
        {	
        	if(CTRL_MODE_MANUAL == true)
        	{
			printf("Shutting Off!\n");
			
			control.motorVel[0] = 0.;
			control.motorVel[1] = 0.;
			isStop = true;
			usleep(200000);
			SYSTEM_RUN = false;
		}
		else if (CTRL_MODE_WAYPOINT == true)
        	{
        		ugv_state.des_x = 1.0;
        		ugv_state.des_y = -1.0;
        		//convertXY2ANGLE();
        		cmd_vel.heading = -10.;
        		printf("waypoint(%e, %e)\n",ugv_state.des_x, ugv_state.des_y);
        		printf("desired heading: %e, dist: %e\n\n",ugv_state.des_heading, ugv_state.des_dist);
        	}
	}
	else if (command_key == 'i' || command_key == 'I')
        {	
        	k_psi.p += 0.02;
        	printf("k_psi increased: %e\n",k_psi.p);
	}
	else if (command_key == 'k' || command_key == 'K')
        {	
        	k_psi.p -= 0.02;
        	printf("k_psi decreased: %e\n",k_psi.p);
	}
	else if (command_key == 'o' || command_key == 'O')
        {	
        	k_psi.d += 0.002;
        	printf("d_psi increased: %e\n",k_psi.d);
	}
	else if (command_key == 'c' || command_key == 'C')
        {	
        	heading_bias = ugv_state.psi;
        	printf("Heading bias corrected!\n");
	}
    }
    pthread_exit(NULL);
}


void convertXY2ANGLE(void)
{
	double e_x = ugv_state.des_x - ugv_state.cur_x;
	double e_y = ugv_state.des_y - ugv_state.cur_y;
	
	/*
	e_psi.p = cmd_vel.heading - ugv_state.psi;  //degree
	e_psi.d =  - ugv_state.psi_dot;
	e_psi.i += e_psi.p*dt;
	*/
	ugv_state.des_dist = sqrt(pow(e_x,2.0) + pow(e_y,2.0));
	ugv_state.des_heading = (180./PI)*atan2(e_y, e_x);
	
	printf("des_heading: %e, dist: %e\n",ugv_state.des_heading, ugv_state.des_dist);
}

int controlHeading(double dt)
{
	cmd_vel.heading = ugv_state.des_heading;
	e_psi.p = cmd_vel.heading - ugv_state.psi;  //degree
	if(e_psi.p > 180.) e_psi.p -= 2.*180.;
        else if (e_psi.p < -180.) e_psi.p += 2.*180.;
	e_psi.d =  - ugv_state.psi_dot;
	e_psi.i += e_psi.p*dt;
	SatVal(10., &e_psi.i);
	
	float e_heading = sqrt(pow(e_psi.p,2.));
	//printf("des_heading: %e,  cur_heading: %e\n",cmd_vel.heading, ugv_state.psi);
	
	printf("heading error: %e\n",e_heading);
	if (e_heading > 4.) 
	{
		contrlPID(e_psi, k_psi, &cmd_vel.angular);
		cmd_vel.linear = 0.;
		//control.motorVel[0] = motorDir1 * (cmd_vel.angular);
		//control.motorVel[1] = motorDir2 * (-cmd_vel.angular);
		return 0;
	}
	else if (e_heading < 4.) 
	{
		cmd_vel.angular = 0.;
		return 1;	
	}
}

int controlDistance(double dt)
{
	e_dist.p = ugv_state.des_dist;// - pos_encoder;
	//e_dist.i += e_dist.p*dt;
	e_dist.i = 0.;
	e_dist.d = -vel_encoder;

	
	if (sqrt(pow(e_dist.p,2.)) > 1.2)
	{
		contrlPID(e_dist, k_dist, &cmd_vel.linear);
		//control.motorVel[0] = motorDir1 * cmd_vel.linear*1.;
		//control.motorVel[1] = motorDir2 * cmd_vel.linear*1.;
		return 0;
	}
	else if (sqrt(pow(e_dist.p,2.)) < 1.2)
	{
		cmd_vel.linear = 0.;
		//control.motorVel[0] = 0.;
		//control.motorVel[1] = 0.;
		return 1;
		printf("Reached!\n");
	} 
	
 	
}

void *control_node(void *thread_id)
{

	geometry_msgs::Twist msg;
	msg.linear.x = 0.;
	msg.linear.x = 0.;
	msg.linear.x = 0.;
	
	msg.angular.x = 0.;
	msg.angular.x = 0.;
	msg.angular.x = 0.;
	
	while(SYSTEM_RUN == true) { 
	ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));

	static ros::Time t_before = ros::Time::now();
	ros::Time t_now = ros::Time::now();
	double dt = (t_now - t_before).toSec();
	t_before = t_now;
	
	usleep(10000);

	if(mode == LOOK)
	{
		llToMeters(lon, lat, &ugv_state.cur_x, &ugv_state.cur_y);
		llToMeters(lonlook, latlook, &ugv_state.des_x, &ugv_state.des_y);
		convertXY2ANGLE();
		controlHeading(dt);
	}
	else if(mode == WP)
	{
		llToMeters(lon, lat, &ugv_state.cur_x, &ugv_state.cur_y);
		llToMeters(lonWP, latWP, &ugv_state.des_x, &ugv_state.des_y);
		convertXY2ANGLE();
		int h = controlHeading(dt);
		int d;
		if(h)
		{
			d = controlDistance(dt);
		}
		
		printf("h: %d,  d: %d\n", h,d);
	}
	else if(mode == MANUAL)
	{
		llToMeters(lon, lat, &ugv_state.cur_x, &ugv_state.cur_y);
		llToMeters(lonWP, latWP, &ugv_state.des_x, &ugv_state.des_y);
		printf("cur_pos: %e %e %e, %e\n",lon, lat, ugv_state.cur_x, ugv_state.cur_y);
		printf("des_pos: %e %e %e, %e\n\n",lonWP, latWP, ugv_state.des_x, ugv_state.des_y);
		convertXY2ANGLE();
		
		e_psi.p = cmd_vel.heading - ugv_state.psi;  //degree
		e_psi.d = -ugv_state.psi_dot;
		e_psi.i += e_psi.p*dt;
		SatVal(10., &e_psi.i);
		contrlPID(e_psi, k_psi, &cmd_vel.angular);
		//cmd_vel.angular=cmd_angular_from_joystick;
		control.motorVel[0] = motorDir1 * (cmd_vel.linear*1. + cmd_vel.angular);
		control.motorVel[1] = motorDir2 * (cmd_vel.linear*1. - cmd_vel.angular);
	}
	else if(mode == STOP)
	{
		cmd_vel.linear = 0.;
		cmd_vel.angular = 0.;
	}

	if(isStop)
	{
		cmd_vel.linear=0.;
		cmd_vel.angular = 0.;
	}
	
	//printf("motor R: %e,  L: %e\n", control.motorVel[0],control.motorVel[1]);
	SatVal(1., &cmd_vel.linear);
	msg.linear.x = cmd_vel.linear;
	msg.angular.z = cmd_vel.angular;
	chatter_pub.publish(msg);
	
	usleep(3000);
	
	}

        pthread_exit(NULL);
}
//
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ugv_ctrl");
	ros::NodeHandle nh;
	chatter_pub = nh.advertise<geometry_msgs::Twist >("/cmd_vel",1);
	encoder_subscriber = nh.subscribe("/odometry/filtered",1,encoderCallback);
	//GPS_subscriber = nh.subscribe("/navsat/fix",1,GPSCallback);
	GPS_subscriber = nh.subscribe("/mavros/global_position/raw/fix",1,GPSCallback);
	GPS_WP_subscriber = nh.subscribe("/jackal_waypoint",1,GPSWPCallback);
	look_subscriber = nh.subscribe("/jackal_look",1,lookCallback);
	mode_subscriber = nh.subscribe("/jackal_mode",1,modeCallback);
	//imu_subscriber = nh.subscribe("/imu/data_raw",1,imuCallback);
	//px4_imu_subscriber = nh.subscribe("/mavros/imu/data",1,px4IMUCallback);
	//ros::Subscriber px4_imu_raw_subscriber = nh.subscribe("/mavros/imu/data_raw",1,px4IMU_RAWCallback);
	//ros::Subscriber RC_subscriber = nh.subscribe("/mavros/rc/in",1,RC_callback);  
	//CEncoder.open_port("/dev/ttyACM0",1);
	gettimeofday(&t_init,NULL);
	pthread_t threads[NUM_THREADS];
        pthread_attr_t attr;
        struct sched_param	param;

        int fifo_max_prio, fifo_min_prio;
        
        printf("Starts UGV Contrl Node\n");
        
         // Initialize mutex and condition variables
	pthread_mutex_init(&data_acq_mutex, NULL);

	// Set thread attributes
	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	fifo_max_prio = sched_get_priority_max(SCHED_FIFO);
	fifo_min_prio = sched_get_priority_min(SCHED_FIFO);
	
	uint16_t numT=0;
	param.sched_priority = fifo_max_prio;
	pthread_attr_setschedparam(&attr, &param);
	pthread_create(&threads[numT++], &attr, control_node, (void *) 0);

	param.sched_priority = fifo_min_prio;
	pthread_attr_setschedparam(&attr, &param);
	pthread_create(&threads[numT++], &attr, command_input, (void *) 1);
	
	for (int i = 0; i < numT; i++)
	{

		pthread_join(threads[i], NULL);
		printf("joining %d thread\n", i);
	}
	
	printf("close destroy pthread attr\n");
	pthread_attr_destroy(&attr);
	printf("close destroying mutex\n");
	pthread_mutex_destroy(&data_acq_mutex);
	return 0;

}


