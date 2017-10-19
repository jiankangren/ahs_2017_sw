#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <string.h>

#define k_turn 3.0

using namespace std;

ros::Publisher vel_pub, st_pub, acc_pub;
ros::Subscriber acc_sub, joy_sub, stop_sub;

ros::Time v_past;

int y_count = 0;
bool last = false;
bool run = false;

double ax,ay;
double vx,vy,vx_g,vy_g,va_g;
double th_g = -M_PI/2;
double x,y,yaw,x_p,y_p;
double dt;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
        if(last && !msg->buttons[14]) {
                run = !run;
                ROS_INFO("Run command status: %i", run);
        }
        last = msg->buttons[14];
}

void accCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
        if(run) {
                vx_g = msg->linear.x;
                vy_g = msg->linear.y;
		va_g = msg->angular.z;
        }else {
                vx_g = 0;
                vy_g = 0;
		va_g = 0;
                th_g = M_PI/2;
                ROS_INFO("Vel X: %f, Y: %f ignored...", msg->linear.x, msg->linear.y);
        }
}

void stopCallback(const std_msgs::Bool::ConstPtr& msg) 
{
        if(msg->data) {
                run = false;
                ROS_INFO("Stopped..");
        }
}

int main(int argc, char** argv) {

        ros::init(argc, argv, "UGV_vel");
        ros::NodeHandle nh;
        ros::Rate loop_rate = 40;
   
        tf::StampedTransform transform;
        tf::TransformListener listener;

        vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        st_pub = nh.advertise<geometry_msgs::Twist>("/mpc_vel", 10);
        acc_sub = nh.subscribe("/cmd_acc", 20, accCallback);
        joy_sub = nh.subscribe("/bluetooth_teleop/joy", 1, joyCallback);
        stop_sub = nh.subscribe("/stop", 1, stopCallback);

        listener.waitForTransform("/map", "/vicon/jackal1/jackal1", ros::Time(0), ros::Duration(3.0));

        x = 0;
        y = 0;
        x_p = 0;
        y_p = 0;

        ROS_INFO("Running: UGV Accel");

        while(ros::ok()) {
                ros::spinOnce();
                x_p = x;
                y_p = y;

                listener.lookupTransform("/map", "/vicon/jackal1/jackal1", ros::Time(0), transform);
                x = transform.getOrigin().x();
                y = transform.getOrigin().y();
                yaw = tf::getYaw(transform.getRotation()) + 2*M_PI*y_count;

                dt = (ros::Time::now().toSec() - v_past.toSec());
                v_past = ros::Time::now();

                vx = (x - x_p) / dt;
                vy = (y - y_p) / dt;

                geometry_msgs::Twist cmd;
                /*cmd.linear.x = sqrt(pow(vx_g,2) + pow(vy_g,2));

                th_g = atan2(vy_g,vx_g);
                
                if(abs(vy_g) < 0.001 && abs(vx_g) < 0.001) {
                        th_g = -M_PI/2;
                }

                if((th_g - yaw) > M_PI) {
                        y_count += 1;
                }else if((th_g - yaw) < -M_PI) {
                        y_count -= 1;
                }

                cmd.angular.z = k_turn*(th_g - yaw);
                */
		if(run) {
			cmd.linear.x = vx_g;
                        cmd.angular.z = va_g;
			vel_pub.publish(cmd);
                }else {
                        th_g = -M_PI/2;
                        vx_g = 0;
                        vy_g = 0;
			va_g = 0;
                        ax = 0;
                        ay = 0;
                }
                geometry_msgs::Twist v_st;
                v_st.linear.x = x;
                v_st.linear.y = y;
                v_st.angular.x = vx;
                v_st.angular.y = vy;
		v_st.angular.z = yaw;

                st_pub.publish(v_st);

                loop_rate.sleep();
        }
}

