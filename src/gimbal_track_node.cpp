#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <queue>
#include <vector>
#include <string>
#include <sstream>//for int to string

using namespace std;

#define C_PI ((double) 3.141592653)
#define CAMERA_FX 367.950989
#define CAMERA_FY 362.819244
int detect_num = 0;
bool point_updated = false,odom_updated = false;
float drone_height = 0;
float drone_orientation = 0;
Eigen::Vector3f drone_position;
queue<geometry_msgs::PoseStamped> robot_pose_buf;
ros::Publisher robot_location_pub;

void robot_pos_estimator(const geometry_msgs::PointStamped::ConstPtr& point_msg)
{   
    //if(!point_updated||!odom_updated)
    //    return;
    Eigen::Vector3f pos_robot;
    static int last_detect_num = 0;
    geometry_msgs::PoseStamped robot_loc_msg;
    static bool pos_last_init = false;
    if(point_msg->point.z == 0)
    {
        last_detect_num = 0;
        robot_loc_msg.pose.position.z = -1;
        robot_location_pub.publish(robot_loc_msg);
        while(!robot_pose_buf.empty()) robot_pose_buf.pop();
        pos_last_init = false;
        printf("z = %.2f\n",robot_loc_msg.pose.position.z);
        return;
    }
    pos_robot << -point_msg->point.x/point_msg->point.z,
                 -point_msg->point.y/point_msg->point.z,
                 1.0;
    pos_robot = drone_height*pos_robot;
    //printf("pos x = %.2f, y = %.2f drone_yaw = %.2f\n",pos_robot(0),pos_robot(1),drone_orientation*180.0/C_PI);

    robot_loc_msg.header.frame_id = "world";
    robot_loc_msg.pose.position.x = pos_robot(0)*cos(drone_orientation) - pos_robot(1)*sin(drone_orientation) + drone_position(0);
    robot_loc_msg.pose.position.y = pos_robot(0)*sin(drone_orientation) + pos_robot(1)*cos(drone_orientation) + drone_position(1);
    //smooth the pos
    static float robot_pos_x_last,robot_pos_y_last;
    if(!pos_last_init)
    {
        pos_last_init = true;
        robot_pos_x_last = robot_loc_msg.pose.position.x;
        robot_pos_y_last = robot_loc_msg.pose.position.y;
    }
    else
    {
        robot_loc_msg.pose.position.x = 0.03*robot_loc_msg.pose.position.x + 0.97*robot_pos_x_last;
        robot_loc_msg.pose.position.y = 0.03*robot_loc_msg.pose.position.y + 0.97*robot_pos_y_last;
        robot_pos_x_last = robot_loc_msg.pose.position.x;
        robot_pos_y_last = robot_loc_msg.pose.position.y;
    }
    robot_loc_msg.pose.position.z = -1; 
    robot_loc_msg.header.stamp = point_msg->header.stamp;
    robot_pose_buf.push(robot_loc_msg);
    float direction = 0;
    if(robot_pose_buf.size()>1)
    {
        float delta_x,delta_y,delta_time;
        float vx,vy;
        delta_x = robot_pose_buf.back().pose.position.x - robot_pose_buf.front().pose.position.x;
        delta_y = robot_pose_buf.back().pose.position.y - robot_pose_buf.front().pose.position.y;
        delta_time = (robot_pose_buf.back().header.stamp - robot_pose_buf.front().header.stamp).toSec();
        if(delta_time>=0.5)
        {
            vx = delta_x/delta_time;
            vy = delta_y/delta_time;
            vx = fabs(vx)<0.05? 0:vx;
            vy = fabs(vy)<0.05? 0:vy;
            //printf("size = %d, delta_time = %.2f\n",robot_pose_buf.size(),delta_time);
            robot_pose_buf.pop();
            //robot_pose_buf.pop();
            //if(point_msg->point.z == 0)
            //    last_detect_num = 0;
            last_detect_num = 1;
        }
        else if(last_detect_num == 0)
        {
            //detect, but wait about 1s to calculate vel
            vx = 0;
            vy = 0;
            robot_loc_msg.pose.position.z = -1;
        }
        
        direction = atan2(vy,vx);
        robot_loc_msg.pose.orientation.x = 0;
        robot_loc_msg.pose.orientation.y = 0;
        robot_loc_msg.pose.orientation.z = sin(direction/2.0);
        robot_loc_msg.pose.orientation.w = cos(direction/2.0);
        //printf("vx = %.2f, vy = %.2f, direction = %.2f\n",vx,vy,direction*180.0/C_PI);

        if(point_msg->point.z != 0)
            robot_loc_msg.pose.position.z = sqrt(vx*vx + vy*vy);
        else
        {
            //lost
            robot_loc_msg.pose.position.z = -1;   
            last_detect_num = 0;
            while(!robot_pose_buf.empty()) robot_pose_buf.pop();    
        }
        
    }
    //printf("pos x = %.2f, y = %.2f\n",pos_robot(0),pos_robot(1));
    //printf("drone: x = %.2f, y = %.2f\n",drone_position(0),drone_position(1));
    printf("x = %.2f, y = %.2f, z = %.2f, dir = %.2f,height = %.2f\n",robot_loc_msg.pose.position.x,
                                                             robot_loc_msg.pose.position.y,
                                                             robot_loc_msg.pose.position.z,
                                                             direction*180.0/C_PI,drone_height);
    robot_location_pub.publish(robot_loc_msg);
}


void pointTrackCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    robot_pos_estimator(msg);
    //printf("heard z= %.2f\n",msg->point.z);
    if(msg->point.z == 0)
        detect_num = 0;
    
    point_updated = true;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    drone_position << msg->pose.pose.position.x,
                      msg->pose.pose.position.y,
                      msg->pose.pose.position.z;
    drone_height =  msg->pose.pose.position.z;
    drone_orientation = atan2(2.0*(msg->pose.pose.orientation.w*msg->pose.pose.orientation.z + msg->pose.pose.orientation.x*msg->pose.pose.orientation.y),
                          -1.0 + 2.0*(msg->pose.pose.orientation.w*msg->pose.pose.orientation.w + msg->pose.pose.orientation.x*msg->pose.pose.orientation.x)); 
    //printf("drone: x = %.2f, y = %.2f z = %.2f, yaw = %.2f\n",drone_position(0),drone_position(1),drone_position(2),drone_orientation*180.0/C_PI);  
    odom_updated = true;              
}

void process()
{    
   if(point_updated == false)
        printf("waiting for Qiuge msg\n");
    else if(odom_updated == false)
        ;//printf("waiting for odom msg\n");
    else
        ;//printf("in process\n");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_tracker");
    ros::NodeHandle n;
    ros::Subscriber sub_EIP,sub_odom;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    sub_EIP = n.subscribe("point", 10, pointTrackCallback);
    sub_odom = n.subscribe("odom", 10, odomCallback);

    robot_location_pub = n.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);

    ros::Rate r(100);
    while (ros::ok())
    {
        process();
        ros::spinOnce();
        r.sleep();
    }
    ros::spin();
    return 0;
}
