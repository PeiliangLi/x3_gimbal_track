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
#define WORLD_FRAME 1
typedef struct
{
    float w;
    float x;
    float y;
    float z;
}quaternion_data_t;

float drone_yaw = 0.0;
float pitch=0,roll=0,yaw=0;
float drone_height;
quaternion_data_t rotation;  //current gimbal att
ros::Publisher gimbal_ctrl_pub,robot_location_pub,debug_pub;
bool drone_yaw_updated = false, gimbal_updated = false, sonar_updated = false, odom_updated = false;
bool init_ok = false;
Eigen::Vector3f pos_robot,drone_position;
float drone_orientation;
float vx = 0,vy = 0;
int detect_num;
int searching_flag = 1;
int searching_flag2 = 1;
bool in_searching = false;
queue<geometry_msgs::PoseStamped> robot_pose_buf;

Eigen::Vector3f vector_rotation_by_quaternion(Eigen::Vector3f init_vector, quaternion_data_t rotation)
{
    Eigen::Vector3f current_vector;
    Eigen::Matrix3f rotationMatrix;
    float qx2 = rotation.x * rotation.x;
    float qz2 = rotation.z * rotation.z;
    float qy2 = rotation.y * rotation.y;
    float qxy = rotation.x * rotation.y;
    float qzw = rotation.z * rotation.w;
    float qxz = rotation.x * rotation.z;
    float qyw = rotation.y * rotation.w;
    float qyz = rotation.y * rotation.z;
    float qxw = rotation.x * rotation.w;

    rotationMatrix << (1.0-2.0*qy2-2.0*qz2), 2.0*(qxy -qzw), 2.0*(qxz +qyw),
                       2.0*(qxy+qzw), (1.0-2.0*qx2-2.0*qz2), 2.0*(qyz -qxw),
                       2.0*(qxz-qyw), 2.0*(qxw+qyz), (1.0-2.0*qx2-2.0*qy2);
    current_vector = rotationMatrix * init_vector;
    return current_vector;
}

quaternion_data_t find_quaternion_from_vector(Eigen::Vector3f init_vector, Eigen::Vector3f desire_vector)
{
    quaternion_data_t desire_quaternion;
    Eigen::Vector3f desire_quaternion_xyz = init_vector.cross(desire_vector);
    desire_quaternion.w = init_vector.norm()*desire_vector.norm() + init_vector.dot(desire_vector);
    
    float length_q = sqrt(desire_quaternion_xyz.squaredNorm() + desire_quaternion.w*desire_quaternion.w);
    desire_quaternion.x = desire_quaternion_xyz(0)/length_q;
    desire_quaternion.y = desire_quaternion_xyz(1)/length_q;
    desire_quaternion.z = desire_quaternion_xyz(2)/length_q;
    desire_quaternion.w/= length_q;
    return desire_quaternion;
}

void gimbal_track(int u, int v)
{
    if((!drone_yaw_updated) || (!gimbal_updated))
        return;
    Eigen::Vector3f init_vector,current_vector,init_target_vector,target_vector;
    init_vector << 1.0,
                   0.0,
                   0.0;
    init_target_vector << 1.0,
                        (float)v/CAMERA_FX,
                        (float)u/CAMERA_FY;

    current_vector = vector_rotation_by_quaternion(init_vector,rotation);  //camera optical axis vector
    target_vector = vector_rotation_by_quaternion(init_target_vector,rotation);
  
    quaternion_data_t target_quaternion = find_quaternion_from_vector(current_vector, target_vector);
  
    float desire_pitch = asin(2.0*(target_quaternion.w*target_quaternion.y - target_quaternion.z*target_quaternion.x));
    float desire_roll = atan2(2.0*(target_quaternion.w*target_quaternion.x + target_quaternion.z*target_quaternion.y),
                            1.0 - 2.0*(target_quaternion.x*target_quaternion.x + target_quaternion.y*target_quaternion.y));
    float desire_yaw = atan2(2.0*(target_quaternion.w*target_quaternion.z + target_quaternion.x*target_quaternion.y),
                            1.0 - 2.0*(target_quaternion.y*target_quaternion.y + target_quaternion.z*target_quaternion.z)); 
    //printf("desire:::pitch = %.2f, roll = %.2f, yaw = %.2f\n",desire_pitch*180.0/C_PI,desire_roll*180.0/C_PI,desire_yaw*180.0/C_PI); 
    
    geometry_msgs::TwistStamped gimbal_ctrl_msg;
    float P = 1.5,D = 0.1;
    static float angular_y_last = 0,angular_x_last = 0,angular_z_last = 0;
    static float desire_pitch_last = 0,desire_roll_last = 0,desire_yaw_last = 0;
    gimbal_ctrl_msg.twist.angular.y = (fabs(desire_pitch)<0.08)? 0.0: P*(-desire_pitch) + D*(desire_pitch_last - desire_pitch)/0.01;
    gimbal_ctrl_msg.twist.angular.x = (fabs(desire_roll)<0.08)? 0.0: P*(desire_roll) + D*(desire_roll - desire_roll_last)/0.01;
    gimbal_ctrl_msg.twist.angular.z = (fabs(desire_yaw)<0.08)? 0.0: P*(-desire_yaw) + D*(desire_yaw_last - desire_yaw)/0.01;
    desire_pitch_last = desire_pitch;
    desire_roll_last = desire_roll;
    desire_yaw_last = desire_yaw;
    if(yaw>0.5&&gimbal_ctrl_msg.twist.angular.z<0)
        gimbal_ctrl_msg.twist.angular.z = 0;
    if(yaw<-0.5&&gimbal_ctrl_msg.twist.angular.z>0)
        gimbal_ctrl_msg.twist.angular.z = 0;
    if(pitch>-C_PI/6.0&&gimbal_ctrl_msg.twist.angular.y<0)
        gimbal_ctrl_msg.twist.angular.y = 0;
    //printf("gimbal_yaw: = %.2f\n",yaw);
    
    gimbal_ctrl_msg.twist.angular.y = 0.2 * gimbal_ctrl_msg.twist.angular.y + 0.8 * angular_y_last;
    gimbal_ctrl_msg.twist.angular.x = 0.2 * gimbal_ctrl_msg.twist.angular.x + 0.8 * angular_x_last;
    gimbal_ctrl_msg.twist.angular.z = 0.2 * gimbal_ctrl_msg.twist.angular.z + 0.8 * angular_z_last;
    angular_y_last = gimbal_ctrl_msg.twist.angular.y;
    angular_x_last = gimbal_ctrl_msg.twist.angular.x;
    angular_z_last = gimbal_ctrl_msg.twist.angular.z;
    gimbal_ctrl_msg.twist.angular.z = 0;
    if(init_ok)
        gimbal_ctrl_pub.publish(gimbal_ctrl_msg);
}

void robot_pos_estimator(const geometry_msgs::PointStamped::ConstPtr& point_msg)
{
    //if((!drone_yaw_updated) || (!gimbal_updated) || (!sonar_updated))
    //    break;

    Eigen::Matrix3f R_camera,Ry,Rr,Rp;
    Eigen::Vector3f p_norm;
    Ry << cos(yaw), sin(yaw), 0,
          -sin(yaw), cos(yaw),0,
          0,            0,   1;

    Rp <<cos(pitch+C_PI/2.0), 0, -sin(pitch+C_PI/2.0),
                0,          1,         0,
         sin(pitch+C_PI/2.0), 0, cos(pitch+C_PI/2.0); 
    
    Rr << 1,      0,        0,
          0, cos(roll),  sin(roll),
          0, -sin(roll), cos(roll);
    R_camera = Rr * Rp * Ry;
    p_norm << (float)-point_msg->point.x/CAMERA_FY,
              (float)point_msg->point.y/CAMERA_FX,
                      1;
    float Zc = drone_height/(R_camera.transpose().row(2) * p_norm);  //camera depth
    
    pos_robot =  Zc * R_camera.transpose() * p_norm;
    //printf("pos x = %.2f, y = %.2f drone_yaw = %.2f\n",pos_robot(0),pos_robot(1),drone_orientation*180.0/C_PI);
    geometry_msgs::PoseStamped robot_loc_msg;
    if(odom_updated == true)
    {
        robot_loc_msg.header.frame_id = "world";
        robot_loc_msg.pose.position.x = pos_robot(0)*cos(drone_orientation) + pos_robot(1)*sin(drone_orientation) + drone_position(0);
        robot_loc_msg.pose.position.y = pos_robot(0)*sin(drone_orientation) - pos_robot(1)*cos(drone_orientation) + drone_position(1);
    }
    else
    {
        robot_loc_msg.header.frame_id = "body";
        robot_loc_msg.pose.position.x = pos_robot(0);
        robot_loc_msg.pose.position.y = -pos_robot(1);
    }
    //smooth the pos
    static bool pos_last_init = false;
    static float robot_pos_x_last,robot_pos_y_last;
    static int last_detect_num = 0;
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
            robot_pose_buf.pop();
            last_detect_num = point_msg->point.z;
        }
        else if(last_detect_num == 0)
        {
            //detect, but wait about 1s to calculate vel
            vx = 0;
            vy = 0;
            robot_loc_msg.pose.position.z = -1;
        }
        direction = atan2(vy,vx);
        std_msgs::Float32 debug_msg;
        debug_msg.data = direction*180.0/C_PI;
        debug_pub.publish(debug_msg);
        robot_loc_msg.pose.orientation.x = 0;
        robot_loc_msg.pose.orientation.y = 0;
        robot_loc_msg.pose.orientation.z = sin(direction/2.0);
        robot_loc_msg.pose.orientation.w = cos(direction/2.0);
        //printf("vx = %.2f, vy = %.2f, direction = %.2f\n",vx,vy,direction*180.0/C_PI);

        if(point_msg->point.z > 0)
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
    printf("x = %.2f, y = %.2f, z = %.2f, direction = %.2f\n",robot_loc_msg.pose.position.x,
                                                             robot_loc_msg.pose.position.y,
                                                             robot_loc_msg.pose.position.z,
                                                             direction*180.0/C_PI);
    robot_location_pub.publish(robot_loc_msg);
}

void droneImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    drone_yaw = -atan2(2.0*(msg->orientation.w*msg->orientation.z + msg->orientation.x*msg->orientation.y),
                          -1.0 + 2.0*(msg->orientation.w*msg->orientation.w + msg->orientation.x*msg->orientation.x));
    drone_yaw_updated = true;
}

void gimbalCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  pitch = msg->vector.y/180.0*C_PI;
  roll = msg->vector.x/180.0*C_PI;
  yaw = msg->vector.z/180.0*C_PI;
  yaw = fmod((yaw - drone_yaw),2*C_PI);
  //printf("drone_yaw = %.2f, gimbal_yaw: = %.2f\n",drone_yaw,yaw);
  rotation.w = cos(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0) + sin(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
  rotation.x = sin(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0) - cos(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
  rotation.y = cos(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0) + sin(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0);
  rotation.z = cos(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0) - sin(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0);
  gimbal_updated = true;
}

void pointTrackCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    gimbal_track(msg->point.x,msg->point.y);
    robot_pos_estimator(msg);
    detect_num = (int)msg->point.z;
    if(detect_num == 0)
    {
        init_ok = false;
    }
        
}

void sonarDisCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    static float height_last = 0;
    //drone_height = 0.1 * msg->range + 0.9 * height_last;
    height_last = drone_height;
    sonar_updated = true;
    printf("height = %.2f\n",drone_height);
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
   // printf("loop\n");
    geometry_msgs::TwistStamped gimbal_ctrl_msg;
    //init gimbal
    if((!drone_yaw_updated) || (!gimbal_updated))
    {
        printf("waiting fot gimbal msg\n");
    }
    else if((fabs(pitch+C_PI/4.0)>0.08 || fabs(roll)>0.08 || fabs(yaw)>0.08)&&init_ok == false&&in_searching==false)
    {
        //printf("pitch = %.2f, roll = %.2f, yaw = %.2f\n",pitch*180.0/C_PI,roll*180.0/C_PI,yaw*180.0/C_PI); 
        //printf("initing\n");
        gimbal_ctrl_msg.twist.angular.y = 1.0*(pitch+C_PI/4.0);
        gimbal_ctrl_msg.twist.angular.x = 1.0*(-roll);
        gimbal_ctrl_msg.twist.angular.z = 1.0*(yaw);
        gimbal_ctrl_pub.publish(gimbal_ctrl_msg);
    }
    else
    {
        init_ok = true;
        if(detect_num == 0)
            in_searching = true;
        else
            in_searching = false;
        //printf("init ok\n");
    }
    //init ok but not detect target
    if(in_searching == true)
    {
        //printf("searching\n");
        geometry_msgs::TwistStamped gimbal_ctrl_msg_for_search;
        
        //yaw searching
        /*
        if(searching_flag == 1)
        {
            gimbal_ctrl_msg_for_search.twist.angular.z = 0.3;
            if(yaw<-0.3)
            {
                searching_flag = -1;
                gimbal_ctrl_msg_for_search.twist.angular.z = 0;
            }
        }
        else if(searching_flag == -1)
        {
            gimbal_ctrl_msg_for_search.twist.angular.z = -0.3;
            if(yaw>0.3)
            {
                searching_flag = 1;
                gimbal_ctrl_msg_for_search.twist.angular.z = 0;
            }
        }
        */
        //pitch searching
        if(searching_flag2 == 1)
        {
            gimbal_ctrl_msg_for_search.twist.angular.y = 0.3;
            if(pitch <-C_PI/3.0)
            {
                searching_flag2 = -1;
                gimbal_ctrl_msg_for_search.twist.angular.y = 0;
            }
        }
        else if(searching_flag2 == -1)
        {
            gimbal_ctrl_msg_for_search.twist.angular.y = -0.3;
            if(pitch>-C_PI/6.0)
            {
                searching_flag2 = 1;
                gimbal_ctrl_msg_for_search.twist.angular.y = 0;
            }
        }
        gimbal_ctrl_pub.publish(gimbal_ctrl_msg_for_search);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_tracker");
    ros::NodeHandle n;
    ros::Subscriber sub_gimbal,sub_imu,sub_point,sub_sonar,sub_odom;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    sub_gimbal = n.subscribe("gimbal", 10, gimbalCallback);
    sub_imu = n.subscribe("imu", 50, droneImuCallback);
    sub_point = n.subscribe("point", 10, pointTrackCallback);
    sub_sonar = n.subscribe("sonar_dis", 10, sonarDisCallback);
    sub_odom = n.subscribe("odom", 10, odomCallback);

    gimbal_ctrl_pub = n.advertise<geometry_msgs::TwistStamped>("/djiros/gimbal_speed_ctrl", 10);
    robot_location_pub = n.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);
    debug_pub = n.advertise<std_msgs::Float32>("/gimbal_track/debug", 10);

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
