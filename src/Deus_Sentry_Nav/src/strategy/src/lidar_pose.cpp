/**
 * @file lidar_pose_radar_only.cpp
 * @brief 只使用雷达定位（订阅 /Odometry），解析保存并打印位姿信息
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <iomanip>

using namespace std;

class lidar_pose
{
public:
    lidar_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_);

    double pi;
    struct attitude
    {
        double pitch;
        double roll;
        double yaw;
    };

    attitude radarAttitude;
    geometry_msgs::PoseStamped radarPose;

    bool radarOdomRec_flag;

    ros::Rate *rate;

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber odom_sub;

    void estimator_odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void start();
};

lidar_pose::lidar_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_)
    :nh(nh_), nh_private(nh_private_), radarOdomRec_flag(false)
{
    pi = 3.1415926;
    rate = new ros::Rate(30);

    odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 2, &lidar_pose::estimator_odom_cb,this);

    radarAttitude.pitch = 0;
    radarAttitude.roll = 0;
    radarAttitude.yaw = 0;
}

void lidar_pose::estimator_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    radarPose.pose = msg->pose.pose;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    radarAttitude.pitch = pitch * 180 / pi;
    radarAttitude.roll = roll * 180 / pi;
    radarAttitude.yaw = yaw * 180 / pi;

    radarOdomRec_flag = true;
}

// void lidar_pose::start()
// {
//     while(ros::ok())
//     {
//         if(!radarOdomRec_flag){
//             cout << "\033[K" << "\033[31m noooooooooooooooooo   lidarrrrrrrrrrrrr     dataaaaaaaaaaaaaaaaaaaaaaaaaa!!! \033[0m" << endl;
//         } else {
//             cout << "\033[K" << "\033[32m yesyesyes      lidar    data   receiveddddddddddddddddddd! \033[0m" << endl;
//             cout << "\033[K" << "       雷达定位" << endl;
//             cout << setiosflags(ios::fixed) << setprecision(7)
//                 << "\033[K" << "x      " << radarPose.pose.position.x << endl;
//             cout << setiosflags(ios::fixed) << setprecision(7)
//                 << "\033[K" << "y      " << radarPose.pose.position.y << endl;
//             cout << setiosflags(ios::fixed) << setprecision(7)
//                 << "\033[K" << "z      " << radarPose.pose.position.z << endl;
//             cout << setiosflags(ios::fixed) << setprecision(7)
//                 << "\033[K" << "pitch  " << radarAttitude.pitch << endl;
//             cout << setiosflags(ios::fixed) << setprecision(7)
//                 << "\033[K" << "roll   " << radarAttitude.roll << endl;
//             cout << setiosflags(ios::fixed) << setprecision(7)
//                 << "\033[K" << "yaw    " << radarAttitude.yaw << endl;
//             cout << "\033[7A" << endl;
//         }
//         ros::spinOnce();
//         rate->sleep();
//     }
//     cout << "\033[7B" << endl;
// }


void lidar_pose::start()
{
    while (ros::ok())
    {
        cout << "\033[2J\033[H";

        if (!radarOdomRec_flag)
        {
            cout << "\033[31mNooooooooooo LiDAR dataaaaaaaaaaaaaaaaaa!a\033[0m" << endl;
        }
        else
        {
            cout << "\033[32mLiDARRRRRRRRRRRRRRRRRRRRRRRRRR data receivedddddddddddddddddddddddddddd!\033[0m" << endl;
            cout << "==================== odometry information ====================" << endl;

            cout << setiosflags(ios::fixed) << setprecision(7);
            cout << "x      : " << radarPose.pose.position.x << endl;
            cout << "y      : " << radarPose.pose.position.y << endl;
            cout << "z      : " << radarPose.pose.position.z << endl;
            cout << "pitch  : " << radarAttitude.pitch << endl;
            cout << "roll   : " << radarAttitude.roll << endl;
            cout << "yaw    : " << radarAttitude.yaw << endl;

            cout << "=====================================================" << endl;
        }

        ros::spinOnce();
        rate->sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_pose_node");
    ros::NodeHandle nh_("");
    ros::NodeHandle nh_private_("~");
    lidar_pose vision(nh_,nh_private_);
    vision.start();

    return 0;
}