/**
 * @file joy_cont.cpp
 * @Code to control a pixhawk based quadcopter using a joystick, written with
 * mavros version 0.14.2, px4 flight stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandTOL.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

/**
 * @brief global variable initialization
 * 
 */
mavros_msgs::State current_state;
sensor_msgs::Joy joyvar;
std_msgs::Float64 cur_yaw;
sensor_msgs::Imu imu;

/**
 * @brief function to read mavros state message
 * 
 * @param msg 
 */
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

/**
 * @brief function to retrieve joystick variable values
 * 
 * @param joy 
 */
void joyfn(const sensor_msgs::Joy::ConstPtr& joy){
    ROS_INFO("1");
    joyvar = *joy;}

/**
 * @brief function to retrieve compasss data
 * 
 * @param msg 
 */
void compass_cb(const std_msgs::Float64::ConstPtr& msg){
    cur_yaw = *msg;
}

/**
 * @brief function to retrieve imu data
 * 
 * @param msg 
 */
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
    imu = *msg;
}

/**
 * @brief function to find the angle of orientation relative to the origin 
 * 
 * @param ori 
 * @return float 
 */
float find_ang(geometry_msgs::Quaternion ori){
    float q[4];
    float ang;
    q[0] = ori.w;
    q[1] = ori.x;
    q[2] = ori.y;
    q[3] = ori.z;
    ang = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
    return(ang);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_cont");
    ros::NodeHandle nh;

    /*subscriber and publisher description*/
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient
            <mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber joy_ip = nh.subscribe("joy", 10, joyfn);
    ros::ServiceClient  land_client = nh.serviceClient<mavros_msgs::CommandTOL> 
	        ("/mavros/cmd/land");
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber angle_sub = nh.subscribe<std_msgs::Float64>
            ("mavros/global_position/compass_hdg", 10, compass_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data", 10, imu_cb);


    /*the setpoint publishing rate MUST be faster than 2Hz*/
    ros::Rate rate(20.0);

    /*wait for FCU connection*/
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    /* variable definitions and initializations*/
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped vel;
    mavros_msgs::SetMode set_mode;
    mavros_msgs::CommandTOL land;
    mavros_msgs::CommandBool arm_cmd;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
    int x_d = 0, y_d = 0, z_d = 2;
    float ang;

    /*send a few setpoints before starting*/
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    /* stay in the loop if ros is okay and land button is not pressed,
        land once land button is pressed*/
    while(ros::ok() && joyvar.buttons[5]!=1){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(set_mode) &&
                set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else{
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        /*calculatio to find out the orientation angle*/
        ang = find_ang(imu.orientation);
        cur_yaw.data = cur_yaw.data-90;

        /*set velocity of the quadcopter based on joystick positions*/
        ROS_INFO("%lf,%lf,%lf,%f",joyvar.axes[3],joyvar.axes[2],joyvar.axes[1],
                ang*180/3.14);
        vel.twist.linear.y = -joyvar.axes[2]*cos(-ang) + joyvar.axes[3]*
                sin(-ang);
        vel.twist.linear.x = -joyvar.axes[3]*cos(-ang) - joyvar.axes[2]*
                sin(-ang);
        vel.twist.linear.z = joyvar.axes[1];
        if(joyvar.buttons[6] != 0)
            vel.twist.angular.z = joyvar.buttons[6];
        else
            vel.twist.angular.z = -joyvar.buttons[7];
    }
    
    /*commands to control the landing*/
    land.request.altitude = 0;
    land.request.min_pitch = 0;
    land_client.call(land);
    ROS_INFO("Landing");
    local_vel_pub.publish(vel);
    ros::spinOnce();
    rate.sleep();
    return 0;
}
