#ifndef ALEXA_VOICE_CONTROL_H
#define ALEXA_VOICE_CONTROL_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"


class alexa_voice_control {

    public:

        alexa_voice_control();
        ~alexa_voice_control();

        void spinner (void);

    private:

        ros::NodeHandle nh;

        ros::Publisher alexa_tts_publisher;
        ros::Publisher manipulator_trajectory_publisher, mobile_base_velocity_publisher;

        ros::Subscriber intent_subscriber;

        void Intent_Callback (const std_msgs::Int32::ConstPtr &);
        
        void Algoritmo_IMA (void);
        void Algoritmo_PRBT (void);

        void Move_MPO (float x_vel, float y_vel, float z_twist, float movement_time);
        void Move_PRBT (double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, float movement_time);

        void speak(std::string text);

};

#endif /* ALEXA_VOICE_CONTROL_H */