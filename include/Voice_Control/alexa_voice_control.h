#ifndef ALEXA_VOICE_CONTROL_H
#define ALEXA_VOICE_CONTROL_H

#include <sstream>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

#include "alexa_voice_control/parameter_msg.h"
#include "alexa_voice_control/movement_msg.h"

extern "C" {
    #include "desired_velocity_qp/solver.h"
}

class alexa_voice_controller {

    public:

        alexa_voice_controller();
        ~alexa_voice_controller();

        void spinner (void);

    private:

        ros::NodeHandle nh;

        ros::Publisher alexa_tts_publisher;
        ros::Publisher manipulator_trajectory_publisher, mobile_base_velocity_publisher;

        ros::Subscriber intent_subscriber, set_parameter_subscriber, precision_movement_subscriber, inizialization_subscriber;

        void Intent_Callback (const std_msgs::Int32::ConstPtr &);
        void Set_Parameter_Callback (const alexa_voice_control::parameter_msg::ConstPtr &);
        void Precision_Movement_Callback (const alexa_voice_control::movement_msg::ConstPtr &);
        void Inizialization_Callback (const std_msgs::Bool::ConstPtr &);


        void speak(std::string text);
        std::string float_to_string(float num, int decimal_precision);

        bool alexa_tts_inizialization;

        float desired_velocity;
        float desired_velocity_QP(float desired_vel);

        void Algoritmo_IMA (void);
        void Algoritmo_PRBT (void);

        void Move_MPO (float x_vel, float y_vel, float z_twist, float movement_time);
        void Move_PRBT (double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, float movement_time);


};

#endif /* ALEXA_VOICE_CONTROL_H */