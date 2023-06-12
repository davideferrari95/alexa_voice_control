#ifndef ALEXA_VOICE_CONTROL_H
#define ALEXA_VOICE_CONTROL_H

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include "alexa_voice_control/String.h"

class alexa_voice_controller {

    public:

        alexa_voice_controller();
        ~alexa_voice_controller();

        void spinner (void);

    private:

        ros::NodeHandle nh;

        ros::Publisher alexa_tts_publisher;
        ros::ServiceClient alexa_tts_client, alexa_tts_inizialization_client;

        ros::Subscriber intent_subscriber, inizialization_subscriber;
        
        void Intent_Callback (const std_msgs::String::ConstPtr &);
        void Inizialization_Callback (const std_msgs::Bool::ConstPtr &);

        void speak(std::string text);
        void speak_on_topic(std::string text);

        bool alexa_tts_inizialization;
        std::string intent_name;

};

#endif /* ALEXA_VOICE_CONTROL_H */