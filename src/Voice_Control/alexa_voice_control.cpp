#include "Voice_Control/alexa_voice_control.h"


//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

alexa_voice_controller::alexa_voice_controller() {

    // ---- ROS PUBLISHERS ---- //
    alexa_tts_publisher = nh.advertise<std_msgs::String>("/alexa/tts", 1);

    // ---- ROS SUBSCRIBERS ---- //
    intent_subscriber = nh.subscribe("/alexa/intents", 1, &alexa_voice_controller::Intent_Callback, this);

    // ---- ROS SERVICE CLIENTS ---- //
    alexa_tts_client = nh.serviceClient<alexa_voice_control::String>("/alexa/text_to_speech");
    alexa_tts_inizialization_client = nh.serviceClient<std_srvs::Trigger>("/alexa/alexa_tts_initialization");

    alexa_tts_inizialization = false;

    // Wait for Alexa TTS
    while (!alexa_tts_inizialization) {
        
        std_srvs::Trigger alexa_tts_inizialization_srv;

        if (alexa_tts_inizialization_client.call(alexa_tts_inizialization_srv)) {alexa_tts_inizialization = true;}
        else {ROS_INFO_THROTTLE(5, "Wait For Alexa TTS Initialization");}

    }

    // Comunication Initialized
    ros::Duration(2).sleep();
    speak("Alexa-TTS-Communication-Initialized");

}

alexa_voice_controller::~alexa_voice_controller() {


}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


void alexa_voice_controller::Intent_Callback (const std_msgs::String::ConstPtr &msg) {

    std_msgs::String temp = *msg;
    intent_name = temp.data;

}


//------------------------------------------------------ FUNCTION ------------------------------------------------------//

void alexa_voice_controller::speak_on_topic(std::string text) {

    std_msgs::String msg;
    msg.data = text;
    alexa_tts_publisher.publish(msg);

}


void alexa_voice_controller::speak(std::string text) {

    alexa_voice_control::String msg;
    msg.request.message_data = text;

    if (alexa_tts_client.call(msg)) {ros::Duration(5).sleep();}
    else {ROS_ERROR("Failed to Call Service: \"/alexa/text_to_speech\"");}

    ROS_INFO("Speak Done");

}


//-------------------------------------------------------- MAIN --------------------------------------------------------//


void alexa_voice_controller::spinner (void) {

    ros::spinOnce();

    speak("example-text,-hyphen-necessary-in-place-of-spaces");

}
