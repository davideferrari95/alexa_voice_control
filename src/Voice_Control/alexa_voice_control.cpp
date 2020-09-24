#include "Voice_Control/alexa_voice_control.h"


//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

alexa_voice_control::alexa_voice_control() {

    //Publisher & Subscriber

    intent_subscriber = nh.subscribe("/alexa/intent_number", 1, &alexa_voice_control::Intent_Callback, this);

    alexa_tts_publisher = nh.advertise<std_msgs::String>("/alexa/text_to_speech", 1);

    manipulator_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/Robot_Bridge/prbt_Planned_Trajectory", 1000);
    mobile_base_velocity_publisher = nh.advertise<geometry_msgs::Twist>("/Robot_Bridge/mpo_500_Planned_Velocity", 1000);

}

alexa_voice_control::~alexa_voice_control() {


}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


void alexa_voice_control::Intent_Callback (const std_msgs::Int32::ConstPtr &msg) {

    std_msgs::Int32 intent_number = *msg;

    if (intent_number.data == 1) {Algoritmo_IMA();}
    else {std::cout << "\nIntento Sconosciuto\n";}

}


//------------------------------------------------------ FUNCTION ------------------------------------------------------//

void alexa_voice_control::speak(std::string text) {

    std_msgs::String msg;
    msg.data = text;
    alexa_tts_publisher.publish(msg);

}

void alexa_voice_control::Move_MPO (float x_vel, float y_vel, float z_twist, float movement_time) {

    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = x_vel;
    cmd_vel.linear.y = y_vel;
    cmd_vel.angular.z = z_twist;

    float x_movement = x_vel * movement_time;
    float z_rotation = z_twist * movement_time;

    mobile_base_velocity_publisher.publish(cmd_vel);

    ros::Duration(movement_time).sleep();

    // if (x_vel != 0) {Odometry_Linear_Wait(x_movement);}
    // if (z_twist != 0) {Odometry_Rotation_Wait(z_rotation);}

    cmd_vel.angular.x = 0; cmd_vel.angular.y = 0; cmd_vel.angular.z = 0;
    cmd_vel.linear.x = 0; cmd_vel.linear.y = 0; cmd_vel.linear.z = 0;

    mobile_base_velocity_publisher.publish(cmd_vel);

}


void alexa_voice_control::Move_PRBT (double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, float movement_time) {

    trajectory_msgs::JointTrajectory trajectory;
    std::vector<double> final_position = {joint1, joint2, joint3, joint4, joint5, joint6};
    
    trajectory.joint_names = {"prbt_joint_1", "prbt_joint_2", "prbt_joint_3", "prbt_joint_4", "prbt_joint_5", "prbt_joint_6"};
    trajectory.points.resize(1);

    trajectory.points[0].positions = final_position;
    ros::Duration time(movement_time);
    trajectory.points[0].time_from_start = time;

    manipulator_trajectory_publisher.publish(trajectory);

}


//------------------------------------------------- MOVEMENT ALGORITHM --------------------------------------------------//


void alexa_voice_control::Algoritmo_IMA (void) {

    float home[6] =  {0.08, 0.75, 2.35, 1.55, 0.00, -0.241};

    //go to home position in 3 seconds
    ROS_WARN("Manipulator Home");
    Move_PRBT(home[0], home[1], home[2], home[3], home[4], home[5], 3.0);
    ros::Duration(3).sleep();

    ros::Duration(2).sleep();
    
    //move mobile base to searching point
    ROS_WARN("Mobile Base GOTO Search Point");
    Move_MPO(0.5,0,0,3);
    Move_MPO(0.5,0,1,3);
    Move_MPO(0.5,0,0,2);
    ROS_INFO("Search Point Reached");
    
    ros::Duration(5).sleep();
    speak("Inizio-la-ricerca-dell\'ArUco");

    //search until joint1 = +1.26 in 7.5 seconds
    ROS_WARN("Starting ArUco Search");
    Move_PRBT(+1.26, home[1], home[2], home[3], home[4], home[5], 7.5);
    ros::Duration(7.5).sleep();

    ros::Duration(2).sleep();
    speak("Inizio-allineamento");

    //align to +1.2 in 5 seconds
    ROS_WARN("Starting ArUco Centering");
    Move_PRBT(+1.87, home[1], home[2], home[3], home[4], home[5], 4.0);
    ros::Duration(4).sleep();

    ros::Duration(2).sleep();

    //compute manipulator rotation  ->  2.967 : 170 = 1.2 : x
    float rotation_rad = 1.76;

    //mpo rotation = 9 [°/s] at 0.3 [rad/s]
    //mpo rotation = 15 [°/s] at 0.5 [rad/s]
    float rotation_vel = +0.5;
    float rotation_time = rotation_rad / rotation_vel;
    rotation_time *= 1.8;

    //rotate mobile base to 1.2 and manipulator to home
    ROS_INFO("Moblie Base Rotation to ArUco");
    Move_PRBT(home[0], home[1], home[2], home[3], home[4], home[5], rotation_time);
    Move_MPO(0,0,rotation_vel,rotation_time);

    ros::Duration(2).sleep();

    //allign mobile base to aruco (0.12m in +y)
    ROS_INFO("Moblie Base Alignment to ArUco");
    float alignment_velocity = +0.05;
    float alignment_time = 0.12 / alignment_velocity;
    Move_MPO(0,alignment_velocity,0,alignment_time);

    ros::Duration(2).sleep();
    speak("Inizio-avvicinamento");

    //time = distance (1.01 m) / velocity (0.5 m/s)
    float linear_velocity = 0.5;
    float time = 1.01 / linear_velocity;

    //move mobile base to aruco
    ROS_WARN("Starting Moblie Base Approaching");
    Move_MPO(linear_velocity, 0, 0, time);
    
    ros::Duration(2).sleep();
    speak("Inizio-prelievo");

    //picking
    ROS_WARN("Starting Manipulator Picking");
    Move_PRBT(0.05, -0.15, 1.45, 0.00, 0.00, 0.00, 4.6);
    ros::Duration(4.6).sleep();

    ros::Duration(2).sleep();

    //return to home
    ROS_WARN("Manipulator Home");    
    Move_PRBT(home[0], home[1], home[2], home[3], home[4], home[5], 3.0);
    
    ros::Duration(2).sleep();
    speak("Ho-terminato-il-movimento,resto-in-attesa-di-un-nuovo-comando");

}

void alexa_voice_control::Algoritmo_PRBT (void) {

    float home[6] =  {0.08, 0.75, 2.35, 1.55, 0.00, -0.241};

    //go to home position in 3 seconds
    ROS_WARN("Manipulator Home");
    Move_PRBT(home[0], home[1], home[2], home[3], home[4], home[5], 3.0);
    ros::Duration(3).sleep();

    ros::Duration(2).sleep();
    
    //move mobile base to searching point
    ROS_WARN("Mobile Base GOTO Search Point");
    // Move_MPO(0.5,0,0,3);
    // Move_MPO(0.5,0,1,3);
    // Move_MPO(0.5,0,0,2);
    ROS_INFO("Search Point Reached");
    
    ros::Duration(3).sleep();
    speak("Inizio-la-ricerca-dell\'ArUco");
    ros::Duration(2).sleep();

    //search until joint1 = +1.26 in 7.5 seconds
    ROS_WARN("Starting ArUco Search");
    Move_PRBT(+1.26, home[1], home[2], home[3], home[4], home[5], 7.5);
    ros::Duration(7.5).sleep();

    ros::Duration(1).sleep();
    speak("Inizio-allineamento");
    ros::Duration(1).sleep();

    //align to +1.2 in 5 seconds
    ROS_WARN("Starting ArUco Centering");
    Move_PRBT(+1.87, home[1], home[2], home[3], home[4], home[5], 4.0);
    ros::Duration(4).sleep();

    ros::Duration(2).sleep();

    //compute manipulator rotation  ->  2.967 : 170 = 1.2 : x
    float rotation_rad = 1.76;

    //mpo rotation = 9 [°/s] at 0.3 [rad/s]
    //mpo rotation = 15 [°/s] at 0.5 [rad/s]
    float rotation_vel = +0.5;
    float rotation_time = rotation_rad / rotation_vel;
    rotation_time *= 1.8;

    //rotate mobile base to 1.2 and manipulator to home
    ROS_INFO("Moblie Base Rotation to ArUco");
    Move_PRBT(home[0], home[1], home[2], home[3], home[4], home[5], rotation_time);
    // Move_MPO(0,0,rotation_vel,rotation_time);

    ros::Duration(2).sleep();

    //allign mobile base to aruco (0.12m in +y)
    ROS_INFO("Moblie Base Alignment to ArUco");
    float alignment_velocity = +0.05;
    float alignment_time = 0.12 / alignment_velocity;
    // Move_MPO(0,alignment_velocity,0,alignment_time);

    ros::Duration(1).sleep();
    speak("Inizio-avvicinamento");
    ros::Duration(1).sleep();
    
    //time = distance (1.01 m) / velocity (0.5 m/s)
    float linear_velocity = 0.5;
    float time = 1.01 / linear_velocity;

    //move mobile base to aruco
    ROS_WARN("Starting Moblie Base Approaching");
    // Move_MPO(linear_velocity, 0, 0, time);
    
    ros::Duration(1).sleep();
    speak("Inizio-prelievo");
    ros::Duration(1).sleep();

    //picking
    ROS_WARN("Starting Manipulator Picking");
    Move_PRBT(0.05, -0.15, 1.45, 0.00, 0.00, 0.00, 4.6);
    ros::Duration(4.6).sleep();

    ros::Duration(2).sleep();

    //return to home
    ROS_WARN("Manipulator Home");    
    Move_PRBT(home[0], home[1], home[2], home[3], home[4], home[5], 3.0);
    
    ros::Duration(2).sleep();
    speak("Ho-terminato-il-movimento,resto-in-attesa-di-un-nuovo-comando");

}

//-------------------------------------------------------- MAIN --------------------------------------------------------//


void alexa_voice_control::spinner (void) {
    
    ros::spinOnce();

    ros::Duration(5).sleep();
    speak("Comunicazione-tra-robot-e-utente-inizializzata");

    ros::spin();
    
    // necessaria pausa tra i comandi
    // ros::Duration(5).sleep();
    // speak("finalmente-ci-sono-riuscito!");

}
