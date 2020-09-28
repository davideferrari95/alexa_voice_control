#include "Voice_Control/alexa_voice_control.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "alexa_voice_controller");

    alexa_voice_controller avc;

    while (ros::ok()) {

        avc.spinner();
        ros::shutdown();

    }

return 0;

}
