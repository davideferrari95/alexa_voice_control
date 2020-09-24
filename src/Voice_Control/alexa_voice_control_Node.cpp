#include "Voice_Control/alexa_voice_control.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "alexa_voice_control");

    alexa_voice_control avc;

    while (ros::ok()) {

        avc.spinner();
        ros::shutdown();

    }

return 0;

}
