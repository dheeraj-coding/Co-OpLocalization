#include <gnc_functions.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "drone_hq");
    ros::NodeHandle hq;

    init_publisher_subscriber(hq);

    wait4connect();
    wait4start();
    initialize_local_frame();
    takeoff(4);

}