#include <gnc_functions.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "drone_hq");
    ros::NodeHandle hq("~");

    init_publisher_subscriber(hq);

    int height;
    int id;
    hq.getParam("id", id);
    hq.getParam("height", height);

    std::cout<<"ID: "<<id<<", Height: "<<height<<std::endl;

    wait4connect();
    // wait4start();
    set_mode("GUIDED");
    initialize_local_frame();
    takeoff(height);

}