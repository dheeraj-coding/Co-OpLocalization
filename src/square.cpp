#include <gnc_functions.hpp>
#include <std_msgs/Bool.h>

// void prevTrackerCallback(const std_msgs::Bool::ConstPtr& msg){
//     ROS_INFO("I heard: [%d]", msg->data);
// }


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
    
    if(id > 1){
        bool prevReached = false;
        auto prevTrackerCallback = [&prevReached](const std_msgs::Bool::ConstPtr& msg){ prevReached = msg->data;};

        char buffer[100];
        sprintf(buffer, "/iris%d/square%d/moving", id-1, id-1);
        std::string topic(buffer);
        ros::Subscriber prevSucceeded = hq.subscribe<std_msgs::Bool>(topic, 100, prevTrackerCallback);
        ros::Rate rateWaiter(2.0);
        while(prevReached == false){
            ros::spinOnce();
            rateWaiter.sleep();
            continue;
        }
    }
        
    ros::Publisher succeededTopic = hq.advertise<std_msgs::Bool>("moving", 100);
    takeoff(height);
    ros::Rate rate(2.0);
    bool reached = false;
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
        std_msgs::Bool moving;
        moving.data = reached;
        succeededTopic.publish(moving);
        if(check_waypoint_reached()==1){
            reached = true;
        }
    }

}