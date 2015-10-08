#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "urdf_gazebo_commander"); // minimal_commander node
    ros::NodeHandle n;
    ros::Publisher command_publisher1 = n.advertise<std_msgs::Float64>("pos_cmd1", 1); // publish to vel_cmd topic
    ros::Publisher command_publisher2 = n.advertise<std_msgs::Float64>("pos_cmd2", 1); // publish to vel_cmd topic
    ros::Rate naptime(10); // update @ 10hz

    double pi = 3.14159; // value of pi
    double t = 0; // current time in calculation
    double dt = 0.01; // timestep for calculation
    double sine1; // sine output
    double sine2;
    double amplitude1 = pi; // amplitude value for joint1
    double frequency1 = 1; // frequency value for joint1
    double amplitude2 = pi; // amplitude value for joint2
    double frequency2 = 4; // frequency value for joint2ouble frequency = 1; // frequency value for sine
 
    std_msgs::Float64 output1; // message wrapper for sine output
    std_msgs::Float64 output2;

    while (ros::ok()) {
        sine1 = amplitude1 * sin(2*pi*frequency1*t); // Calculate sine1
        sine2 = amplitude2 * sin(2*pi*frequency2*t); // Calculate sine2
        
        output1.data = sine1; // Store sine value in proper message format
        output2.data = sine2;
        command_publisher1.publish(output1); // Publish value to vel_cmd topic
        command_publisher2.publish(output2); // Publish value to vel_cmd topic
        t += dt; // Increment t by timeset dt
        naptime.sleep();
    }
}
