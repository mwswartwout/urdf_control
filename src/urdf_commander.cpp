#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <vector>

int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_commander"); // minimal_commander node
    ros::NodeHandle n;
    ros::Publisher joint_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1); // publish to vel_cmd topic
    ros::Rate naptime(10); // update @ 10hz

    double pi = 3.14159; // value of pi
    double t = 0; // current time in calculation
    double dt = 0.1; // timestep for calculation
    double sine1; // sine output for joint1
    double sine2; // sine output for joint2
    double amplitude1 = pi; // amplitude value for joint1
    double frequency1 = 1; // frequency value for joint1
    double amplitude2 = pi; // amplitude value for joint2
    double frequency2 = 4; // frequency value for joint2
    int seq = 0;
    std::vector<double> position;
    std::vector<std::string> name;
    sensor_msgs::JointState output; // message wrapper for sine output
    while (ros::ok()) {
        sine1 = amplitude1 * sin(2*pi*frequency1*t); // Calculate sine1
        sine2 = amplitude2 * sin(2*pi*frequency2*t); // Calculate sine2
        
        output.header.seq = seq;
        name[0] = "joint1";
        name[1] = "joint2";
        output.name = name;
        position[0] = sine1; // Store sine value in proper message format
        position[1] = sine2;
        output.position = position;
        joint_publisher.publish(output); // Publish value to vel_cmd topic
        t += dt; // Increment t by timeset dt
        naptime.sleep();
    }
}
