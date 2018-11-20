#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/UInt16MultiArray.h"
#include "sensor_msgs/Range.h"
#include <time.h>
#include <string> //std::string, std::to_string

#define FIELD_OF_VIEW 0.5
#define MAX_RANGE 0.8
#define MIN_RANGE 0.02

std::vector<sensor_msgs::Range> sonar_array;

void range_callback(const std_msgs::UInt16MultiArray& ut){
    for(size_t i = 0; i < ut.data.size(); ++i){
        float u=(float)ut.data[i]/100.0f;
        sonar_array[i].range = u;//>ultra_msg1.max_range?ultra_msg1.max_range:u1;
    }
}

int main(int argc, char **argv){
	ros::init(argc,argv,"sonar_pub");
	ros::NodeHandle nh;
	ros::Subscriber range_msg = nh.subscribe("sonar", 1000, range_callback); //subscribe ultra data set
	ros::Publisher range_pub = nh.advertise<sensor_msgs::Range>("range",20);
	
	ros::Rate loop_rate(20);
	
	while(ros::ok()){
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        for(size_t i = 0; i < sonar_array.size(); ++i){
            header.frame_id = "sonar_link_" + std::to_string((int)i+1);
            sonar_array[i].header = header;
            sonar_array[i].field_of_view = FIELD_OF_VIEW;
            sonar_array[i].min_range = MIN_RANGE;
            sonar_array[i].max_range = MIN_RANGE;
            range_pub.publish(sonar_array[i]);
        }

		ros::spinOnce();
		loop_rate.sleep();

	}
}
