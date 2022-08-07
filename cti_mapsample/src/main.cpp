#include <ros/ros.h>
#include "mapsample/mapsample.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cti_mapsample_node");
    MapSample sample;
    sample.run();
    return 0;
}
