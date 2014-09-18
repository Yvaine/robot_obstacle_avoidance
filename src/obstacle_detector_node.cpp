#include "obstacle_detector.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "obstacle_detector");

    ObstacleDetector detector;

    while(ros::ok())
    {
        detector.publishDetection();
        ros::spinOnce();
    }
    return 0;
}
