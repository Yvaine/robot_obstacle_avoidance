#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Range.h"
#include "cv_bridge/cv_bridge.h"

#include "obstacle_avoidance/Detection.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "geometry_msgs/Twist.h"

enum Command
{
    FRONT,
    LEFT,
    RIGHT
};

enum Situation_t
{
    LEFT_ONE,
    LEFT_TWO,
    RIGHT_ONE,
    RIGHT_TWO,
    MIDDLE,
    LEFT_RIGHT,
    ALL,
    NONE
};

struct Obstacle
{
    Situation_t type;
    ros::Time time;
};

class ObstacleDetector
{
    public:
        /*!
         * ObstacleDetector ctor. It initializes the node handle, 
         * the ImageTransport object and the publishers and subscribers.
         */
        ObstacleDetector();

        /*!
         * Dtor
         */
        ~ObstacleDetector();

        /*!
         * Callback to handle the depth map for obstacle detection.
         *
         * \param depth_map The depth map message.
         */
        void depthMapCallBack(const sensor_msgs::ImageConstPtr& depth_map);

        void ultrassonicNtCallBack(const sensor_msgs::RangePtr& ultrassonic_nt);
        void ultrassonicStCallBack(const sensor_msgs::RangePtr& ultrassonic_st);
        void ultrassonicEtCallBack(const sensor_msgs::RangePtr& ultrassonic_et);
        void ultrassonicWtCallBack(const sensor_msgs::RangePtr& ultrassonic_wt);

        void publishDetection();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Publisher pub_; //this shall publish the detected areas

        image_transport::ImageTransport it_;

        //this shall subscribe to a depth map.
        image_transport::Subscriber depth_sub_; 

        ros::Subscriber ultrassonic_nt_sub_;
        ros::Subscriber ultrassonic_st_sub_;
        ros::Subscriber ultrassonic_et_sub_;
        ros::Subscriber ultrassonic_wt_sub_;

        ros::Publisher detected_areas_pub_;
        ros::Publisher cmd_vel_pub_;

        //The cv::Mat images used for detection.
        //The algorithm for detection will threshold the depth map in a 
        //pre-defined range (min distance for object detection) and it will
        //set rois to check whrere this object is (int front, int the right or in
        //the left).
        cv::Mat binary_;
        cv::Mat roi_left_;
        cv::Mat roi_center_;
        cv::Mat roi_right_; 

        cv::Moments moments_; //used to calculate the area of an object detection.

        //mininimum moment area for detection. Note that low values
        //may cause the system to have undefined behavior, since low levels
        //are noise sensible.
        int min_area_;
        int min_dist_; //the threshold distance for obstacle detection.

        //message to hold the detected areas.
        obstacle_avoidance::Detection detected_areas_;

        //Última situação de obstáculo
        Obstacle last_obstacle;
        
        Command last_command_;
        void front();
        void left();
        void right();
};

#endif
