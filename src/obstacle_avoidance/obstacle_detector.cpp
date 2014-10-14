#include "obstacle_detector.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib>

ObstacleDetector::ObstacleDetector()
    :nh_(),
    nh_private_("~"),
    it_(nh_),
    binary_(480, 640, CV_8UC1),
    roi_left_(binary_, cv::Rect(0,0,128,480)), //initialize the left roi
    roi_center_(binary_, cv::Rect(256,0,128,480)), //initialize the center roi
    roi_right_(binary_, cv::Rect(512,0,128,480)) //initialize the right roi
{
    ROS_INFO("Initializing");
    ROS_INFO("Reading parameters");

    nh_private_.param<int>("detection_area", min_area_, 1000);
    nh_private_.param<int>("detection_distance", min_dist_, 800);
    nh_private_.param<double>("delta_time", delta_time_, 0.01);

    ROS_INFO("detection area: %d", min_area_);
    ROS_INFO("detection distance: %d", min_dist_);
    ROS_INFO("delta time: %f", delta_time_);

    depth_sub_ = it_.subscribe("/camera/depth/image_raw", 1, &ObstacleDetector::depthMapCallBack, this);
    binary_pub_ = it_.advertise("/kinect/detected_areas", 1);
    detected_areas_pub_ = nh_.advertise<obstacle_avoidance::Detection>("obstacle_detector/detected_areas", 1);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ultrassonic_nt_sub_ = nh_.subscribe("sensor/ultrassonic/north", 1, &ObstacleDetector::ultrassonicNtCallBack, this);
    ultrassonic_st_sub_ = nh_.subscribe("sensor/ultrassonic/south", 1, &ObstacleDetector::ultrassonicStCallBack, this);
    ultrassonic_et_sub_ = nh_.subscribe("sensor/ultrassonic/east", 1, &ObstacleDetector::ultrassonicEtCallBack, this);
    ultrassonic_wt_sub_ = nh_.subscribe("sensor/ultrassonic/west", 1, &ObstacleDetector::ultrassonicWtCallBack, this);

    detected_areas_.kinect[0] = false;
    detected_areas_.kinect[1] = false;
    detected_areas_.kinect[2] = false;

    detected_areas_.ultrassonic[0] = false;
    detected_areas_.ultrassonic[1] = false;
    detected_areas_.ultrassonic[2] = false;
    detected_areas_.ultrassonic[3] = false;

    last_obstacle_.type = NONE;
    last_command_ = NONE_CMD;
}

ObstacleDetector::~ObstacleDetector(){}

void ObstacleDetector::depthMapCallBack(const sensor_msgs::ImageConstPtr& depth_map)
{
    cv_bridge::CvImagePtr depth_cv_ptr;
    cv_bridge::CvImage binary_cv;
    try
    {
        depth_cv_ptr = cv_bridge::toCvCopy(depth_map, sensor_msgs::image_encodings::TYPE_16UC1); //grabs the depth_map        

        cv::inRange(depth_cv_ptr->image, cv::Scalar(300), cv::Scalar(min_dist_), binary_); //thresholds the depth map to find obstacles

	binary_cv.image = binary_;
	binary_cv.encoding = sensor_msgs::image_encodings::TYPE_8UC1;

        //Now we calculate the moments to find out the area of the non-zero values in the map, that is
        //the regions below the defined distance threshold.
        //if the moment area is higher than the min area, we say that we are facing an obstacle and sets the 
        //corresponnding variable. This is important because we may face some a noisy area.

        moments_ = cv::moments(roi_left_, true);
        if(moments_.m00 > min_area_) {detected_areas_.kinect[0] = true; cv::add(roi_left_, cv::Scalar(100), roi_left_);}
        else detected_areas_.kinect[0] = false;

        moments_ = cv::moments(roi_center_, true);
        if(moments_.m00 > min_area_) {detected_areas_.kinect[1] = true; cv::add(roi_center_, cv::Scalar(100), roi_center_);}
        else detected_areas_.kinect[1] = false;

        moments_ = cv::moments(roi_right_, true);
        if(moments_.m00 > min_area_) {detected_areas_.kinect[2] = true; cv::add(roi_right_, cv::Scalar(100), roi_right_);}
        else detected_areas_.kinect[2] = false;

        //De acordo com o artigo do Ortiz Correa, temos 5 situações
        //de reconhecimento de obstáculos
        
        Obstacle current_obstacle;

        if( (!detected_areas_.kinect[0]) && (!detected_areas_.kinect[1]) && (!detected_areas_.kinect[2]) ){ //Nenhum obstáculo
            ObstacleDetector::front(); //va para a frente
        }
        else if( (!detected_areas_.kinect[0]) && (!detected_areas_.kinect[1]) && (detected_areas_.kinect[2]) ){ // obstáculo à direita

            current_obstacle.type = RIGHT_ONE;
            current_obstacle.time = ros::Time::now();

            if( ( (last_obstacle_.type == LEFT_ONE) || (last_obstacle_.type == LEFT_TWO) ) && ( (last_obstacle_.time - ros::Time::now()) < ros::Duration(delta_time_) ) ){
                ObstacleDetector::right();
            }
            else
            {
                ObstacleDetector::left(); // vire à esquerda
            }

            last_obstacle_ = current_obstacle;

        }
        else if( (!detected_areas_.kinect[0]) && (detected_areas_.kinect[1]) && (detected_areas_.kinect[2]) ){ // dois obstáculos à direita

            current_obstacle.type = RIGHT_TWO;
            current_obstacle.time = ros::Time::now();

            if( ( (last_obstacle_.type == LEFT_ONE) || (last_obstacle_.type == LEFT_TWO) ) && ( (last_obstacle_.time - ros::Time::now()) < ros::Duration(delta_time_) ) ){
                ObstacleDetector::right();
            }
            else
            {
                ObstacleDetector::left(); // vire à esquerda
            }

            last_obstacle_ = current_obstacle;
        }
        else if( (detected_areas_.kinect[0]) && (!detected_areas_.kinect[1]) && (!detected_areas_.kinect[2]) ){ // um obstáculo à esquerda

            current_obstacle.type = LEFT_ONE;
            current_obstacle.time = ros::Time::now();

            if( ( (last_obstacle_.type == RIGHT_ONE) || (last_obstacle_.type == RIGHT_TWO) ) && ( (last_obstacle_.time - ros::Time::now()) < ros::Duration(delta_time_) ) ){
                ObstacleDetector::left();
            }
            else
            {
                ObstacleDetector::right(); // vire à esquerda
            }

            last_obstacle_ = current_obstacle;

        }
        else if( (detected_areas_.kinect[0]) && (detected_areas_.kinect[1]) && (!detected_areas_.kinect[2]) ){ // dois obstáculos à esquerda

            current_obstacle.type = LEFT_TWO;
            current_obstacle.time = ros::Time::now();

            if( ( (last_obstacle_.type == RIGHT_ONE) || (last_obstacle_.type == RIGHT_TWO) ) && ( (last_obstacle_.time - ros::Time::now()) < ros::Duration(delta_time_) ) ){
                ObstacleDetector::left();
            }
            else
            {
                ObstacleDetector::right(); // vire à esquerda
            }

            last_obstacle_ = current_obstacle;

        }
        else if( (!detected_areas_.kinect[0]) && (detected_areas_.kinect[1]) && (!detected_areas_.kinect[2]) ){ // obstaculo no centro

            current_obstacle.type = MIDDLE;
            current_obstacle.time = ros::Time::now();

            if(last_command_ == RIGHT){
                ObstacleDetector::right();
            }
            else if(last_command_ == LEFT){
                ObstacleDetector::left();
            }
            else if(last_command_ == FRONT){

                if(rand()%2) ObstacleDetector::right();
                else ObstacleDetector::left();
            }

            last_obstacle_ = current_obstacle;
        }
        else if( (detected_areas_.kinect[0]) && (!detected_areas_.kinect[1]) && (detected_areas_.kinect[2]) ){ //obstaculo nas extremidades

            current_obstacle.type = LEFT_RIGHT;
            current_obstacle.time = ros::Time::now();

            if(last_command_ == RIGHT){
                ObstacleDetector::right();
            }
            else if(last_command_ == LEFT){
                ObstacleDetector::left();
            }
            else
            { 
                if(rand()%2) {
                    ObstacleDetector::right();
                }
                else {
                    ObstacleDetector::left();
                }
            }         

            last_obstacle_ = current_obstacle;
        }
        else if( (detected_areas_.kinect[0]) && (detected_areas_.kinect[1]) && (detected_areas_.kinect[2]) ){ //obstaculos todos

            current_obstacle.type = ALL;
            current_obstacle.time = ros::Time::now();

            if(last_command_ == RIGHT){
                ObstacleDetector::right();
            }
            else if(last_command_ == LEFT){
                ObstacleDetector::left();
            }
            else if(last_command_ == FRONT){

                if(rand()%2) ObstacleDetector::right();
                else ObstacleDetector::left();
            }         

            last_obstacle_ = current_obstacle;
        }

	binary_pub_.publish(binary_cv.toImageMsg());
        cv::waitKey(1);
    }

    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

}

void ObstacleDetector::front()
{ 
    geometry_msgs::Twist velocity;

    velocity.linear.x = 2.0;
    velocity.angular.z = 0.0;

    cmd_vel_pub_.publish(velocity);

    last_command_ = FRONT;
}

void ObstacleDetector::left()
{
    geometry_msgs::Twist velocity;

    velocity.linear.x = 0.0;
    velocity.angular.z = 2.0;

    cmd_vel_pub_.publish(velocity);

    last_command_ = LEFT;
}

void ObstacleDetector::right()
{
    geometry_msgs::Twist velocity;

    velocity.linear.x = 0.0;
    velocity.angular.z = -2.0;

    cmd_vel_pub_.publish(velocity);

    last_command_ = RIGHT;
}

void ObstacleDetector::ultrassonicNtCallBack( const sensor_msgs::RangePtr& ultrassonic_nt)
{ 
}
void ObstacleDetector::ultrassonicStCallBack( const sensor_msgs::RangePtr& ultrassonic_st)
{ 
}
void ObstacleDetector::ultrassonicEtCallBack( const sensor_msgs::RangePtr& ultrassonic_et)
{ 
}
void ObstacleDetector::ultrassonicWtCallBack( const sensor_msgs::RangePtr& ultrassonic_wt)
{ 
}

void ObstacleDetector::publishDetection()
{
    detected_areas_pub_.publish(detected_areas_);
}
