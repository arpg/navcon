/**
 * This x,y coordinates in pixel space and uses an Octomap to projec
 * into free space
**/
#ifndef PROJECT_COORDINATES_H
#define PROJECT_COORDINATES_H


//Ros Includes
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/console.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/transform_listener.h>
#include <octomap/octomap.h>
#include <rough_octomap/RoughOcTree.h>
#include <rough_octomap/conversions.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/CompressedImage.h>

//Logging
#include <log4cxx/logger.h>

//Standard Libraray Includes
#include <vector>
#include <map>
#include <string>
#include <queue>
#include <regex>
#include <limits>
#include <tuple>
#include <assert.h>

//OpenCV
#include <opencv2/core/types.hpp>



using namespace ros;

class ProjectCoordinates
{
    public:
        ProjectCoordinates(){

        }
        ProjectCoordinates(ros::NodeHandle nh, ros::NodeHandle nh_private, tf2_ros::Buffer* tf_buffer, tf2_ros::TransformListener* tf_listener){
            _nh = nh;
            _nh_private = nh_private;
            _logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
            _tf_buffer = tf_buffer;
            _tf_listener = tf_listener;
            _have_map = false;
            initialize_subscribers();
            initialize_publishers(); 
	        _image_id = 0; 

        }

        //Initializing parameters
        void initialize_params();
        std::string getParamAsString(auto val);
        void initialize_subscribers();
        void initialize_publishers();

        void map_callback( const octomap_msgs::Octomap::ConstPtr &msg );
        void center_point_callback( const geometry_msgs::PointStamped::ConstPtr &msg );
        void cam_info_callback( const sensor_msgs::CameraInfo::ConstPtr &msg);

        //Functions for localization
        cv::Point3d project_cam_xy(std::string &cam_frame, cv::Point2d &center_point);
        bool lookup_transform(std::string &frame1, std::string &frame2, ros::Time time, geometry_msgs::TransformStamped& transfrom);
        bool project_cam_world(cv::Point3d initial_ray, std::string &reference_frame, std::string &cam_frame, ros::Time time, cv::Point3d &world_ray);
        void round_point(octomap::point3d &point, double resolution);


        //Publishers
        void publish_point(octomap::point3d &point);
        void publish_nav_goal(octomap::point3d &point);
        void run();
    
    private:
        octomap_msgs::Octomap _map;
        std::map<std::string, image_geometry::PinholeCameraModel> _cam_models_map;
        int _num_cam_models;
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_private;
        std::map<std::string, std::string> _param_vals;
        log4cxx::LoggerPtr  _logger;
        //Image Id
        int _image_id;
        bool _have_map;

        //Queue of detections to process
        //std::tuple<std_msgs::Header, Image::BoundingBox, sensor_msgs::Image>
        std::queue<geometry_msgs::PointStamped> _detection_queue;

        //subscribers
        ros::Subscriber _octomap_sub;
        ros::Subscriber _center_point_sub;
        std::vector<ros::Subscriber> _cam_info_sub_array;

        //publishers
        ros::Publisher _ray_publisher;
        ros::Publisher _nav_goal_publisher;
        ros::Publisher _nav_point_publisher;
        ros::Publisher _ray_transform_publisher;
	    ros::Publisher _transform_publisher;

        //TF Listner
        tf2_ros::Buffer* _tf_buffer;
        tf2_ros::TransformListener* _tf_listener;


};


#endif //indef LOCALIZE_ARTIFACTS_H


	


