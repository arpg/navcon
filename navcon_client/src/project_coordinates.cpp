/**
 * Projects the center of a bounding box into free space
 **/

#include <navcon_client/project_coordinates.h>

/**
 * Ros Callbacks
 * */

void ProjectCoordinates::initialize_params()
{
    std::vector<std::string> keys;
    _nh_private.getParamNames(keys);
    // Generate map of keys to values
    for (std::string s : keys)
    {
        std::string next_param;
        _nh_private.getParam(s, next_param);
        _param_vals.insert({s, getParamAsString(s)});
    }
    ROS_INFO("[Project Coordinates] Read Parameters");
    // Print out values for testing
    // TODO prevent iteration of this loop if lower than debug level
    for (auto const &s : _param_vals)
    {
        // ROS_DEBUG("[Project Coordinates] Read Param: %s with a value of %s ", s.first.c_str(), s.second.c_str());
    }
}

std::string ProjectCoordinates::getParamAsString(auto val)
{
    std::string next_param;
    if (_nh_private.getParam(val, next_param))
    {
        return next_param;
    }
    float next_param_f;
    if (_nh_private.getParam(val, next_param_f))
    {
        return std::to_string(next_param_f);
    }
    double next_param_d;
    if (_nh_private.getParam(val, next_param_d))
    {
        return std::to_string(next_param_d);
    }
    int next_param_i;
    if (_nh_private.getParam(val, next_param_i))
    {
        return std::to_string(next_param_i);
    }
    bool next_param_b;
    if (_nh_private.getParam(val, next_param_b))
    {
        return std::to_string(next_param_b);
    }
    return "false";
}

/**
 *Initialize Subscribers
 **/
void ProjectCoordinates::initialize_subscribers()
{
    // Center Point Sub
    std::string cam_center_point_topic;
    _nh_private.param<std::string>("cam_center_point_topic", cam_center_point_topic, "center_point");
    _center_point_sub = _nh.subscribe(cam_center_point_topic, 10, &ProjectCoordinates::center_point_callback, this);

    // Octomap
    std::string octomap_topic;
    _nh_private.param<std::string>("octomap_topic", octomap_topic, "marble_mapping");
    _octomap_sub = _nh.subscribe(octomap_topic, 10, &ProjectCoordinates::map_callback, this);

    // Get the number of cameras
    int num_cam;
    _nh_private.param("num_cam", num_cam, 1);

    for (int i = 0; i < num_cam; i++)
    {
        std::string cam_info_topic;
        if (_nh_private.getParam("cam" + std::to_string(i) + "_info", cam_info_topic))
        {
            ROS_DEBUG("[Project Coordinates:] Subscribing to %s topic", cam_info_topic.c_str());
            ros::Subscriber current_sub = _nh.subscribe(cam_info_topic, 10, &ProjectCoordinates::cam_info_callback, this);
            _cam_info_sub_array.push_back(current_sub);
        }
        else
        {
            ROS_ERROR("[Project Coordinates]: Failed to get param %s", ("cam" + std::to_string(i) + "_info").c_str());
        }
    }
}

/**
 * Initialize Publishers
 * */

void ProjectCoordinates::initialize_publishers()
{
    _ray_publisher = _nh.advertise<visualization_msgs::Marker>("camera_projected_ray", 1);
    _transform_publisher = _nh.advertise<geometry_msgs::TransformStamped>("camera_transfrom", 1);
    _nav_goal_publisher = _nh.advertise<geometry_msgs::PoseStamped>("guiGoalPoint", 1);
    _nav_point_publisher = _nh.advertise<geometry_msgs::PointStamped>("nav_lang_point", 1);
    _ray_transform_publisher = _nh.advertise<geometry_msgs::PoseStamped>("camera_pose_ray", 1);
}

/**
 * Octomap Callback
 * Takes in an octomap and stores it in the object.
 * Topic is controlled in launch file
 */

void ProjectCoordinates::map_callback(const octomap_msgs::Octomap::ConstPtr &msg)
{
    // ROS_DEBUG("[Localize Artfiacts:] Recieved Map");
    if (!_have_map)
    {
        _have_map = true;
    }
    _map = *msg;
}

/**
 * Center Cam Callback
 */

void ProjectCoordinates::center_point_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    ROS_DEBUG("[Project Coordiantes:] Recieved Center Point");
    _detection_queue.push(*msg);
}

/**
 * Camera Info Callback
 * Takes in the camera info messages from each camera stream
 * Model is stored in a map containg the frame id and the model
 */

void ProjectCoordinates::cam_info_callback(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
    std::string cam_frame = msg->header.frame_id;
    // If Cam frame not in map add it
    if (!_cam_models_map.count(cam_frame))
    {
        ROS_DEBUG("[Project Coordinates]: Insesrting %s into camera map.", cam_frame.c_str());
        image_geometry::PinholeCameraModel current_camera_model;
        current_camera_model.fromCameraInfo(*msg);
        _cam_models_map.emplace(cam_frame, current_camera_model);
    }

    // TODO unsubscribe after all camera info messages recieved
}

/**
 * Projects the camera into a ray in xy coordiantes
 * Note this is in the camera frame X(right), Y(down), Z(forward)
 */

cv::Point3d ProjectCoordinates::project_cam_xy(std::string &cam_frame, cv::Point2d &center_point)
{
    cv::Point3d projected_ray(-1, -1, -1);
    ROS_DEBUG("Project Function cam model: %s", cam_frame.c_str());
    if (_cam_models_map.count(cam_frame))
    {
        ROS_DEBUG("Found camera");
        auto current_model = _cam_models_map.at(cam_frame);
        cv::Point2d rectified_point;
        bool rectify_point;
        _nh_private.param<bool>("rectify_point", rectify_point, false);
        if (rectify_point)
        {
            rectified_point = current_model.rectifyPoint(center_point);
        }
        projected_ray = current_model.projectPixelTo3dRay(center_point);
    }

    return projected_ray;
}

bool ProjectCoordinates::lookup_transform(std::string &frame1, std::string &frame2, ros::Time time, geometry_msgs::TransformStamped &transfrom)
{
    try
    {
        transfrom = _tf_buffer->lookupTransform(frame1, frame2, time, ros::Duration(1.0));
        return true;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_DEBUG("TF Lookup failed %s", ex.what());
        ROS_WARN("%s", ex.what());
        return false;
    }
}

// Apply transform to camera ray
bool ProjectCoordinates::project_cam_world(cv::Point3d initial_ray, std::string &reference_frame, std::string &cam_frame, ros::Time time, cv::Point3d &world_ray)
{

    geometry_msgs::TransformStamped transform;
    geometry_msgs::TransformStamped transform_back;
    if (lookup_transform(reference_frame, cam_frame, time, transform))
    {
        ROS_DEBUG("Projecting Cam Ray into World Ray");
        ROS_DEBUG("Transfrom X: %f, Y:%f, Z: %f", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
        geometry_msgs::Vector3 initial_ray_vector;
        initial_ray_vector.x = initial_ray.z;
        initial_ray_vector.y = -initial_ray.x;
        initial_ray_vector.z = -initial_ray.y;

        geometry_msgs::PoseStamped initial_ray_pose;
        geometry_msgs::Quaternion initial_ray_orientation;
        initial_ray_orientation.w = 1;
        initial_ray_pose.pose.position.x = initial_ray.z;
        initial_ray_pose.pose.position.y = -initial_ray.x;
        initial_ray_pose.pose.position.z = -initial_ray.y;
        initial_ray_pose.pose.orientation = initial_ray_orientation;
        geometry_msgs::PoseStamped world_ray_pose;

        // Vector
        geometry_msgs::Vector3 world_ray_vector;
        ROS_DEBUG("World Ray before transform X: %f, Y: %f Z: %f", world_ray_vector.x, world_ray_vector.y, world_ray_vector.z);
        tf2::doTransform(initial_ray_vector, world_ray_vector, transform);

        ROS_DEBUG("World Ray after transform X: %f, Y: %f Z: %f", world_ray_vector.x, world_ray_vector.y, world_ray_vector.z);
        //_ray_transform_publisher.publish(world_ray_pose);
        _transform_publisher.publish(transform);
        // world_ray.x =  world_ray_pose.pose.position.x;
        // world_ray.y = world_ray_pose.pose.position.y;
        // world_ray.z = world_ray_pose.pose.position.z;
        world_ray.x = world_ray_vector.x;
        world_ray.y = world_ray_vector.y;
        world_ray.z = world_ray_vector.z;

        return true;
    }
    return false;
}

/**
 * Publishes a located artifact
 *
 */

void ProjectCoordinates::publish_nav_goal(octomap::point3d &point)
{
    geometry_msgs::PoseStamped nav_goal;
    nav_goal.header.frame_id = "world";
    nav_goal.pose.position.x = point.x();
    nav_goal.pose.position.y = point.y();
    nav_goal.pose.position.z = point.z();
    _nav_goal_publisher.publish(nav_goal);

    geometry_msgs::PointStamped nav_point;
    nav_point.header.frame_id = "world";
    nav_point.point.x = point.x();
    nav_point.point.y = point.y();
    nav_point.point.z = point.z();
    _nav_point_publisher.publish(nav_point);
}

/*
 * Used to publish a point to rviz for visualization
 */

void ProjectCoordinates::publish_point(octomap::point3d &point)
{
    visualization_msgs::Marker direction_viz;
    direction_viz.header.frame_id = "world";
    direction_viz.header.stamp = ros::Time();
    direction_viz.type = visualization_msgs::Marker::SPHERE;
    direction_viz.action = visualization_msgs::Marker::ADD;
    direction_viz.pose.position.x = point.x();
    direction_viz.pose.position.y = point.y();
    direction_viz.pose.position.z = point.z();
    direction_viz.pose.orientation.x = 0.0;
    direction_viz.pose.orientation.y = 0.0;
    direction_viz.pose.orientation.z = 0.0;
    direction_viz.pose.orientation.w = 1.0;
    direction_viz.scale.x = 0.5;
    direction_viz.scale.y = 0.5;
    direction_viz.scale.z = 0.5;
    direction_viz.color.a = 1.0; // Don't forget to set the alpha!
    direction_viz.color.r = 1.0;
    direction_viz.color.g = 0.0;
    direction_viz.color.b = 0.0;
    direction_viz.lifetime = ros::Duration(100);
    ROS_DEBUG("[Project Coordinates]: Publishing ray");
    _ray_publisher.publish(direction_viz);
}

/**
 * Rounds a point to the nearest point in an octomap.
 * Points are rounded based on the resolution of the octomap
 */

void ProjectCoordinates::round_point(octomap::point3d &point, double resolution)
{
    resolution = 100.0 * resolution;
    point.x() = std::round(point.x() * resolution) / resolution;
    point.y() = std::round(point.y() * resolution) / resolution;
    point.z() = std::round(point.z() * resolution) / resolution;
}

/**
 * Main Procesing fucntion
 * 1) Takes the latest detection from the queue of bouding boxes
 * 2) Projects to XY space
 * 3) Performs raycast
 * 4) Publishes artifact
 */
void ProjectCoordinates::run()
{
    // If there are detections in the queue process them
    if (!_detection_queue.empty())
    {
        // Obtain current detection
        auto current_detection = _detection_queue.front();
        // Remove from queue
        _detection_queue.pop();

        auto current_namespace = _nh.getNamespace();
        current_namespace.erase(0, 1);
        std::string cam_frame = current_detection.header.frame_id;
        std::string cam_frame_id = cam_frame;
        if (!(cam_frame.find(current_namespace) != std::string::npos))
        {
            ROS_DEBUG("Adding namespace to tf");
            cam_frame_id = current_namespace + "/" + cam_frame;
        }
        // 1) Get center point of the bounding box
        cv::Point2d center_point(current_detection.point.x, current_detection.point.y);
        ROS_DEBUG("[Project Coordinates]: Processing point with center at %f, %f, Frame %s",
                  current_detection.point.x, current_detection.point.y, cam_frame_id.c_str());

        // 2) Project point using camera model
        auto projected_cam_ray = project_cam_xy(cam_frame, center_point);
        // projected_cam_ray.z = 0;
        ROS_DEBUG("[Project Coordinates]: Projected ray from camera in LOCAL: %s, %s, %s.", std::to_string(projected_cam_ray.x).c_str(),
                  std::to_string(projected_cam_ray.y).c_str(), std::to_string(projected_cam_ray.z).c_str());
        // Convert to world
        std::string reference_frame = "world";
        bool cam_ray_status = project_cam_world(projected_cam_ray, reference_frame, cam_frame_id, ros::Time::now(), projected_cam_ray);

        ROS_DEBUG("[Project Coordinates]: Projected ray from camera in WORLD: %s, %s, %s.", std::to_string(projected_cam_ray.x).c_str(),
                  std::to_string(projected_cam_ray.y).c_str(), std::to_string(projected_cam_ray.z).c_str());

        // 3) Use TF Tree to transpose ray to world coordinates
        // Only if transform succeded
        if (cam_ray_status)
        {
            std::string lidar_link = _nh_private.param("lidar_link", lidar_link);
            geometry_msgs::TransformStamped lidar_to_world;
            std::string lidar_frame_id = current_namespace + "/" + lidar_link;
            std::string world_frame_id = "world";
            ROS_DEBUG("Lidar Frame Id: %s", lidar_frame_id.c_str());
            // world_frame_id = "world";
            if (lookup_transform(world_frame_id, lidar_frame_id, ros::Time::now(), lidar_to_world))
            {
                ROS_DEBUG("Lookup world to lidar succeded");
                // 4) Get depth from octomap
                octomap::point3d artifact_location;
                octomap::point3d origin(lidar_to_world.transform.translation.x, lidar_to_world.transform.translation.y, lidar_to_world.transform.translation.z);
                octomap::point3d direction(projected_cam_ray.x, projected_cam_ray.y, projected_cam_ray.z);

                ROS_DEBUG("[Project Coordinates]: Direction: %f, %f, %f Origin: %f, %f, %f",
                          direction.x(), direction.y(), direction.z(), origin.x(), origin.y(), origin.z());
                auto current_map = octomap_msgs::binaryMsgToMap(_map);
                auto current_octomap = dynamic_cast<octomap::RoughOcTree *>(current_map);
                if (current_octomap != NULL)
                {
                    // Get Resolution
                    double resolution = current_octomap->getResolution();
                    // round_point(origin, resolution);
                    // round_point(direction, resolution);

                    ROS_DEBUG("[Project Coordinates]: Direction: %f, %f, %f Origin: %f, %f, %f",
                              direction.x(), direction.y(), direction.z(), origin.x(), origin.y(), origin.z());

                    bool projection_status = current_octomap->castRay(origin, direction, artifact_location, true, 12.0);
                    auto searched_node = current_octomap->search(artifact_location);
                    if (searched_node == NULL)
                    {
                        ROS_DEBUG("Searched Node is Null!");
                    }
                    // only successful projection
                    if (projection_status)
                    {
                        publish_point(artifact_location);
                    }
                    // For debug set to origin
                    // Clean up
                    delete current_octomap;

                    ROS_DEBUG("[Project Coordinates]:Point is at: %f, %f, %f", origin.x(), origin.y(), origin.z());
                    if (true)
                    {
                        ROS_DEBUG("[Project Coordinates]: Publishing goal point");
                        publish_nav_goal(artifact_location);
                    }
                }
            }
        }
    }
}
