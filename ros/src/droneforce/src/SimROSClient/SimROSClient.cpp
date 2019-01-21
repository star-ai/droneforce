/**
 * @file   SyntheticImagesPulisher.cpp
 * @author Muyuan Lin, adapted from @Winter Guerra
 * @brief  Pulls images from Unity and publish them as ROS images.
 *
 **/

#include "SimROSClient.hpp"

#define SHOW_DEBUG_IMAGE_FEED false
typedef const boost::function< void(const nav_msgs::Odometry::ConstPtr &)> callback;

// Constructor
SimROSClient::SimROSClient(){
  startTime = flightGoggles.getTimestamp();
  std::cout << "ROS Client started!" << std::endl;
}

void SimROSClient::populateRenderSettings(){
  // Scene/Render settings
  flightGoggles.state.maxFramerate = 60; 
  /*
  Available scenes:
  sceneFilename = "Butterfly_World";
  sceneFilename = "FPS_Warehouse_Day";
  sceneFilename = "FPS_Warehouse_Night";
  sceneFilename = "Hazelwood_Loft_Full_Day";
  sceneFilename = "Hazelwood_Loft_Full_Night";
   */
  flightGoggles.state.sceneFilename = "Hazelwood_Loft_Full_Night";
  
  // Prepopulate metadata of cameras
  unity_outgoing::Camera_t cam_RGB;
  cam_RGB.ID = "Camera_RGB";
  cam_RGB.channels = 3;
  cam_RGB.isDepth = false;
  cam_RGB.outputIndex = 0;

  unity_outgoing::Camera_t cam_D;
  cam_D.ID = "Camera_D";
  cam_D.channels = 1;
  cam_D.isDepth = true;
  cam_D.outputIndex = 1;

  // Add cameras to persistent state
  flightGoggles.state.cameras.push_back(cam_RGB);
  flightGoggles.state.cameras.push_back(cam_D);
  
}


void SimROSClient::poseSubscriber(const nav_msgs::Odometry::ConstPtr& msg){
    ROS_INFO("Seq: [%d]", msg->header.seq);
    // Sends render requests to FlightGoggles indefinitely
    geometry_msgs::Pose cam_pose_tf = msg->pose.pose;

    // Old hack to convert ROS TF to ENU poses.
    // float x = cam_pose_tf.position.x;
    // cam_pose_tf.position.x = -cam_pose_tf.position.y;
    // cam_pose_tf.position.y = x;

    Transform3 cam_pose_eigen;
    poseMsgToEigen(cam_pose_tf, cam_pose_eigen);

    // Populate status message with new pose
    flightGoggles.setCameraPoseUsingROSCoordinates(cam_pose_eigen, 0);
    flightGoggles.setCameraPoseUsingROSCoordinates(cam_pose_eigen, 1);

    // Update timestamp of state message (needed to force FlightGoggles to rerender scene)
    flightGoggles.state.utime = flightGoggles.getTimestamp();
    // request render
    flightGoggles.requestRender();
    
}

void posePublisher(SimROSClient *self){
  // Sends render requests to FlightGoggles indefinitely
  while (true){
    // Update camera position
    self->updateCameraTrajectory();
    // Update timestamp of state message (needed to force FlightGoggles to rerender scene)
    self->flightGoggles.state.utime = self->flightGoggles.getTimestamp();
    // request render
    self->flightGoggles.requestRender();
    // Throttle requests to framerate.
    // usleep(1e6/self->flightGoggles.state.maxFramerate);
    }
}

void SimROSClient::updateCameraTrajectory(){
  double period = 15.0f;
  double r = 2.0f;
  double t = (flightGoggles.getTimestamp()-startTime)/1000000.0f;
  double theta = -((t/period)*2.0f*M_PI);
  
  Transform3 camera_pose;
  camera_pose.translation() = Vector3(r*cos(theta), r*sin(theta), 1.5f);
  // Set rotation matrix using pitch, roll, yaw
  camera_pose.linear() = Eigen::AngleAxisd(theta-M_PI, Eigen::Vector3d(0,0,1)).toRotationMatrix();

  // Populate status message with new pose
  flightGoggles.setCameraPoseUsingROSCoordinates(camera_pose, 0);
  flightGoggles.setCameraPoseUsingROSCoordinates(camera_pose, 1);
}

///////////////////////
// Example Client Node
///////////////////////

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_listener");

    ros::NodeHandle node;
    
    // Create client
    SimROSClient client;

    // Load params
    client.populateRenderSettings();

    // callback poseCallback = boost::bind(&SimROSClient::poseSubscriber, &client, _1);
    // ros::Subscriber sub = node.subscribe("odom", 1, poseCallback);

    // Spin
    // ros::spin();


    ros::Rate r(30);
    while(node.ok()) {
      posePublisher(&client);

      ros::spinOnce();
      r.sleep();
    }



    return 0;
}
