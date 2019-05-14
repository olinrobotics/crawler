#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class HitchOdom {
public:
  ros::NodeHandle node;
  ros::Subscriber odom_sub = node.subscribe("/gps/odom", 1, &HitchOdom::odomCB, this);
  ros::Publisher hitch_pub = node.advertise<geometry_msgs::Pose>("/hitch_pose", 10);
  ros::Rate rate = ros::Rate(100);

  void spin() {
    while (node.ok()) {
      ros::spinOnce();  // Update callbacks
      rate.sleep();     // Standardize output rate
    }
  }

private:
  nav_msgs::Odometry odom_msg;

  void odomCB(const nav_msgs::Odometry& msg) {
    odom_msg = msg;
    geometry_msgs::Pose hitch_pose_msg = computeHitchPose();
    hitch_pub.publish(hitch_pose_msg);
  }

  geometry_msgs::Pose computeHitchPose() {
    // Point representing hitch "zero" based on base link
    // TODO: Make this more parameterized
    Eigen::Vector3d p1(-0.1524,0,0);

    Eigen::Quaterniond q(odom_msg.pose.pose.orientation.x,
                         odom_msg.pose.pose.orientation.y,
                         odom_msg.pose.pose.orientation.z,
                         odom_msg.pose.pose.orientation.w);
    q.normalize();

    // convert a quaternion to a 3x3 rotation matrix:
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d R_result = R * p1;

    geometry_msgs::Pose pose_msg = geometry_msgs::Pose();
    // Return pose in global frame
    pose_msg.position.x = R_result(0) + odom_msg.pose.pose.position.x;
    pose_msg.position.y = R_result(1) + odom_msg.pose.pose.position.y;
    pose_msg.position.z = R_result(2) + odom_msg.pose.pose.position.z;

    return pose_msg;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "HitchOdom");
  HitchOdom hitch;
  hitch.spin();
}



//   /gps/odom
