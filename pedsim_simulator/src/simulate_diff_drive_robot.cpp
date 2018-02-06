#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

double g_updateRate, g_simulationFactor, g_timeStep;
std::string g_worldFrame, g_robotFrame;
geometry_msgs::Twist g_currentTwist;
tf::Transform g_currentPose;
boost::shared_ptr<tf::TransformBroadcaster> g_transformBroadcaster;
boost::mutex mutex;

/// Simulates robot motion of a differential-drive robot with translational and
/// rotational velocities as input
/// These are provided in the form of a geometry_msgs::Twist, e.g. by
/// turtlebot_teleop/turtlebot_teleop_key.
/// The resulting robot position is published as a TF transform from world -->
/// base_footprint frame.
void updateLoop()
{
    ros::WallRate rate(g_updateRate);
    double dt = g_timeStep;

    while (true) {
        // Get current pose
        double x = g_currentPose.getOrigin().x();
        double y = g_currentPose.getOrigin().y();
        double theta = tf::getYaw(g_currentPose.getRotation());

        // Get requested translational and rotational velocity
        double v, omega;
        {
            boost::mutex::scoped_lock lock(mutex);
            v = g_currentTwist.linear.x;
            omega = g_currentTwist.angular.z;
        }

        // Simulate robot movement
        x += cos(theta) * v * dt;
        y += sin(theta) * v * dt;
        theta += omega * dt;

        // Update pose
        g_currentPose.getOrigin().setX(x);
        g_currentPose.getOrigin().setY(y);
        g_currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, theta));

        // Broadcast transform
        g_transformBroadcaster->sendTransform(tf::StampedTransform(
            g_currentPose, ros::Time::now(), g_worldFrame, g_robotFrame));

        rate.sleep();
    }
}

void onTwistReceived(const geometry_msgs::Twist::ConstPtr& twist)
{
    boost::mutex::scoped_lock lock(mutex);
    g_currentTwist = *twist;
}

void onPoseReceived(const geometry_msgs::Pose::ConstPtr& twist)
{
    boost::mutex::scoped_lock lock(mutex);
    // Stop speed also
    g_currentTwist.linear.x = 0;
		g_currentTwist.angular.z = 0;
    
		g_currentPose.getOrigin().setX(twist->position.x);
		g_currentPose.getOrigin().setY(twist->position.y);
		g_currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, twist->orientation.z));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simulate_diff_drive_robot");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    // Process parameters
    privateHandle.param<std::string>("world_frame", g_worldFrame, "odom");
    privateHandle.param<std::string>("robot_frame", g_robotFrame,
        "base_footprint");

    privateHandle.param<double>("simulation_factor", g_simulationFactor,
        1.0); // set to e.g. 2.0 for 2x speed
    privateHandle.param<double>("update_rate", g_updateRate, 25.0); // in Hz
    privateHandle.param<double>("time_step", g_timeStep, 0.05);

    double initialX = 0.0, initialY = 0.0, initialTheta = 0.0;
    privateHandle.param<double>("pose_initial_x", initialX, 0.0);
    privateHandle.param<double>("pose_initial_y", initialY, 0.0);
    privateHandle.param<double>("pose_initial_theta", initialTheta, 0.0);

    g_currentPose.getOrigin().setX(initialX);
    g_currentPose.getOrigin().setY(initialY);
    g_currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, initialTheta));

    // Create ROS subscriber and TF broadcaster
    g_transformBroadcaster.reset(new tf::TransformBroadcaster());
    ros::Subscriber twistSubscriber = nodeHandle.subscribe<geometry_msgs::Twist>("cmd_vel", 3, onTwistReceived);
    ros::Subscriber cmdPoseSubscriber = nodeHandle.subscribe<geometry_msgs::Pose>("cmd_pose", 3, onPoseReceived);

    // Run
    boost::thread updateThread(updateLoop);
    ros::spin();
}
