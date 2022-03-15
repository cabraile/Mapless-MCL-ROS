#include <cmath>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_datatypes.h"
class RelativeOffsetNode {

private:
    const double MAX_DELTA_SIZE = 10.0;

    ros::Publisher pub_;
    ros::Subscriber sub_;

    tf::Pose pose_last_;
    
    bool flag_received_once_ = false;
    std::array<double, 36> covariance_;

public:

    /* ROS nav_msgs::Odometry poses are provided with respect to a static frame (usually 'odom').
     * However, in some applications, knowing the relative estimated pose due to motion is desired.
     * Therefore, this method converts the odometry message to the pose relative to the previous timestamp.
     */ 
    void odomCallback(
        const nav_msgs::Odometry::ConstPtr& odom
    ) {
        // Might be useful for future reference
        // https://answers.ros.org/question/60658/calculate-difference-between-two-poses/
        // http://docs.ros.org/en/diamondback/api/tf/html/c++/transform__datatypes_8h.html
        // 

        // Convert Odometry to a tf::Pose (easier to convert)
        tf::Pose pose;
        tf::poseMsgToTF(odom->pose.pose, pose);

        // If no previous message was received, skip processing
        if(!flag_received_once_) {
            pose_last_ = pose;
            // Copy the first covariance matrix
            for(std::size_t i = 0; i < 36; i++)
                covariance_[i] = odom->pose.covariance[i];
            flag_received_once_ = true;
            return ;
        }
        
        // Compute the relative transform between both poses
        tf::Transform transform = pose_last_.inverseTimes(pose);
        double delta_translation = transform.getOrigin().length();
        double delta_rotation = transform.getRotation().length();
        // Odometry might have been lost...
        if ( ( delta_rotation < 1e-2 ) || (delta_translation > MAX_DELTA_SIZE ) ) {
            flag_received_once_ = false;
            return ;
        }
        // Store the results on the delta_pose object, which will be published
        geometry_msgs::PoseWithCovarianceStamped delta_pose;
        tf::poseTFToMsg(transform, delta_pose.pose.pose);
        
        delta_pose.header = odom->header;

        // Assign covariance between previous and current frames
        for(std::size_t i = 0; i < 36; i++)
            delta_pose.pose.covariance[i] = covariance_[i];

        // Publish
        pub_.publish(delta_pose);
        pose_last_ = pose;
        return ;
    }
    

    RelativeOffsetNode(ros::NodeHandle & nh) {

        this->pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/relative/pose", 100, false);

        std::string odom_topic;
        ros::param::param<std::string>("~odom_topic", odom_topic, "odom");
        sub_ = nh.subscribe(odom_topic, 1000, &RelativeOffsetNode::odomCallback, this);

        return ;
    }

};

int main(int ac, char * av[]) {
   
    ros::init(ac, av, "odometry_node");
    ros::NodeHandle n;

    RelativeOffsetNode node(n);
    ros::spin();
    return 0;
}