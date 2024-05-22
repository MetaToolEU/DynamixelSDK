#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/tf_angles.hpp"
#include <math.h>

class TFPublisherNode : public rclcpp::Node
{
public:
    TFPublisherNode() : Node("static_tf_publisher")
    {
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Subscribe to the present tf angles topic
        tf_angles_subscriber_ = this->create_subscription<dynamixel_sdk_custom_interfaces::msg::TfAngles>(
            "tf_angles", 10,
            std::bind(&TFPublisherNode::angleCallback, this, std::placeholders::_1));

        // Initialize present positions
        angle1 = 0;
        angle2 = 0;

        // Publish static transforms
        publish_static_transforms();
    }

private:
	void angleCallback(const dynamixel_sdk_custom_interfaces::msg::TfAngles::SharedPtr msg) 
    {
		angle1 = msg->tf_angle_1;
		angle2 = msg->tf_angle_2;
		RCLCPP_INFO(this->get_logger(), "angle1: %d, angle2: %d", angle1, angle2);
		
		// Republish static transforms when positions are updated
        publish_static_transforms();
	}

    void publish_static_transforms()
    {
        // Clear previous static transforms
        static_transforms_.clear();

        // Static transform 1
        geometry_msgs::msg::TransformStamped static_transform_stamped1;
        static_transform_stamped1.header.stamp = this->now();
        static_transform_stamped1.header.frame_id = "neck_link";
        static_transform_stamped1.child_frame_id = "dx_link";
        static_transform_stamped1.transform.translation.x = -0.001621;
        static_transform_stamped1.transform.translation.z = 0.1195;
        // Set dynamic rotation around Z-axis using angle_1
        tf2::Quaternion quat1;
        quat1.setRPY(0, 0, angle1*(M_PI/180));
        static_transform_stamped1.transform.rotation.x = quat1.x();
        static_transform_stamped1.transform.rotation.y = quat1.y();
        static_transform_stamped1.transform.rotation.z = quat1.z();
        static_transform_stamped1.transform.rotation.w = quat1.w();
        static_transforms_.push_back(static_transform_stamped1);

        // Static transform 2
        geometry_msgs::msg::TransformStamped static_transform_stamped2;
        static_transform_stamped2.header.stamp = this->now();
        static_transform_stamped2.header.frame_id = "dx_link";
        static_transform_stamped2.child_frame_id = "tilt_link";
        static_transform_stamped2.transform.translation.x = 0.024;
        // Set dynamic rotation around Z-axis using angle_2
        tf2::Quaternion quat2;
        quat2.setRPY(0, -angle2*(M_PI/180), 0); // Roll, Pitch, Yaw
        static_transform_stamped2.transform.rotation.x = quat2.x();
        static_transform_stamped2.transform.rotation.y = quat2.y();
        static_transform_stamped2.transform.rotation.z = quat2.z();
        static_transform_stamped2.transform.rotation.w = quat2.w();
        static_transforms_.push_back(static_transform_stamped2);

        // Static transform 3
        geometry_msgs::msg::TransformStamped static_transform_stamped3;
        static_transform_stamped3.header.stamp = this->now();
        static_transform_stamped3.header.frame_id = "tilt_link";
        static_transform_stamped3.child_frame_id = "head_link";
        static_transform_stamped3.transform.translation.x = 0.028;
        // Set rotation around Z-axis using present_position_2
        tf2::Quaternion quat3;
        quat3.setRPY(0, 0, 0); // Roll, Pitch, Yaw
        // RCLCPP_INFO(this->get_logger(), "quat1: [%f, %f, %f, %f]\n quat2: [%f, %f, %f, %f]\n quat3: [%f, %f, %f, %f]\n",
        //     quat1.x(), quat1.y(), quat1.z(), quat1.w(), quat2.x(), quat2.y(), quat2.z(), quat2.w(), quat3.x(), quat3.y(), quat3.z(), quat3.w());
        static_transform_stamped3.transform.rotation.x = quat3.x();
        static_transform_stamped3.transform.rotation.y = quat3.y();
        static_transform_stamped3.transform.rotation.z = quat3.z();
        static_transform_stamped3.transform.rotation.w = quat3.w();
        static_transforms_.push_back(static_transform_stamped3);

        // Static transform 4
	    geometry_msgs::msg::TransformStamped static_transform_stamped4;
	    static_transform_stamped4.header.stamp = this->now();
	    static_transform_stamped4.header.frame_id = "head_link";
	    static_transform_stamped4.child_frame_id = "camera_optical";
        // Set translation and rotation obtained from eye-in-hand camera calibration
	    static_transform_stamped4.transform.translation.x = 0.0180;
	    static_transform_stamped4.transform.translation.y = 0.0336;
	    static_transform_stamped4.transform.translation.z = 0.0297;
	    static_transform_stamped4.transform.rotation.x = -0.4931;
	    static_transform_stamped4.transform.rotation.y = 0.4960;
	    static_transform_stamped4.transform.rotation.z = -0.5029;
	    static_transform_stamped4.transform.rotation.w = 0.5079;
	    static_transforms_.push_back(static_transform_stamped4);

        // Static transform 5
        geometry_msgs::msg::TransformStamped static_transform_stamped5;
	    static_transform_stamped5.header.stamp = this->now();
	    static_transform_stamped5.header.frame_id = "camera_optical";
	    static_transform_stamped5.child_frame_id = "camera_link";
        // Set rotation from optical frame to ROS2 frame
        tf2::Quaternion quat5;
        quat5.setRPY(90*(M_PI/180), -90*(M_PI/180), 0); // Roll, Pitch, Yaw
	    static_transform_stamped5.transform.rotation.x = quat5.x();
        static_transform_stamped5.transform.rotation.y = quat5.y();
        static_transform_stamped5.transform.rotation.z = quat5.z();
        static_transform_stamped5.transform.rotation.w = quat5.w();
	    static_transforms_.push_back(static_transform_stamped5);

        // Publish all static transforms
        for (const auto &static_transform : static_transforms_)
        {
            static_broadcaster_->sendTransform(static_transform);
        }
    }

    rclcpp::Subscription<dynamixel_sdk_custom_interfaces::msg::TfAngles>::SharedPtr tf_angles_subscriber_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms_;
    int angle1;
    int angle2;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
