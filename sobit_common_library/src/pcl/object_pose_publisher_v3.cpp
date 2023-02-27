#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <ros/ros.h>

#include <sobit_common_library/utils/print.h>
#include <sobit_common_msg/BoundingBoxes.h>
#include <sobit_common_msg/ObjectPose.h>
#include <sobit_common_msg/ObjectPoseArray.h>
#include <sobit_common_msg/RunCtrl.h>

#include <std_msgs/Bool.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <iostream>


typedef pcl::PointXYZ           PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sobit_common_msg::BoundingBoxes, sensor_msgs::PointCloud2, sensor_msgs::Image>
    BBoxesCloudSyncPolicy;


class ObjPosePublisher {
    private:
        ros::NodeHandle               nh_;

        tf2_ros::Buffer               tfBuffer_;
        tf2_ros::TransformListener    tfListener_;
        tf2_ros::TransformBroadcaster tfBroadcaster_;

        cv_bridge::CvImagePtr cv_ptr_;
        cv::Mat img_raw_;
        
        std::string              base_frame_name_;
        std::string              map_frame_name_;
        std::string              cloud_topic_name_;
        std::string              img_topic_name_;

        double                   cluster_tolerance;
        int                      min_clusterSize;
        int                      max_clusterSize;
        double                   leaf_size;

        ros::Publisher  pub_obj_poses_;
        ros::Subscriber sub_ctr_;
        ros::ServiceServer srv_subscriber_switch_;

        bool execute_flag_;
        // bool pub_result_flag_;

        std::unique_ptr<message_filters::Subscriber<sobit_common_msg::BoundingBoxes>> sub_bboxes_;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>        sub_cloud_;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>              sub_img_;

        std::shared_ptr<message_filters::Synchronizer<BBoxesCloudSyncPolicy>>         sync_;


        bool checkNanInf(PointT pt) {
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
                return false;

            } else if (std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z)) {
                return false;

            }

            return true;
        }


        bool checkNanInf(geometry_msgs::Point pt) {
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
                return false;

            } else if (std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z)) {
                return false;

            }
            
            return true;
        }


        void callback_BBoxCloud(const sobit_common_msg::BoundingBoxesConstPtr &bbox_msg,
                                const sensor_msgs::PointCloud2ConstPtr        &cloud_msg,
                                const sensor_msgs::ImageConstPtr              &img_msg ) {
            // if(execute_flag_ == false){	return;	}

            std::string     frame_id = cloud_msg->header.frame_id;
            geometry_msgs::TransformStamped transformStampedFrame_;

            PointCloud::Ptr cloud_transform(new PointCloud());
            pcl::fromROSMsg(*cloud_msg, *cloud_transform);
            
            try {
                transformStampedFrame_ = tfBuffer_.lookupTransform(base_frame_name_, frame_id, ros::Time(0), ros::Duration(1.0));
                pcl_ros::transformPointCloud(*cloud_transform, *cloud_transform, transformStampedFrame_.transform);

            } catch (tf2::TransformException &ex) {
                ROS_ERROR("Could NOT transform tf to: %s", ex.what());
                return;

            } catch (...) {
                ROS_ERROR("ERROR obj_point_publisher 54 line");
                return;

            }

            try {
                cv_ptr_ = cv_bridge::toCvCopy( img_msg, sensor_msgs::image_encodings::BGR8 );
                img_raw_ = cv_ptr_->image.clone();

            } catch ( cv_bridge::Exception &e ) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;

            }
            if (img_raw_.empty() == true) {
                ROS_ERROR("Input_image error");
                return;

            }
            
            int width = img_raw_.cols;
            ROS_INFO("width : %i", width);

            sobit_common_msg::ObjectPoseArray object_pose_array;
            object_pose_array.header = bbox_msg->header;

            // visualization_msgs::MarkerArray marker_array;

            for (int i = 0; i < bbox_msg->bounding_boxes.size(); i++) {
                const sobit_common_msg::BoundingBox& bbox = bbox_msg->bounding_boxes[i];

                sobit_common_msg::ObjectPose obj_pose;
                obj_pose.Class = bbox.Class;
                
                int x_ctr = ( bbox.xmin + bbox.xmax ) / 2;
                int y_ctr = ( bbox.ymin + bbox.ymax ) / 2;
                int array_num = ( width * y_ctr ) + x_ctr;
                // ROS_INFO("x_ctr : %i", x_ctr);
                // ROS_INFO("y_ctr : %i", y_ctr);

                if(std::isnan( cloud_transform->points[ array_num ].x ) || std::isnan( cloud_transform->points[ array_num ].y ) || std::isnan( cloud_transform->points[ array_num ].z )){
                    int x_ctr_min = ( x_ctr + bbox.xmin ) / 2;
                    int x_ctr_max = ( x_ctr + bbox.xmax ) / 2;
                    int y_ctr_min = ( y_ctr + bbox.ymin ) / 2;
                    int y_ctr_max = ( y_ctr + bbox.ymax ) / 2;
                    // ROS_INFO("x_ctr_min : %i", x_ctr_min);
                    // ROS_INFO("x_ctr_max : %i", x_ctr_max);
                    // ROS_INFO("y_ctr_min : %i", y_ctr_min);
                    // ROS_INFO("y_ctr_max : %i", y_ctr_max);

                    int array_num_y_ctr_min = ( width * y_ctr_min ) + x_ctr;
                    int array_num_y_ctr_max = ( width * y_ctr_max ) + x_ctr;
                    int array_num_x_ctr_min = ( width * y_ctr ) + x_ctr_min;
                    int array_num_x_ctr_max = ( width * y_ctr ) + x_ctr_max;
                    // ROS_INFO("array_num_y_ctr_min : %i", array_num_y_ctr_min);
                    // ROS_INFO("array_num_y_ctr_max : %i", array_num_y_ctr_max);
                    // ROS_INFO("array_num_x_ctr_min : %i", array_num_x_ctr_min);
                    // ROS_INFO("array_num_x_ctr_max : %i", array_num_x_ctr_max);

                    int num_pt = 0;
                    PointT pt;
                    if( !(std::isnan(cloud_transform->points[ array_num_y_ctr_min ].x )||std::isnan(cloud_transform->points[ array_num_y_ctr_min ].y)||std::isnan( cloud_transform->points[array_num_y_ctr_min].z))) {
                        num_pt++;
                        pt.x += cloud_transform->points[ array_num_y_ctr_min ].x;
                        pt.y += cloud_transform->points[ array_num_y_ctr_min ].y;
                        pt.z += cloud_transform->points[ array_num_y_ctr_min ].z;
                    } if( !(std::isnan(cloud_transform->points[ array_num_y_ctr_max ].x )||std::isnan(cloud_transform->points[ array_num_y_ctr_max ].y)||std::isnan( cloud_transform->points[array_num_y_ctr_max].z))) {
                        num_pt++;
                        pt.x += cloud_transform->points[ array_num_y_ctr_max ].x;
                        pt.y += cloud_transform->points[ array_num_y_ctr_max ].y;
                        pt.z += cloud_transform->points[ array_num_y_ctr_max ].z;
                    } if(!(std::isnan(cloud_transform->points[ array_num_x_ctr_min ].x )||std::isnan(cloud_transform->points[ array_num_x_ctr_min ].y)||std::isnan( cloud_transform->points[array_num_x_ctr_min].z))) {
                        num_pt++;
                        pt.x += cloud_transform->points[ array_num_x_ctr_min ].x;
                        pt.y += cloud_transform->points[ array_num_x_ctr_min ].y;
                        pt.z += cloud_transform->points[ array_num_x_ctr_min ].z;
                    } if(!(std::isnan(cloud_transform->points[ array_num_x_ctr_max ].x )||std::isnan(cloud_transform->points[ array_num_x_ctr_max ].y)||std::isnan( cloud_transform->points[array_num_x_ctr_max].z))) {
                        num_pt++;
                        pt.x += cloud_transform->points[ array_num_x_ctr_max ].x;
                        pt.y += cloud_transform->points[ array_num_x_ctr_max ].y;
                        pt.z += cloud_transform->points[ array_num_x_ctr_max ].z;
                    }
                    if ( num_pt == 0 ) continue;
                    obj_pose.pose.position.x = pt.x/num_pt;
                    obj_pose.pose.position.y = pt.y/num_pt;
                    obj_pose.pose.position.z = pt.z/num_pt;
                } else {
                    obj_pose.pose.position.x = cloud_transform->points[ array_num ].x;
                    obj_pose.pose.position.y = cloud_transform->points[ array_num ].y;
                    obj_pose.pose.position.z = cloud_transform->points[ array_num ].z;
                }

                obj_pose.detect_id          = i;
                obj_pose.pose.orientation.x = 0.0;
                obj_pose.pose.orientation.y = 0.0;
                obj_pose.pose.orientation.z = 0.0;
                obj_pose.pose.orientation.w = 1.0;

                geometry_msgs::TransformStamped transformStampedGeoObj;
                transformStampedGeoObj.header.frame_id = map_frame_name_;
                transformStampedGeoObj.child_frame_id = bbox.Class;
                transformStampedGeoObj.header.stamp = ros::Time::now();
                transformStampedGeoObj.transform.translation.x = obj_pose.pose.position.x;
                transformStampedGeoObj.transform.translation.y = obj_pose.pose.position.y;
                transformStampedGeoObj.transform.translation.z = obj_pose.pose.position.z;
                transformStampedGeoObj.transform.rotation.x = obj_pose.pose.orientation.x;
                transformStampedGeoObj.transform.rotation.y = obj_pose.pose.orientation.y;
                transformStampedGeoObj.transform.rotation.z = obj_pose.pose.orientation.z;
                transformStampedGeoObj.transform.rotation.w = obj_pose.pose.orientation.w;

                object_pose_array.object_poses.push_back(obj_pose);
                tfBroadcaster_.sendTransform(transformStampedGeoObj);
            }
            pub_obj_poses_.publish(object_pose_array);
        }

        // bool callbackSubscriberSwitch( sobit_common_msg::RunCtrl::Request &req, sobit_common_msg::RunCtrl::Response &res ) {
        //     if ( req.request ) {
        //         ROS_INFO ("[ PlaceablePoseDetection ] Turn on the PlaceablePoseDetection" );
        //     } else {
        //         ROS_INFO ("[ PlaceablePoseDetection ] Turn off the PlaceablePoseDetection" );
        //     }
        //     execute_flag_ = req.request;
        //     res.response = true;
        //     return true;
        // }

    public:
        ObjPosePublisher() : tfListener_(tfBuffer_) {
            nh_.param("map_frame_name", map_frame_name_, std::string("map"));
            nh_.param("base_frame_name", base_frame_name_, std::string("base_footprint"));
            nh_.param("cloud_topic_name", cloud_topic_name_, std::string("/points2"));
            nh_.param("img_topic_name", img_topic_name_, std::string("/rgb/image_raw"));

            nh_.param("cluster_tolerance", cluster_tolerance, 0.01);
            nh_.param("min_clusterSize", min_clusterSize, 100);
            nh_.param("max_clusterSize", max_clusterSize, 20000);
            nh_.param("leaf_size", leaf_size, 0.005);

            nh_.param("execute_flag", execute_flag_, true);
            // nh_.param("pub_result_flag", pub_result_flag_, true);

            sub_bboxes_.reset(new message_filters::Subscriber<sobit_common_msg::BoundingBoxes>(nh_, "objects_rect", 5));
            sub_cloud_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud_topic_name_, 5));
            sub_img_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, img_topic_name_, 5));
            
            sync_.reset(new message_filters::Synchronizer<BBoxesCloudSyncPolicy>(
                BBoxesCloudSyncPolicy(200), *sub_bboxes_, *sub_cloud_, *sub_img_));
            sync_->registerCallback(boost::bind(&ObjPosePublisher::callback_BBoxCloud, this, _1, _2, _3));
            // srv_subscriber_switch_ = nh_.advertiseService( "run_ctr", &ObjPosePublisher::callbackSubscriberSwitch, this);

        }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "obj_point_publisher_node");
    ObjPosePublisher  obj_point_publisher;
    ros::AsyncSpinner spinner(1);

    spinner.start();
    ros::waitForShutdown();
    
    return 0;
}