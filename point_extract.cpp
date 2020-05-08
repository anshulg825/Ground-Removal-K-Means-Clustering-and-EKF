#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#define PCL_NO_PRECOMPILE
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <velodyne_pointcloud/point_types.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
using namespace std;

#define VPoint velodyne_pointcloud::PointXYZIR

pcl::PointCloud<VPoint>::Ptr CloudOut(new pcl::PointCloud<VPoint>());


class extract{
public:
	extract();
private:
	ros::NodeHandle n;
	ros::Subscriber extracted_points; 
    ros::Publisher updated_points;
	void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);    
};

	extract::extract(){
		extracted_points = n.subscribe("/kitti/velo/pointcloud", 10, velodyne_callback);
		updated_points = n.advertise<sensor_msgs::PointCloud2>("/updated_cloud",10);
	}
	void extract::velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

		pcl::PointCloud<VPoint> laserCloudInput;
		pcl::fromROSMsg(*laserCloudMsg, laserCloudInput);
		velodyne_pointcloud::PointXYZIR point;
		for(int i = 0; i < laserCloudInput.points.size(); i++){
			if(laserCloudInput.points[i].x > -10 && laserCloudInput.points[i].x < 10){
				point.x = laserCloudInput.points[i].x;
				point.y = laserCloudInput.points[i].y;
				point.z = laserCloudInput.points[i].z;
				point.intensity = laserCloudInput.points[i].intensity;
	        	point.ring = laserCloudInput.points[i].ring;
				CloudOut -> points.push_back(point);
			}
		}
		sensor_msgs::PointCloud2 updated;
	    pcl::toROSMsg(*CloudOut, updated);       
        updated_points.publish(updated);
}

int main(int argc, char** argv){
	 ros::init (argc, argv, "extraction");	
	 extract node;
	 ros::spin();
}