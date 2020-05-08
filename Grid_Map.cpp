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
#define VPoint velodyne_pointcloud::PointXYZIR
namespace clustering
{
  struct PointXYZIRL
  {
    PCL_ADD_POINT4D;                    
    float    intensity;    
    float    height;             
    uint16_t ring;                      
    uint16_t label;                     
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     
  } EIGEN_ALIGN16;
};
POINT_CLOUD_REGISTER_POINT_STRUCT(clustering::PointXYZIRL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, height, height)
                                  (uint16_t, ring, ring)
                                  (uint16_t, label, label))
#define ClusterPointXYZIRL clustering::PointXYZIRL
pcl::PointCloud<clustering::PointXYZIRL>::Ptr grid_cloud(new pcl::PointCloud<clustering::PointXYZIRL>);
class grid{
public:
	grid();
private:
	ros::NodeHandle n;
    ros::Publisher grid_points;
    ros::Subscriber grid_subscribe;
    void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msgs);
};
grid::grid(){
	grid_subscribe = n.subscribe("/points_no_ground", 2, &grid::callback, this);
	grid_points = n.advertise<sensor_msgs::PointCloud2>("/grid_points", 2);
}
bool MaxHeightCompare(clustering::PointXYZIRL a, clustering::PointXYZIRL b){
	return a.z > b.z;
}
bool Ysort(clustering::PointXYZIRL a, clustering::PointXYZIRL b){
	return a.y < b.y;
}
void grid::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msgs){
	pcl::PointCloud<ClusterPointXYZIRL> grid_point_data;
	pcl::fromROSMsg(*cloud_msgs, grid_point_data);	
	sort(grid_point_data.points.begin(), grid_point_data.points.end(), MaxHeightCompare);
	clustering::PointXYZIRL point;
	//removing ovehead points
	pcl::PointCloud<ClusterPointXYZIRL>::iterator it = grid_point_data.points.begin();
	sort(grid_point_data.points.begin(), grid_point_data.points.end(), Ysort);
	for(size_t i = 0; i < grid_point_data.points.size(); i++){
		if(grid_point_data.points[i].y < -8){
			it++;
		}else {
			break;
		}
	}
	grid_point_data.erase(grid_point_data.begin(), it);
	it = grid_point_data.points.begin();
	for(size_t i = 0; i < grid_point_data.points.size(); i++){
		if(grid_point_data.points[i].y < 8){
			it++;
		}else {
			break;
		}
	}
	grid_point_data.erase(it, grid_point_data.points.end());

	std::cout << "after_removing x's" << grid_point_data.points.size() << std::endl;

	it = grid_point_data.points.begin();
	for(int i = 0; i < grid_point_data.points.size(); i++){
		if(grid_point_data.points[i].z > 0){
			it++;                                 
		}else {
			break;
		}
	}
	std::cout << "after_removing above 0 is" << grid_point_data.points.size() << std::endl;

	grid_point_data.erase(grid_point_data.begin(), it);
	it = grid_point_data.points.begin();
	for(int i = 0; i < grid_point_data.points.size(); i++){
		point.x = grid_point_data.points[i].x;
		point.y = grid_point_data.points[i].y;
		point.z = 0;
		point.height = grid_point_data.points[i].z;
	    point.intensity = grid_point_data.points[i].intensity;
	    point.ring = grid_point_data.points[i].ring;
	    point.label = 0u;
		grid_cloud -> points.push_back(point);
		it = grid_point_data.points.begin() + i;
		for(int j = i+1; j < grid_point_data.points.size(); j++){
			it ++;
			if(grid_point_data.points[j].x == point.x && grid_point_data.points[j].y == point.y){
				grid_point_data.points.erase(it);
			}
		}
	}
	std::cout << "grid_cloud size is" << grid_cloud->points.size() << std::endl;
	sensor_msgs::PointCloud2 updated_cloud;
	pcl::toROSMsg(*grid_cloud, updated_cloud);
	updated_cloud.header.stamp = cloud_msgs->header.stamp;
   	updated_cloud.header.frame_id = cloud_msgs->header.frame_id;
   	grid_points.publish(updated_cloud);
   	grid_cloud -> clear();
}
int main(int argc, char **argv){
	std::cout<<"4";
	ros::init(argc, argv, "GridpointsNode");
	grid node;
	ros::spin();
}
