#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#define PCL_NO_PRECOMPILE
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <velodyne_pointcloud/point_types.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>

namespace clustering
{
  struct PointXYZIRL
  {
    PCL_ADD_POINT4D;                    
    float    intensity;                 
    uint16_t ring;                      
    uint16_t label;                     
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     
  } EIGEN_ALIGN16;

};
POINT_CLOUD_REGISTER_POINT_STRUCT(clustering::PointXYZIRL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity1)
                                  (uint16_t, ring, ring)
                                  (uint16_t, label, label))

#define ClusterPointXYZIRL clustering::PointXYZIRL

pcl::PointCloud<ClusterPointXYZIRL>::Ptr cluster(new pcl::PointCloud<ClusterPointXYZIRL>());
std::vector<ClusterPointXYZIRL> centroid;


class Kmeans{
private:
	ros::NodeHandle nh;
	ros::Subscriber points_sub;
	ros::Publisher ground_points_pub;

	int iter_no=100;

	void callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
	double distance(ClusterPointXYZIRL a , ClusterPointXYZIRL b);
	
public:
Kmeans();
};

Kmeans::Kmeans(){
	points_sub = nh.subscribe("/velodyne_points", 2, &Kmeans::callback,this);
	
	cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("/cluster",2);
}

double Kmeans::distance(ClusterPointXYZIRL a , ClusterPointXYZIRL b){

	double dis;
	double X_real = a.x - b.x;
	double Y_real = a.y - b.y; 
	dis = (pow(X_real,2) + pow(Y_real,2));
	double d = sqrt(dis); 
	return d;
}

void Kmeans::callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

	int n=10;

	pcl::PointCloud<ClusterPointXYZIRL> laserCloudInput;
 	pcl::fromROSMsg(*laserCloudMsg, laserCloudInput);

 	ClusterPointXYZIRL point;
 	int j=0;
 	//choose n random points
 	for(int i=0;i<n;i++){
       
       j = rand()%(laserCloudInput.points.size());
       
       //verify whether j is already used or not
       point.x = laserCloudInput.points[i].x;
	   point.y = laserCloudInput.points[i].y;
	   point.z = 0;
	   point.intensity = laserCloudInput.points[i].intensity;
	   point.ring = laserCloudInput.points[i].ring;
	   point.label = i;// 0 means uncluster

	   centroid.push_back(point);

 	}
 	
 	double dist=100,min_dist;
 	for(int i=0;i<iter_no;i++){
 	// change cluster_id a/c to nearest centroid
 		
 		for(int k=0;i<laserCloudInput.points.size();k++){
 			
 			point.x = laserCloudInput.points[k].x;
	        point.y = laserCloudInput.points[k].y;
	       	int count=0;
	        for(int q=0;q<10;q++){

	        	double a = distance(point,centroid[q]);
	        	if(a<dist){
	
	        		count=q;
	        	}
	        }
	    point.label = count;
	   	cluster->points.push_back(point);
	   	count=0;
 		}


 	// Recalculating the center of each cluster
 		for (int i = 0; i < laserCloudInput.points.size(); i++)
 		{
 			
 			for(int q=0;q<10;q++){

 				if( (laserCloudInput.points[i].label)==q ){

 					
 				}

 			}
 		}



 	}

	sensor_msgs::PointCloud2 cluster_msg;
	pcl::toROSMsg(*cluster,cluster_msg);
	cluster_msg.header.stamp = laserCloudMsg->header.stamp;
    cluster_msg.header.frame_id = laserCloudMsg->header.frame_id;
    ground_points_pub.publish(cluster_msg);

}

int main(int argc, char **argv)
{

    ros::init(argc, argv,"k_means_node");
    Kmeans node;
    ros::spin();
    return 0;
}