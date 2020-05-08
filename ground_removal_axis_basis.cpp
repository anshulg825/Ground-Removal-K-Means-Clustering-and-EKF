 

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#define PCL_NO_PRECOMPILE
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
//#include <pcl/ros/conversions.h>
#include <velodyne_pointcloud/point_types.h>

#include <Eigen/Dense>
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

#define VPoint velodyne_pointcloud::PointXYZIR

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
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (uint16_t, label, label))

#define ClusterPointXYZIRL clustering::PointXYZIRL



pcl::PointCloud<VPoint>::Ptr g_seeds_points(new pcl::PointCloud<VPoint>()); // after seed extraction 
pcl::PointCloud<VPoint>::Ptr g_ground_points(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr g_not_ground_points(new pcl::PointCloud<VPoint>());
pcl::PointCloud<ClusterPointXYZIRL>::Ptr g_all_pc(new pcl::PointCloud<ClusterPointXYZIRL>());

std::vector < pcl::PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <PointCloud <VPoint>::Ptr > > pg_seeds_points;
std::vector < pcl::PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <PointCloud <VPoint>::Ptr > > pg_ground_points;
std::vector < pcl::PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <PointCloud <VPoint>::Ptr > > pg_not_ground_points;
std::vector < pcl::PointCloud<ClusterPointXYZIRL>::Ptr, Eigen::aligned_allocator <PointCloud <ClusterPointXYZIRL>::Ptr > > pg_all_pc;

//std::vector<pcl::PointCloud<VPoint>::Ptr > pcs; or this one

//sorting points based on z points 
bool z_comparison(VPoint a , VPoint b){
	return a.z<b.z;
};

bool x_comparison(VPoint a , VPoint b){
	return a.x<b.x;
};

class plane_fit{
public:
	plane_fit();
private:
	ros::NodeHandle nh;
	ros::Subscriber points_sub;
	ros::Publisher ground_points_pub;
	ros::Publisher groudless_points_pub;
	ros::Publisher all_points_pub;

	// Parameters for sensors and plane fit 
	//CHeck once with papere

	
	double sensor_height = 2.5;
	int seg_no = 1; /// to be  implemented after tuning 
	int iter_no = 4;
	int lpr_no = 20;
	double th_seeds = 1.4;
	double th_dist = 0.7;
	int plane_parts=9;

	void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
	void estimate_plane(void);
	void extract_initial_seeds(const pcl::PointCloud<VPoint>& p_sorted);

	float d; /// equation parameter 
	// Normal  parameters
	MatrixXf normal; // current size 0x0;

	float th_dist_d;





};

	plane_fit::plane_fit(){
		points_sub = nh.subscribe("/velodyne_points", 2, &plane_fit::velodyne_callback,this);

		groudless_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_no_ground",2);
		ground_points_pub= nh.advertise<sensor_msgs::PointCloud2>("/points_ground",2);
		all_points_pub=nh.advertise<sensor_msgs::PointCloud2>("/all_points",2);

	}

	void plane_fit::extract_initial_seeds(const pcl::PointCloud<VPoint>& p_sorted){
		double sum = 0;
		int count = 0;
		
		for(int i =0; i<p_sorted.points.size() ; i++){
			sum += p_sorted.points[i].z;
			count++;

		}
		//cout count;
		double lpr_height = count!=0?sum/count:0;
		g_seeds_points->clear();
		for(int i=0; i<p_sorted.points.size(); i++){
			if(p_sorted.points[i].z<lpr_height + th_seeds){
				g_seeds_points->points.push_back(p_sorted.points[i]);
			}
		}




	}

	void plane_fit::estimate_plane(void){
		Eigen::Matrix3f covariance_matrix;
		Eigen::Vector4f points_mean;

		pcl::computeMeanAndCovarianceMatrix(*g_ground_points, covariance_matrix, points_mean); /////Amazing Amazing amazing Amazing 
		JacobiSVD<MatrixXf> svd(covariance_matrix,Eigen::DecompositionOptions::ComputeFullU);

		////Check these function from Matrixf 
		normal = svd.matrixU().col(2);
		// A= UBUt 
		// last eigen vector will give normal 

		// check from paper once 

		Eigen::Vector3f seeds_mean = points_mean.head<3>();

		d= -(normal.transpose()*seeds_mean)(0,0);
		 th_dist_d = th_dist -d ;

		// Check for covariance 



	}

	void plane_fit::velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
		pcl::PointCloud<VPoint> laserCloudInput;
		std::vector < PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <PointCloud <VPoint>::Ptr > > parts;
		std::vector < PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <PointCloud <VPoint>::Ptr > > parts_original;
		// conversion of velodyne point struct to pcl data point 
		pcl::fromROSMsg(*laserCloudMsg, laserCloudInput);

		pcl::PointCloud<VPoint> laserCloudInput_original;
		// conversion of velodyne point struct to pcl data point 
		pcl::fromROSMsg(*laserCloudMsg, laserCloudInput_original);
		ClusterPointXYZIRL point;
	
    	for(size_t i=0;i<laserCloudInput.points.size();i++){
	        point.x = laserCloudInput.points[i].x;
	        point.y = laserCloudInput.points[i].y;
	        point.z = laserCloudInput.points[i].z;
	        point.intensity = laserCloudInput.points[i].intensity;
	        point.ring = laserCloudInput.points[i].ring;
	        point.label = 0u;// 0 means uncluster
	        g_all_pc->points.push_back(point);
	    }


		// Sort on basis of z axis value to get initial seeds 

		sort(laserCloudInput.points.begin(), laserCloudInput.points.end(),z_comparison);

		pcl::PointCloud<VPoint>::iterator it = laserCloudInput.points.begin();
		for(int i =0;i<laserCloudInput.points.size();i++){
			if(laserCloudInput.points[i].z < -1.5*sensor_height){
				it++;
			}
			else {
				break;
			}
		}


		laserCloudInput.points.erase(laserCloudInput.points.begin(),it);
	
	
		//initilize all 9 pcl
		sort(laserCloudInput.points.begin(), laserCloudInput.points.end(),x_comparison);
		
		for(int s=0,int n=1;s<plane_parts;s++,n++){
			int u= (laserCloudInput.points.size())/9;
	    	for(size_t i=((n-1)*u);i<(n*u);i++){
	        
	        (*parts[s]).x = laserCloudInput.points[i].x;
	        (*parts_original[s]).x = laserCloudInput.points[i].x;
	        (*parts[s]).y = laserCloudInput.points[i].y;
	        (*parts_original[s]).y = laserCloudInput.points[i].y;
	        (*parts[s]).z = laserCloudInput.points[i].z;
	        (*parts_original[s]).z = laserCloudInput.points[i].z;
	        (*parts[s]).intensity = laserCloudInput.points[i].intensity;
	        (*parts_original[s]).intensity = laserCloudInput.points[i].intensity;
	        (*parts[s]).ring = laserCloudInput.points[i].ring;
	        (*parts_original[s]).ring = laserCloudInput.points[i].ring;
			       
	    	}

	    }
		//now new 9 vector storing diffrent parts of pcl
		
		for(int i=0;i<plane_parts<i++){

		extract_initial_seeds(*parts[i]);
		pg_ground_points[i]=pg_seeds_points[i];
		for(int k=0;k< iter_no ;k++){
			estimate_plane();
			pg_ground_points[i]->clear();
			pg_not_ground_points[i]->clear();

			Eigen::MatrixXf points((*parts_original[i]).points.size(),3);

        	int j =0;
        	for(auto p:(*parts_original[i]).points){
            	points.row(j++)<<p.x,p.y,p.z;
        	}
			/// need to find distace of points from the plane 

			Eigen::VectorXf result = points*normal;

			for(int r=0;r<result.rows();r++){
				if(result[r]<th_dist_d){
					pg_all_pc[i]->points[r].label = 1u;// means ground
					pg_ground_points[i]->points.push_back((*parts_original[i])[r]);
				}
				else{
					pg_all_pc[i]->points[r].label = 0u;// means not ground and non clusterred
					pg_not_ground_points[i]->points.push_back((*parts_original[i])[r]);
				}
			}

		}


		}

		//concanate all 9 planes


		


		// Publish the fucking points

		sensor_msgs::PointCloud2 ground_msg;
		pcl::toROSMsg(*g_ground_points,ground_msg);
		ground_msg.header.stamp = laserCloudMsg->header.stamp;
    	ground_msg.header.frame_id = laserCloudMsg->header.frame_id;
    	ground_points_pub.publish(ground_msg);

	    sensor_msgs::PointCloud2 groundless_msg;
	    pcl::toROSMsg(*g_not_ground_points, groundless_msg);
	    groundless_msg.header.stamp = laserCloudMsg->header.stamp;
	    groundless_msg.header.frame_id = laserCloudMsg->header.frame_id;
	    groudless_points_pub.publish(groundless_msg);
	    
	    sensor_msgs::PointCloud2 all_points_msg;
	    pcl::toROSMsg(*g_all_pc, all_points_msg);

	    all_points_msg.header.stamp = laserCloudMsg->header.stamp;
	    all_points_msg.header.frame_id = laserCloudMsg->header.frame_id;
	    all_points_pub.publish(all_points_msg);
	    g_all_pc->clear();




	}


int main(int argc, char **argv)
{

    ros::init(argc, argv,"plane_fit_node");
    plane_fit node;
    ros::spin();

    return 0;

}


/// Model parameter for ground is assumed to be  ax+ by +cz+d =0
//normal = a,b,c 
/// th_dist_d = threshold_dist -d ;

/*vector < PointCloud<PointXYZ>::Ptr, Eigen::aligned_allocator <PointCloud <PointXYZ>::Ptr > > sourceClouds;

    //save PointClouds to array
    for (int i = 1; i < (argc - 1); i++)
    {
        PointCloud<PointXYZ>::Ptr sourceCloud(new PointCloud<PointXYZ>);
        if (io::loadPCDFile<PointXYZ>(argv[i], *sourceCloud) != 0)
        {
            return -1;
        }
        sourceClouds.push_back(sourceCloud);
       // sourceCloud->clear();
    }


*/
