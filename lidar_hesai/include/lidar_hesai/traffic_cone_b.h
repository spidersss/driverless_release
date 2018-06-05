#ifndef __FILTER_TEST_H_
#define __FILTER_TEST_H_

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#ifndef M_PI
#define M_PI 3.1415926
#endif
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;  
PointCloud space_part(PointCloud cloud, double x_distance, double y_distance, double z_distance);
PointCloud outlier_filter(PointCloud cloud, int MeanK, double Thresh);
PointCloud center_cluster(PointCloud cloud, double Tolerance, int MinSize, int MaxSize);
double steerCreator(PointCloud cloud);
#endif

PointCloud space_part(PointCloud cloud, double x_distance, double y_distance, double z_distance)
{
	sensor_msgs::PointCloud2 output;
	PointCloud cloud_filtered;
	//std::cout<<cloud.points.size()<<std::endl;
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator it;
	for(it = cloud.points.begin(); it != cloud.points.end(); it++)
	{	
		
		/*if(it->x > (-1.0 * x_distance) && it->x < x_distance && it->y < 0 && it->y > y_distance && it->z > z_distance && it->z < 1 && it->x !=NAN && it->y  != NAN && it->z != NAN){*/
		if((it->y < (it->x * 13.45 + 51.4375-10)) && (it->y > (it->x*13.45 + 51.4375 - 100 ))){
			//it->x = 0;
			cloud_filtered.points.push_back (*it);
			//std::cout<<it->x<<"\t"<<it->y<<"\t"<<it->z<<std::endl;
		}
	}
	cloud_filtered.header = cloud.header;
	cloud_filtered.width = cloud_filtered.points.size ();
  	cloud_filtered.height = 1;
  	cloud_filtered.is_dense = false;
	//std::cout<<cloud_filtered.points.size()<<std::endl;
	
	return cloud_filtered;
}

PointCloud outlier_filter(PointCloud cloud, int MeanK, double Thresh)
{
	PointCloud cloud_filtered;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
	statFilter.setInputCloud(cloud.makeShared());
	statFilter.setMeanK(MeanK);
	statFilter.setStddevMulThresh(Thresh);
	statFilter.filter(cloud_filtered);
	return cloud_filtered;
}

PointCloud center_cluster(PointCloud cloud, double Tolerance, int MinSize, int MaxSize)
{
	//PointCloud cloud_cluster;//存储每个类
	PointCloud cloud_center;// 存储每个类的质心
	pcl::PointXYZ point_center;//存储质心
	
  	// 创建用于提取搜索方法的kdtree树对象
 	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new 		pcl::search::KdTree<pcl::PointXYZ>);
  	tree->setInputCloud (cloud.makeShared());

  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //欧式聚类对象
  	ec.setClusterTolerance (Tolerance);                     // 设置近邻搜索的搜索半径为0.2m
  	ec.setMinClusterSize (MinSize);                 //设置一个聚类需要的最少的点数目为5
  	ec.setMaxClusterSize (MaxSize);               //设置一个聚类需要的最大点数目为2500
  	ec.setSearchMethod (tree);                    //设置点云的搜索机制
  	ec.setInputCloud (cloud.makeShared());
  	ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中
  	//迭代访问点云索引cluster_indices,直到分割处所有聚类
  	int j = 0;
  	int count = 0;
  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  { 
  	//迭代容器中的点云的索引，并且分开每个点云的高度值
  	count = 0;
  	point_center.x = 0;
  	point_center.y = 0;
  	point_center.z = 0;
  	std::cout<<"id:"<<j+1<<"\t";
  	if(it->indices.size() > 300) continue;//将较大的物体，例如人排除掉
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)	{
     //设置保存点云的属性问题
	count ++;
    //cloud_cluster.points.push_back (cloud.points[*pit]);
    
    point_center.x += cloud.points[*pit].x;
    point_center.y += cloud.points[*pit].y;
    point_center.z = 0.0;
    }
    point_center.x /=  (double)count;
    point_center.y /=  (double)count;
    cloud_center.points.push_back(point_center);
    
  	j++;
     
    std::cout<<"number:"<<count<<std::endl;
  }
   
  //cloud_cluster.header = cloud.header;
  cloud_center.header.frame_id = "pandar";
  cloud_center.width = cloud_center.points.size ();
  //std::cout<<"size:"<<cloud_cluster.points.size()<<std::endl;
  cloud_center.height = 1;
  cloud_center.is_dense = false;
  return cloud_center;
}

double steerCreator(PointCloud cloud)
{
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator iter;
	for(iter = cloud.points.begin(); iter != cloud.points.end(); iter++){
		std::cout<<"x:"<<iter->x<<"\t"<<"y:"<<iter->y<<std::endl;
	}
  	for(int i = 0; i < cloud.points.size(); i += 2){
  		double center_x = (cloud.points[i].x+cloud.points[i+1].x)/2.0;
  		double center_y = (cloud.points[i].y+cloud.points[i+1].y)/2.0;
		std::cout<<"center"<<i/2+1<<"("<<center_x<<","<<center_y<<")"<<std::endl;
		double disToNext = sqrt(center_x*center_x + center_y*center_y);
		if( disToNext > 1.0){
			double theta = (atan2(-1.0, 0) - atan2(center_y, center_x))/M_PI*180.0;	
			std::cout<<"theta"<<i/2+1<<":"<<theta<<"\tdisToNext:"<<disToNext<<std::endl;
			return theta;
		}
	}
	return -1;	
}
