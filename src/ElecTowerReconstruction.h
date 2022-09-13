#pragma once
#pragma once
#include<vector>
#include<string>
#include<iostream>
#include<fstream>
#include<stack>
#include <Eigen/Core>
#include<pcl/common/common.h>
#include <pcl/common/centroid.h>
#include<pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/io/ply_io.h>
#include<pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/kdtree/kdtree_flann.h>

//opencv
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class ElecTowerReconsrutction {

public:
	void TopReconstrution(string &filepath);
	void SetOutPath(string &filepath);
	void SetTowerType(int i);
private:

//===============================塔头重建================================
//准备数据
	PointCloudT::Ptr DataInput(string& filepath);
	PointCloudT::Ptr PointCloudFilter(PointCloudT::Ptr &cloud);
	PointCloudT::Ptr Projection_X(PointCloudT::Ptr &cloud);//点云投影，两种投方式
	PointCloudT::Ptr Projection_Y(PointCloudT::Ptr &cloud);														 
	void Rerotate_X(PointCloudT::Ptr &cloud);
	void Rerotate_Y(PointCloudT::Ptr &cloud);
	void AABBCompute(PointCloudT::Ptr &cloud);
	void PointScale(PointCloudT::Ptr &cloud,
	double scale_x, double scale_y);//缩放
	void TowerJudge();

	Mat PointGrid(PointCloudT::Ptr &cloud);//二维图构建
	void CornorExtract(Mat& img);
	PointCloudT::Ptr  Img2Points(Mat& Image);
	PointCloudT::Ptr PointSymmetry(PointCloudT::Ptr &points,int state);
	void CordinateFill(PointCloudT::Ptr &cornor, double width);//直接通过给定的宽度对称
	void DataOutput(PointCloudT::Ptr &ipt, PointCloudT::Ptr &opt,
	PointCloudT::Ptr &Sipt, PointCloudT::Ptr &Sopt, string& filepath);
//===============================塔身重建================================




private:
	string FilePath;
	int TowerType;


	PointT OriginMaxAABB;
	PointT OriginMinAABB;
	string filepath;

	double PX_xmin;
	double PY_ymin;
	vector<pair<int,int>>CornorPOints;
	vector<pair<int, int>> outerpoints;
	PointCloudT::Ptr oupts;//外角点
	vector<pair<int, int>> innerpoints;
	PointCloudT::Ptr inpts; //内角点
	PointCloudT::Ptr Syminpts;
	PointCloudT::Ptr Symoupts;

};