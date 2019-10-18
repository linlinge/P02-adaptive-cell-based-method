#pragma once
#include <iostream>
#include <stdio.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <opencv2/opencv.hpp>
#include "PCLExtend.h"
using namespace cv;
using namespace std;
class SurfaceFitting
{
	public:
		Mat P_;
		double x_min_,x_max_;
		double y_min_,y_max_;
		double z_min_,z_max_;
		vector<double> errors_;
		pcl::PointCloud<PointType>::Ptr cloud_;
		
		Mat Poly33(Mat& x, Mat& y,Mat& z);
		Mat Poly33(pcl::PointCloud<PointType>::Ptr cloud);
		double Calculate(double datx,double daty);
		double AlgebraicDistacne(double datx,double daty, double datz);
		void CalculateErrors();
		vector<int> Ransac(pcl::PointCloud<PointType>::Ptr cloud); 
		void DrawSurface(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
};
