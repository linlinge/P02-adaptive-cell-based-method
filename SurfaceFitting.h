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
#include "Statistics.h"
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include "BasicGeometry.h"
#include "V3.hpp"
#include <set>
using namespace cv;
using namespace std;

enum MODEL{PLANE,POLY33};
class SurfaceFitting
{
	public:		
		double x_min_,x_max_;
		double y_min_,y_max_;
		double z_min_,z_max_;
		vector<double> errors_;
		Mat P_;
		MODEL mode_;
		pcl::PointCloud<PointType>::Ptr cloud_;		
		vector<int> inlier_idx_;
		vector<int> outlier_idx_;
		
		// Internal Function
		void Poly33(Mat& x, Mat& y,Mat& z);
		void PlaneFitting(pcl::PointCloud<PointType>::Ptr cloud);
		double AlgebraicDistacne(double datx, double daty, double datz);
		double EuclideanDistance(double datx, double daty, double datz);
		double CalculateZ(double datx,double daty);
		void DrawSurface(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
		
		
		// External Function
		void FittingBasedOnPoly33(pcl::PointCloud<PointType>::Ptr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer=NULL);
		void FittingBasedOnRansac(pcl::PointCloud<PointType>::Ptr cloud, MODEL type=PLANE, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer=NULL);
		void GetModelError();
};