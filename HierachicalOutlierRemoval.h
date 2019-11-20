#pragma once
#include "PCLExtend.h"
#include "Statistics.h"
#include "SurfaceFitting.h"
#include "Color.h"
#define ELAPSED(START,END)  (((END.tv_sec  - START.tv_sec) * 1000000u + END.tv_usec - START.tv_usec) / 1.e6)

class HierachicalOutlierRemoval
{
	public:
		pcl::PointCloud<PointType>::Ptr cloud_;
		pcl::PointCloud<PointType>::Ptr cloud_filtered_;
		pcl::PointCloud<PointType>::Ptr cloud_outlier_;		
		MODEL mode_;
		double mean_dist_;
		pcl::octree::OctreePointCloudSearch<PointType>::Ptr octree_;
		
		// Internal Function
		HierachicalOutlierRemoval(pcl::PointCloud<PointType>::Ptr cloud=NULL);
		void SplitIntoPrimitives(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer=NULL);
		void FittingBasedOutlierRemoval(pcl::PointCloud<PointType>::Ptr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer=NULL);
		void PatchFittingAndRendering(pcl::PointCloud<PointType>::Ptr cloud,boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer=NULL);
		
		// External Function
		void Type3RemovalBasedOnRansac(MODEL mode=PLANE, pcl::PointCloud<PointType>::Ptr cloud=NULL,  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer=NULL);
};