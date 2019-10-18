#pragma once
#include "PCLExtend.h"
#include "Statistics.h"
#include "SurfaceFitting.h"
#include "Color.h"
class HierachicalOutlierRemoval
{
	public:
		void GetLeafShow(pcl::PointCloud<PointType>::Ptr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
		void FittingBasedOutlierRemoval(pcl::PointCloud<PointType>::Ptr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
		void PatchFittingAndRendering(pcl::PointCloud<PointType>::Ptr cloud,boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
		void RANSACFittingBasedOutlierRemoval(pcl::PointCloud<PointType>::Ptr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
};