#include "PCLExtend.h"
double ComputeMeanDistance(const pcl::PointCloud<PointType>::ConstPtr cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!std::isfinite((*cloud)[i].x))
		{
			continue;
		}
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}


double ComputeMaxDistance(const pcl::PointCloud<PointType>::ConstPtr cloud)
{
	double rst = 0.0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!std::isfinite((*cloud)[i].x))
		{
			continue;
		}
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			double dist_tmp=sqrt(sqr_distances[1]);
			rst = rst>dist_tmp ? rst:dist_tmp;
		}
	}

	return rst;
}

vector<double> StatisticNearestDistance(const pcl::PointCloud<PointType>::ConstPtr cloud)
{
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	vector<double> rst;
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{ 
		if (!std::isfinite((*cloud)[i].x))
		{
			continue;
		}
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{			
			rst.push_back(sqr_distances[1]);
			++n_points;
		}
	}	
	
	return rst;
}

void TransformPointCloud(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_tf,Eigen::Affine3f tf)
{
	cloud_tf->points.resize(cloud->points.size());
	
	#pragma omp parallel for
	for(int i=0;i<cloud->points.size();i++)
	{
		Eigen::Vector3f v1(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);		
		Eigen::Vector3f v2=tf*v1;
		cloud_tf->points[i].x=v2(0,0);
		cloud_tf->points[i].y=v2(1,0); 
		cloud_tf->points[i].z=v2(2,0);
	}
}
