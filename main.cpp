#include <iostream>
#include <map>
#include <ostream>
#include "PCLExtend.h"
#include <time.h>
#include <stdio.h>
#include "HierachicalOutlierRemoval.h"
using namespace std;

int main(int argc, char** argv)
{
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);

    if (pcl::io::loadPLYFile<PointType>(argv[1], *cloud) == -1) // load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
	
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	
	HierachicalOutlierRemoval hor;
	hor.GetLeafShow(cloud,viewer);
	hor.PatchFittingAndRendering(cloud,viewer);
		
    viewer->setBackgroundColor (1.0f, 1.0f, 1.0f);	
	
    //设置点云颜色，该处为单一颜色设置
    pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color(cloud);

    //添加需要显示的点云数据
    viewer->addPointCloud<PointType> (cloud, multi_color, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");

    //viewer->showCloud(cloud);
    while(!viewer->wasStopped()){
        viewer->spinOnce (10);
        boost::this_thread::sleep (boost::posix_time::microseconds (10));
    }
	return 0;
}