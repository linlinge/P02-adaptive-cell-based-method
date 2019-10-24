#include "HierachicalOutlierRemoval.h"

void HierachicalOutlierRemoval::GetLeafShow(pcl::PointCloud<PointType>::Ptr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	// voxel record
	pcl::PointCloud<PointType>::Ptr voxel_record(new pcl::PointCloud<PointType>);
		
	// 存储体素的下标
	map<int, vector<Eigen::Vector3f>> _bodyVoxel;

	// 存储叶子节点
	//pcl::PointCloud<PointType>::Ptr  cloudLeaf(new pcl::PointCloud<PointType>);
	double mean_dist=ComputeMeanDistance(cloud);
    
	// 最小体素的边长
	float resolution = 80*mean_dist;
	pcl::octree::OctreePointCloudSearch<PointType> octree(resolution);

	//octree.setTreeDepth(8);
	octree.setInputCloud(cloud);
	
	// 从输入点云构建八叉树
	octree.addPointsFromInputCloud();
	
	// PointType searchPoint;
	int depth = octree.getTreeDepth();
	vector<Eigen::Vector3f> list_min;
	vector<Eigen::Vector3f> list_max;
	
	// 求出体素边界
	srand( (unsigned)time( NULL ) );
	for (auto it = octree.begin(depth);it != octree.end();++it){	
		if(it.isLeafNode()){
			// caclulate voxel centre
			Eigen::Vector3f  voxel_min, voxel_max;
			octree.getVoxelBounds(it, voxel_min, voxel_max);
			Eigen::Vector3f voxel_centre;
			voxel_centre=(voxel_min+voxel_max)/2.0f;		
			
			// establish voxel point cloud
			PointType pt = PointType(255, 0, 0);
			pt.x=voxel_centre.x();
			pt.y=voxel_centre.y();
			pt.z=voxel_centre.z();
			voxel_record->points.push_back(pt);							
			list_min.push_back(voxel_min);
			list_max.push_back(voxel_max);
			
			// Get points indices in each voxel
			auto leaf = it.getLeafContainer();
			std::vector<int> indices; 
			leaf.getPointIndices(indices);
			
			// extract point set in voxel
			
				pcl::PointCloud<PointType>::Ptr cloud_tmp(new pcl::PointCloud<PointType>);
				float r=rand()%100/100.0*255;
				float g=rand()%100/100.0*255;
				float b=rand()%100/100.0*255;
				for(int i=0;i<indices.size();i++)
				{
					cloud->points[indices[i]].r=r;
					cloud->points[indices[i]].g=g;
					cloud->points[indices[i]].b=b;
					cloud_tmp->points.push_back(cloud->points[indices[i]]);
				}
				
				/* SurfaceFitting sf;
				sf.Poly33(cloud_tmp);
				for(int i=0;i<indices.size();i++)
				{
					double dist_tmp=sf.AlgebraicDistacne(cloud->points[indices[i]].x,cloud->points[indices[i]].y,cloud->points[indices[i]].z);
					//cout<<dist_tmp<<endl;
					if(dist_tmp>0.0004)
					{
						cloud->points[indices[i]].r=255;
						cloud->points[indices[i]].g=0;
						cloud->points[indices[i]].b=0;
					}
					else
					{
						cloud->points[indices[i]].r=0;
						cloud->points[indices[i]].g=255;
						cloud->points[indices[i]].b=0;
					}
				} 
			*/
		}
	}
	
	// find voxel who is outlier
	 vector<double> statistic_tmp=StatisticNearestDistance(voxel_record);
	 
	/*
	ofstream fout("1.txt");
	for(int i=0;i<statistic_tmp.size();i++)
	{
		fout<<statistic_tmp[i]<<endl;
	}
	fout.close(); 
	*/
	
	
	// show outlier voxel  
	/* Statistics st(statistic_tmp);
	double list_mean=st.Mean();
	double list_var=st.Variance(1);
	double thresh=list_mean+50*list_var;
	for(int i=0;i<list_min.size();i++){
		if(statistic_tmp[i]>thresh)
			viewer->addCube(list_min[i].x(), list_max[i].x(), list_min[i].y(), list_max[i].y(), list_min[i].z(), list_max[i].z(), 1, 0, 0, std::to_string(i));
		else
			viewer->addCube(list_min[i].x(), list_max[i].x(), list_min[i].y(), list_max[i].y(), list_min[i].z(), list_max[i].z(), 0, 1, 0, std::to_string(i));
	} 
	*/
	
	
	
	/* srand( (unsigned)time( NULL ) );	
	for(int i=0;i<list_min.size();i++){
		float r=rand()%100/100.0;
		float g=rand()%100/100.0;
		float b=rand()%100/100.0;
		viewer->addCube(list_min[i].x(), list_max[i].x(), list_min[i].y(), list_max[i].y(), list_min[i].z(), list_max[i].z(), r,g,b, std::to_string(i));
	}   */
	

	// print the result
	pcl::io::savePLYFileASCII("color_patch.ply",*cloud);
	cout << "深度: "<< depth << endl;
	cout << "叶子节点个数:" << octree.getLeafCount() << endl;
	cout<<"number of points in cloud: "<<cloud->points.size()<<endl;
	cout<<"voxel size:"<<voxel_record->points.size()<<endl;
}

void HierachicalOutlierRemoval::FittingBasedOutlierRemoval(pcl::PointCloud<PointType>::Ptr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	vector<int> idx;
	for(int i=0;i<cloud->points.size();i++) idx.push_back(i);
	
	double mean_dist=ComputeMeanDistance(cloud);
	int K=500;
	vector<int> pointIdxNKNSearch(K);
	vector<float> pointNKNSquaredDistance(K); 	
	pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>());
	kdtree->setInputCloud(cloud);

	for(int i=0;i<idx.size();i++)
	{	
		if(idx[i]!=-1)
		{
			if ( kdtree->nearestKSearch (cloud->points[idx[i]], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){				
				
				// extract temp cloud
				pcl::PointCloud<PointType>::Ptr cloud_tmp(new pcl::PointCloud<PointType>);
				for(int j=0;j<pointIdxNKNSearch.size();j++){
					// fill temp cloud
					cloud_tmp->points.push_back(cloud->points[pointIdxNKNSearch[j]]);
					// disable neighbour points
					idx[pointIdxNKNSearch[j]]=-1;
				}
				
				SurfaceFitting sf;
				sf.FittingBasedOnPoly33(cloud_tmp);
				for(int j=0;j<pointIdxNKNSearch.size();j++)
				{
					int itmp=pointIdxNKNSearch[j];
					double dist_tmp=sf.AlgebraicDistacne(cloud->points[itmp].x,cloud->points[itmp].y,cloud->points[itmp].z);
					if(dist_tmp>0.1*mean_dist)
					{
						cloud->points[itmp].r=255;
						cloud->points[itmp].g=0;
						cloud->points[itmp].b=0;
					}
					else
					{
						cloud->points[itmp].r=0;
						cloud->points[itmp].g=255;
						cloud->points[itmp].b=0;
					}
				}				
			}
		}	
	}
}

void HierachicalOutlierRemoval::RANSACFittingBasedOutlierRemoval(pcl::PointCloud<PointType>::Ptr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	vector<int> idx;
	for(int i=0;i<cloud->points.size();i++) idx.push_back(i);
	
	double mean_dist=ComputeMeanDistance(cloud);
	int K=500;
	vector<int> pointIdxNKNSearch(K);
	vector<float> pointNKNSquaredDistance(K); 	
	pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>());
	kdtree->setInputCloud(cloud);

	for(int i=0;i<idx.size();i++)
	{	
		if(idx[i]!=-1)
		{
			if ( kdtree->nearestKSearch (cloud->points[idx[i]], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){				
				
				// extract temp cloud
				pcl::PointCloud<PointType>::Ptr cloud_tmp(new pcl::PointCloud<PointType>);
				for(int j=0;j<pointIdxNKNSearch.size();j++){
					// fill temp cloud
					cloud_tmp->points.push_back(cloud->points[pointIdxNKNSearch[j]]);
					// disable neighbour points
					idx[pointIdxNKNSearch[j]]=-1;
				}
				
				SurfaceFitting sf;
				sf.FittingBasedOnPoly33(cloud_tmp);
				for(int j=0;j<pointIdxNKNSearch.size();j++)
				{
					int itmp=pointIdxNKNSearch[j];
					double dist_tmp=sf.AlgebraicDistacne(cloud->points[itmp].x,cloud->points[itmp].y,cloud->points[itmp].z);
					if(dist_tmp>0.1*mean_dist)
					{
						cloud->points[itmp].r=255;
						cloud->points[itmp].g=0;
						cloud->points[itmp].b=0;
					}
					else
					{
						cloud->points[itmp].r=0;
						cloud->points[itmp].g=255;
						cloud->points[itmp].b=0;
					}
				}
			}
		}
	}
}

void HierachicalOutlierRemoval::PatchFittingAndRendering(pcl::PointCloud<PointType>::Ptr cloud,boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	SurfaceFitting sf;	
	sf.FittingBasedOnPoly33(cloud);
	
	vector<float> dist;
	for(int i=0;i<cloud->points.size();i++)
	{
		float dist_tmp=sf.AlgebraicDistacne(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
		dist.push_back(dist_tmp);
	}
	Statistics st(dist);
	cout<<st.min_<<" "<<st.max_<<endl;

	// Rendering
	for(int i=0;i<cloud->points.size();i++)
	{
		V3 ctmp=get_color(st.min_,st.max_,dist[i]);
		cloud->points[i].r=ctmp.r;
		cloud->points[i].g=ctmp.g;
		cloud->points[i].b=ctmp.b;
	}
	sf.DrawSurface(viewer);
	
	pcl::io::savePLYFileASCII("Patch_fitting_rendered.ply",*cloud);
	cout<<"End!"<<endl;
}