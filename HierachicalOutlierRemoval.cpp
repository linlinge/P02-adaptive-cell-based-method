#include "HierachicalOutlierRemoval.h"

HierachicalOutlierRemoval::HierachicalOutlierRemoval(pcl::PointCloud<PointType>::Ptr cloud)
{
	cloud_filtered_=pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);	
	cloud_outlier_=pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);	
	if(cloud!=NULL)
		cloud_=cloud;
	
	mean_dist_=ComputeMeanDistance(cloud_);
}

/*********************************************************************************************************
										Internal Function
**********************************************************************************************************/
void HierachicalOutlierRemoval::SplitIntoPrimitives(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	// voxel record
	pcl::PointCloud<PointType>::Ptr voxel_record(new pcl::PointCloud<PointType>);	
    
	// 最小体素的边长
	float resolution = 15*mean_dist_;
	octree_=pcl::octree::OctreePointCloudSearch<PointType>::Ptr(new pcl::octree::OctreePointCloudSearch<PointType>(resolution));
	octree_->setInputCloud(cloud_);	
	octree_->addPointsFromInputCloud();// 从输入点云构建八叉树
	
	/* if(viewer!=NULL){
		for (auto it = octree_->begin(depth);it != octree_->end();++it){	
			if(it.isLeafNode()){
				// Get all points indices in each voxel
				auto leaf = it.getLeafContainer();
				std::vector<int> indices;
				leaf.getPointIndices(indices);
			}
		}
	} */
}

/*********************************************************************************************************
										External Function
**********************************************************************************************************/
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

void HierachicalOutlierRemoval::Type3RemovalBasedOnRansac(MODEL mode, pcl::PointCloud<PointType>::Ptr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	struct timeval start, end;
	mode_=mode;
	if(cloud!=NULL)
		cloud_=cloud;
	
	gettimeofday(&start, NULL);
	SplitIntoPrimitives();
	gettimeofday(&end, NULL);
	cout<<"SplitIntoPrimitives:"<<ELAPSED(start,end)<<" (s)"<<endl;
			
	int depth = octree_->getTreeDepth();
	int count=0;	
	// Traversing leaf node
	//#pragma omp parallel for
	pcl::PointCloud<PointType>::Ptr ctmp(new pcl::PointCloud<PointType>);
	/* #pragma omp parallel num_threads(6)
	{ */	
		for(auto it = octree_->begin(depth);it!= octree_->end();++it){
			/* #pragma omp single nowait
			{ */
				if(it.isLeafNode()){
					// Get all points indices in each voxel
					//gettimeofday(&start, NULL);
					auto leaf = it.getLeafContainer();
					std::vector<int> indices; 			
					leaf.getPointIndices(indices);

					// Step 01: Get all points in this cloud
					gettimeofday(&start, NULL);
					ctmp->points.clear();
					for(int i=0;i<indices.size();i++){
						ctmp->points.push_back(cloud_->points[indices[i]]);
						cloud_->points[indices[i]].r=255;
						cloud_->points[indices[i]].g=0;
						cloud_->points[indices[i]].b=0;
					}
					gettimeofday(&end, NULL);
					//cout<<"Step01:"<<ELAPSED(start,end)<<" (s)"<<endl;					
					//cout<<count++<<" "<<ctmp->points.size()<<endl;
					
					if(ctmp->points.size()>=4){
						// Step 02: Is it required to load use Ransac
						gettimeofday(&start, NULL);
						double eigval=CalculateEigenvalue(ctmp);
						gettimeofday(&end, NULL);
						//cout<<"Step02:"<<ELAPSED(start,end)<<" (s)"<<endl;
						
						if(eigval>0.0000005){
							// Step 03: Ransac and Plane
							gettimeofday(&start, NULL);
							SurfaceFitting sf;
							sf.FittingBasedOnRansac(ctmp,mode);
							//sf.GetModelError();
							for(int i=0;i<sf.inlier_idx_.size();i++)
								cloud_filtered_->points.push_back(ctmp->points[sf.inlier_idx_[i]]);															

							for(int i=0;i<sf.outlier_idx_.size();i++)
								cloud_outlier_->points.push_back(ctmp->points[sf.outlier_idx_[i]]);
							
							gettimeofday(&end, NULL);
							//cout<<"Step03:"<<ELAPSED(start,end)<<" (s)"<<endl;
						}
						else{
							count++; 
							for(int i=0;i<ctmp->points.size();i++)
								cloud_filtered_->points.push_back(ctmp->points[i]); 
						}
					}
					else{
						for(int i=0;i<ctmp->points.size();i++)
							cloud_filtered_->points.push_back(ctmp->points[i]);
					}
				}
			//}
		}		
	//}
	printf("(%d,%d,%d)",cloud_->points.size(),cloud_filtered_->points.size(),cloud_outlier_->points.size());
	pcl::io::savePLYFileASCII("cloud_.ply",*cloud_);
	pcl::io::savePLYFileASCII("cloud_filtered.ply",*cloud_filtered_);
	pcl::io::savePLYFileASCII("cloud_outlier.ply",*cloud_outlier_);		
	cout<<endl<<"end!"<<endl;
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