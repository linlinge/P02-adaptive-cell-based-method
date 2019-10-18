#pragma once
#include <vector>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <math.h>
#include <algorithm>
using namespace std;

class V2I
{
	public:
		int d1_,d2_;
		float dist_;
		V2I(){};		
		V2I(int dat1,int dat2,float dist)
		{
			d1_=dat1;
			d2_=dat2;
			dist_=dist;
		}
		
		V2I operator=(V2I& e)
		{
			d1_=e.d1_;
			d2_=e.d2_;
			dist_=e.dist_;
		}
		friend ostream& operator<<(ostream& output, V2I& e)
		{
			cout<<e.d1_<<" "<<e.d2_<<" "<<e.dist_;
			return output;
		}
};



class CloudCompare
{
	public:
		vector<V2I> correspondance_;
		float min_dist_;
		float max_dist_;
		
		void find_correspondance(pcl::PointCloud<pcl::PointXYZRGBA>& cloud1, pcl::PointCloud<pcl::PointXYZRGBA>& cloud2)
		{	
			max_dist_=-INT_MAX;
			min_dist_=INT_MAX;
			for(int i=0;i<cloud1.size();i++)
			{
				float min_dist=INT_MAX;
				int min_j=0;
				
				#pragma omp parallel for
				for(int j=0;j<cloud2.size();j++)
				{
					float cur_dist=distance(cloud1[i],cloud2[j]);
					if(min_dist > cur_dist)
					{
						min_dist=cur_dist;
						min_j=j;
					}
				}
				min_dist_=min_dist_<min_dist ? min_dist_:min_dist;
				max_dist_=max_dist_>min_dist ? max_dist_:min_dist;
				correspondance_.push_back(V2I(i,min_j,min_dist));
			}						
		}
		
	/*	void find_correspondance(vector<Point>& ps1, pcl::PointCloud<pcl::PointXYZRGBA>& cloud)
		{
			vector<Point> ps2;
			correspondance_.resize(ps1.size());
			
			for(int i=0;i< cloud->points.size();i++)
			{
				Point ptmp;
				ptmp.id_=i;
				ptmp.x_=cloud->points[i].x;
				ptmp.y_=cloud->points[i].y;
				ptmp.z_=cloud->points[i].z;
				
				ptmp.r_=cloud->points[i].r;
				ptmp.g_=cloud->points[i].g;
				ptmp.b_=cloud->points[i].b;
				ps2.push_back(ptmp);
			}
			
			sort(ps2.begin(),ps2.end(),[](Point& e1, Point& e2){ return e1.x_< e2.x_});
			
			for(int i=0; i< ps1.size(); i++)
			{
				vector<Point>::iterator it=find(ps2.begin(),ps2.end(),ps1[i]);
				correspondance_[i].d1_=i;
				correspondance_[i].d2_=it->id_;
			}
			
		}*/
		
		void find_correspondance(pcl::PointCloud<pcl::PointXYZRGBA>& cloud, vector<Point>& ps)
		{
			max_dist_=-INT_MAX;
			min_dist_=INT_MAX;
				
			for(int i=0;i<cloud.size();i++)
			{			
				float min_dist=INT_MAX;
				int min_j=0;
				
				//#pragma omp parallel for
				for(int j=0;j<ps.size();j++)
				{
					float cur_dist=distance(cloud[i],ps[j]);
					if(min_dist > cur_dist)
					{
						min_dist=cur_dist;
						min_j=j;
					}
					if(min_dist<0.000001)
						break;
				}
				min_dist_=min_dist_<min_dist ? min_dist_:min_dist;
				max_dist_=max_dist_>min_dist ? max_dist_:min_dist;
				correspondance_.push_back(V2I(i,min_j,min_dist));
			}
		}
		
		void find_correspondance(vector<Point>& ps1, vector<Point>& ps2)
		{
			max_dist_=-INT_MAX;
			min_dist_=INT_MAX;
				
			for(int i=0;i<ps1.size();i++)
			{			
				float min_dist=INT_MAX;
				int min_j=0;
				
				//#pragma omp parallel for
				for(int j=0;j<ps2.size();j++)
				{
					float cur_dist=distance(ps1[i],ps2[j]);
					if(min_dist > cur_dist)
					{
						min_dist=cur_dist;
						min_j=j;
					}
					if(min_dist<0.000001)
						break;
				}
				min_dist_=min_dist_<min_dist ? min_dist_:min_dist;
				max_dist_=max_dist_>min_dist ? max_dist_:min_dist;
				correspondance_.push_back(V2I(i,min_j,min_dist));
			}
		}
		
		float distance(pcl::PointXYZRGBA& p1, pcl::PointXYZRGBA& p2)
		{
			return sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
		}
		
		float distance(pcl::PointXYZRGBA& p1, Point& p2)
		{
			return sqrt(pow(p1.x-p2.x_,2)+pow(p1.y-p2.y_,2)+pow(p1.z-p2.z_,2));
		}
		
		float distance(Point& p1, Point& p2)
		{
			return sqrt(pow(p1.x_-p2.x_,2)+pow(p1.y_-p2.y_,2)+pow(p1.z_-p2.z_,2));
		}
};