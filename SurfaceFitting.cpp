#include "SurfaceFitting.h"
/********************************************************************************************************************/
//												   Internal Function
/********************************************************************************************************************/
void SurfaceFitting::Poly33(Mat& x, Mat& y, Mat& z)
{	
	x_min_=y_min_=z_min_=INT_MAX;
	x_max_=y_max_=z_max_=-INT_MAX;
	for(int i=0;i<x.rows;i++)
	{
		if(x_min_>x.at<double>(i,0))
			x_min_=x.at<double>(i,0);
		if(x_max_<x.at<double>(i,0))
			x_max_=x.at<double>(i,0);
		
		if(y_min_>y.at<double>(i,0))
			y_min_=y.at<double>(i,0);
		if(y_max_<y.at<double>(i,0))
			y_max_=y.at<double>(i,0);
		
		if(z_min_>z.at<double>(i,0))
			z_min_=z.at<double>(i,0);
		if(z_max_<z.at<double>(i,0))
			z_max_=z.at<double>(i,0);			
	}
	
   
	Mat A=Mat_<double>(x.rows,10);	
	for(int i=0;i<x.rows;i++)
	{
		double datx=x.at<double>(i,0);
		double daty=y.at<double>(i,0);
		
		A.at<double>(i,0)=1;
		A.at<double>(i,1)=datx;
		A.at<double>(i,2)=daty;
		A.at<double>(i,3)=pow(datx,2);
		A.at<double>(i,4)=datx*daty;
		A.at<double>(i,5)=pow(daty,2);
		A.at<double>(i,6)=pow(datx,3);
		A.at<double>(i,7)=pow(datx,2)*daty;
		A.at<double>(i,8)=datx*pow(daty,2);
		A.at<double>(i,9)=pow(daty,3);
	}
	P_=(A.t()*A).inv(CV_SVD)*A.t()*z;	
}

void SurfaceFitting::PlaneFitting(pcl::PointCloud<PointType>::Ptr cloud)
{
	x_min_=y_min_=INT_MAX;
	x_max_=y_max_=-INT_MAX;
	for(auto ptmp:cloud->points){
		x_min_=x_min_<ptmp.x ? x_min_:ptmp.x;
		x_max_=x_max_>ptmp.x ? x_max_:ptmp.x;
		y_min_=y_min_<ptmp.y ? y_min_:ptmp.y;
		y_max_=y_max_>ptmp.y ? y_max_:ptmp.y;
	}
		
	Mat A=Mat_<double>(cloud->points.size(),3);
	Mat b=Mat_<double>(cloud->points.size(),1);
	for(int i=0;i<cloud->points.size();i++)
	{
		A.at<double>(i,0)=cloud->points[i].x;
		A.at<double>(i,1)=cloud->points[i].y;
		A.at<double>(i,2)=1;
		b.at<double>(i,0)=cloud->points[i].z;
	}
	P_=(A.t()*A).inv(CV_SVD)*A.t()*b;
}


double SurfaceFitting::AlgebraicDistacne(double datx, double daty, double datz)
{
	double denominator= abs(datz - CalculateZ(datx,daty));
	double partial_x=-(P_.at<double>(1,0) + 2*P_.at<double>(3,0)*datx + P_.at<double>(4,0)*daty + 3*P_.at<double>(6,0)*pow(daty,2) + 2*P_.at<double>(7,0)*datx*daty + P_.at<double>(8,0)*pow(daty,2));
	double partial_y=-(P_.at<double>(2,0) + P_.at<double>(4,0)*datx + 2*P_.at<double>(5,0)*daty + P_.at<double>(7,0)*pow(datx,2) + 2*P_.at<double>(8,0)*datx*daty + 3*P_.at<double>(9,0)*pow(daty,2));
	double numerator= sqrt(pow(partial_x,2)+pow(partial_y,2)+1);	
	return denominator/numerator;
}

double SurfaceFitting::EuclideanDistance(double datx, double daty, double datz)
{
	if(mode_==PLANE){
		/* cout<<"P rows"<<P_.rows<<endl;
		cout<<"P columns"<<P_.cols<<endl; */
		return abs((P_.at<double>(0)*datx + P_.at<double>(1)*daty - datz + P_.at<double>(2))/sqrt(pow(P_.at<double>(0),2)+pow(P_.at<double>(1),2)+pow(1.0f,2)));
	}
	else if(mode_==POLY33){
		
	}
	else
		cout<<"Error: mode error!"<<endl;
}

double SurfaceFitting::CalculateZ(double datx, double daty)
{
	if(mode_==POLY33) // Poly33
	{ 
		Mat dat=(Mat_<double>(10,1)<< 1, datx, daty, pow(datx,2), datx*daty, pow(daty,2), pow(datx,3), pow(datx,2)*daty, datx*pow(daty,2), pow(daty,3));
		Mat rst=P_.t()*dat;
		return rst.at<double>(0,0);
	}
	else if(mode_==PLANE)
	{
		Mat dat=(Mat_<double>(3,1)<< datx,daty,1);
		Mat rst=P_.t()*dat;
		return rst.at<double>(0,0);
	}
	else
		return -1;
}


void SurfaceFitting::DrawSurface(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	double num=200;
	double xstep=(x_max_-x_min_)/num;
	double ystep=(y_max_-y_min_)/num;	
	
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	for(double xval=x_min_; xval<=x_max_; xval+=xstep){
		for(double yval=y_min_; yval<=y_max_; yval+=ystep){
			PointType ptmp;
			ptmp.x=xval;
			ptmp.y=yval;
			ptmp.z=CalculateZ(xval,yval);
			ptmp.r=0;
			ptmp.g=0;
			ptmp.b=0;
			cloud->points.push_back(ptmp);
			
			//viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(), pcl::PointXYZ(), 255, 0, 0, std::tostring(xval+yval));
		}
	}
	
	cout<<cloud->points.size()<<endl;
	//Set multi-color for point cloud
	pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color(cloud);  	
	
	//Add the demostration point cloud data
	viewer->addPointCloud<PointType> (cloud, multi_color, "surface");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "surface");	
	pcl::io::savePLYFileASCII("fitting-surface.ply",*cloud);
}


/********************************************************************************************************************/
//												   External Function
/********************************************************************************************************************/
void SurfaceFitting::FittingBasedOnPoly33(pcl::PointCloud<PointType>::Ptr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{	
	cloud_=cloud;
	int n=cloud->points.size();
	Mat x=Mat_<double>(n,1);
	Mat y=Mat_<double>(n,1);
	Mat z=Mat_<double>(n,1);
	
	for(int i=0;i<n;i++)
	{
		x.at<double>(i,0)=cloud->points[i].x;
		y.at<double>(i,0)=cloud->points[i].y;
		z.at<double>(i,0)=cloud->points[i].z;
	}
	
	Poly33(x,y,z);	
	if(viewer!=NULL)
		DrawSurface(viewer);
}

void SurfaceFitting::FittingBasedOnRansac(pcl::PointCloud<PointType>::Ptr cloud, MODEL type, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	//cout<<"1"<<endl;
	int n=10;
	mode_=type;
	cloud_=cloud;
	double mean_dist=ComputeMeanDistance(cloud);
	double distance_threshold=3.0f*mean_dist;
	float best_inlier_ratio=0.0f;
	Mat best_P;
	//cout<<"2"<<endl;
	//srand((int)time(NULL));
	if(type==POLY33){
		//cout<<"POLY33 mode"<<endl;
		for(int k=0;k<n;k++){
			
			set<int> idx_buffer; // store current available idx
			for(int i=0;i<cloud->points.size();i++) idx_buffer.insert(i);
			
			// Step01: Random Selection
			pcl::PointCloud<PointType>::Ptr maybe_inlier(new pcl::PointCloud<PointType>);
			Statistics st;
			vector<int> maybe_inlier_idx=st.GenerateRandomIntSet(20,0,cloud->points.size()-1);
			sort(maybe_inlier_idx.begin(),maybe_inlier_idx.end());
			vector<int>::iterator it=unique(maybe_inlier_idx.begin(),maybe_inlier_idx.end());
			maybe_inlier_idx.erase(it,maybe_inlier_idx.end());
			for(int i=0;i<maybe_inlier_idx.size();i++)
				maybe_inlier->points.push_back(cloud->points[maybe_inlier_idx[i]]);
			
			// update idx_buffer
			for(int i=0;i<maybe_inlier_idx.size();i++){
				idx_buffer.erase(maybe_inlier_idx[i]);
			}
			
			// Step02: Maybe Model Estimation
			FittingBasedOnPoly33(maybe_inlier);
			
			// Step03: Maybe Model Error
			int count=0;
			for(set<int>::iterator it=idx_buffer.begin();it!=idx_buffer.end();it++){
				double dtmp=AlgebraicDistacne(cloud->points[*it].x,cloud->points[*it].y,cloud->points[*it].z);
				if(dtmp<distance_threshold){
					maybe_inlier->points.push_back(cloud->points[*it]);
					count++;
				}
			}
			double inlier_ratio=(count+maybe_inlier_idx.size())*1.0/cloud->points.size();
						
			// Step04: whether it is a better model
			if(best_inlier_ratio<inlier_ratio){
				best_inlier_ratio=inlier_ratio;
				FittingBasedOnPoly33(maybe_inlier); // re-estimate model
				best_P=P_;
			}
		}
		
		// update inlier and outlier
		inlier_idx_.clear();
		outlier_idx_.clear();
		for(int i=0;i<cloud_->points.size();i++){
			double dtmp=AlgebraicDistacne(cloud_->points[i].x,cloud_->points[i].y,cloud_->points[i].z);
			if(dtmp<distance_threshold)
				inlier_idx_.push_back(i);			
			else
				outlier_idx_.push_back(i);
		}
		
		// show result
		if(viewer!=NULL){
			P_=best_P;
			DrawSurface(viewer);
		}
	}
	else if(type==PLANE){		
		//cout<<"plane mode"<<endl;		
		for(int k=0;k<n;k++)
		{
		 	pcl::PointCloud<PointType>::Ptr maybe_inlier(new pcl::PointCloud<PointType>);
			set<int> idx_buffer;
			for(int i=0;i<cloud->points.size();i++) idx_buffer.insert(i);
			
			// Step01: Random Selection			
			vector<int> maybe_inlier_idx;
			int init_n=(int) ((cloud->points.size()*0.15)>4 ? (cloud->points.size()*0.15):4);
			//cout<<init_n<<endl;
			//int init_n=10;
			while(maybe_inlier_idx.size()<init_n){
			maybe_inlier_idx.push_back(rand()%cloud->points.size());
			sort(maybe_inlier_idx.begin(),maybe_inlier_idx.end());
			vector<int>::iterator it=unique(maybe_inlier_idx.begin(),maybe_inlier_idx.end());
			maybe_inlier_idx.erase(it,maybe_inlier_idx.end());
			}
			
			for(int i=0;i<maybe_inlier_idx.size();i++){
				idx_buffer.erase(maybe_inlier_idx[i]);
				maybe_inlier->points.push_back(cloud->points[maybe_inlier_idx[i]]);
			}
			
			// Step02: Maybe Model Estimation			
			PlaneFitting(maybe_inlier);
						
			// Step03: Distance
			int count=0;
			for(set<int>::iterator it=idx_buffer.begin();it!=idx_buffer.end();it++){				
				double dist_tmp=EuclideanDistance(cloud->points[*it].x,cloud->points[*it].y,cloud->points[*it].z);
				
				// statistic maybe_inlier
				if(dist_tmp<1.5*mean_dist){
					count++; 
					
					//maybe_inlier_idx.push_back(*it);
					maybe_inlier->points.push_back(cloud->points[*it]);
				}
			}
			
			float current_inlier_ratio=(count+maybe_inlier_idx.size())*1.0/cloud->points.size();			
			
			// Step04: whether it is better model or not			
			if(best_inlier_ratio < current_inlier_ratio){
				best_inlier_ratio = current_inlier_ratio;
				
				//PlaneFitting(maybe_inlier); // re-estimate
				PlaneFitting(maybe_inlier);
				best_P=P_;
			}
		}
		
		// update inlier
		inlier_idx_.clear();
		outlier_idx_.clear();
		for(int i=0;i<cloud_->points.size();i++){
			double dtmp=EuclideanDistance(cloud_->points[i].x,cloud_->points[i].y,cloud_->points[i].z);
			if(dtmp<distance_threshold)
				inlier_idx_.push_back(i);			
			else
				outlier_idx_.push_back(i);
		} 
		//printf("(%d,%d,%d)\n",cloud_->points.size(),inlier_idx_.size(),outlier_idx_.size());
		
		// show result
		if(viewer!=NULL){
			P_=best_P;
			DrawSurface(viewer);
		}
		
	}
}


void SurfaceFitting::GetModelError()
{
	if(mode_==PLANE){
		
		for(int i=0;i<cloud_->points.size();i++){
			
			double dist_tmp=EuclideanDistance(cloud_->points[i].x,cloud_->points[i].y,cloud_->points[i].z);
			
			errors_.push_back(dist_tmp); 
		}
	}
	else if(mode_==POLY33){
		for(int i=0;i<cloud_->points.size();i++){
			double dist_tmp=AlgebraicDistacne(cloud_->points[i].x,cloud_->points[i].y,cloud_->points[i].z);
			errors_.push_back(dist_tmp); 
		}
	}
	
}