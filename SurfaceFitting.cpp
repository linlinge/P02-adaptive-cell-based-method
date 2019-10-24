#include "SurfaceFitting.h"
/********************************************************************************************************************/
//												   Internal Function
/********************************************************************************************************************/
Mat SurfaceFitting::Poly33(Mat& x, Mat& y, Mat& z)
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
	Mat P=(A.t()*A).inv(CV_SVD)*A.t()*z;
	return P;
}

double SurfaceFitting::AlgebraicDistacne(double datx, double daty, double datz)
{
	double denominator= abs(datz - CalculateZ(datx,daty));
	double partial_x=-(P_.at<double>(1,0) + 2*P_.at<double>(3,0)*datx + P_.at<double>(4,0)*daty + 3*P_.at<double>(6,0)*pow(daty,2) + 2*P_.at<double>(7,0)*datx*daty + P_.at<double>(8,0)*pow(daty,2));
	double partial_y=-(P_.at<double>(2,0) + P_.at<double>(4,0)*datx + 2*P_.at<double>(5,0)*daty + P_.at<double>(7,0)*pow(datx,2) + 2*P_.at<double>(8,0)*datx*daty + 3*P_.at<double>(9,0)*pow(daty,2));
	double numerator= sqrt(pow(partial_x,2)+pow(partial_y,2)+1);	
	return denominator/numerator;
}

double SurfaceFitting::CalculateZ(double datx, double daty)
{
	if(P_.rows==10) // Poly33
	{
		Mat dat=(Mat_<double>(10,1)<< 1, datx, daty, pow(datx,2), datx*daty, pow(daty,2), pow(datx,3), pow(datx,2)*daty, datx*pow(daty,2), pow(daty,3));
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
	for(double xval=x_min_; xval<=x_max_; xval+=xstep)
	{
		for(double yval=y_min_; yval<=y_max_; yval+=ystep)
		{
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

void SurfaceFitting::FittingBasedOnRansac(pcl::PointCloud<PointType>::Ptr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	// Random Selection
	pcl::PointCloud<PointType>::Ptr maybe_inlier(new pcl::PointCloud<PointType>);
	Statistics st;
	vector<int> maybe_inlier_idx=st.GenerateRandomIntSet(20,0,cloud->points.size()-1);
	sort(maybe_inlier_idx.begin(),maybe_inlier_idx.end());
	vector<int>::iterator it=unique(maybe_inlier_idx.begin(),maybe_inlier_idx.end());
	maybe_inlier_idx.erase(it,maybe_inlier_idx.end());
	for(int i=0;i<maybe_inlier_idx.size();i++)
		maybe_inlier->points.push_back(cloud->points[maybe_inlier_idx[i]]);
	
	// maybe model
	FittingBasedOnPoly33(maybe_inlier);
	
	
}

void  SurfaceFitting::GetModelError()
{
	for(int i=0;i<cloud_->points.size();i++)
	{
		double dist_tmp=AlgebraicDistacne(cloud_->points[i].x,cloud_->points[i].y,cloud_->points[i].z);
		errors_.push_back(dist_tmp); 
	}
}