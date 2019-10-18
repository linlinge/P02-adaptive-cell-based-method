#pragma once
#include <iostream>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
using namespace std;
class Point
{
	public:
		int id_;
		float x_,y_,z_;
		float r_,g_,b_;
		float confidence_;
		float error_;
		
		friend ostream& operator <<(ostream& out, Point& obj)
		{
			cout<<obj.x_<<" "<<obj.y_<<" "<<obj.z_<<" "<<obj.r_<<" "<<obj.g_<<" "<<obj.b_<<" ";
		}
		friend float Distance(Point& p1, Point& p2)
		{
			return (pow(p1.x_-p2.x_,2)+pow(p1.y_-p2.y_,2)+pow(p1.z_-p2.z_,2));
		}
		friend float Distance(Point& p1, pcl::PointXYZRGBA& p2)
		{
			return (pow(p1.x_-p2.x,2)+pow(p1.y_-p2.y,2)+pow(p1.z_-p2.z,2));
		}
		friend float Distance( pcl::PointXYZRGBA& p2,Point& p1)
		{
			return (pow(p1.x_-p2.x,2)+pow(p1.y_-p2.y,2)+pow(p1.z_-p2.z,2));
		}
		
		Point operator=(Point& e)
		{
			x_=e.x_;
			y_=e.y_;
			z_=e.z_;
			r_=e.r_;
			g_=e.r_;
			b_=e.b_;
			confidence_=e.confidence_;
		}
		
		bool operator==(Point& e)
		{
			if(abs(this->x_-e.x_)<0.000001)
				return true;
			else
				return false;
		}
};