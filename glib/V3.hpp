#pragma once
#include <string>
#include <vector>
#include <math.h>
#include <limits.h>
#include <iostream>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#define PI 3.1415926535897
#define MAX2(a,b) (a>b? a:b)
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define XOY 3
#define XOZ 4
#define YOZ 5

class V3
{
public:
	union {
		struct {
			float x, y, z;
		};

		struct {
			float r, g, b;
		};
	};
	V3() :r(0.0f), g(0.0f), b(0.0f) {};
	V3(float r1, float g1, float b1)
	{
		r = r1;
		g = g1;
		b = b1;
	}
	V3(cv::Mat dat)
	{
		int rows=dat.rows;
		int cols=dat.cols;
		if(rows==1 && dat.type()==CV_32F)
		{
			x=dat.at<float>(0,0);
			y=dat.at<float>(0,1);
			z=dat.at<float>(0,2);
		}
		else if(rows==1 && dat.type()==CV_64F)
		{
			x=dat.at<double>(0,0);
			y=dat.at<double>(0,1);
			z=dat.at<double>(0,2);
		}
		else if(cols==1 && dat.type()==CV_32F)
		{
			x=dat.at<float>(0,0);
			y=dat.at<float>(1,0);
			z=dat.at<float>(2,0);
		}
		else if(cols==1 && dat.type()==CV_64F)
		{
			x=dat.at<double>(0,0);
			y=dat.at<double>(1,0);
			z=dat.at<double>(2,0);
		}		
	}
	
	V3& operator=(const V3 & obj)
	{
		r = obj.r;
		g = obj.g;
		b = obj.b;
		return (*this);
	}
	
	V3(pcl::PointXYZRGBA & dat)
	{
		x=dat.x;
		y=dat.y;
		z=dat.z;
	}
	
	V3(pcl::PointXYZ & dat)
	{
		x=dat.x;
		y=dat.y;
		z=dat.z;
	}
	
	bool operator==(V3& dat)
	{
		return ((x == dat.x) && (y == dat.y) && (z == dat.z));
	}
	bool operator!=(V3& dat)
	{
		if ((x == dat.x) && (y == dat.y) && (z == dat.z))
			return false;
		else
			return true;		
	}
	

	float DistanceXY(V3 & dat)
	{
		return sqrt(pow(x - dat.x, 2.0f) + pow(y - dat.y, 2.0f));
	}
	friend float Distance(V3& dat1,V3& dat2)
	{
		return sqrt(pow(dat1.x - dat2.x, 2.0f) + pow(dat1.y - dat2.y, 2.0f) + pow(dat1.z - dat2.z, 2.0f));
	}

	V3 operator *(float scale)
	{
		V3 temp;
		temp.x = x * scale;
		temp.y = y * scale;
		temp.z = z * scale;
		return temp;
	}

	friend V3 operator*(float scale, V3 dat)
	{
		dat.x = scale * dat.x;
		dat.y = scale * dat.y;
		dat.z = scale * dat.z;
		return dat;
	}

	V3 operator*(V3 dat)
	{
		dat.x = dat.x*x;
		dat.y = dat.y*y;
		dat.z = dat.z*z;
		return dat;
	}


	V3& operator /(float scale)
	{
		x = x / scale;
		y = y / scale;
		z = z / scale;
		return *this;
	}
	friend V3 operator +(V3 dat1, V3 dat2)
	{
		V3 rst;
		rst.x = dat1.x + dat2.x;
		rst.y = dat1.y + dat2.y;
		rst.z = dat1.z + dat2.z;
		return rst;
	}

	V3 operator+ (float dat)
	{
		V3 temp;
		temp.x = x + dat;
		temp.y = y + dat;
		temp.z = z + dat;

		return temp;
	}

	friend V3 operator+(float dat1, V3 dat2)
	{
		V3 temp;
		temp.x = dat1 + dat2.x;
		temp.y = dat1 + dat2.y;
		temp.z = dat1 + dat2.z;
		return temp;
	}


	V3  operator -(V3  dat)
	{
		V3 rst;
		rst.x = x - dat.x;
		rst.y = y - dat.y;
		rst.z = z - dat.z;
		return rst;
	}
	V3 operator-(float dat)
	{
		V3 rst;
		rst.x -= dat;
		rst.y -= dat;
		rst.z -= dat;
		return rst;
	}


	friend float GetArc(V3 dat1, V3 dat2)
	{
		float cos_theta = Dot(dat1.GetNormalize(), dat2.GetNormalize());
		float theta = acos(cos_theta);
		return theta;
	}

	friend float Dot(V3 dat1,V3 dat2)
	{	
		return (dat1.x*dat2.x+dat1.y*dat2.y+dat1.z*dat2.z);
	}


	friend V3 Cross(V3& dat1, V3& dat2)
	{
		V3 rst;
		rst.x = dat1.y*dat2.z - dat2.y*dat1.z;
		rst.y = dat2.x*dat1.z - dat1.x*dat2.z;
		rst.z = dat1.x*dat2.y - dat1.y*dat2.x;
		return rst;
	}


	float GetLength()
	{		
		return sqrt(x*x + y * y + z * z);
	}
	

	V3 GetNormalize()
	{
		V3 rst;
		float a = sqrt(x*x + y * y + z * z);
		rst.y = y / a;
		rst.z = z / a;
		rst.x = x / a;
		return rst;
	}
	
	void Normalize()
	{
		float a = sqrt(x*x + y * y + z * z);
		x = x / a;
		y = y / a;
		z = z / a;		
	}

	void Clear()
	{
		x = y = z = INT_MAX;
	}

	friend std::ostream& operator <<(std::ostream& os, const V3 dat)
	{
		os << dat.x << " " << dat.y << " " << dat.z;
		return os;
	}	
	
	float GetArcToPlane(int rotation_axis, int plane)
	{
		float arc;		
		if(rotation_axis == X_AXIS && plane == XOZ)
		{	
			if(y==0)
				arc=CV_PI/2.0f;
			else if(z==0)
				arc=0;
			else if(y*z>0)
			{
				arc=atan(y/z);
			}
			else
			{
				arc=CV_PI+atan(y/z);
			}
		}
		else if(rotation_axis == X_AXIS && plane == XOY)
		{		
			if(y==0)
				arc=0;
			else if(z==0)
				arc=CV_PI/2.0f;
			else 
				arc=CV_PI/2.0f+atan(y/z);
		}		
		else if(rotation_axis == Y_AXIS && plane == XOY)
		{		
			if(x==0)
				arc=CV_PI/2.0f;
			else if(z==0)
				arc=0;
			else if(x*z>0)
				arc=atan(z/x);
			else 
				arc=CV_PI+atan(z/x);
		}
		else if(rotation_axis == Y_AXIS && plane == YOZ)
		{		
			if(x==0)
				arc=0;
			else if(z==0)
				arc=CV_PI/2.0f;
			else
				arc=CV_PI/2.0f+atan(z/x);
		}
		else if(rotation_axis == Z_AXIS && plane == YOZ)
		{		
			if(x==0)
				arc=CV_PI/2.0f;
			else if(y==0)
				arc=0;
			else if(x*y>0)
				arc=atan(x/y);
			else
				arc=CV_PI+atan(x/y);
		}
		else if(rotation_axis == Z_AXIS && plane == XOZ)
		{			
			if(x==0)				
				arc=CV_PI/2.0f;			
			else if(y==0)				
				arc=0;			
			else
				arc=CV_PI/2.0f+atan(x/y);			
		}
		
		return arc;
	}
	
	cv::Mat ToMat(int row_or_col)
	{
		if(row_or_col==0)
		{
			cv::Mat dat=(cv::Mat_<float>(1,3)<<x,y,z);
			return dat;
		}
		else
		{
			cv::Mat dat=(cv::Mat_<float>(3,1)<<x,y,z);
			return dat;
		}
	}	
};