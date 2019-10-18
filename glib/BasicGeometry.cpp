#include "BasicGeometry.h"
/*****************************************************************
 Line
******************************************************************/
Line::Line(V3 dat1, V3 dat2, LineInitType mode)
{
	switch(mode)
	{
		case PD: // represent line with point and direction
			Point_=dat1;
			Direction_=dat2;
			break;
			
		case PP: // represent line with two points
			Point_=dat1;
			Direction_=dat2-dat1;
			break;
	} 
}

V3 Line::IsIntersect(Plane& dat)
{
	float t = -(dat.A_*Point_.x + dat.B_*Point_.y + dat.C_*Point_.z + dat.D_) / (dat.A_*Direction_.x + dat.B_*Direction_.y + dat.C_*Direction_.z);
	return V3(Direction_.x * t + Point_.x, Direction_.y * t + Point_.y, Direction_.z * t + Point_.z);
}

V3 Line::GetProjectionVector(ProjectionType mode)
{
	if(mode==XY)
	{
		return V3(Direction_.x, Direction_.y, 0);
	}
	else if(mode==XZ)
	{
		return V3(Direction_.x, 0, Direction_.z);
	}
	else if(mode==YZ)
	{
		return V3(0, Direction_.y, Direction_.z);
	}
}

float Line::GetProjectionArc(ProjectionType mode)
{
	if(mode==XY)
	{
		return atan(Direction_.z/sqrt(pow(Direction_.x,2)+pow(Direction_.y,2)));
	}
	else if(mode==XZ)
	{
		return atan(Direction_.y/sqrt(pow(Direction_.x,2)+pow(Direction_.z,2)));
	}
	else if(mode==YZ)
	{
		return atan(Direction_.x/sqrt(pow(Direction_.y,2)+pow(Direction_.z,2)));
	}
}

V3 Line::TransformTo(ProjectionType mode)
{
	if(mode==XY)
	{
		return GetProjectionVector(XY).GetNormalize()*Direction_.GetLength() + Point_;
	} 
	else if(mode == YZ)
	{
		return GetProjectionVector(YZ).GetNormalize()*Direction_.GetLength() + Point_;
	}
	else if(mode == XZ)
	{
		return GetProjectionVector(XZ).GetNormalize()*Direction_.GetLength() + Point_;
	}
}

/*****************************************************************
 Plane
******************************************************************/
Plane::Plane(V3 P1, V3 P2, V3 P3)
{
	A_ = (P2.y - P1.y)*(P3.z - P1.z) - (P3.y - P1.y)*(P2.z - P1.z);
	B_ = (P2.z - P1.z)*(P3.x - P1.x) - (P3.z - P1.z)*(P2.x - P1.x);
	C_ = (P2.x - P1.x)*(P3.y - P1.y) - (P3.x - P1.x)*(P2.y - P1.y);
	D_ = -A_ * P1.x - B_ * P1.y - C_ * P1.z;
}

V3 Plane::IsIntersect(Line& dat)
{
	float t = -(A_*dat.Point_.x + B_ * dat.Point_.y + C_ * dat.Point_.z + D_) / (A_*dat.Direction_.x + B_ * dat.Direction_.y + C_ * dat.Direction_.z);
	return V3(dat.Direction_.x * t + dat.Point_.x, dat.Direction_.y * t + dat.Point_.y, dat.Direction_.z * t + dat.Point_.z);
}

/*****************************************************************
 Angle
******************************************************************/
Angle::Angle(V3& mid, V3& left, V3& right)
{
	V3 line1 = left - mid;
	V3 line2 = right - mid;
	arc_=GetArc(line1,line2);
	angle_ = arc_ * 180.0 / PI;
}



cv::Mat VectorToRotation(V3 orientation_and_arc)
{	
	cv::Mat rotation_vector = (cv::Mat_<float>(3, 1) << orientation_and_arc.x,orientation_and_arc.y,orientation_and_arc.z);   // rotation vector
	cv::Mat rotation_matrix;                                            // rotaiton matrix
	Rodrigues(rotation_vector, rotation_matrix);                    // calculate 
	return rotation_matrix;
}

cv::Mat GetRotationMatrixToAxis(V3 vec, int axis)
{
	if(axis==X_AXIS)
	{
		float alpha=vec.GetArcToPlane(X_AXIS,XOZ);
		cout<<vec<<endl;
		cout<<"alpha:"<<alpha/CV_PI*180<<endl;
		
		cv::Mat R1=VectorToRotation(V3(1,0,0)*alpha);
		cv::Mat vec1=R1*vec.ToMat(1);
		cout<<V3(vec1)<<endl;
		
		float beta=V3(vec1).GetArcToPlane(Y_AXIS,XOY);	
		cout<<"beta:"<<beta/CV_PI*180<<endl;
		
		
		cv::Mat R2=VectorToRotation(V3(0,1,0)*beta);
		
		return R2*R1;
	}
	else if(axis == Y_AXIS)
	{
		float alpha=vec.GetArcToPlane(Y_AXIS,XOY);
		cv::Mat R1=VectorToRotation(V3(0,1,0)*alpha);
		cv::Mat vec1=R1*vec.ToMat(1);
		
		float beta=V3(vec1).GetArcToPlane(Z_AXIS,YOZ);
		cv::Mat R2=VectorToRotation(V3(0,0,1)*beta);
		
		return R2*R1;
	}
	else if(axis == Z_AXIS)
	{
		float alpha=vec.GetArcToPlane(Z_AXIS,YOZ);
		cv::Mat R1=VectorToRotation(V3(0,0,1)*alpha);
		cv::Mat vec1=R1*vec.ToMat(1);
		
		float beta=V3(vec1).GetArcToPlane(X_AXIS,XOZ);
		cv::Mat R2=VectorToRotation(V3(1,0,0)*beta);
		
		return R2*R1;
	}
}


Eigen::MatrixXf MatToMatrixXf(cv::Mat dat)
{
	Eigen::MatrixXf rst(dat.rows,dat.cols);
	
	if(dat.type()==CV_32F)
	{
		for(int i=0;i<dat.rows;i++)
		{
			for(int j=0;j<dat.cols;j++)
			{
				rst(i,j)=dat.at<float>(i,j);
			}
		}
	}
	else if(dat.type()==CV_64F)
	{
		for(int i=0;i<dat.rows;i++)
		{
			for(int j=0;j<dat.cols;j++)
			{
				rst(i,j)=dat.at<double>(i,j);
			}
		}
	}
	return rst;
}