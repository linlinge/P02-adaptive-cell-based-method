#pragma once
#include "V3.hpp"
#include <iostream>
#include <Eigen/Core>
using namespace std;

enum LineInitType{PD,PP};
enum ProjectionType{XY,XZ,YZ};
class Plane;

/* 
	Pointwise Equation:
 		(x-x1)/d.x=(y-y1)/d.y=(z-z1)/d.z
*/
class Line
{
public:
	// represent by parameter	
	V3 Point_,Direction_;
	
	// convert point-direction equation to parameter equation
	Line(V3 dat1, V3 dat2, LineInitType mode);
	V3 IsIntersect(Plane& dat);
	V3 GetProjectionVector(ProjectionType mode);
	float GetProjectionArc(ProjectionType mode);
	V3 TransformTo(ProjectionType mode);
};

class Plane
{
public:
	float A_, B_, C_, D_;
	Plane(V3 P1, V3 P2, V3 P3);
	V3 IsIntersect(Line& dat);
};

class Angle
{
public:
	float arc_, angle_;	
	Angle(V3& mid,V3& left,V3& right);
};


// 
cv::Mat VectorToRotation(V3 orientation_and_arc);
cv::Mat GetRotationMatrixToAxis(V3 vec, int axis);
Eigen::MatrixXf MatToMatrixXf(cv::Mat dat);