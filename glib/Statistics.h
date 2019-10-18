#pragma once
#include <iostream>
#include <vector>
#include <math.h>

#include<limits.h>
using namespace std;
class Statistics
{
	public:
		vector<float> dat_;
		float min_,max_;
		float sum_;
		float mean_;
		float stdev_;
		float stdevp_;
		
		Statistics(vector<float> dat);
};