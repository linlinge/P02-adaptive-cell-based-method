#include "Statistics.h"
Statistics::Statistics(vector<float> dat)
{
	dat_=dat;		
	float sum=0;
	float min=INT_MAX;
	float max=-INT_MAX;
	
	// statistics
	for(int i=0;i<dat.size();i++)
	{
		sum+=dat[i];
		min=min<dat[i] ? min:dat[i];
		max=max>dat[i] ? max:dat[i];
	}
	sum_=sum;
	min_=min;
	max_=max;
	mean_=sum_/dat.size();
	
	sum=0;
	for(int i=0;i<dat.size();i++)
	{
		sum+=pow(dat[i]-mean_,2);				
	}
	stdevp_=sqrt(sum/dat.size());
	stdev_=sqrt(sum/(dat.size()-1));
}

vector<int> Statistics::GenerateRandomIntSet(int n,int min,int max)
{
	vector<int> dat;
	dat.resize(n);
	int range=max-min;
	for(int i=0;i<n;i++){
		dat[i]=rand()%range+min;
	}
	return dat;
}