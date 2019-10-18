#include "FileExtend.h"
// remove adjacent duplicate substring in string
void remove_adjacent_duplicate(string& dat1, string dat2)
{
	vector<int> pos;
	int cur = dat1.find(dat2, 0);
	if (cur != -1)
	{
		while (cur != -1)
		{
			if (cur != -1)
				pos.push_back(cur);
			cur = dat1.find(dat2, cur + 1);
		}

		// remove duplicate
		int adjust = 0;
		for (int i = 0; i < pos.size() - 1; i++)
		{
			if (pos[i] + 1 == pos[i + 1])
			{
				dat1.erase(pos[i] + adjust, 1);
				adjust--;
			}
		}
	}
}

vector<string> split(string dat, string separator)
{
	vector<string> rst;
	int start, end;
	do
	{
		start = dat.find(separator);
		end = start + separator.size();
		rst.push_back(dat.substr(0, start));
		dat.erase(0, end);
	} while (start != -1);
	return rst;
}