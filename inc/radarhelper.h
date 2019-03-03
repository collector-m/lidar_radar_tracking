#ifndef RADARHELPER_H
#define RADARHELPER_H

#include "global.h"
#include "object.h"

static int compute_anchor(BoxObject &obj)
{
	int anchor_index = -1;
	int index = 0;
	float min_dist = FLT_MAX;
	for (auto pt : obj.corner)
	{
		cv::Point2f newpt;
		Eigen::Vector3f vec(pt.x, pt.y, 0);
		vec = obj.yaw * vec;  // rotate by z axis
		newpt.x = vec(0) + obj.rx;  // offset
		newpt.y = vec(1) + obj.ry;
		float dist = sqrt(pow(newpt.x, 2) + pow(newpt.y, 2));
		if (dist < min_dist)
		{
			min_dist = dist;
			anchor_index = index;
		}
		++index;
	}
	assert(anchor_index >= 0 && anchor_index <= 3);
	return anchor_index;
}

void anchor2center(std::vector<BoxObject> &v)
{
	for (auto &obj : v)
	{
		float rx = obj.rx;
		float ry = obj.ry;
		float size_l = obj.corner[0].x - obj.corner[1].x;
		float size_w = obj.corner[2].y - obj.corner[1].y;
		// printf("[anchor2center] object size: l %f, w %f\n", size_l, size_w);
		
		Eigen::Vector3f offset(0,0,0);
		if (fabs(ry) < 2)
		{
			offset(0) = +size_l / 2;
			offset(1) = 0;
		}
		else
		{
			// define anchor index
			// 0-left_top, 1-left_down, 2-right_down, 3-right_top
			int anchor_index = compute_anchor(obj);	
			if (anchor_index == 0)
			{
				offset(0) = -size_l / 2;
				offset(1) = +size_w / 2;
			}
			else if (anchor_index == 1)
			{
				offset(0) = +size_l / 2;
				offset(1) = +size_w / 2;
			}
			else if (anchor_index == 2)
			{
				offset(0) = +size_l / 2;
				offset(1) = -size_w / 2;
			}
			else
			{
				offset(0) = -size_l / 2;
				offset(1) = -size_w / 2;
			}
			offset = obj.yaw * offset;
		}
		printf("[anchor2center] offset: %f, %f\n", offset(0), offset(1));
		obj.rx += offset(0);
		obj.ry += offset(1);
	}
}

#endif // RADARHELPER_H
