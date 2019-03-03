#ifndef POINTCLOUDHELPER_H
#define POINTCLOUDHELPER_H

#include "global.h"
#include "object.h"
#include "visualizer.h"

const float ccl_win_w = 2 / map_scale;
const float ccl_win_h = 2 / map_scale;

static cv::Scalar label2color(int label, int label_num)
{
	cv::Scalar color(0, 0, 0);
    if (!label_num)  return color;

	label *= (256 / (label_num));
	if (!label)
	{
		color[0] = 0;
		color[1] = 0;
		color[2] = 0;
	}
	else if (label <= 51)
	{
		color[0] = 255;
		color[1] = label * 5;
		color[2] = 0;
	}
	else if (label <= 102)
	{
		label -= 51;
		color[0] = 255 - label * 5;
		color[1] = 255;
		color[2] = 0;
	}
	else if (label <= 153)
	{
		label -= 102;
		color[0] = 0;
		color[1] = 255;
		color[2] = label * 5;
	}
	else if (label <= 204)
	{
		label -= 153;
		color[0] = 0;
		color[1] = 255 - uchar(128.0*label / 51.0 + 0.5);
		color[2] = 255;
	}
	else
	{
		label -= 204;
		color[0] = 0;
		color[1] = 127 - uchar(127.0*label / 51.0 + 0.5);
		color[2] = 255;
	}

	return color;
}

static void ccl_dfs(int row, int col, cv::Mat &m, 
                    std::vector<bool> &visited, 
                    std::vector<int> &label, 
                    std::vector<int> &area,  
                    int label_cnt)
{
	visited[row * m.cols + col] = 1;

    for (int i=-ccl_win_w/2; i<=ccl_win_w/2; ++i)
    {
        for (int j=-ccl_win_h/2; j<=ccl_win_h/2; ++j)
        {
            int u = std::max(0, col+i);
            u = std::min(m.cols-1, u);
            int v = std::max(0, row+j);
            v = std::min(m.rows-1, v);

            if (!visited[v*m.cols + u] && (m.at<uchar>(row, col) == m.at<uchar>(v, u)))
            {
                label[v*m.cols + u] = label_cnt;
                ++area[label_cnt];
                ccl_dfs(v, u, m, visited, label, area, label_cnt);
            }
        }
    }
}

static void ccl(cv::Mat &map)
{
    cv::Mat m;
    cv::cvtColor(map, m, CV_BGR2GRAY);
	// printf("[pointcloud_labelling] map cols %d, rows %d\n", m.cols, m.cols);

    std::vector<bool> visited(m.rows * m.cols, 0);
    std::vector<int> label(m.rows * m.cols, 0);
    std::vector<int> area(m.rows * m.cols, 0);

	int label_cnt = 0;

	for (int i = 0; i < m.rows; i++)
	{
		for (int j = 0; j < m.cols; j++)
		{
			if (!m.at<uchar>(i, j))  // no lidar -> label 0
			{
				label[i*m.cols + j] = 0;
				area[0]++;
				continue;
			}
			if (visited[i*m.cols + j])
			{
				continue;
			}

			label[i*m.cols + j] = ++label_cnt;
			++area[label_cnt];
			ccl_dfs(i, j, m, visited, label, area, label_cnt);
		}
	}

	printf("[pointcloud_labelling] there is %d cluster at all\n", label_cnt);
	// for (int i = 0; i <= label_cnt; i++)
	// {
	// 	printf("[pointcloud_labelling] cluster %d: %d\n", i, area[i]);
	// }

	for (int i = 0; i < m.rows; i++)
	{
		for (int j = 0; j < m.cols; j++)
		{
			cv::Scalar color = label2color(label[i*m.cols + j], label_cnt);
			map.at<cv::Vec3b>(i,j)[0] = color[0];
			map.at<cv::Vec3b>(i,j)[1] = color[1];
			map.at<cv::Vec3b>(i,j)[2] = color[2];
		}
	}
}

void pointcloud_labelling(std::vector<LidarPoint> &lidarpts)
{
    Visualizer pcviser;
    pcviser.Init();
    pcviser.DrawLidarPts(lidarpts, cv::Scalar(0,255,255));
    cv::Mat lidarmap = pcviser.GetMap();

	printf("[pointcloud_labelling] %zu pointclouds received\n", lidarpts.size());
	if (lidarpts.size())
		ccl(lidarmap);
	
	// // test ccl
	// cv::Mat lidarmap(500, 500, CV_8UC3, cv::Scalar(0,0,0));
	// cv::rectangle(lidarmap, cv::Rect(100, 100, 100, 100), cv::Scalar(255, 255, 255));
	// ccl(lidarmap);
	// printf("ccl end\n");

    cv::namedWindow("pointcloud_label");
	cv::imshow("pointcloud_label", lidarmap);
    cv::waitKey(5);
}

#endif // POINTCLOUDHELPER_H
