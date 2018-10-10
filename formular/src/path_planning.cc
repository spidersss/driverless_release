#include "formular/path_planning.h"

/*************************主函数*************************/
int main()
{
	/**路径边缘拟合**/
	Mat map(500, 500, CV_8UC3, Scalar::all(0));  //用于显示的图像
	int* input_data = new int[5];
	string s_temp;
	vector<Point>side_1, side_2;
	ifstream infile;
	infile.open("./测试数据/弯道上部边界1.txt");  //上部边界点
	while (getline(infile, s_temp))
	{
		string_to_num(s_temp, input_data);
		int x = input_data[0];
		int y = input_data[1];
		side_1.push_back(Point(x, y));
	}
	infile.close();
	infile.open("./测试数据/弯道下部边界1.txt");  //下部边界点
	while (getline(infile, s_temp))
	{
		string_to_num(s_temp, input_data);
		int x = input_data[0];
		int y = input_data[1];
		side_2.push_back(Point(x, y));
	}
	infile.close();
	vector<Point> point_set_1(side_1.begin(), side_1.end()), point_set_2(side_2.begin(), side_2.end());
	int num = fit_num(side_1);  //多项式阶数，如路径点数少，而阶数过高，拟合会出错！
	Mat mat_k1 = polyfit(point_set_1, fit_num(side_1));
	Mat mat_k2 = polyfit(point_set_2, fit_num(side_2));
	for (int i = 0; i < side_1.size(); ++i)  //画出上部路径边缘的离散点
	{
		Point center_1 = side_1[i];
		circle(map, center_1, 4, Scalar(50,100,255), CV_FILLED, CV_AA);
	}
	for (int i = 0; i < side_2.size(); ++i)  //画出下部路径边缘的离散点
	{
		Point center_2 = side_2[i];
		circle(map, center_2, 4, Scalar(255,100,50), CV_FILLED, CV_AA);
	}
	vector<double>x1, y1;
	for (int i = 0; i < map.cols; ++i)  //上部路径边缘拟合曲线的坐标，i决定多项式的底数
	{
		Point2d center;
		center.x = i;  //要画的点的x,y坐标值，圆心坐标
		center.y = 0;
		for (int j = 0; j < num + 1; ++j)  //j决定多项式中每一项的幂次
		{
			center.y += mat_k1.at<double>(j, 0)*pow(i, j);
		}
		x1.push_back(center.x);
		y1.push_back(center.y);
		circle(map, center, 1, Scalar(255, 255, 255), CV_FILLED, CV_AA);
	}
	vector<double>x2, y2;  //y1,y2的下标对应0-499，共500个数
	for (int i = 0; i < map.cols; ++i)  //下部路径边缘拟合曲线的坐标
	{
		Point2d center;
		center.x = i;  //要画的点的x,y坐标值，圆心坐标
		center.y = 0;
		for (int j = 0; j < num + 1; ++j)
		{
			center.y += mat_k2.at<double>(j, 0)*pow(i, j);
		}
		x2.push_back(center.x);
		y2.push_back(center.y);
		circle(map, center, 1, Scalar(255, 255, 255), CV_FILLED, CV_AA);
	}
	/**路径区域划分**/
	int x_start = (side_1[0].x + side_2[0].x) / 2;
	int y_start = (side_1[0].y + side_2[0].y) / 2;
	int startnode[2] = { x_start,y_start };  //起点
	int x_end = (side_1[side_1.size() - 1].x + side_2[side_2.size() - 1].x) / 2;
	int y_end = (side_1[side_1.size() - 1].y + side_2[side_2.size() - 1].y) / 2;
	int endnode[2] = { x_end,y_end };  //终点
	for (int col = 0; col <map.cols; ++col)
	{
		for (int row = 0; row < map.rows; ++row)
		{
			if (col < x_start)
			{
				map.at<Vec3b>(row, col) = Vec3b(0, 0, 255);  //x_start左边区域标红
			}
			else
			{
				if (col <= x_end)
				{
					if (row <y1[col])
					{
						map.at<Vec3b>(row, col) = Vec3b(0, 0, 255);  //BGR，上部障碍线以上区域标红
					}
					if (row > y2[col])
					{
						map.at<Vec3b>(row, col) = Vec3b(0, 0, 255);  //下部障碍线以下区域标红
					}
				}
				else
				{
					map.at<Vec3b>(row, col) = Vec3b(0, 0, 255);  //x_end右边区域标红
				}
			}
		}
	}
									   /**路径关键参数设定**/
	clock_t start, finish;
	start = clock();  //开始计时
	srand((unsigned)time(NULL));  //这句不能放在主函数外面;如没有这句，每次编译产生的随机数是一样的！
	int filled_num = 0;
	int greed_flag = 1;
	double newpoint[2] = { 0,0 };
	vector<vector<int> >tree(1000);
	for (int i = 0; i < 1000; ++i)
	{
		tree[i].resize(2);
	}
	tree[0][0] = startnode[0];
	tree[0][1] = startnode[1];
	int x_picture = map.cols;  //图像大小
	int y_picture = map.rows;
	//**贪婪RRT路径规划**//
	vector<int>path_x, path_y;  //路径节点的x,y坐标
	int success = 0;
	while (success == 0)
	{
		filled_num = 0;  //每次统计前先归零
		for (int i = 0; i != tree.size(); ++i)
		{
			if ((tree[i][0] != 0) & (tree[i][1] != 0))
			{
				filled_num = filled_num + 1;  //统计tree写到哪一行了
			}
		}
		for (int i = 0; i != 2; ++i)  //每次产生两个可行点
		{
			bias_extend_tree(x_picture, y_picture, map, endnode, newpoint, tree, y1, y2, filled_num, greed_flag, x_start, x_end);
		}
		for (int i = 0; i != tree.size(); ++i)
		{
			double x_final_deviation = tree[i][0] - endnode[0];
			double y_final_deviation = tree[i][1] - endnode[1];  //计算每次找到的新节点与终点的距离，决定是否继续寻找新节点
			double final_deviation = sqrt(x_final_deviation*x_final_deviation + y_final_deviation * y_final_deviation);
			if (final_deviation < step)
			{
				cout << "已到达终点！" << endl;
				success = 1;  //说明已经到达终点，不必再继续拓展节点
				break;
			}
			else
			{
				success = 0;  //返回0后，主函数中会重新进入这个while循环，从而搜索下一个节点
			}
		}
	}
	finish = clock();  //结束计时
	double time_consuming = double(finish - start) / CLOCKS_PER_SEC;  //计算执行程序所用时间（秒）
	cout << "本次求解时间为：" << time_consuming << "秒" << endl;
	cout << "开始最优路径回溯" << endl;
	path_x.push_back(endnode[0]);  //记录终点坐标
	path_y.push_back(endnode[1]);
	find_path(path_x, path_y, tree, startnode, x_start, x_end);
	path_x.push_back(startnode[0]);  //记录起点坐标
	path_y.push_back(startnode[1]);
	cout << "最优路径回溯结束！" << endl;
	//**最终路径绘制**//
	Point p_start, p_end;
	p_start.x = startnode[0];
	p_start.y = startnode[1];
	circle(map, p_start, 5, Scalar(255, 255, 255), -1);   //画出起点
	p_end.x = endnode[0];
	p_end.y = endnode[1];
	circle(map, p_end, 5, Scalar(255, 255, 255), -1);  //画出终点
	int lastnode_num = 0;
	for (int i = 0; i != tree.size(); ++i)
	{
		if ((tree[i][0] != 0) && (tree[i][1] != 0))
		{
			lastnode_num = lastnode_num + 1;  //统计tree写到哪一行
		}
	}
	for (int i = 0; i != lastnode_num-1; ++i)  //画出搜索树的所有节点
	{
		Point p;
		p.x = tree[i][0];
		p.y = tree[i][1];    
		circle(map, p, 4, Scalar(200, 200, 0), -1); //画点：第三个参数为线宽，第五个参数设为-1，表明是个实点
		Point start_point = Point(tree[i][0], tree[i][1]);
		Point end_point = Point(tree[i + 1][0], tree[i + 1][1]);
	}
	for (int i = 0; i != path_x.size()-1;++i)  //画出回溯得到的最优路径
	{
		Point p;
		p.x = path_x[i];
		p.y = path_y[i];   
		circle(map, p, 2, Scalar(255, 50, 0), -1); //画点：第三个参数为线宽，第五个参数设为-1，表明是个实点
		Point start_point = Point(path_x[i], path_y[i]);
		Point end_point = Point(path_x[i + 1], path_y[i + 1]);
		line(map, start_point, end_point, Scalar(255, 0, 0), 3);  //画线
	}
	//**计算导向线**//
	vector<Point> point_set_line;
	for (int i = path_x.size() - 3; i < path_x.size(); ++i)
	{
		int x = path_x[i];
		int y = path_y[i];
		point_set_line.push_back(Point(x, y));
	}
	int num_line = fit_num(point_set_line);  //初始点开始的几个点的拟合
	Mat mat_k_line = polyfit(point_set_line, num_line);
	vector<double>x_direction_line, y_direction_line;
	for (int i = path_x[path_x.size() - 3]; i >= path_x[path_x.size() - 1]; --i)  //画出上部路径边缘拟合曲线，i决定多项式的底数
	{
		Point2d center;
		center.x = i;  //要画的点的x,y坐标值，圆心坐标
		center.y = 0;
		for (int j = 0; j < num_line + 1; ++j)  //j决定多项式中每一项的幂次
		{
			center.y += mat_k_line.at<double>(j, 0)*pow(i, j);
		}
		x_direction_line.push_back(center.x);
		x_direction_line.push_back(center.y);
		circle(map, center, 1, Scalar(160, 160, 0), CV_FILLED, CV_AA);
	}
	cout << "程序运行结束！" << endl;
	imshow("rrt", map);//贪婪RRT得到的路径
	waitKey(0);
	return 0;
}
