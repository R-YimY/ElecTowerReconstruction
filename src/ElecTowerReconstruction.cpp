#include"ElecTowerReconstruction.h"

//读取数据，数据格式为xyz;
PointCloudT::Ptr ElecTowerReconsrutction::DataInput(string& filepath) {
	ifstream file;
	file.open(filepath);
	double x, y, z;
	PointCloudT::Ptr cloud(new PointCloudT);
	while (file >> x >> y >> z) {
		PointT p;
		p.x = x;
		p.y = y;
		p.z = z;
		cloud->push_back(p);
	}

	double X(0), Y(0), Z(0);
	for (int i(0); i < cloud->size(); i++) {
		X += cloud->points[i].x;
		Y += cloud->points[i].y;
		Z += cloud->points[i].z;
	}

	X = X / cloud->points.size();
	Y = Y / cloud->points.size();
	Z = Z / cloud->points.size();


	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x = cloud->points[i].x - X;
		cloud->points[i].y = cloud->points[i].y - Y;
		cloud->points[i].z = cloud->points[i].z - Z;
	}
	//pcl::io::savePCDFile("cloud.pcd", *cloud);
	return cloud;
}

//计算原始点云的包围盒
void ElecTowerReconsrutction::AABBCompute(PointCloudT::Ptr &cloud) {
	pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	PointT min_point_AABB;//AABB包围盒
	PointT max_point_AABB;
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);

	OriginMaxAABB = max_point_AABB;
	OriginMinAABB = min_point_AABB;

}

//计算中心点
PointT CentorPoint(PointCloudT::Ptr &cloud) {
	PointT centor;
	double X(0), Y(0), Z(0);
	for (int i(0); i < cloud->size(); i++) {
		X += cloud->points[i].x;
		Y += cloud->points[i].y;
		Z += cloud->points[i].z;
	}

	X = X / cloud->points.size();
	Y = Y / cloud->points.size();
	Z = Z / cloud->points.size();

	centor.x = X;
	centor.y = Y;
	centor.z = Z;

	return centor;
}




//点云滤波
PointCloudT::Ptr ElecTowerReconsrutction::PointCloudFilter(PointCloudT::Ptr & cloud)
{
	PointCloudT::Ptr cloud_filtered(new PointCloudT);
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(cloud);// 填入点云
	sor.setMeanK(8);        // 设置均值参数K
	sor.setStddevMulThresh(0.6);// 设置
	sor.filter(*cloud_filtered);
	return cloud_filtered;
}

PointCloudT::Ptr ElecTowerReconsrutction::Projection_X(PointCloudT::Ptr &cloud) {

	PointCloudT::Ptr cloud_bev(new PointCloudT);//投影后点云
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = 1.0;
	coefficients->values[1] = 0.0;
	coefficients->values[2] = 0.0;
	coefficients->values[3] = 0;
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType(pcl::SACMODEL_PLANE); // 三维平面参数
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_bev);
	double angle = M_PI / 2;//旋转90°
	Eigen::Affine3f transform_Z = Eigen::Affine3f::Identity();
	Eigen::Affine3f transform_X = Eigen::Affine3f::Identity();
	transform_Z.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
	transform_X.rotate(Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitX()));

	PointCloudT::Ptr cloud_transformed(new PointCloudT);
	pcl::transformPointCloud(*cloud_bev, *cloud_transformed, transform_Z);
	pcl::transformPointCloud(*cloud_transformed, *cloud_transformed, transform_X);

	pcl::io::savePCDFile("result/Projection_X.pcd", *cloud_transformed);
	return	cloud_transformed;
}

PointCloudT::Ptr ElecTowerReconsrutction::Projection_Y(PointCloudT::Ptr &cloud) {
	PointCloudT::Ptr cloud_bev(new PointCloudT);//投影后点云
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = 0.0;
	coefficients->values[1] = 1.0;
	coefficients->values[2] = 0.0;
	coefficients->values[3] = 0;
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType(pcl::SACMODEL_PLANE); // 三维平面参数
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_bev);
	double angle = M_PI / 2;//旋转90°
	Eigen::Affine3f transform_X = Eigen::Affine3f::Identity();
	transform_X.rotate(Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitX()));
	PointCloudT::Ptr cloud_transformed(new PointCloudT);
	pcl::transformPointCloud(*cloud_bev, *cloud_transformed, transform_X);
	//pcl::io::savePCDFile("result/Projection_Y.pcd", *cloud_transformed);
	return	cloud_transformed;
}

void ElecTowerReconsrutction::Rerotate_Y(PointCloudT::Ptr &cloud) {
	double angle = M_PI / 2;//旋转90°
	Eigen::Affine3f transform_X = Eigen::Affine3f::Identity();
	PointCloudT::Ptr cloud_transformed(new PointCloudT);
	transform_X.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitX()));
	pcl::transformPointCloud(*cloud, *cloud_transformed, transform_X);
	cloud = cloud_transformed;
}
void ElecTowerReconsrutction::Rerotate_X(PointCloudT::Ptr &cloud) {
	double angle = M_PI / 2;//旋转90°
	PointCloudT::Ptr cloud_transformed(new PointCloudT);

	Eigen::Affine3f transform_Z = Eigen::Affine3f::Identity();
	Eigen::Affine3f transform_X = Eigen::Affine3f::Identity();
	transform_X.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitX()));
	transform_Z.rotate(Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*cloud, *cloud_transformed, transform_X);
	pcl::transformPointCloud(*cloud_transformed, *cloud_transformed, transform_Z);

	//pcl::io::savePCDFile("result/Rerotate_X.pcd", *cloud_transformed);
	cloud = cloud_transformed;
}

PointCloudT::Ptr ElecTowerReconsrutction::PointScale(PointCloudT::Ptr &cloud) {

	pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	PointT min_point_AABB;//AABB包围盒
	PointT max_point_AABB;
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);
	pcl::PointCloud<PointT>::Ptr cloudScale(new pcl::PointCloud<PointT>);
	Eigen::Isometry3d S = Eigen::Isometry3d::Identity();

	double dx1 = sqrt(pow(double(min_point_AABB.x - max_point_AABB.x), 2));
	double dx2 = sqrt(pow(double(OriginMaxAABB.x - OriginMinAABB.x), 2));
	double dy1= sqrt(pow(double(min_point_AABB.y - max_point_AABB.y), 2));
	double dy2 = sqrt(pow(double(OriginMaxAABB.y - OriginMaxAABB.y), 2));

	double scale_x = dx1/ dx2;
	double scale_y = dy1/ dy2;
	double scale_z=0;

	S = Eigen::Scaling(scale_x, scale_y, scale_z);
	// 执行缩放变换，并将结果保存在 s_cloud 中
	pcl::transformPointCloud(*cloud, *cloudScale, S.matrix());
	return cloudScale;
}

Mat ElecTowerReconsrutction::PointGrid(PointCloudT::Ptr & cloud)
{

	float Xmax, Xmin, Ymax, Ymin;
	float step = 0.05; // 格网边长
	PointT min_pt, max_pt;
	pcl::getMinMax3D(*cloud, min_pt, max_pt);
	
	PX_xmin = Xmin = min_pt.x;
	PY_ymin = Ymin = min_pt.y;
	Xmax = max_pt.x;
	Ymax = max_pt.y;

	cout << "最小点坐标：" << min_pt.x << " " << min_pt.y << endl;
	cout << "最大点坐标：" << max_pt.x << " " << max_pt.y << endl;
	int Row = ceil((Ymax - Ymin) / step);//格网行数
	int Col = ceil((Xmax - Xmin) / step);//格网列数

	cout << "格网总数：" << Row << " " << Col << " " << Row * Col << endl;
	//包含点的格网
	vector<PointT>** PointsInGrid = new vector<PointT>*[Row];
	for (int i = 0; i < Row; i++)
	{
		PointsInGrid[i] = new vector<PointT>[Col];
	}
	for (int i = 0; i < (*cloud).size(); i++)
	{
		int rowID = static_cast<int>((cloud->points[i].y - Ymin) / step);
		int colID = static_cast<int>((cloud->points[i].x - Xmin) / step);
		PointsInGrid[rowID][colID].push_back(cloud->points[i]);
	}

	cv::Size imagesize;
	imagesize.width = Col + 100; //短的
	imagesize.height = Row + 100;

	Mat preImage = Mat::zeros(imagesize, CV_8UC1);

	for (int i(0); i < Col; i++) {       //宽度循环
		for (int j(1); j < Row; j++) {
			if (PointsInGrid[j][i].size() > 0)			
				preImage.at<uchar>(Row - j + 50, i + 50) = 255;
			else 
				preImage.at<uchar>(Row - j + 50, i + 50) = 0;
		}
	}
	cout << "preImage.size " << preImage.size() << endl;
	imwrite("二值图.png", preImage);
	return preImage;
}

void  ElecTowerReconsrutction::CornorExtract(Mat& img) {

	PointCloudT::Ptr inp(new PointCloudT);
	PointCloudT::Ptr outp(new PointCloudT);

	Mat kernel, gradient_Img, out, edgeout;
	kernel = getStructuringElement(MORPH_RECT, Size(10, 10));
	morphologyEx(img, gradient_Img, MORPH_GRADIENT, kernel);

	imwrite("闭合前.jpg", gradient_Img);
	Mat element = getStructuringElement(MORPH_RECT, Size(30, 30));
	morphologyEx(gradient_Img, out, MORPH_CLOSE, element);
	imwrite("闭合后.jpg", out);
	
	//2. 边缘检测
	Mat CannyImg;
	cv::Canny(out, CannyImg, 30.0, 70.0 , 3 , false);
	imwrite("边缘.jpg", CannyImg);

	//3.角点检测
	vector<vector<cv::Point>> contours;
	vector<Vec4i> hierachy;
	findContours(CannyImg, contours, hierachy, RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(-1, -1));
	cout << "contours.size():  " << contours.size() << endl;
	vector<vector<cv::Point> >poly(contours.size());
	for (int i = 0; i < contours.size(); i++){
		approxPolyDP(Mat(contours[i]), poly[i], 5, true);
	}

	int nl = CannyImg.rows; //行数
	int nc = CannyImg.cols * CannyImg.channels();

	if (contours.size() > 2) {
		for (int j(0); j < poly[0].size(); j++) {
			pair<int, int> pt;
			pt.second = poly[0][j].x;
			pt.first = poly[0][j].y;
			outerpoints.push_back(pt);
		}
		for (int j(0); j < poly[2].size(); j++) {
			pair<int, int> pt;
			pt.second = poly[2][j].x;
			pt.first = poly[2][j].y;
			innerpoints.push_back(pt);
		}

	}
	else {
		for (int j(0); j < poly[0].size(); j++) {
			pair<int, int> pt;
			pt.second = poly[0][j].x;
			pt.first = poly[0][j].y;
			outerpoints.push_back(pt);
		}
		for (int j(0); j < poly[1].size(); j++) {
			pair<int, int> pt;
			pt.second = poly[1][j].x;
			pt.first = poly[1][j].y;
			innerpoints.push_back(pt);
		}
	}

	for (int i(0); i < outerpoints.size(); i++) {
		PointT pt;
		pt.x = (outerpoints[i].second)*0.05 + PX_xmin - 2.5;
		pt.y = (nl - outerpoints[i].first)*0.05 + PY_ymin - 2.5;
		outp->push_back(pt);
	}
	for (int i(0); i < innerpoints.size(); i++) {
		PointT pt;
		pt.x = (innerpoints[i].second)*0.05 + PX_xmin - 2.5;
		pt.y = (nl - innerpoints[i].first)*0.05 + PY_ymin - 2.5;
		inp->push_back(pt);
	}

	inpts = inp;
	oupts = outp;

	cout << "oupts.size(): " << outp->size() << endl;
	cout << "inpts.size(): " << inp->size() << endl;

	//pcl::io::savePCDFile("result/oupts.pcd", *oupts);
	//pcl::io::savePCDFile("result/inpts.pcd", *inpts);

}




PointCloudT::Ptr  ElecTowerReconsrutction::Img2Points(Mat& Image) {
	PointCloudT::Ptr re(new PointCloudT);
	//遍历图片，转换为点云
	int nl = Image.rows; //行数
	int nc = Image.cols * Image.channels(); //每行元素的总元素数量
	for (int j = 0; j < nl; j++) {
		uchar* data = Image.ptr<uchar>(j);
		for (int i = 0; i < nc; i++) {
			if (data[i] > 10)
			{
				PointT pt;
				pt.x = i / 3 * 0.05 + PX_xmin;
				pt.y = (nl - j)*0.05 + PY_ymin;
				re->push_back(pt);
			}
		}
	}

	return re;
}

//点云对称
PointCloudT::Ptr ElecTowerReconsrutction::PointSymmetry(PointCloudT::Ptr &points, int state) {
	//设置状态进行角点链接 state=1表示关于X轴对称 ；2表示关于Y。。。。
	PointCloudT::Ptr transformedCloud(new PointCloudT);
	float A(0), B(0), C(0), D(0);//A,B,C,D为平面Ax+By+Cz=D
	switch (state)
	{
	case 1:
		A = 1;
		break;
	case 2:
		B = 1;
		break;
	case 3:
		C = 1;
		break;
	default:
		break;
	}
	float e = sqrt(A * A + B * B + C * C);//为了后续将平面法向量转化为单位向量
	float a = A / e, b = B / e, c = C / e;
	float r = D / e;
	Eigen::Matrix4f n;//旋转矩阵n
	n << 1 - 2 * a*a, -2 * a*b, -2 * a*c, -2 * a*r,
		-2 * a*b, 1 - 2 * b*b, -2 * b*c, -2 * b*r,
		-2 * a*c, -2 * b*c, 1 - 2 * c*c, -2 * c*r,
		0, 0, 0, 1;
	pcl::transformPointCloud(*points, *transformedCloud, n);

	pcl::io::savePCDFile("result/PointSymmetry.pcd", *transformedCloud);
	return transformedCloud;
}

//按照塔身宽度赋予x坐标
void ElecTowerReconsrutction::CordinateFill(PointCloudT::Ptr &cornor,double width) {
	
	Rerotate_X(cornor);//点云重新回到初始方向 ，此时X坐标为0
	for (int i(0); i < cornor->size(); i++) {
		cornor->points[i].x = width;
	}
	pcl::io::savePCDFile("result/CordinateFill.pcd", *cornor);
}




void ElecTowerReconsrutction::TopReconstrution(string & filepath)
{
	PointCloudT::Ptr cloud = DataInput(filepath);
	//cloud = PointCloudFilter(cloud);
	PointCloudT::Ptr cloud_projectionX = Projection_X(cloud);//沿着X投影的点云
	cv::Mat Grid_X = PointGrid(cloud_projectionX);
	CornorExtract(Grid_X);//角点检测并保存到目标指针
	CordinateFill(oupts, 2.0);
	CordinateFill(inpts, 2.0);
	PointCloudT::Ptr Syin = PointSymmetry(inpts,1);
	PointCloudT::Ptr Syou = PointSymmetry(oupts,1);
	

	string file = "result/re.obj";
	DataOutput(inpts, oupts, Syin, Syou,file);
}

void ElecTowerReconsrutction::DataOutput(PointCloudT::Ptr &ipt, PointCloudT::Ptr &opt, 
	PointCloudT::Ptr &Sipt, PointCloudT::Ptr &Sopt, string& filepath) {
	int m1 = ipt->points.size();
	int m2 = Sipt->points.size();
	int m3 = opt->points.size();
	int m4 = Sopt->points.size();

	cout << m1 << " " << m2 << " " << m3 << " " << m4 << " " << endl;
	//保存为obj格式的数据
	ofstream fileout;
	fileout.open(filepath);
//===================================点保存==================================
	//保存内点
	for (int i(0); i < ipt->size(); i++) {
		fileout << "v " << ipt->points[i].x << " " << ipt->points[i].y << " "<< ipt->points[i].z << endl;
	}
	for (int i(0); i < Sipt->size(); i++) {
		fileout << "v " << Sipt->points[i].x << " " << Sipt->points[i].y << " " << Sipt->points[i].z << endl;
	}

	//保存外点
	for (int i(0); i < opt->size(); i++) {
		fileout << "v " << opt->points[i].x << " " << opt->points[i].y << " " << opt->points[i].z << endl;
	}
	for (int i(0); i < Sopt->size(); i++) {
		fileout << "v " << Sopt->points[i].x << " " << Sopt->points[i].y << " " << Sopt->points[i].z << endl;
	}

//===================================线保存==================================

	for (int i(0); i < m1; i++) {
		if (i != m1 - 1)
			fileout << "l " << i + 1 << " " << i + 2 << endl;
		else
			fileout << "l " << i + 1 << " " << 1 << endl;
	}
	for (int i(0); i < m2; i++) {
		if (i != m2 - 1)
			fileout << "l " << m1+i + 1 << " " << m1+i + 2 << endl;
		else
			fileout << "l " << m1+i + 1 << " " << m1+1 << endl;
	}
	for (int i(0); i < m3; i++) {
		if (i != m3 - 1)
			fileout << "l " << m1+m2 + i + 1 << " " << m1 + m2 + i + 2 << endl;
		else
			fileout << "l " << m1 + m2 + i + 1 << " " << m1 + m2+1 << endl;
	}
	for (int i(0); i < m4; i++) {
		if (i != m4 - 1)
			fileout << "l " << m1 + m2+m3 + i + 1 << " " << m1 + m2 + m3 + i + 2 << endl;
		else
			fileout << "l " << m1 + m2 + m3 + i + 1 << " " << m1 + m2 + m3+1 << endl;
	}
}