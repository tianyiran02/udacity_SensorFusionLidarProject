/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	int pointSize = cloud->points.size();
	std::cout << "Cloud point counts: " << pointSize << std::endl;

	// For max iterations 
	for (int round = 0; round < maxIterations; round ++)
	{
		std::unordered_set<int> tempResult;
		int pointa = rand() % pointSize + 1;
		int pointb = rand() % pointSize + 1;
		
		// Randomly sample subset and fit line
		pcl::PointXYZ &pa = cloud->points[pointa];
		pcl::PointXYZ &pb = cloud->points[pointb];

		// Calculate a and b
		float a = pa.y - pb.y;
		float b = pa.x - pb.x;
		float c = pa.x * pb.y - pb.x * pa.y;
		float tmp = sqrt(pow(a, 2) + pow(b, 2));

		for (int point = 0; point < pointSize; point ++)
		{
			// Measure distance between every point and fitted line
			float dis = fabs(a * cloud->points[point].x + b * cloud->points[point].y + c) / tmp;
			std::cout << "Calculated distance: " << dis << std::endl;

			// If distance is smaller than threshold count it as inlier
			if (dis < distanceTol)
			{
				tempResult.insert(point);
			}
		}

		if (tempResult.size() > inliersResult.size())
		{
			inliersResult = tempResult;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	int pointSize = cloud->points.size();
	std::cout << "Cloud point counts: " << pointSize << std::endl;

	// For max iterations 
	for (int round = 0; round < maxIterations; round ++)
	{
		std::unordered_set<int> tempResult;
		std::unordered_set<int> pointsTmp;
		if (pointSize < 3)
		{
			std::cout << "Two few points, return." << std::endl;
			return inliersResult;
		}
		while(pointsTmp.size()<3)
		{
			pointsTmp.insert(rand() % pointSize + 1);
		}

		auto itr = pointsTmp.begin();

		float x1 = (cloud->points[*itr]).x;
		float y1 = (cloud->points[*itr]).y;
		float z1 = (cloud->points[*itr]).z;
		itr ++;
		float x2 = (cloud->points[*itr]).x;
		float y2 = (cloud->points[*itr]).y;
		float z2 = (cloud->points[*itr]).z;
		itr ++;
		float x3 = (cloud->points[*itr]).x;
		float y3 = (cloud->points[*itr]).y;
		float z3 = (cloud->points[*itr]).z;

		float tmpi = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		float tmpj = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		float tmpk = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		float tmpd = -(tmpi * x1 + tmpj * y1 + tmpk * z1);

		float tmp = sqrt(pow(tmpi, 2) + pow(tmpj, 2) + pow(tmpk, 2));

		for (int point = 0; point < pointSize; point ++)
		{
			// Measure distance between every point and fitted line
			float dis = fabs(tmpi * cloud->points[point].x + tmpj * cloud->points[point].y + tmpk * cloud->points[point].z + tmpd) / tmp;
			std::cout << "Calculated distance: " << dis << std::endl;

			// If distance is smaller than threshold count it as inlier
			if (dis < distanceTol)
			{
				tempResult.insert(point);
			}
		}

		if (tempResult.size() > inliersResult.size())
		{
			inliersResult = tempResult;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 200, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
