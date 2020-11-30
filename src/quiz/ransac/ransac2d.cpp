/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cmath>

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
    // distance between the line and a point
    double distance;
    // line parameters in ax + by + c = 0
    double a, b, c;
    // loop till max number of iterations
    while(maxIterations--)
    {
        // Create a new inliers set
        std::unordered_set<int> inliers;
        // Randomly sample two points and fit line
        while(inliers.size()<2)
        {
            inliers.insert(rand()%(cloud->points.size()));
        }
        auto itr = inliers.begin();
        auto firstPoint = cloud->points[*itr];
        auto secondPoint = cloud->points[*++itr];
        a = firstPoint.y - secondPoint.y;
        b = secondPoint.x - firstPoint.y;
        c = firstPoint.x * secondPoint.y - secondPoint.x * firstPoint.y;
        // loop through the points to calculate the distance
        int index = 0;
        for(auto point: cloud->points)
        {
            if(inliers.count(index)==0)
            {
                // Measure distance between every point and fitted line
                distance = fabs(a * point.x + b * point.y + c) / sqrt(pow(a,2) + pow(b,2));
                // If distance is smaller than threshold, add the point to the inliers
                if(distance<=distanceTol)
                    {
                        inliers.insert(index);
                     }
             }
            index++;

         }
        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }

    }


    // Return indicies of inliers from fitted line with most inliers

    return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function
	// distance between the plane and a point
	double distance;
	// plane parameters in ax + by + cz + d = 0
	double a, b, c, d;
	// loop till max number of iterations
	while(maxIterations--)
	{
		// Create a new inliers set
		std::unordered_set<int> inliers;
		// Randomly sample three points and fit a plane
		while(inliers.size()<3)
		{
			inliers.insert(rand()%(cloud->points.size()));
		}
		auto itr = inliers.begin();
		auto firstPoint = cloud->points[*itr];
		auto secondPoint = cloud->points[*++itr];
		auto thirdPoint = cloud->points[*++itr];
		a = (secondPoint.y - firstPoint.y)*(thirdPoint.z - firstPoint.z) - (secondPoint.z - firstPoint.z) * (thirdPoint.y - firstPoint.y);
		b = (secondPoint.z - firstPoint.z)*(thirdPoint.x - firstPoint.x) - (secondPoint.x - firstPoint.x) * (thirdPoint.z - firstPoint.z);
		c = (secondPoint.x - firstPoint.x)*(thirdPoint.y - firstPoint.y) - (secondPoint.y - firstPoint.y) * (thirdPoint.x - firstPoint.x);
		d =  -(a * firstPoint.x + b * firstPoint.y + c * firstPoint.z);
		// loop through the points to calculate the distance
		int index = 0;
		for(auto point: cloud->points)
		{
			if(inliers.count(index)==0)
			{
				// Measure distance between every point and fitted line
				distance = fabs(a * point.x + b * point.y + c * point.z + d) / sqrt(pow(a,2) + pow(b,2) + pow(c,2));
				// If distance is smaller than threshold, add the point to the inliers
				if(distance<=distanceTol)
					{
						inliers.insert(index);
					 }
			 }
			index++;

		 }
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
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
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 10, 0.5);
	std::unordered_set<int> inliers = RansacPlane(cloud, 10, 0.5);


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
