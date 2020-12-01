// PCL lib Functions for processing point clouds

#include "processPointClouds.h"




// ProcessPointClouds
//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // downsample the cloud points into voxels
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxGrid;
    voxGrid.setInputCloud(cloud);
    voxGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxGrid.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
    // Crop a box
    pcl::CropBox<PointT> regionOfInterest(true);
    regionOfInterest.setMin(minPoint);
    regionOfInterest.setMax(maxPoint);
    regionOfInterest.setInputCloud(cloudFiltered);
    regionOfInterest.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.5, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.5, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int pointIndex : indices)
    {
        inliers->indices.push_back(pointIndex);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    // Create filtering object to extract the obstacles points from the cloud given the inliers
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    // The planeCloud will contain all points of cloud that are indexed by inliers
    extract.setNegative(false);
    extract.filter(*planeCloud);
    // Setting setNegative to true will do the opposite
    extract.setNegative(true);
    extract.filter(*obstCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // TODO:: Fill in this function to find inliers for the cloud.
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // RANSAC will return the indices of the points that belongs to a plane
    // Create a pointer that points to the points indices
	pcl::PointIndices::Ptr inliers  {new pcl::PointIndices};
    // Custom RANSAC implementation
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
		std::unordered_set<int> inliers_set;
		// Randomly sample three points and fit a plane
		while(inliers_set.size()<3)
		{
			inliers_set.insert(rand()%(cloud->points.size()));
		}
		auto itr = inliers_set.begin();
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
			if(inliers_set.count(index)==0)
			{
				// Measure distance between every point and fitted line
				distance = fabs(a * point.x + b * point.y + c * point.z + d) / sqrt(pow(a,2) + pow(b,2) + pow(c,2));
				// If distance is smaller than threshold, add the point to the inliers
				if(distance<=distanceThreshold)
					{
						inliers_set.insert(index);
					 }
			 }
			index++;

		 }
    // save the indices described by the best model.(largest number of point indices)
		if (inliers_set.size() > inliersResult.size())
		{
			inliersResult = inliers_set;
		}

	}
    // save the result in a pcl::PointIndices variable
    for(auto idx : inliersResult)
    {
        inliers->indices.push_back(idx);
    }

    if(inliers->indices.size()==0)
    {
        std::cerr<<"Could not estimate a planer model for the given data"<<std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    // separate the cloud using the inliers indices
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    // return a pair of clouds(obstacles, road).
    return segResult;
}

// PCL built-in RANSAC segmentation function

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlanePcl(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // TODO:: Fill in this function to find inliers for the cloud.
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // RANSAC will return the indices of the points that belongs to a plane
    // Create a pointer that points to the points indices
	  pcl::PointIndices::Ptr inliers  {new pcl::PointIndices};
    // pcl built-in RANSAC
    pcl::ModelCoefficients::Ptr coefficients { new pcl::ModelCoefficients};
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    // Segement the largest planer component from the given cloud data
    seg.setInputCloud(cloud);
    // the coefficients of the plane can be used to render the plane in the pcl viewer.
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size()==0)
    {
        std::cerr<<"Could not estimate a planer model for the given data"<<std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    // pcl clustering

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    //typename std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
    typename std::vector<pcl::PointIndices> clusterIndices;
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // 3d tree for the obstcale points
    typename pcl::search::KdTree<PointT>::Ptr tree (new  pcl::search::KdTree<PointT>);
    // feed the tree with the cloud
    tree->setInputCloud(cloud);
    // euclidean clustering object
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    // any points within 0.1 will be clustered together
    ec.setClusterTolerance (clusterTolerance); //  in m
    // minimum number of points to form a cluster
    ec.setMinClusterSize (minSize);
    // maximum number of points to form a cluster
    ec.setMaxClusterSize (maxSize);
    // set the search method to 3d tree
    ec.setSearchMethod (tree);
    // use the obstacle cloud as input
    ec.setInputCloud (cloud);
    // extract the clusters
    ec.extract (clusterIndices);
    for(auto Indices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for(auto index : Indices.indices)
            cloudCluster->push_back(cloud->points[index]);
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        // save a cluster cloud in cloudCluster
        clusters.push_back(cloudCluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

//custom clustering
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering3D(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    KDTree* tree = new KDTree;
    // insert the cloud points in the tree
    for (int i=0; i<cloud->points.size(); i++)
    	tree->insert((cloud->points[i]),i);
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::vector<int>> clustersIndices;
	// vector to track unporcessed points
	std::vector<bool> processedPoints(cloud->points.size(),false);
	int pointIndex = 0;
	// loop over the points
    for(auto  point: cloud->points)
	{
        // if not processed
		if(!processedPoints[pointIndex++])
		{
            // create a cluster
			std::vector<int> cluster;
            // recursive function  to assign points to the created cluster
			clusterHelper(pointIndex, cloud, cluster, tree, clusterTolerance, processedPoints);
			// set limits to cluster size
            if((cluster.size()>=minSize) && cluster.size()<= maxSize)
            // add to clusters vector
            clustersIndices.push_back(cluster);

		}
	}
    // create a point cloud for each cluster
    for(auto cluster : clustersIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new typename pcl::PointCloud<PointT>);
        for (auto pointIndex : cluster)
            clusterCloud->points.push_back(cloud->points[pointIndex]);
        clusters.push_back(clusterCloud);
    }
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;


	return clusters;

}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

// recursive function
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int pointIndex, typename pcl::PointCloud<PointT>::Ptr const & points, std::vector<int> &cluster, KDTree*& tree, float distanceTol, std::vector<bool>& processedPoints)
{
    // mark the point as processed
    processedPoints[pointIndex] = true;
    // add it to the cluster
    cluster.push_back(pointIndex);
    // search for nearby points
    std::vector<int> nearbyPointIndices = tree->search(points->points[pointIndex], distanceTol);
	// for every nearby point
    for (auto nearbyPointIndex: nearbyPointIndices)
	{
        // check if it has not been processed yet
		if(!processedPoints[nearbyPointIndex])
		{
            // recurse  till every point of the nearby points is processed
			clusterHelper(nearbyPointIndex, points, cluster, tree, distanceTol, processedPoints);
		}
	}


}
