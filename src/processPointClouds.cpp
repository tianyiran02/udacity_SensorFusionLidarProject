// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#define _USE_MATH_DEFINES
#include <math.h>


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
    typename pcl::PointCloud<PointT>::Ptr bodyFiltered (new pcl::PointCloud<PointT>);
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Get roof point out of it
    pcl::CropBox<PointT> roofFilter;
    std::vector<int> indice;
    roofFilter.setMin(Eigen::Vector4f(-3, -2, -3, 1));
    roofFilter.setMax(Eigen::Vector4f(3, 2, 1, 1));
    roofFilter.setInputCloud(cloud);
    roofFilter.filter(indice);

    // Remove roof points
    pcl::ExtractIndices<PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr roofFiltered (new pcl::PointCloud<PointT>);
    pcl::PointIndices::Ptr roofIndice (new pcl::PointIndices);
    for (int point : indice)
    {
        roofIndice->indices.push_back(point);
    }
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (roofIndice);

    extract.setNegative (true);
    extract.filter (*roofFiltered);

    // Done:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::CropBox<PointT> boxFilter;
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.setInputCloud(roofFiltered);
    boxFilter.filter(*bodyFiltered);

    // Voxels
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr finalFiltered (new pcl::PointCloud<PointT>);
    vg.setInputCloud (bodyFiltered);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*finalFiltered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return finalFiltered;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Done: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr inliner_cloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr obst_cloud (new pcl::PointCloud<PointT> ());

    for (int i : inliers->indices)
    {
        inliner_cloud->points.push_back(cloud->points[i]);
    }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);

    // extract.setNegative (false);
    // extract.filter (*cloud_p);
    extract.setNegative (true);
    extract.filter (*obst_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obst_cloud, inliner_cloud);
    return segResult;
}

#define FOR_UDACITY_PROJECT
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, bool wall)
{
#ifndef FOR_UDACITY_PROJECT
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Done:: Fill in this function to find inliers for the cloud.
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    if (wall == true)
    {
        Eigen::Vector3f axis = Eigen::Vector3f(0.0, 1.0, 0.0); //z axis
        seg.setAxis(axis);
        seg.setEpsAngle(  60.0f * (M_PI/180.0f) ); // plane can be within 30 degrees of X-Y plane
        seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    }
    else
    {
        seg.setModelType (pcl::SACMODEL_PLANE);
    }

    // Mandatory
    seg.setOptimizeCoefficients(true);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if(inliers->indices.size() == 0)
    {
        std::cout << "No planar model found for given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
#else
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function
    int pointSize = cloud->points.size();
    // std::cout << "Cloud point counts: " << pointSize << std::endl;

    // For max iterations 
    for (int round = 0; round < maxIterations; round ++)
    {
        std::unordered_set<int> tempResult;
        std::unordered_set<int> pointsTmp;
        if (pointSize < 3)
        {
            // std::cout << "Two few points, return." << std::endl;
            goto output;
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
            // std::cout << "Calculated distance: " << dis << std::endl;

            // If distance is smaller than threshold count it as inlier
            if (dis < distanceThreshold)
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

output:

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        auto point = cloud->points[index];
        if(inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    return std::make_pair(cloudOutliers, cloudInliers);
#endif
}

template<typename PointT>
void ProcessPointClouds<PointT>::proxyPoints(std::vector<int>& pointCluster, std::unordered_set<int>& pointLog, const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol, int index)
{
    std::vector<int> nearPoint;

    pointLog.insert(index);
    pointCluster.push_back(index);

    nearPoint = tree->search(points[index], distanceTol);

    for (int i : nearPoint)
    {
        if (pointLog.count(i) == 0)
        {
            proxyPoints(pointCluster, pointLog, points, tree, distanceTol, i);
        }
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
#ifndef FOR_UDACITY_PROJECT
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Done:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (pcl::PointIndices getIndices: cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (int index: getIndices.indices)
        {
            cloud_cluster->points.push_back (cloud->points[index]); //*
        }
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
#else
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters_rtn;
    std::vector<std::vector<int>> clusters;
    std::unordered_set<int> pointLog;
    std::vector<std::vector<float>> points;

    KdTree* tree = new KdTree;

    for(int index = 0; index < cloud->points.size(); index++)
    {
        std::vector<float> pointvector;
        auto point = cloud->points[index];

        pointvector.push_back(point.x);
        pointvector.push_back(point.y);
        pointvector.push_back(point.z);

        points.push_back(pointvector);
    }

    for (int i=0; i<points.size(); i++) 
        tree->insert(points[i],i); 

    for (int index = 0; index < points.size(); index ++)
    {
        if (pointLog.count(index) == 0)
        {
            // point has not been process yet
            std::vector<int> cluster;

            // find the cluster relate to the points
            proxyPoints(cluster, pointLog, points, tree, clusterTolerance, index);

            // add clusters
            clusters.push_back(cluster);
        }
    }

    // generate return value according to result
    for (int i = 0; i < clusters.size(); i ++)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>());

        int clusterSiz = clusters.at(i).size();
        if ((clusterSiz>= minSize) && (clusterSiz <= maxSize))
        {
            for(int j = 0; j < clusterSiz; j++)
            {
                auto point = cloud->points[clusters.at(i).at(j)];
                cluster_cloud->points.push_back(point);
            }

            clusters_rtn.push_back(cluster_cloud);
        }
    }

    return clusters_rtn;
#endif
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
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    BoxQ box;

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                    ///    the signs are different and the box doesn't get correctly oriented in some cases.

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    box.bboxTransform = bboxTransform;
    box.bboxQuaternion = bboxQuaternion;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

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