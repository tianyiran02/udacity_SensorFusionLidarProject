/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered;

    /* Print the box and voxel processing */
    Box box = {-3, -2, -3, 3, 2, 1};
    renderBox(viewer, box, 1, Color(0,1,1));
    cloudFiltered = pointProcessorI->FilterCloud(inputCloud, 0.3f, Eigen::Vector4f(-30, -15, -5, 1), Eigen::Vector4f(30, 15, 10, 1));

    /* Plane segment */
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(cloudFiltered, 100, 0.2);
    // renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
    // renderPointCloud(viewer, segmentCloud.first, "objectCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    /* Clustering */
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 1.0, 10, 500);
    // rendering
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    int clusterId = 0;
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), Color(1,0,0));
        
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, Color(1,0,0));

        ++clusterId;
    }

    /* */
    // renderPointCloud(viewer, cloudFiltered, "cloudFiltered");
    // renderPointCloud(viewer, cloudFiltered, "cloudFiltered", {-1, 0 ,0});
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Done:: Create lidar sensor 
    Lidar * lidar = new Lidar(cars, 0);
    ProcessPointClouds<pcl::PointXYZ> * PointCloudProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    // Done:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr = lidar->scan();
    // renderRays(viewer,lidar->position,cloudPtr);
    // renderPointCloud(viewer, cloudPtr, "car");

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = PointCloudProcessor->SegmentPlane(cloudPtr, 100, 0.2);
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = PointCloudProcessor->Clustering(segmentCloud.first, 1.0, 10, 200);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        PointCloudProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        
        Box box = PointCloudProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::visualization::PCLVisualizer::Ptr viewerRaw (new pcl::visualization::PCLVisualizer ("3D Raw Viewer"));

    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    initCamera(setAngle, viewerRaw);

    // simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewerRaw->removeAllPointClouds();
        viewerRaw->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        // display raw data in viewerRaw
        renderPointCloud(viewerRaw, inputCloudI, "raw cloud");

        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
        viewerRaw->spinOnce ();
    }
}