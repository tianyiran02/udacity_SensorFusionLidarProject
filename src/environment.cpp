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
    Box box = {-3, -2, -1, 3, 2, 1};
    renderBox(viewer, box, 500, Color(0, 0.5, 0.5));
    cloudFiltered = pointProcessorI->FilterCloud(inputCloud, 0.3f, Eigen::Vector4f(-30, -10, -5, 1), Eigen::Vector4f(30, 10, 10, 1));

    /* Plane segment */
    // get the ground plane out
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(cloudFiltered, 100, 0.2, false);
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
    // get all the wall out
    bool cloudWall = true;
    int wallIndex = 1;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> tmpCloud = segmentCloud;
    while (cloudWall == true)
    {
        std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentWallCloud = pointProcessorI->SegmentPlane(tmpCloud.first, 100, 0.2, true);
        if (segmentWallCloud.second->points.size() >= 200)
        {
            tmpCloud = segmentWallCloud;
            // renderPointCloud(viewer, segmentWallCloud.second, "wallPlaneCloud" + wallIndex, Color(1,1,0));
            wallIndex++;
            if (wallIndex >= 3)
            {
                cloudWall = false;
            }
        }
        else
        {
            tmpCloud = segmentWallCloud;
            cloudWall = false;
        }
    }

#if 1
    /* Clustering */
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(tmpCloud.first, 1.0, 10, 500);
    // rendering
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    int clusterId = 0;
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), Color(1,0,0));
        
        BoxQ box = pointProcessorI->BoundingBoxQ(cluster);
        renderBox(viewer, box, clusterId, Color(1,0,0));

        ++clusterId;
    }
#endif
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

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = PointCloudProcessor->SegmentPlane(cloudPtr, 100, 0.2, false);
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

#define SHOW_RAW

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
#ifdef SHOW_RAW
    pcl::visualization::PCLVisualizer::Ptr viewerRaw (new pcl::visualization::PCLVisualizer ("3D Raw Viewer"));
#endif
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
#ifdef SHOW_RAW
    initCamera(setAngle, viewerRaw);
#endif
    // simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
#ifdef SHOW_RAW
        viewerRaw->removeAllPointClouds();
        viewerRaw->removeAllShapes();
#endif

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
#ifdef SHOW_RAW
        // display raw data in viewerRaw
        renderPointCloud(viewerRaw, inputCloudI, "raw cloud");
#endif

        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
#ifdef SHOW_RAW
        viewerRaw->spinOnce ();
#endif
    }
}