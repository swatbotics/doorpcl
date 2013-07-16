#ifndef EDGE_DETECTOR
#define EDGE_DETECTOR

#include <string>

#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include "plane_segmenter.h"

#include "SimpleConfig.h"


//mouse click callback- currently prints out location of mouse click
//on viewer
void mouseClick(const pcl::visualization::MouseEvent &event, void* detector);


//keyboard event handler callback
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent
                            &event, void* detector );


class EdgeDetector
{
public:
    typedef pcl::PointXYZRGBA Point;  
    typedef pcl::PointCloud<Point> PointCloud;
    typedef std::vector< cv::Vec4i > LineArray;
    typedef pcl::visualization::PointCloudColorHandlerRGBField<Point> 
                ColorHandler;
    typedef std::vector< pcl::PointXYZ > LinePosArray;

    std::string filename;

    const float deviceFocalLength;
    const float pixel_size;
    bool viewerIsInitialized, doWrite;
    bool showImage;
    double radius;
    pcl::

    float fx, fy, u0, v0;
    bool waiting;

    Eigen::Vector2i handle0, handle1;

    //these hold information on the current plane 
    //segmented picture.
    std::vector<plane_data> planes;
    int frame_index;    //the index of the plane currently
                        //being viewed
    //the xyz points of the corners of the doors in 3D space
    std::vector< pcl::PointXYZ > doorPoints;
    //the u, v points of the corners of the doors in 
    //the picture plane
    std::vector< Eigen::Vector2i > drawPoints;


    //a simple constructor using a config file.
    EdgeDetector ( const std::string & configFile );

    //this function will run until the reader throws an error about 
    //a non-existant file.
    void runWithInputFile();

    //run with input from the camera
    void run ();

    //the doorPos is the position of the center of the door,
    void getDoorInfo(double & height, double & width,
                     Eigen::Vector3f & doorPos, 
                     Eigen::Vector3f & doorRot );

    //add a u, v point to the set of points,
    int addDoorPoint ( int u, int v);

    //draw the lines that the segmenter found
    void drawLines ();

    pcl::PointXYZ projectPoint( int u, int v, int p );

    //if the points do not follow a counter clockwise ordering,
    //reorder them so that they do.
    void orderPoints();

    int current_grasp_index;

    void getHandlePoints( pcl::PointIndices & indices ){
    


private:
    SimpleConfig config;


    //this holds a set of colors for visualization
    std::vector< cv::Vec3i > colors;

    int view1, view2;
    pcl::visualization::PCLVisualizer * line_viewer;
    pcl::visualization::ImageViewer * image_viewer;
    pcl::visualization::ImageViewer * plane_viewer;

    PlaneSegmenter segmenter;


    //create a viewer that holds lines and a point cloud.
    void updateViewer( const PointCloud::ConstPtr &cloud,
                       const std::vector< LinePosArray > & planarLines );

    void initViewer( const PointCloud::ConstPtr & cloud );
    


    //this implements a left of test, and switches any 
    //points that fail the left-of test.
    void left_of_switch( const int index1, const int index2, const int index3 );

    //point cloud callback function gets new pointcloud and runs segmentation
    //algorithm
    void cloud_cb_ (const PointCloud::ConstPtr &cloud);

    //This is the main door specifying event loop.
    void waitAndDisplay();

    //clear the plane viewer of the lines.
    void removeAllDoorLines();

    //saves pcd files from grabbed pointclouds
    void savePointCloud(const PointCloud & cloud);

    //reads existing pcd files
    void readPointCloud(PointCloud::Ptr & cloud);

    //a utility function to change the color of a point cloud.
    //this is currently unused.
    inline void convertColor( PointCloud::Ptr & cloud,
                       cv::Mat & mat,
                       pcl::PointIndices::Ptr inliers);

};




#endif
