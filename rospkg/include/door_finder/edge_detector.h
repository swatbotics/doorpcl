#ifndef EDGE_DETECTOR
#define EDGE_DETECTOR

#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include "plane_segmenter.h"
#include "SimpleConfig.h"


//mouse click callback- currently prints out location of mouse click
//on viewer
//void mouseClick(const pcl::visualization::MouseEvent &event, void* detector);


//keyboard event handler callback
/*
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent
                            &event, void* detector );
*/

class EdgeDetector
{
public:
    //I think this still makes it recompile
    typedef PlaneSegmenter::Point Point;  
    typedef PlaneSegmenter::PointCloud PointCloud;
    typedef std::vector< cv::Vec4i > LineArray;
    typedef std::vector< pcl::PointXYZ > LinePosArray;

    std::string filename;

    float deviceFocalLength;
    float pixel_size;
    bool doWrite;
    double point_radius, minDistOffPlane, maxDistOffPlane;

    float fx, fy, u0, v0;
    bool waiting;

    int current_grasp_index;

    Eigen::Vector3f handlePos;
    Eigen::Vector3f handleAxis;
    double handleRadius;

    Eigen::Vector2i handle0, handle1;

    cv::Mat planeImage, currentIntensityImage, displayImage;

    //these hold information on the current plane 
    //segmented picture.
    PointCloud::ConstPtr curr_cloud;
    std::vector< pcl::ModelCoefficients > planes;

    int frame_index;    //the index of the plane currently
                        //being viewed
    //the xyz points of the corners of the doors in 3D space
    std::vector< pcl::PointXYZ > doorPoints;
    pcl::IndicesPtr handleIndices;

    pcl::ModelCoefficients handleCoeffs;

    //the u, v points of the corners of the doors in 
    //the picture plane
    std::vector< Eigen::Vector2i > drawPoints;

    ~EdgeDetector(){} 
    //a simple constructor using a config file.
    EdgeDetector ( const std::string & configFile );
    //find the closest point and return the index and distance squared
    void findClosestDrawPoint( int u, int v, 
                               int & closestIndex,
                               int & minDist2 );


    void inputPointCloud( const PointCloud::ConstPtr & cloud,
                          bool view );


    //the doorPos is the position of the center of the door,
    void getDoorInfo(double & height, double & width,
                     Eigen::Vector3f & doorPos, 
                     Eigen::Vector3f & doorRot );

    //the length is how long the door hangle is, the
    //upward thickness of the door handle.
    //The offset is an (x,y,z) vector offset from the center of the door.
    //Or maybe 
    void getHandleInfo( double & length, double & height,
                        Eigen::Vector3f & center );

    //This is for getting the door plane.
    int getDoorPlane();
    bool isWithinBounds( int u, int v );

    //add a u, v point to the set of door points. 
    //if the index is anything but -1, then the
    //point at that index will be modified
    //returns false if an invalid index was given
    bool addDoorPoint ( int u, int v, const int index = -1);
    void doorMouseClick( int u, int v );
    void doorMouseMovement( int u, int v );

    //draw the lines that the segmenter found
    pcl::PointXYZ projectPoint( int u, int v, int p );
    void handleMouseSelection( int u1, int v1, int u2, int v2 );

    //if the points do not follow a counter clockwise ordering,
    //reorder them so that they do.
    void orderPoints();

    void setDoorPoints();


    void getHandlePoints();
    double distanceFromPlane( const Point & point,
                              const pcl::ModelCoefficients & coeffs);
    void makeDisplayImage();


private:
    SimpleConfig config;

    

    //this holds a set of colors for visualization
    std::vector< cv::Vec3i > colors;

    PlaneSegmenter segmenter;


    //this implements a left of test, and switches any 
    //points that fail the left-of test.
    void left_of_switch( const int index1, const int index2, const int index3 );

    //This is the main door specifying event loop.
    void waitAndDisplay();

    //clear the plane viewer of the lines.
    void removeAllDoorLines();

    //saves pcd files from grabbed pointclouds
    void savePointCloud(const PointCloud & cloud);

    //reads existing pcd files
    void readPointCloud(PointCloud::Ptr & cloud);

};


#endif
