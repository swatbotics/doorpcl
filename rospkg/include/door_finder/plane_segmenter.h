#ifndef PLANE_SEGMENTER
#define PLANE_SEGMENTER

#include <iostream>
#include <exception>
#include <assert.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/filter.h>
#include <pcl/common/common_headers.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include "SimpleConfig.h"


class PlaneSegmenter{


public:
    typedef pcl::PointXYZI Point;  
    typedef pcl::PointCloud<Point> PointCloud;
    typedef std::vector< cv::Vec4i > LineArray;
    typedef std::vector< pcl::PointXYZ > LinePosArray;

    bool getLines;
    
    PlaneSegmenter( const std::string & configFileName );
    PlaneSegmenter(int maxNumPlanes=6, int minSize=50000,
                   bool optimize=false, float threshold=0.03, 
                   int sacMethod=0);

    //call this to actually run the segmentation algorithm.
    //if the user wants to display an image of the lines and planes in 2d, then
    //the user can input a pointer to an image viewer.
    void segment(const PointCloud::ConstPtr &cloud, 
                 std::vector< pcl::ModelCoefficients > & planes, 
                 std::vector< LinePosArray > & linePositions,
                  cv::Mat & planeImage, cv::Mat & intensityImage );

    //set the hough line parameters
    void setHoughLinesBinary( float rho, float theta, int threshold,
                                    int minLineLength, int maxLineGap);
    void setHoughLinesIntensity( float rho, float theta, int threshold,
                                    int minLineLength, int maxLineGap);

    //set the camera intrinsics
    void setCameraIntrinsics( float focus_x, float focus_y,
                              float origin_x, float origin_y );

    //sets parameters for several openCV filters that are used inside
    //the findLines function
    void setFilterParams(int blur, int filterSize,
                         int intensityErosion, int lineDilation);

    void setCannyParams( int binarySize, int binaryLowerThreshold,
                         int binaryUpperThreshold,
                         int intensitySize, int intensityLowerThreshold,
                         int intensityUpperThreshold );

private:

    int maxPlaneNumber;
    int minPlaneSize; 

    //these are the parameters for the filters which are applied to images.
    int blurSize;              //this controls the size of the blurring kernel
                               //used to blur the intensity image;
    int filterSize;            //this is the size of the kernel used in the
                               //morphological closing operation on the binary
                               //image.
    int intensityErosionSize; //this is the size of the dilation on the intensity
                               //image.
    int lineDilationSize;      //This controls the amout by which the mask image
                               //is dilated to remove the edges of the plane
                               //from the canny edge detection consideration.

    //these control the parameters for canny edge detection
    int cannyIntensitySize, cannyBinarySize;
    int cannyIntensityLowThreshold, cannyIntensityHighThreshold;
    int cannyBinaryLowThreshold, cannyBinaryHighThreshold;

    //these variables control the parameters of the HoughLines function.
    float binary_rhoRes, binary_thetaRes, intensity_rhoRes, intensity_thetaRes;
    int binary_threshold, binary_minLineLength, binary_maxLineGap ;
    int intensity_threshold, intensity_minLineLength, intensity_maxLineGap ;

    //these are the intrinsics of the camera, they must be set for the
    //function to work. Not setting these values results in an assertion failure.
    float fx, fy, u0, v0;

    bool haveSetCamera;
 
    pcl::SACSegmentation<Point> seg;

    //this modifies the vector "larger" in place, and resizes it.
    //This function assumes that both structures hold integer
    //values that get larger. 
    inline void filterOutIndices( std::vector< int > & larger,
                           const std::vector<int> & remove,
                           cv::Mat & planeImage,
                           size_t index     );

    //this takes a set of indices (validPoints) and sets the corresponding
    //cells in a matrix to 255. The matrix should start out as a matrix of all
    //zeros.
    //This creates a binary image that can be easily used to threshold and
    //find lines or features.
    inline void cloudToMatBinary(const std::vector< int > & validPoints,
                           cv::Mat &mat                            );

    inline void cloudToMatIntensity(const std::vector< int > & validPoints,
                                    cv::Mat &mat,
                                    const PointCloud::ConstPtr & cloud);


    //This takes an image (preferably a binary image) and performs the canny
    //edge detection algorithm. Then a houghLine algorithm is run to extract lines
    inline void findLines(const pcl::PointIndices::Ptr & inliers,
                          const PointCloud::ConstPtr & cloud,
                          std::vector< pcl::ModelCoefficients > & planes, 
                          LineArray & planarLines,
                          LineArray & intensityLines);
    

    //this takes the equation of a plane (Ax + By + Cz + D = 0) as coeffs,
    //and the positions of the endpoints of lines in a picture.
    //These endpoints are then projected onto the plane to convert the 2d 
    //line positions into 3d lines on the plane.
    //the projection equations have been solved analytically. 
    inline void linesToPositions( const pcl::ModelCoefficients & coeffs,
                                  const LineArray & lines, 
                                  LinePosArray & linePositions               );

    inline void matrixLinesToPositions( const pcl::ModelCoefficients::Ptr & coeffs,
                                       const LineArray & lines, 
                                       LinePosArray & linePositions               );

    inline void orderPoints();

};

#endif 
