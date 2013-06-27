#include "plane_segmenter.h"


PlaneSegmenter::PlaneSegmenter( int maxNumPlanes, int minSize,
                                bool optimize, float threshold ) : 
        maxPlaneNumber( maxNumPlanes ), minPlaneSize( minSize) 
{
    
    // Optional
    seg.setOptimizeCoefficients (optimize );
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold ( threshold );

    HL_rhoRes = 8;
    HL_thetaRes = CV_PI/180;
    HL_threshold = 40;
    HL_minLineLength = 75;
    HL_minLineGap = 4;

    haveSetCamera = false;


}

void PlaneSegmenter::setCameraIntrinsics( float focus_x, float focus_y,
                                          float origin_x, float origin_y ){

    fx = focus_x;
    fy = focus_y;
    u0 = origin_x;
    v0 = origin_y;

    haveSetCamera = true;
}


void PlaneSegmenter::setHoughLines( float rho, float theta, int threshold,
                                    int minLineLength, int minLineGap){

    HL_rhoRes = rho;
    HL_thetaRes = theta;
    HL_threshold = threshold;
    HL_minLineLength = minLineLength;
    HL_minLineGap = minLineGap;
}


void PlaneSegmenter::segment(const PointCloud::ConstPtr & cloud, 
                             std::vector< LinePosArray > & linePositions,
                             pcl::visualization::ImageViewer * viewer ) 
{   

    assert( haveSetCamera );

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    seg.setInputCloud ( cloud->makeShared() );

    //initialize the indices containers, set outliers to be all of the
    //points inside the point cloud. 
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::IndicesPtr outliers ( new std::vector<int> );
    outliers->resize( cloud->height * cloud->width );

    for ( int i = 0; i < outliers->size(); i ++ ){
        (*outliers)[i] = i;
    }


    //We get two planes, or the size of the inliers is reduced to
    //a very low amout;
    do{

        //this performs segmentation on only the indices that are 
        //in outliers
        seg.setIndices( outliers );
        seg.segment (*inliers, *coefficients);
        
        //copy plane 
        if (inliers->indices.size () == 0)
        {
            if ( linePositions.size() == 0 ){
                PCL_ERROR("Could not estimate a plane for the given data.");
            }
            return;
        }
        
        //create a binary picture from the points in inliers.
        cv::Mat majorPlane = cv::Mat::zeros(cloud->height *
                                            cloud->width,
                                            1 , CV_8UC1 );
        cloudToMat(inliers->indices, majorPlane );
        //reshape the matrix into the shape of the image and run the
        //canny edge detector on the resulting image.
        majorPlane.rows = cloud->height;
        majorPlane.cols = cloud->width;

        LineArray lines;
        findLines( majorPlane, lines, viewer );

        //convertColor(cannyLineMat, inliers);  
        
        //transforms the lines in the plane into lines in space.
        linePositions.resize( linePositions.size() + 1 );
        linesToPositions(coefficients, lines, linePositions.back() );

        filterOutIndices( *outliers, inliers->indices );

    }
    while( linePositions.size() < maxPlaneNumber ||
           inliers->indices.size() > minPlaneSize   );
}


inline void PlaneSegmenter::filterOutIndices( std::vector< int > & larger,
                       const std::vector<int> & remove){
    int j = 0;
    int k = 0;
    for( int i = 0; i < larger.size() ; i ++ ){
        if ( j < remove.size() &&
             larger[i] == remove[j] ){
            j++;
        }
        else{
            larger[k] = larger[i];
            k ++;
        }
    }

    larger.resize( k );
}


inline void PlaneSegmenter::cloudToMat(const std::vector< int > & validPoints,
                       cv::Mat &mat)
{
  //set the values of mat that correspond to being on the major
  // plane of interest.
  //These values should be 1, we will end up with a binary matrix:
  //a value of 1 is on the plane,
  //a value of 0 is off the plane.
  for (int i=0; i < validPoints.size(); i++){
        mat.at<uint8_t>( validPoints[i], 1 ) = 255;
  }
  
}

//change this
inline void PlaneSegmenter::findLines(cv::Mat & src, LineArray & lines,
                                      pcl::visualization::ImageViewer * viewer )
{
    
    try{
        //TODO : is the copy necessary
        //cv::blur( src, dst, cv::Size(5,5) );
        src.copyTo(dst);
        cv::Canny(dst, dst, 25, 230, 3);
    }
    catch ( std::exception & e ){
        cout << "Error with canny edge detector" << e.what() << "\n";
        return;
    }
    
    //int size = 4 ;
    //cv::Mat kernel = cv::Mat::ones( size, size, CV_64F ); 
    //cv::dilate( dst, dst, kernel);

    //dst, lines, rho_resolution, theta_resolution, threshold,
    //minLinLength, maxLineGap
    cv::HoughLinesP(dst, lines, HL_rhoRes, HL_thetaRes, HL_threshold,
                    HL_minLineLength, HL_minLineGap);
    
    //draw the lines;
    if ( viewer != NULL ){     
        cv::Mat cdst;
        cv::cvtColor(src, cdst, CV_GRAY2BGR);

        for( size_t i = 0; i < lines.size(); i++ )
        {
            cv::Vec4i l = lines[i];
            cv::line( cdst, cv::Point(l[0], l[1]),
                            cv::Point(l[2], l[3]),
                            cv::Scalar(0,0,255),
                            3, CV_AA);
        }
        viewer->showRGBImage( cdst.data, cdst.cols, cdst.rows );
    }          
}


//this solves for the position of all of the line endpoint in the
//these equations have been solved analytically. 
inline void PlaneSegmenter::linesToPositions( 
                              const pcl::ModelCoefficients::Ptr & coeffs,
                              const LineArray & lines, 
                              LinePosArray & linePositions               ){

    //extract the coefficients of the plane
    const float A = coeffs->values[0];
    const float B = coeffs->values[1];
    const float C = coeffs->values[2];
    const float D = coeffs->values[3];
     
    //Project each point onto the plane
    for( int i = 0; i < lines.size(); i ++ ){
        for ( int j = 0; j < 2; j ++ ){

            const int u = lines[i][0 + j*2];
            const int v = lines[i][1 + j*2];

            const float delta_u = u0 - u;
            const float delta_v = v0 - v;

            const float z = ( A*fy*delta_u + B*fy*delta_v - C ) / D;
            const float x = - delta_u * z / fx;
            const float y = - delta_v * z / fy;

            linePositions.push_back( pcl::PointXYZ( x, y, z ) );
        }
    }
}

