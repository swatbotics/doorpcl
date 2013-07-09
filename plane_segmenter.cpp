#include "plane_segmenter.h"


PlaneSegmenter::PlaneSegmenter( const std::string & configFileName ){
    SimpleConfig config( configFileName );

    double planeThreshold;
    int sacMethod;
    bool optimize;
   
    frame_index = 0;

    waiting = true;

    //get plane segmentation parameters
    config.get("maxPlaneNumber", maxPlaneNumber);
    config.get("minPlaneSize", minPlaneSize);
    optimize = config.getBool("optimize");
    config.get("planeThreshold", planeThreshold);
    config.get("sacMethod" , sacMethod );

    // Optional
    seg.setOptimizeCoefficients (optimize );
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType ( sacMethod );
    seg.setDistanceThreshold ( planeThreshold );

    //get Hough parameters from config file 
    config.get("binary_rhoRes", binary_rhoRes);
    config.get("binary_thetaRes", binary_thetaRes);
    config.get("binary_threshold", binary_threshold);
    config.get("binary_minLineLength", binary_minLineLength);
    config.get("binary_maxLineGap", binary_maxLineGap);

    //get Hough parameters from config file 
    config.get("intensity_rhoRes", intensity_rhoRes);
    config.get("intensity_thetaRes", intensity_thetaRes);
    config.get("intensity_threshold", intensity_threshold);
    config.get("intensity_minLineLength", intensity_minLineLength);
    config.get("intensity_maxLineGap", intensity_maxLineGap);

    //get the canny stuff.
    config.get( "cannyIntensitySize", cannyIntensitySize);
    config.get( "cannyBinarySize", cannyBinarySize );
    config.get( "cannyIntensityLowThreshold", cannyIntensityLowThreshold);
    config.get( "cannyIntensityHighThreshold", cannyIntensityHighThreshold);
    config.get( "cannyBinaryLowThreshold", cannyBinaryLowThreshold);
    config.get( "cannyBinaryHighThreshold", cannyBinaryHighThreshold);

    //get the filter parameters.
    config.get( "blurSize", blurSize);
    config.get( "filterSize", filterSize);
    config.get( "intensityErosionSize", intensityErosionSize);
    config.get( "lineDilationSize", lineDilationSize );



    haveSetCamera = false;
}
    


//PlaneSegmenter constructor
PlaneSegmenter::PlaneSegmenter( int maxNumPlanes, int minSize,
                                bool optimize, float threshold,
                                int sacMethod ) : 
        maxPlaneNumber( maxNumPlanes ), minPlaneSize( minSize) 
{
    // Optional
    seg.setOptimizeCoefficients (optimize );
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType ( sacMethod );
    seg.setDistanceThreshold ( threshold );

    haveSetCamera = false;
}

//Sets focal length and initial points for vision algorithm
void PlaneSegmenter::setCameraIntrinsics( float focus_x, float focus_y,
                                          float origin_x, float origin_y ){
    fx = focus_x;
    fy = focus_y;
    u0 = origin_x;
    v0 = origin_y;

    haveSetCamera = true;
}

//Sets parameters for rgb HoughLines algorithm
void PlaneSegmenter::setHoughLinesIntensity( float rho, float theta, int threshold,
                                    int minLineLength, int maxLineGap){

    intensity_rhoRes = rho;
    intensity_thetaRes = theta;
    intensity_threshold = threshold;
    intensity_minLineLength = minLineLength;
    intensity_maxLineGap = maxLineGap;
}

//Sets parameters for binary HoughLines algorithm
void PlaneSegmenter::setHoughLinesBinary( float rho, float theta, int threshold,
                                    int minLineLength, int maxLineGap){

    binary_rhoRes = rho;
    binary_thetaRes = theta;
    binary_threshold = threshold;
    binary_minLineLength = minLineLength;
    binary_maxLineGap = maxLineGap;
}

//Sets parameters for binary Canny algorithm
void PlaneSegmenter::setCannyParams( int binarySize, int binaryLowerThreshold,
                     int binaryUpperThreshold,
                     int intensitySize, int intensityLowerThreshold,
                     int intensityUpperThreshold ){

    cannyIntensitySize = intensitySize;
    cannyBinarySize = binarySize;
    cannyIntensityLowThreshold = intensityLowerThreshold;
    cannyIntensityHighThreshold = intensityUpperThreshold;
    cannyBinaryLowThreshold = binaryLowerThreshold;
    cannyBinaryHighThreshold = binaryUpperThreshold;
}

//Sets parameters for noise filter
void PlaneSegmenter::setFilterParams ( int blur, int filterSize,
                                       int intensityErosion, int lineDilation )
{
    this->blurSize = blur;
    this->filterSize = filterSize;
    this->intensityErosionSize = intensityErosion;
    this->lineDilationSize = lineDilation;
}

void PlaneSegmenter::addDoorPoint ( int u, int v)
{
    //extract the coefficients of the plane
    const float A = planes[ frame_index ].coeffs.values[0];
    const float B = planes[ frame_index ].coeffs.values[1];
    const float C = planes[ frame_index ].coeffs.values[2];
    const float D = planes[ frame_index ].coeffs.values[3];

    //this corrects for the inversion of the axes in
    //the pcl image viewer point indexing.
    //const int _u = u0 * 2 - u;
    const int _v = v0 * 2 - v;

    const float delta_u = u0 - u;
    const float delta_v = v0 - _v;

    //These are the analytical solutions for x y and z.
    //They were solved from the following three equations
    //      Ax + By + Cz + D = 0
    //      ( fx * x ) + ( z * delta_u ) = 0 
    //      ( fy * y ) + ( z * delta_v ) = 0 

    const float z = D / ( A*delta_u/fx + B*delta_v/fy - C );
    const float x = - delta_u * z / fx;
    const float y = - delta_v * z / fy;
    
    if ( doorPoints.size() < 4 ){
        doorPoints.push_back( pcl::PointXYZ( x, y, z ) );
        drawPoints.push_back( Eigen::Vector2i( u , v ) );
    }else {
       //replace the point that it is closest to, or 
       int closestIndex = -1;
       double minDistance = 1000000;

       for ( int i = 0; i < 4; i ++ ){
           int delta_u = drawPoints[i][0] - u ;
           int delta_v = drawPoints[i][1] - v ;

}

//Planar segmentation function
void PlaneSegmenter::segment(const PointCloud::ConstPtr & cloud, 
                             std::vector< LinePosArray > & linePositions,
                             pcl::visualization::ImageViewer * viewer ) 
{   
    
    //if the camera parameters have not been set, the program will not work, so abort
    assert( haveSetCamera );
    
    frame_index = 0;
    planes.clear();
    doorPoints.clear();
    drawPoints.clear();
   
    //initialize the model coefficients for the plane and 
    //send the cloud to the segmenter for segmentation
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


    //This do while loop is the main segmentation loop.
    //The loop quits once the max number of planes has been reached, or
    //until the segmenter returns a plane that is smaller than the 
    //minPlaneSize.
    do{

        //this performs segmentation on only the indices that are 
        //in outliers. 
        seg.setIndices( outliers );

        //Perform segmentation of the plane. store the coefficients of the plane 
        //, and the inliers on the plane.
        //THe coefficients are in Ax + By + Cz + D = 0 form. 
        seg.segment (*inliers, *coefficients);


        //If the size of the found plane is too small, exit the segmenter.
        if ( inliers->indices.size () <= minPlaneSize ) { 
            return;
        }


        planes.resize( planes.size() + 1 );
        planes.back().coeffs = *coefficients;

        //Find the lines in the plane and store them in the planarLines and
        //intensityLines vectors.
        LineArray planarLines;
        LineArray intensityLines;
        findLines( inliers, cloud, planarLines, intensityLines, viewer );
 
        //transforms the lines in the plane into lines in space.
        linePositions.resize( linePositions.size() + 1 );
        linesToPositions(coefficients, planarLines, linePositions.back() );
        linePositions.resize( linePositions.size() + 1 );        
        linesToPositions(coefficients, intensityLines, linePositions.back() );

        //remove the indices in from outliers that are in inliers.
        //This allows plane segmentation to be repeated on all of the points
        //that are not in planes that have already been found.
        filterOutIndices( *outliers, inliers->indices );

    }
    //if the number of planes found is greater than or equal to the max number of 
    //planes, then quit
    while( linePositions.size() < maxPlaneNumber );

}

//Removes segmented planes from the point cloud
//This algorithm has runs in linear time in the amount of outliers.
//This algorithm modifies the 'larger' vector in place
//Several assumptions must hold for this algorithm to work: 
//      1. As the index increases, the value of the ints stored must increase
//          as well.
//      2. The vector 'remove' must be a subset of 'larger' 
//      3. 
inline void PlaneSegmenter::filterOutIndices( std::vector< int > & larger,
                       const std::vector<int> & remove){
    int j = 0; // the index into the 'remove' vector
    int k = 0; // the index into the 'larger' vector
               // This index holds the next place in 'larger' that an index will
               // be stored, facilitating the in place modification of 'larger'

    for( int i = 0; i < larger.size() ; i ++ ){

        if ( j < remove.size() && larger[i] == remove[j] ){
            j++;
        }
        else{
            larger[k] = larger[i];
            k ++;
        }
    }

    larger.resize( k );
}

//Transforms point cloud to a binary matrix
inline void PlaneSegmenter::cloudToMatBinary(const std::vector< int > & validPoints,
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

//Transforms point cloud to an intensity matrix
inline void PlaneSegmenter::cloudToMatIntensity(const std::vector< int > & 
                                                validPoints,
                                                cv::Mat &mat,
                                                const PointCloud::ConstPtr & cloud)
{
    //set the values of mat that correspond to being on the major
    // plane of interest.
    //A non-zero value is on the plane and the value corresponds to the average of
    //its rgb values, aka the intensity
    //a value of 0 is off the plane.
    for (int i=0; i < validPoints.size(); i++){
        const int index = validPoints[i];
        const Point p = cloud->points[ index ];
        const Eigen::Vector3i rgb( p.getRGBVector3i() );
        const uint8_t intensity = ( rgb[0] + rgb[1] + rgb[2] ) / 3; 
        mat.at<uint8_t>( index, 1 ) = intensity;
    }
}

//Find depth and color lines from segmented plane
inline void PlaneSegmenter::findLines( const pcl::PointIndices::Ptr & inliers,
                                       const PointCloud::ConstPtr & cloud,
                                      LineArray & planarLines,
                                      LineArray & intensityLines,
                                      pcl::visualization::ImageViewer * viewer )
{
     
    cv::Mat binary, intensity, mask, copyBinary, copyIntensity, maskedIntensity;
   

    //initialize the matrices
    //create a binary picture from the points in inliers.
    copyBinary    = cv::Mat::zeros(cloud->height * cloud->width, 1 , CV_8UC1 );
    copyIntensity = cv::Mat::zeros(cloud->height * cloud->width, 1 , CV_8UC1 );

    cloudToMatBinary   (inliers->indices, copyBinary );
    cloudToMatIntensity(inliers->indices, copyIntensity, cloud );

    //reshape the matrix into the shape of the image and run the
    //canny edge detector on the resulting image.
    copyBinary.rows = cloud->height;
    copyBinary.cols = cloud->width;

    copyIntensity.rows = cloud->height;
    copyIntensity.cols = cloud->width;

    copyBinary.copyTo( mask );
    copyBinary.copyTo(binary);
    copyIntensity.copyTo( intensity );
   


    bool getIntensity = true;
    ///////////////////////////////////////////////////////////////////////////
    //Perform Canny Edge Detection
    if ( getIntensity ){
        //the blur will smooth out the intensity edges.
        cv::blur( intensity, intensity, cv::Size(blurSize , blurSize) );
        cv::Canny(intensity, intensity, cannyIntensityLowThreshold,
                                        cannyIntensityHighThreshold,
                                        cannyIntensitySize );
        
        //remove the noise added by including the edges.
        //This will increase the size of the mask image so that 
        //it can get rid of the edges when copied over.
        cv::Mat intensityKernel = cv::Mat::ones( intensityErosionSize,
                                                 intensityErosionSize,
                                                                 CV_8U ); 

        cv::erode( mask, mask, intensityKernel);
        intensity.copyTo( maskedIntensity, mask );
        
    }

    //TODO : find out why the copy is necessary: For some reason,
    //without the copy, the canny edge detector does not work.
    //binary.copyTo(binary);

    //this filter cleans up the noise from the sensor.        
    //cv::blur( dst, dst, cv::Size(size , size) );
    cv::Mat kernel = cv::Mat::ones( filterSize, filterSize, CV_8U ); 
    cv::dilate( binary, binary, kernel);
    cv::erode( binary, binary, kernel );
    cv::Canny(binary, binary, cannyBinaryLowThreshold,
                              cannyBinaryHighThreshold,
                              cannyBinarySize);


    /////////////////////////////////////////////////////////////////////
    //Perform the hough lines detection algorithm
    cv::Mat kern = cv::Mat::ones( lineDilationSize, lineDilationSize, CV_8U ); 
    cv::dilate( binary, binary, kern);

    //run HoughLines on noise-filtered color and depth matrices
    cv::HoughLinesP(binary, planarLines, binary_rhoRes, binary_thetaRes,
              binary_threshold, binary_minLineLength, binary_maxLineGap);
    cv::HoughLinesP(maskedIntensity, intensityLines, intensity_rhoRes,
                    intensity_thetaRes, intensity_threshold,
                    intensity_minLineLength, intensity_maxLineGap);


    //if there is a viewer, then display a set of lines on the viewer.
    if ( viewer != NULL ){     
        cv::Mat cdst;

        //TODO : make this a configurable option. Right now, this is simply
        //a convenience mechanism.
        bool seeBinary = true;

        if ( seeBinary ){
            cv::cvtColor(binary, cdst, CV_GRAY2BGR);

            for( size_t i = 0; i < planarLines.size(); i++ )
            {
                cv::Vec4i l = planarLines[i];
                cv::line( cdst, cv::Point(l[0], l[1]),
                                cv::Point(l[2], l[3]),
                                cv::Scalar(0,0,255),
                                3, CV_AA);
            }
        }
        else {
            cv::cvtColor(maskedIntensity, cdst, CV_GRAY2BGR);

            for( size_t i = 0; i < intensityLines.size(); i++ )
            {
                cv::Vec4i l = intensityLines[i];
                cv::line( cdst, cv::Point(l[0], l[1]),
                                cv::Point(l[2], l[3]),
                                cv::Scalar(0,0,255),
                                3, CV_AA);
            }
        }
        viewer->showRGBImage( cdst.data, cdst.cols, cdst.rows );
        //planes.back().image = cv::Mat::zeros( cloud->width, cloud->height , CV_8UC1 );
        //copyIntensity.copyTo( planes.back().image, copyBinary ); 
        copyIntensity.copyTo( planes.back().image, copyBinary );
        cv::cvtColor( planes.back().image, planes.back().image, CV_GRAY2BGR );

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

            //These are the analytical solutions for x y and z.
            //They were solved from the following three equations
            //      Ax + By + Cz + D = 0
            //      ( fx * x ) + ( z * delta_u ) = 0 
            //      ( fy * y ) + ( z * delta_v ) = 0 

            const float z = D / ( A*delta_u/fx + B*delta_v/fy - C );
            const float x = - delta_u * z / fx;
            const float y = - delta_v * z / fy;

            linePositions.push_back( pcl::PointXYZ( x, y, z ) );
        }
    }
}

//Transforms 2D lines returned by HoughLines into lines we can draw in viewer
void PlaneSegmenter::matrixLinesToPositions( const pcl::ModelCoefficients::Ptr 
                             & coeffs,
                             const LineArray & lines, 
                             LinePosArray & linePositions
                            ){

    //the b vector;
    const cv::Matx31f b ( -coeffs->values[3] , 0.0, 0.0 );
    cv::Matx33f A( 
           coeffs->values[0], coeffs->values[1], coeffs->values[2],
           fx               , 0.0              , 0.0,
           0.0              , fy               , 0.0         );

    for( int i = 0; i < lines.size(); i ++ ){
        cv::Matx31f position [2];
        
        for ( int j = 0; j < 2; j ++ ){
            int u, v;
            u = lines[i][0 + j*2];
            v = lines[i][1 + j*2];
            A( 1, 2) = u0 - u;
            A( 2, 2) = v0 - v;
            position[j] = A.inv() * b;
        }

        linePositions.push_back( pcl::PointXYZ( position[0](0,0), 
                                   position[0](1,0),
                                   position[0](2,0) ) );
        linePositions.push_back( pcl::PointXYZ( position[1](0,0), 
                                   position[1](1,0),
                                   position[1](2,0) ));

    }
}


