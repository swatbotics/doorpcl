#include "edge_detector.h"
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <iostream>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>


//This takes in the name of a configuration file as the input.
EdgeDetector::EdgeDetector( const std::string & configFile )
                      : deviceFocalLength( 530.551),
                        pixel_size( 1.075 ),
                        viewerIsInitialized( false ),
                        doWrite( false ), showImage( false ),
                        u0( -1), v0(-1), config( configFile )
{
    //get the handle parameters
    config.get( "minDistOffPlane", minDistOffPlane );
    config.get( "maxDistOffPlane", maxDistOffPlane );

    //initialize the segmenter class
    segmenter = PlaneSegmenter( configFile );
         
    //the index of the current plane that is being viewed.
    frame_index = 0;

    //the radius of the tag points
    radius = 10;

    //initalize the handle bounding points
    handle0[0] = -1;
    handle0[1] = -1;
    handle1[0] = -1;
    handle1[1] = -1;
    
    //the current tag point that is being grasped.
    //All negative numbers mean that a tag point is not being grasped.
    current_grasp_index = -1;

    //The frame start
    waiting = true;

    //Initialize the separate views for the camera.
    view1 = 0;
    view2 = 0;
    line_viewer = new pcl::visualization::PCLVisualizer( "Line Viewer" ) ;
    line_viewer->initCameraParameters();
    
    handleIndices = pcl::IndicesPtr ( new std::vector< int > );

    image_viewer = new pcl::visualization::ImageViewer( "Image Viewer" );
    plane_viewer = new pcl::visualization::ImageViewer( "Plane Viewer" );

    config.get( "filename", filename );

    //array of colors we will use to draw edge lines
    colors.push_back( cv::Vec3i ( 0  , 255,   0 ));
    colors.push_back( cv::Vec3i (   0,   0, 255 ));
    colors.push_back( cv::Vec3i ( 255, 255,   0 ));
    colors.push_back( cv::Vec3i ( 255,   0, 255 ));
    colors.push_back( cv::Vec3i (   0, 255, 255 ));
    colors.push_back( cv::Vec3i ( 255, 125, 125 ));
    colors.push_back( cv::Vec3i ( 125, 255, 125 ));
    colors.push_back( cv::Vec3i ( 125, 125, 255 ));

}






pcl::PointXYZ EdgeDetector::projectPoint( int u, int v, int p )
{

    //extract the coefficients of the plane
    const float & A = planes[ p ].coeffs.values[0];
    const float & B = planes[ p ].coeffs.values[1];
    const float & C = planes[ p ].coeffs.values[2];
    const float & D = planes[ p ].coeffs.values[3];
    
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

    return pcl::PointXYZ( x, y, z );
}

int EdgeDetector::addDoorPoint ( int u, int v)
{
    pcl::PointXYZ point3D = projectPoint( u, v, frame_index );
    int indexAdded;

    if ( doorPoints.size() < 4 ){
        doorPoints.push_back( point3D );
        drawPoints.push_back( Eigen::Vector2i( u , v ) );
        indexAdded = doorPoints.size() - 1;

    }else {
       //replace the point that it is closest to, or 
       int closestIndex = -1;
       int minDistance = 1000000;

       for ( int i = 0; i < 4; i ++ ){
           const int delta_u = drawPoints[i][0] - u ;
           const int delta_v = drawPoints[i][1] - v ;
           const int dist2 = delta_u * delta_u + delta_v * delta_v;
           if ( dist2 < minDistance ){
               minDistance = dist2;
               closestIndex = i;
           }
       }
       doorPoints[ closestIndex ] = point3D;
       drawPoints[ closestIndex ] = Eigen::Vector2i( u, v );

       indexAdded = closestIndex;
    }
    return indexAdded;
}

//this function uses a left of test to make sure that all of the
//points are in the correct order.
void EdgeDetector::orderPoints()
{
    
    left_of_switch( 0, 1, 3 );
    left_of_switch( 1, 2, 3 );
    left_of_switch( 0, 1, 3 );
}

//this implements a left of test, and switches any 
//points that fail the left-of test.
void EdgeDetector::left_of_switch( const int index1, const int index2, const int index3 )
{
    const Eigen::Vector2i to1 = drawPoints[index1] - drawPoints[ index3 ];
    const Eigen::Vector2i to2 = drawPoints[index2] - drawPoints[ index3 ];
    
    const Eigen::Vector3i to1_3D (to1[0], to1[1], 0 );
    const Eigen::Vector3i to2_3D (to2[0], to2[1], 0 );
    const Eigen::Vector3i test = to1_3D.cross( to2_3D );


    //if the points fail the 'left of' test, then swap them
    if ( test[2] < 0 )
    {
        std::swap( drawPoints[index1], drawPoints[ index2 ] );
        std::swap( doorPoints[index1], doorPoints[ index2 ] );
    }

}



void EdgeDetector::waitAndDisplay ()
{
    if (planes.size() == 0 ){
        return;
    }

    frame_index = 0;
    while (this->waiting)
    {

        const cv::Mat & matrix = 
                planes[ frame_index ].image;

        plane_viewer->showRGBImage( matrix.data, matrix.cols, matrix.rows);
        plane_viewer->spinOnce();
    }
    doorPoints.clear();
    drawPoints.clear();
    removeAllDoorLines();
}

double EdgeDetector::distanceFromPlane( const pcl::PointXYZRGBA & point,
                                        const pcl::ModelCoefficients 
                                        & coeffs ){

    //extract the coefficients of the plane
    const float & A = planes[ frame_index ].coeffs.values[0];
    const float & B = planes[ frame_index ].coeffs.values[1];
    const float & C = planes[ frame_index ].coeffs.values[2];
    const float & D = planes[ frame_index ].coeffs.values[3];
    
    const float numer = A * point.x + B * point.y + C * point.z + D;
    const float denom = sqrt( A*A + B*B + C*C );

    const float distance = numer / denom;

    if ( distance < 0 )
    {
        return -distance;
    }
    return distance;
}


void EdgeDetector::getHandlePoints( )
{
    cout << "getting handle points\n";
    if (handle0[1] > handle1[1] ){ std::swap( handle0[1], handle1[1] ); }
    if (handle0[0] > handle1[0] ){ std::swap( handle0[0], handle1[0] ); }
    
    cout << "Assumed cloud size: " << curr_cloud->width * curr_cloud->height;
    cout << "Cloud Size: " << curr_cloud->points.size() << "\n";
    for ( int i = handle0[0]; i <= handle1[0] ; i ++ ){
        for ( int j = handle0[1] ; j <= handle1[1] ; j ++ ){
            //index through cloud 
            //the below indexing is invalid
            const pcl::PointXYZRGBA & p = curr_cloud->at( i, j );
            const double distance = distanceFromPlane( p,
                                                 planes[frame_index].coeffs );

            //if the distance is far off the plane, then it is on 
            if ( distance > minDistOffPlane && distance < maxDistOffPlane )
            {
                handleIndices->push_back( j * curr_cloud->width + i );
            }
        }
    }
    if (handleIndices->size() > 0 ){
        pcl::PointIndices inliers;
        
        pcl::SACSegmentation<Point> sac_seg;
        sac_seg.setOptimizeCoefficients( true );
        sac_seg.setInputCloud( curr_cloud->makeShared() );
        sac_seg.setIndices( handleIndices );
        sac_seg.setModelType( pcl::SACMODEL_LINE );
        sac_seg.setMethodType( pcl::SAC_RANSAC );
        sac_seg.setRadiusLimits( 0, 0.03 );
        sac_seg.setDistanceThreshold( 0.5 );
        sac_seg.segment( inliers, handleCoeffs );
        //sac_seg.segment( , handleCoeffs );
        
        handlePos[0] = handleCoeffs.values[0];
        handlePos[1] = handleCoeffs.values[1];
        handlePos[2] = handleCoeffs.values[2];
        
        handleAxis[0] = handleCoeffs.values[3];
        handleAxis[1] = handleCoeffs.values[4];
        handleAxis[2] = handleCoeffs.values[5];
    }

    drawHandle();
}


//the doorPos is the position of the center of the door,
//the 
void EdgeDetector::getDoorInfo(double & height, double & width,
                 Eigen::Vector3f & doorPos, Eigen::Vector3f & doorRot )
{
    Eigen::Vector3f p0 (doorPoints[0].x,
                        doorPoints[0].y,
                        doorPoints[0].z);

    Eigen::Vector3f p1 (doorPoints[1].x,
                        doorPoints[1].y,
                        doorPoints[1].z);

    Eigen::Vector3f p2 (doorPoints[2].x,
                        doorPoints[2].y,
                        doorPoints[2].z);

    Eigen::Vector3f p3 (doorPoints[3].x,
                        doorPoints[3].y,
                        doorPoints[3].z);
  
  
    //since most doors are taller than they are wide, we set the height
    //of the door equal to the larger dimension and the width equal
    //to the smaller one
    Eigen::Vector3f up = ((p0 - p1) + (p3 - p2)) / 2;
    Eigen::Vector3f across = ((p1 - p2) + (p0 - p3)) / 2;
    Eigen::Vector3f through = up.cross( across );

    cout << "up: " << up << "\tacross: " << across << endl;

    height = up.norm();
    width = across.norm();

    if ( width > height )
    {
        std::swap(width, height);
        std::swap(up, across);
    }

    cout << "width: " << width << "\theight: " << height << endl;

    doorPos = ((p0 + p1 + p2 + p3) / 4);
    
    
    //TODO : This may not be correct, the indices could be wrong.
    //Find the correct indices, or find the opencv functions
    //this is a roll, pitch, yaw vector of angles.
    cout << "The doorRot values are liable to be wrong, this needs"
        << " to be checked\n";

    doorRot[ 0 ] = atan2( through[2], through[1] ) + M_PI / 2;
    doorRot[ 1 ] = atan2(  across[2],  across[0] );
    doorRot[ 2 ] = atan2(      up[1],      up[0] ) - M_PI / 2;

    cout << "Roll: " << doorRot[0] << "\tPitch: " << doorRot[1] << "\tYaw: "
         << doorRot[2] << endl;


}

void EdgeDetector::getHandleInfo( double & length, double & height,
                    Eigen::Vector3f & center ){

    Eigen::Vector3f lowerBounds, upperBounds;

    for ( int i = 0; i < handleIndices->size(); i ++ ){

        //get the point on the doorknob
        const int index = handleIndices->at( i );
        const Point & currentPoint = curr_cloud->points[ index ];

        if ( currentPoint.x > upperBounds[0] ){ upperBounds[0] = currentPoint.x; }
        if ( currentPoint.y > upperBounds[1] ){ upperBounds[1] = currentPoint.y; }
        if ( currentPoint.z > upperBounds[2] ){ upperBounds[2] = currentPoint.z; }
        
        if ( currentPoint.x < lowerBounds[0] ){ lowerBounds[0] = currentPoint.x; }
        if ( currentPoint.y < lowerBounds[1] ){ lowerBounds[1] = currentPoint.y; }
        if ( currentPoint.z < lowerBounds[2] ){ lowerBounds[2] = currentPoint.z; }

    }

    center = (upperBounds + lowerBounds) / 2;

    const int heightIndex = 1;
    height = upperBounds[ heightIndex ] - lowerBounds[ heightIndex ];

    const int lengthIndex = 0;
    length = upperBounds[ lengthIndex ] - lowerBounds[ lengthIndex ];

}

void EdgeDetector::drawLines ()
{

    removeAllDoorLines();
    
    for ( int i = 0; i < drawPoints.size() ; i ++ ){
    
        const pcl::visualization::Vector3ub red_color(255,0,0);
        const double opacity = 1.0;
        const std::string shape_id = "points";
    
        plane_viewer->markPoint( drawPoints[i][0], drawPoints[i][1],
            red_color, red_color, radius, shape_id, opacity);
    }

    if ( doorPoints.size() < 2 ){
        return;
    }

    assert(drawPoints.size() == doorPoints.size());

    for ( int i = 0; i < drawPoints.size() ; i ++ ){
        if ( i == drawPoints.size() - 1 &&
            drawPoints.size() != 4        ){
            return;
        }

        pcl::PointXYZ start3D, end3D;
        Eigen::Vector2i start2D, end2D;
        start2D = drawPoints[ i ];
        start3D = doorPoints[ i ];
        
        if ( i + 1 >= drawPoints.size() ){
            end2D   = drawPoints[ 0 ];
            end3D = doorPoints[ 0 ];
        } else{
            end2D   = drawPoints[ i+1 ];
            end3D = doorPoints[ i + 1 ];
        }
        

        plane_viewer->addLine(start2D[0], start2D[1], end2D[0], end2D[1],
                        1.0, 0, 0, "points");
        line_viewer->addLine(start3D, end3D,
                        255, 0, 0,
                        "doorLine" +  boost::to_string( i ),
                         view1 );

    }
}

void EdgeDetector::removeAllDoorLines(){
    plane_viewer->removeLayer( "points" );
    for ( int i = 0; i < doorPoints.size() ; i ++ ){
        std::string id = "doorLine" + boost::to_string( i );
        line_viewer->removeShape( id ); 
    }
}
//create a viewer that holds lines and a point cloud.
void EdgeDetector::updateViewer( const PointCloud::ConstPtr &cloud,
                   const std::vector< LinePosArray > & planarLines )
{
    //remove the shapes so that they can be updated.
    line_viewer->removeAllShapes( view1);
    //update the point cloud
    line_viewer->updatePointCloud( cloud, "cloud");

    //print out the number of planes.
    cout << "Number of Planes: " << planes.size() << endl;

    //Display all of the edge lines in the line_viewer.
    for( int i = 0; i < planarLines.size(); i ++ ){
        const LinePosArray lines = planarLines[i];

        //The every two points constitute a lines (two endpoints)
        for ( int j = 0; j < lines.size() ; j += 2 ){
            const pcl::PointXYZ start = lines[ j ];
            const pcl::PointXYZ end   = lines[ j+1 ];
            
            cv::Vec3i color = colors[ (i / 2) % colors.size() ];
            if ( i % 2 == 1 ){
                color *= 0.2;
            }
            line_viewer->addLine(start, end,
                            color[0], color[1], color[2],
                            "line" +  boost::to_string( j*100 +i ),
                             view1 );
        }
    }
    line_viewer->spinOnce (100);
}

void EdgeDetector::drawHandle(){

    cout << "handlePoints Size: " << handleIndices->size() << endl;
    for ( int i = 0; i < 40 && i < handleIndices->size(); i ++ ){
        const int a = rand() % handleIndices->size();
        const int b = rand() % handleIndices->size();

        const Point & p0 = curr_cloud->points[ handleIndices->at( a ) ];
        const Point & p1 = curr_cloud->points[ handleIndices->at( b ) ];
        
        line_viewer->addLine(p0, p1, 0, 0, 255,
                            "handleSecond" +  boost::to_string( i ),
                             view1 );


        cout << "added handle lines" << endl;
    
    }
    line_viewer->addLine( handleCoeffs, "handle", view1 );

    line_viewer->spinOnce (100);
}


//initialize point cloud viewer
void EdgeDetector::initViewer( const PointCloud::ConstPtr & cloud )
{

    // set the intrinsics for the camera. This is necessary for 
    // projecting the points into real space.
    u0 = cloud->width / 2;
    v0 = cloud->height / 2;
    
    fx = deviceFocalLength;
    fy = deviceFocalLength;
    segmenter.setCameraIntrinsics( fx, fy, u0, v0 );

    //set up the color handler for the point cloud viewer. this will
    //enable showing color
    ColorHandler rgb( cloud );  
     
    //create the dual view ports for the viewer.
    line_viewer->createViewPort( 0.0 , 0.0, 0.5, 1.0, view1 );
    line_viewer->createViewPort( 0.5 , 0.0, 1.0, 1.0, view2 );
     
    //Set up the cloud viewer;
    //  set the bg colors, camera position, coordinate system, and the 
    line_viewer->addPointCloud<Point> ( cloud, rgb,  "cloud", view2);
    line_viewer->setPointCloudRenderingProperties 
       (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        1, "cloud", view2);
    line_viewer->setBackgroundColor( 0, 0, 0, view2 );
    line_viewer->setBackgroundColor( 0.1, 0.1, 0.1 , view1 );
    line_viewer->addCoordinateSystem (0.1);
    line_viewer->setCameraPosition(0,0,-3.5, 0,-1, 0);

    //set the size of the image viewers. 
    image_viewer->setPosition( 700, 10 );
    image_viewer->setSize( 640, 480 );
    plane_viewer->setPosition( 60, 10 );
    plane_viewer->setSize( 640, 480 );

    //set the callbacks for the gui.
    plane_viewer->registerMouseCallback(mouseClick, (void*)this);
    plane_viewer->registerKeyboardCallback( keyboardEventOccurred,
                                            (void*)this );
    viewerIsInitialized = true;

}


void EdgeDetector::run()
{

#ifndef __APPLE__ 
    pcl::OpenNIGrabber* interface = new pcl::OpenNIGrabber();
    boost::function<
        void (const PointCloud::ConstPtr&)> f =
        boost::bind (&EdgeDetector::cloud_cb_, this, _1);
    interface->registerCallback (f);

    fx = interface->getDevice()->getDepthFocalLength() / pixel_size;
    fy = fx;


    interface->start ();

    while (!line_viewer->wasStopped())
    {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
    }

    interface->stop();

#endif
}

//point cloud callback function gets new pointcloud and runs segmentation
//algorithm
void EdgeDetector::cloud_cb_ (const PointCloud::ConstPtr &cloud)
{
    curr_cloud = cloud;
    if ( !viewerIsInitialized ){
        initViewer( cloud );
    }

    if ( !line_viewer->wasStopped() ){
        planes.clear();
        std::vector< LinePosArray > planarLines;
        if ( doWrite ){
            savePointCloud( *cloud );
        } else {
            
            segmenter.segment( cloud, planes, planarLines, image_viewer );
        }
        updateViewer( cloud, planarLines );
    }
    waitAndDisplay();
    cout << "ended Call back\n";
}

//this program will run until the reader throws an error about 
//a non-existant file.
void EdgeDetector::runWithInputFile()
{
    while ( true ){
        if ( !line_viewer->wasStopped() ){
            planes.clear();
            std::vector< LinePosArray > planarLines;
            PointCloud::Ptr cloud (new PointCloud );
            readPointCloud( cloud );
            curr_cloud = cloud;

            if ( !viewerIsInitialized ){
                initViewer( cloud );
            }

            segmenter.segment( cloud, planes, planarLines, image_viewer );
            updateViewer( cloud, planarLines );
        }  
        waitAndDisplay();        
    }

}

//aves pcd files from grabbed pointclouds
void EdgeDetector::savePointCloud(const PointCloud & cloud)
{
    static int index = 0;

    pcl::io::savePCDFileBinary(filename + 
                               boost::to_string( index ) + ".pcd"
                               , cloud);
    index ++;
    cout << "Saving point cloud number: " << index << "\n";
}

//reads existing pcd files
void EdgeDetector::readPointCloud(PointCloud::Ptr & cloud)
{
    static int index = 0;
    try {
        if (pcl::io::loadPCDFile<Point> (filename + 
                                         boost::to_string( index )
                                         +".pcd", *cloud) == -1)
        {
            cerr << "Couldn't read file "+filename+
                     boost::to_string( index ) +".pcd" << endl;
            exit(-1);
        }
    }
    catch (std::exception &e){
        cout << "Error reading pcd file" << e.what() << endl;
    }
    index ++;
}



inline void EdgeDetector::convertColor( PointCloud::Ptr & cloud,
                   cv::Mat & mat,
                   pcl::PointIndices::Ptr inliers)
{
    cv::Mat newMat = mat.reshape( 1, mat.cols * mat.rows );
    
    for (int i = 0; i < inliers->indices.size(); i++)
    {
        uint8_t colorVal = newMat.at<uint8_t>( 
                           inliers->indices[i] , 0 );

        uint8_t r(255), g( 255 - colorVal ), b( 255 - colorVal );
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
             static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
    }
}



//mouse click callback- currently prints out location of mouse click
//on viewer
//TODO: return plane that mouse click location is a member of
void mouseClick(const pcl::visualization::MouseEvent &event,
                    void* detector)
{  

    EdgeDetector * detect = ( EdgeDetector *) detector;
    int & index = detect->current_grasp_index;
    
    if (event.getButton () == 
        pcl::visualization::MouseEvent::LeftButton)
    {    
        cout << "Left Button\n";
        if (event.getType() == pcl::visualization::MouseEvent::MouseButtonPress )
        {
            if (index >= 0 ) {
                index = -1;
                cout << "Index reset\n";
            } else {

                const int radius2 = detect->radius * detect->radius;
                for (int i = 0; i < detect->doorPoints.size(); i++ )
                {
                   const int delta_u = detect->drawPoints[i][0] - event.getX() ;
                   const int delta_v = detect->drawPoints[i][1] - event.getY() ;

                   const int dist2 = delta_u * delta_u + delta_v * delta_v;
                   if ( dist2 < radius2 ){
                       cout << "Grabbing Point\n";
                       index = i;
                       break;
                   }
                }
                if (index < 0 ){
                    detect->addDoorPoint( event.getX() , event.getY() );
                }
            }
        }         
        if (detect->doorPoints.size() == 4)
        {
            //if the points are out of order, switch their order
            detect->orderPoints();

            //TODO : this is just test code, eventually,
            //we will remove this.
            double height, width;
            Eigen::Vector3f doorPos, doorRot;
            detect->getDoorInfo(height, width, doorPos, doorRot);
        }

        detect->drawLines();
    }

    else if (event.getButton () == 
        pcl::visualization::MouseEvent::RightButton)
    {
        /*  Here we want to manually set a door handle by right clicking.
         *  The door handle will not be within the door plane, so we want to
         *  change our segmentation algorithm and have our door handle belong
         *  to a different plane.
         *  Method: pick two points to form a bounding rectangle around the
         *  handle. For each point in the entire point cloud that exists
         *  within this square, check to see whether it lies within the door
         *  plane. If not, add it to a Handle object which contains points
         *  that are a part of the handle.
         */
        
        cout << "Right Button" << endl;

        if (detect->handle0[0] == -1 || detect->handle0[1] == -1)
        {
            cout << "getting first handle point\n";
            detect->handle0[0] = event.getX();
            detect->handle0[1] = detect->v0 * 2 - event.getY();
        }

        else if (detect->handle1[0] == -1 || detect->handle1[1] == -1)
        {
            cout << "getting second handle point\n";
            detect->handle1[0] = event.getX();
            detect->handle1[1] = detect->v0 * 2 - event.getY();
            detect->getHandlePoints( );
        }
    }

    if( index >= 0 ){
        detect->drawPoints[ index ][0] = event.getX();
        detect->drawPoints[ index ][1] = event.getY();
        detect->doorPoints[ index ] = 
                detect->projectPoint( event.getX(), event.getY(), 
                                      detect->frame_index );
        detect->drawLines();                
    }


} 


//keyboard event handler callback
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent
                            &event, void* detector )
{
    EdgeDetector * detect = ( EdgeDetector *) detector;

    //show previous plane
    if (event.getKeySym () == "a" && event.keyDown ())
    {
        //do things
        detect->frame_index--;
        if (detect->frame_index < 0)
        {
            detect->frame_index = detect->planes.size() - 1;
        }
        cout << "displaying previous frame" << endl;
        
    }

    //show next plane
    else if (event.getKeySym () == "d" && event.keyDown ())
    {
        //do different things
        detect->frame_index++;
        if (detect->frame_index >= detect->planes.size())
        {
            detect->frame_index = 0;
        }
        cout << "displaying next frame" << endl;
    }

    //pause and unpause
    else if (event.getKeySym () == "p" && event.keyDown ())
    {
        if (detect->waiting)
        {
            detect->waiting = false;
            cout << "resuming" << endl;
        }
        else
        {
            detect->waiting = true;
            cout << "pausing" << endl;
        }
    }
}



