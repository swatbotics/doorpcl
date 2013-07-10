#include <iostream>
#include <string>

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include "plane_segmenter.h"

#include "SimpleConfig.h"

void mouseClick(const pcl::visualization::MouseEvent &event,
                    void* viewer);

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent
                            &event, void* segmenter );


class SimpleOpenNIViewer
{
  public:
    typedef pcl::PointXYZRGBA Point;  
    typedef pcl::PointCloud<Point> PointCloud;
    typedef std::vector< cv::Vec4i > LineArray;
    typedef pcl::visualization::PointCloudColorHandlerRGBField<Point> 
                ColorHandler;
    typedef std::vector< pcl::PointXYZ > LinePosArray;

    SimpleConfig config;

    //this holds a set of colors for visualization
    std::vector< cv::Vec3i > colors;

    int view1, view2;
    pcl::visualization::PCLVisualizer * line_viewer;
    pcl::visualization::ImageViewer * image_viewer;
    pcl::visualization::ImageViewer * plane_viewer;

    std::string filename;

    const float deviceFocalLength;
    const float pixel_size;
    bool viewerIsInitialized, doWrite;
    bool showImage;

    float fx, fy, u0, v0;

    PlaneSegmenter segmenter;
   
    std::vector<plane_data> planes;
    int frame_index;
    
    std::vector< pcl::PointXYZ > doorPoints;
    std::vector< Eigen::Vector2i > drawPoints;

    int maxNumPlanes, minSize;
    bool optimize;
    float planeThreshold;

    bool waiting;

    Eigen::Vector3f doorPosition, doorRotation;


  
    SimpleOpenNIViewer ( const std::string & configFile )
                          : deviceFocalLength( 530.551),
                            pixel_size( 1.075 ),
                            viewerIsInitialized( false ),
                            doWrite( false ), showImage( false ),
                            u0( -1), v0(-1), config( configFile )
                            {
 
        //initialize the segmenter class
        segmenter = PlaneSegmenter( configFile );
             
        frame_index = 0;

        waiting = true;

        view1 = 0;
        view2 = 0;
        line_viewer = new pcl::visualization::PCLVisualizer( "Line Viewer" ) ;
        line_viewer->initCameraParameters();
        
        image_viewer = new pcl::visualization::ImageViewer( "Image Viewer" );
        plane_viewer = new pcl::visualization::ImageViewer( "Plane Viewer" );

        filename = "../drexelFrames/sample";

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


    
    //create a viewer that holds lines and a point cloud.
    void updateViewer( const PointCloud::ConstPtr &cloud,
                       const std::vector< LinePosArray > & planarLines )
    {

        line_viewer->removeAllShapes( view1);
        line_viewer->updatePointCloud( cloud, "cloud");

        cout << "Number of Planes: " << planes.size() << endl;

        for( int i = 0; i < planarLines.size(); i ++ ){
            const LinePosArray lines = planarLines[i];

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

        //cloud_viewer->showCloud( cloud );
        

        line_viewer->spinOnce (100);
        
    }
    

    
    //initialize point cloud viewer
    void initViewer( const PointCloud::ConstPtr & cloud ){
        u0 = cloud->width / 2;
        v0 = cloud->height / 2;
        
        segmenter.setCameraIntrinsics( deviceFocalLength, deviceFocalLength,
                                       u0, v0 );
        fx = deviceFocalLength;
        fy = deviceFocalLength;
        ColorHandler rgb( cloud );  
         
        line_viewer->createViewPort( 0.0 , 0.0, 0.5, 1.0, view1 );
        line_viewer->createViewPort( 0.5 , 0.0, 1.0, 1.0, view2 );
         
        line_viewer->addPointCloud<Point> ( cloud, rgb,  "cloud", view2);
        line_viewer->setPointCloudRenderingProperties 
           (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            1, "cloud", view2);
        line_viewer->setBackgroundColor( 0, 0, 0, view2 );
        line_viewer->setBackgroundColor( 0.1, 0.1, 0.1 , view1 );
        line_viewer->addCoordinateSystem (0.1);
        line_viewer->setCameraPosition(0,0,-3.5, 0,-1, 0);

        image_viewer->setPosition( 700, 10 );
        image_viewer->setSize( 640, 480 );
        
        plane_viewer->setPosition( 60, 10 );
        plane_viewer->setSize( 640, 480 );

        /*
        boost::function<
          void (const PointCloud::ConstPtr&)> keys =
          boost::bind (&SimpleOpenNIViewer::keyboardEventOccurred, this, _1);
        */

        plane_viewer->registerMouseCallback(mouseClick, (void*)this);
        plane_viewer->registerKeyboardCallback( keyboardEventOccurred,
                                                (void*)&segmenter );
        viewerIsInitialized = true;

    }

    void addDoorPoint ( int u, int v)
    {
        //extract the coefficients of the plane
        const float A = planes[ frame_index ].coeffs.values[0];
        const float B = planes[ frame_index ].coeffs.values[1];
        const float C = planes[ frame_index ].coeffs.values[2];
        const float D = planes[ frame_index ].coeffs.values[3];
        
        cout << "A: " << A << "\tB: " << B << "\tFx: " << fx << endl;
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
           doorPoints[ closestIndex ] = pcl::PointXYZ( x, y, z );
           drawPoints[ closestIndex ] = Eigen::Vector2i( u, v );
        }
    }


    //point cloud callback function gets new pointcloud and runs segmentation
    //algorithm
    void cloud_cb_ (const PointCloud::ConstPtr &cloud)
    {

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
    void runWithInputFile(){
        while ( true ){
            if ( !line_viewer->wasStopped() ){
                planes.clear();
                std::vector< LinePosArray > planarLines;
                PointCloud::Ptr cloud (new PointCloud );
                readPointCloud( cloud );

                if ( !viewerIsInitialized ){
                    initViewer( cloud );
                }

                segmenter.segment( cloud, planes, planarLines, image_viewer );
                updateViewer( cloud, planarLines );
            }  
            waitAndDisplay();        
        }

    }

    void waitAndDisplay ()
    {
        if (planes.size() == 0 ){
            return;
        }

        frame_index = 0;
        while (waiting)
        {

            const cv::Mat & matrix = 
                    planes[ frame_index ].image;

            plane_viewer->showRGBImage( matrix.data, matrix.cols, matrix.rows);
            plane_viewer->spinOnce();
        }
        doorPoints.clear();
        drawPoints.clear();
    }

    void getDoorInfo(Eigen::Vector3f doorPos, Eigen::Vector3f doorRot )
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

        cout << "up: " << up << "\tacross: " << across << endl;

        float height = up.norm();
        float width = across.norm();

        if ( width > height )
        {
            std::swap(width, height);
            std::swap(up, across);
        }

        cout << "width: " << width << "\theight: " << height << endl;

        Eigen::Vector3f center = ((p0 + p1 + p2 + p3) / 4);
        
        doorPos[0] = center[0];
        doorPos[1] = center[1];
        doorPos[2] = center[2];
        
        /* 
        doorRot[0];
        doorRot[1];
        doorRot[2];
        */
    }

    void drawLines ()
    {

        removeAllDoorLines();
        
        for ( int i = 0; i < drawPoints.size() ; i ++ ){
        
            const pcl::visualization::Vector3ub red_color(255,0,0);
            const pcl::visualization::Vector3ub blu_color(0,0,255);
            const double radius = 10, opacity = 1.0;
            const std::string shape_id = "points";
        
            plane_viewer->markPoint( drawPoints[i][0], drawPoints[i][1],
                red_color, blu_color, radius, shape_id, opacity);
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

    void removeAllDoorLines(){
        plane_viewer->removeLayer( "points" );
        for ( int i = 0; i < doorPoints.size() ; i ++ ){
            std::string id = "doorLine" + boost::to_string( i );
            line_viewer->removeShape( id ); 
        }
    }

    void run ()
    {

#ifndef __APPLE__ 
      pcl::OpenNIGrabber* interface = new pcl::OpenNIGrabber();
      boost::function<
          void (const PointCloud::ConstPtr&)> f =
          boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
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

    //aves pcd files from grabbed pointclouds
    void savePointCloud(const PointCloud & cloud)
    {
        static int index = 0;

        pcl::io::savePCDFileBinary(filename + 
                                   boost::to_string( index ) + ".pcd"
                                   , cloud);
        index ++;
        cout << "Saving point cloud number: " << index << "\n";
    }

    //reads existing pcd files
    void readPointCloud(PointCloud::Ptr & cloud)
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



    inline void convertColor( PointCloud::Ptr & cloud,
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

};


//mouse click callback- currently prints out location of mouse click
//on viewer
//TODO: return plane that mouse click location is a member of
void mouseClick(const pcl::visualization::MouseEvent &event,
                    void* viewer)
{  

    SimpleOpenNIViewer * view = ( SimpleOpenNIViewer *) viewer;
     
    if (event.getButton () == 
        pcl::visualization::MouseEvent::RightButton)
    {    

        view->addDoorPoint( event.getX() , event.getY() );

        //cout << "Door Point: " << seg->doorPoints.back() << "\n";
        if (view->doorPoints.size() == 4)
        {
            view->getDoorInfo(view->doorPosition, view->doorRotation);
        }
        view->drawLines();
    }
} 


//keyboard event handler callback
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent
                            &event, void* viewer )
{
    SimpleOpenNIViewer * view = ( SimpleOpenNIViewer *) viewer;

    //show previous plane
    if (event.getKeySym () == "a" && event.keyDown ())
    {
        //do things
        view->frame_index--;
        if (view->frame_index < 0)
        {
            view->frame_index = view->planes.size() - 1;
        }
        cout << "displaying previous frame" << endl;
        
    }

    //show next plane
    else if (event.getKeySym () == "d" && event.keyDown ())
    {
        //do different things
        view->frame_index++;
        if (view->frame_index >= view->planes.size())
        {
            view->frame_index = 0;
        }
        cout << "displaying next frame" << endl;
    }

    //pause and unpause
    else if (event.getKeySym () == "p" && event.keyDown ())
    {
        if (view->waiting)
        {
            view->waiting = false;
            cout << "resuming" << endl;
        }
        else
        {
           view->waiting = true;
            cout << "pausing" << endl;
        }
    }
}


void printUsage(){
    cout << "Usage: ./edge_detector <mode: [1, 3]> <filename>\n"
         << "This program can run with 0, 1, or 2 arguements\n"
         << "With no arguments, this program will not write any data and"
            << " will read data from a device\n"
         << "If the first argument is a 1, then the program will function "
            << "like it would with no arguments\n"
         << "If the first argument is 2, then the program will write the "
            << "Point Cloud data to a file\n"
         << "If the first argument is 3, then the program will read "
            << "Point Cloud data from a file\n"
         << "The third argument sets the filename to be read or written to\n";
}


int main (int argc, char * argv[])
{

  SimpleOpenNIViewer v( "../config.txt" );

  //if there are no arguments, print the usage and run the file;
  if ( argc == 1 ){
      v.run();
      printUsage();
  }
  
  //if there are two or three arguments, execute the code normally;
  else if (argc <= 4){
      //if there are two extra arguments, then set the 
      if ( argc >= 3 ){
          v.filename = argv[2];
      }

      if ( argc == 4 ){
        int value = atoi( argv[1] );
        if (value == 1 ){
            v.showImage = false;
        }
      }

      
      int value = atoi( argv[1] );
      if (value == 1 ){
          cout << "Running edge detection" << "\n";
          v.run();
      }
      else if ( value == 2 ){
          cout << "Running edge detection and saving data to the file: "
               << v.filename << "\n";
          v.doWrite = true;
          v.run();
      }
      else if( value == 3 ){
          cout << "Running edge detection with the data from the file: "
               << v.filename << "\n";
          v.runWithInputFile();
      }
      else {
          printUsage();
      }
  }
  else{
      printUsage();
  }
  
  return 0;
}


