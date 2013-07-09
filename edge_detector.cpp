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

unsigned int text_id = 0;


//mouse click callback- currently prints out location of mouse click
//on viewer
//TODO: return plane that mouse click location is a member of
void mouseClick(const pcl::visualization::MouseEvent &event,
                    void* segmenter)
{  
    PlaneSegmenter * seg = ( PlaneSegmenter *) segmenter;
     
    if (event.getButton () == 
        pcl::visualization::MouseEvent::RightButton)
    {    

        seg->addDoorPoint( event.getX() , event.getY() );
        //cout << "Door Point: " << seg->doorPoints.back() << "\n";
        if (seg->doorPoints.size() == 4)
        {
        }

    }
} 


//keyboard event handler callback
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent
                            &event, void* segmenter )
{
    PlaneSegmenter * seg = ( PlaneSegmenter *) segmenter;
     
    //show previous plane
    if (event.getKeySym () == "a" && event.keyDown ())
    {
        //do things
        seg->frame_index--;
        if (seg->frame_index < 0)
        {
            seg->frame_index = seg->planes.size() - 1;
        }
        cout << "displaying previous frame" << endl;
        
    }

    //show next plane
    else if (event.getKeySym () == "d" && event.keyDown ())
    {
        //do different things
        seg->frame_index++;
        if (seg->frame_index >= seg->planes.size())
        {
            seg->frame_index = 0;
        }
        cout << "displaying next frame" << endl;
    }

    //pause and unpause
    else if (event.getKeySym () == "p" && event.keyDown ())
    {
        if (seg->waiting)
        {
            seg->waiting = false;
            cout << "resuming" << endl;
        }
        else
        {
            seg->waiting = true;
            cout << "pausing" << endl;
        }
    }
}


class SimpleOpenNIViewer
{
  public:
    typedef pcl::PointXYZRGBA Point;  
    typedef pcl::PointCloud<Point> PointCloud;
    typedef std::vector< cv::Vec4i > LineArray;
    typedef pcl::visualization::PointCloudColorHandlerRGBField<Point> 
                ColorHandler;
    typedef std::vector< pcl::PointXYZ > LinePosArray;

    //pcl::visualization::CloudViewer viewer;
    //SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

    //this holds a set of colors for visualization
    std::vector< cv::Vec3i > colors;

    int view1, view2;
    pcl::visualization::PCLVisualizer * line_viewer;
    //pcl::visualization::CloudViewer * cloud_viewer;
    pcl::visualization::ImageViewer * image_viewer;
    pcl::visualization::ImageViewer * plane_viewer;

    std::string filename;

    const float deviceFocalLength;
    const float pixel_size;
    bool viewerIsInitialized, doWrite;
    bool showImage;

    float fx, fy, u0, v0;

    PlaneSegmenter segmenter;
   
    int maxNumPlanes, minSize;
    bool optimize;
    float planeThreshold;

    SimpleConfig config;

  
    SimpleOpenNIViewer ( const std::string & configFile )
                          : deviceFocalLength( 530.551),
                            pixel_size( 1.075 ),
                            viewerIsInitialized( false ),
                            doWrite( false ), showImage( false ),
                            u0( -1), v0(-1), config( configFile )
                            {
 
        //initialize the segmenter class
        segmenter = PlaneSegmenter( configFile );
       
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
                       const std::vector< LinePosArray > & planes )
    {

        line_viewer->removeAllShapes( view1);
        line_viewer->updatePointCloud( cloud, "cloud");

        cout << "Number of Planes: " << planes.size() << endl;

        for( int i = 0; i < planes.size(); i ++ ){
            const LinePosArray lines = planes[i];

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

        plane_viewer->registerMouseCallback(mouseClick, (void*)&segmenter);
        plane_viewer->registerKeyboardCallback( keyboardEventOccurred,
                                                (void*)&segmenter );
        viewerIsInitialized = true;

    }
    
    //point cloud callback function gets new pointcloud and runs segmentation
    //algorithm
    void cloud_cb_ (const PointCloud::ConstPtr &cloud)
    {

        if ( !viewerIsInitialized ){
            initViewer( cloud );
        }

        if ( !line_viewer->wasStopped() ){

            std::vector< LinePosArray > planes;
            if ( doWrite ){
                savePointCloud( *cloud );
            } else {
                segmenter.segment( cloud, planes, image_viewer );
            }
            updateViewer( cloud, planes );
        }
        waitAndDisplay();
        cout << "ended Call back\n";
    }
   
    //this program will run until the reader throws an error about 
    //a non-existant file.
    void runWithInputFile(){
        while ( true ){
            if ( !line_viewer->wasStopped() ){

                std::vector< LinePosArray > planes;
                PointCloud::Ptr cloud (new PointCloud );
                readPointCloud( cloud );

                if ( !viewerIsInitialized ){
                    initViewer( cloud );
                }


                segmenter.segment( cloud, planes, image_viewer );
                updateViewer( cloud, planes );
            }  
            waitAndDisplay();        
        }

    }

    void waitAndDisplay ()
    {
        if (segmenter.planes.size() == 0 ){
            return;
        }

        segmenter.frame_index = 0;
        while (segmenter.waiting)
        {

            const cv::Mat & matrix = 
                    segmenter.planes[ segmenter.frame_index ].image;

            plane_viewer->showRGBImage( matrix.data, matrix.cols, matrix.rows);
            plane_viewer->spinOnce();
            drawLines();
        }
    }


    void drawLines ()
    {
        removeAllDoorLines();

        cout << "entering draw Lines with " << segmenter.doorPoints.size()
             << " points." << endl;
        if ( segmenter.doorPoints.size() < 2 ){
            cout << "\texiting due to lack of points" << endl;
            return;
        }


        assert(segmenter.drawPoints.size() == segmenter.doorPoints.size());

        for ( int i = 0; i < segmenter.drawPoints.size() ; i ++ ){
            if ( i == segmenter.drawPoints.size() - 1 &&
                 segmenter.drawPoints.size() != 4        ){
                return;
            }

            pcl::PointXYZ start3D, end3D;
            Eigen::Vector2i start2D, end2D;
            start2D = segmenter.drawPoints[ i ];
            start3D = segmenter.doorPoints[ i ];
            
            if ( i + 1 >= segmenter.drawPoints.size() ){
                end2D   = segmenter.drawPoints[ 0 ];
                end3D = segmenter.doorPoints[ 0 ];
            } else{
                end2D   = segmenter.drawPoints[ i+1 ];
                end3D = segmenter.doorPoints[ i + 1 ];
            }
            

            plane_viewer->addLine(start2D[0], start2D[1], end2D[0], end2D[1],
                            1.0, 0, 0,
                            "doorLine" +  boost::to_string( i ) );
            line_viewer->addLine(start3D, end3D,
                            255, 0, 0,
                            "doorLine" +  boost::to_string( i ),
                             view1 );

        }
    }

    void removeAllDoorLines(){
        for ( int i = 0; i < segmenter.doorPoints.size() ; i ++ ){
            std::string id = "doorLine" + boost::to_string( i );
            plane_viewer->removeLayer( id );
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
