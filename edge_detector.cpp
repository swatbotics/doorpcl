#include <iostream>
#include <string>

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/common/common_headers.h>

#include <boost/thread/thread.hpp>
#include "plane_segmenter.h"

#include "~/code/e91/e91/hubomz/mzcommon/SimpleConfig.h"

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

    std::string filename;

    const float deviceFocalLength;
    const float pixel_size;
    bool viewerIsInitialized, doWrite;
    bool showImage;

    float fx, fy, u0, v0;
    PointCloud::Ptr inlierCloud;

    Display * viewer;
    ImageDisplay * imageViewer;

    PlaneSegmenter segmenter;

    SimpleOpenNIViewer () : deviceFocalLength( 530.551),
                            pixel_size( 1.075 ),
                            viewerIsInitialized( false ),
                            doWrite( false ), showImage( false ),
                            u0( -1), v0(-1)
                            {
        view1 = 0;
        view2 = 0;
        line_viewer = new pcl::visualization::PCLVisualizer( "Line Viewer" ) ;
        line_viewer->initCameraParameters();
        
        //cloud_viewer = new pcl::visualization::CloudViewer( "Cloud Viewer" );
        image_viewer = new pcl::visualization::ImageViewer( "Image Viewer" );

        filename = "pcd_frames/sample";
        segmenter = PlaneSegmenter( 5, 50000, true, 0.06 );
        segmenter.setHoughLines( 1, 3.14286/720, 60, 80, 10 );


    SimpleOpenNIViewer (){
        colors.push_back( cv::Vec3i ( 255,   0,   0 ));
        colors.push_back( cv::Vec3i ( 0  , 255,   0 ));
        colors.push_back( cv::Vec3i (   0,   0, 255 ));
        colors.push_back( cv::Vec3i ( 255, 255,   0 ));
        colors.push_back( cv::Vec3i ( 255,   0, 255 ));
        colors.push_back( cv::Vec3i (   0, 255, 255 ));
        colors.push_back( cv::Vec3i ( 255, 125, 125 ));
        colors.push_back( cv::Vec3i ( 125, 255, 125 ));
        colors.push_back( cv::Vec3i ( 125, 125, 255 ));

        pixel_size = 1.075;
        
        //these are initialized to invalid starting points for the 
        //purposes of error checking
        u0 = -1;
        v0 = -1; 

        PointCloud::Ptr temp(new PointCloud );
        inlierCloud = temp;

        viewer = new Display;
        imageViewer = new ImageDisplay;
        imageViewer->setPosition( 700, 10 );
        imageViewer->setSize( 640, 480 );

        //viewer->setBackgroundColor (0,0,0);
        ColorHandler rgb(inlierCloud);
        //pcl::visualization::PointCloudColorHandlerCustom<Point>
            //inlierColor( inlierCloud, 255, 255, 0 );

        viewer->addPointCloud<Point> (inlierCloud, rgb,  "cloud");
        viewer->setPointCloudRenderingProperties 
           (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        viewer->addCoordinateSystem (0.5);
        viewer->initCameraParameters();
        viewer->setCameraPosition(0,0,-1.3, 0,-1,0);
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
                
                cv::Vec3i color = colors[ i % colors.size() ];
                line_viewer->addLine(start, end,
                                color[0], color[1], color[2],
                                "line" +  boost::to_string( j*100 +i ) , view1 );
            }

        }

        //cloud_viewer->showCloud( cloud );
        

        line_viewer->spinOnce (100);
        
    }
    
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
           (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud", view2);
        line_viewer->setBackgroundColor( 0, 0, 0, view2 );
        line_viewer->setBackgroundColor( 0.1, 0.1, 0.1 , view1 );
        line_viewer->addCoordinateSystem (0.1);
        line_viewer->setCameraPosition(0,0,-3.5, 0,-1, 0);

        image_viewer->setPosition( 700, 10 );
        image_viewer->setSize( 640, 480 );
        
        viewerIsInitialized = true;

    }
    
    void cloud_cb_ (const PointCloud::ConstPtr &cloud)
    {
        if ( !viewerIsInitialized ){
            initViewer( cloud );
        }

        if ( !line_viewer->wasStopped() ){
            if ( doWrite ){
                savePointCloud( *cloud );
            }

            std::vector< LinePosArray > planes;
            segmenter.segment( cloud, planes, image_viewer );
            updateViewer( cloud, planes );

        }
    void cloud_cb_ (const PointCloud::ConstPtr &cloud)
    {
      if ( !viewer->wasStopped() ){
        if ( u0 == -1 || v0 == -1 ){
            u0 = cloud->width / 2;
            v0 = cloud->height / 2;
        }
        std::vector< LinePosArray > linePositions;
        segment( cloud, linePositions, 50000 );
        updateViewer( cloud, linePositions );
        viewer->spinOnce (100);
      }

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



                segmenter.segment( cloud, planes );
                updateViewer( cloud, planes );
            }
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


    void savePointCloud(const PointCloud & cloud)
    {
        static int index = 0;
        pcl::io::savePCDFileBinary(filename + 
                                   boost::to_string( index ) + ".pcd"
                                   , cloud);
        index ++;
    }

    
    void readPointCloud(PointCloud::Ptr & cloud)
    {
        static int index = 0;
        if (pcl::io::loadPCDFile<Point> (filename + 
                                         boost::to_string( index )
                                         +".pcd", *cloud) == -1)
        {
            cerr << "Couldn't read file "+filename+
                     boost::to_string( index ) +".pcd" << endl;
            exit(-1);
        }
        index ++;
    }


    inline void convertColor( PointCloud::Ptr & cloud,
                       cv::Mat & mat,
                       pcl::PointIndices::Ptr inliers)
  private:

    
    void segment(const PointCloud::ConstPtr &cloud, 
                 std::vector< LinePosArray > & linePositions, 
                 int size)
    {   

        pcl::ModelCoefficients::Ptr coefficients
                                             (new pcl::ModelCoefficients);

        // Create the segmentation object
        pcl::SACSegmentation<Point> seg;
        // Optional
        seg.setOptimizeCoefficients (false);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.03);

        seg.setInputCloud ( cloud->makeShared() );

        pcl::ExtractIndices< Point > extractor;
        extractor.setInputCloud( cloud );

        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::PointIndices::Ptr outliers (new pcl::PointIndices);
        outliers = seg.getIndices();
        cout << "Starting Length of outliers is " 
             << outliers->indices.size() << "\n" ;

        while( inlierCloud->width > size){
            pcl::PointIndices::Ptr remaining( new PointCloud );
            filterOutIndices( outliers, inliers, remaining );

            outliers = extractor.getRemovedIndices();
            seg.setIndices( outliers );
            seg.segment (*inliers, *coefficients);

            //copy plane 
            if (inliers->indices.size () == 0)
            {
              PCL_ERROR("Could not estimate a plane for the given data.");
                cout << "Invalid cloud data" << endl;
                return;
            }
            
            pcl::copyPointCloud<Point>(*cloud, 
                                        inliers->indices,
                                        *inlierCloud);


            cv::Mat majorPlane = cv::Mat::zeros(cloud->height *
                                                cloud->width,
                                                1 , CV_8UC1 );
            cloudToMat(inliers, majorPlane );
      
            majorPlane.rows = cloud->height;
            majorPlane.cols = cloud->width;

            //std::cout << "majorPlane is " << majorPlane.cols <<
            //             "x" << majorPlane.rows << "\n";

            cv::Mat cannyLineMat;
            LineArray lines;
            findLines( majorPlane, lines, cannyLineMat );
            convertColor(cannyLineMat, inliers);  
            
            LinePosArray currentLinePos;
            linesToPositions(coefficients, lines, currentLinePos );

            linePositions.push_back( currentLinePos );

            outliers = remaining;
        }
    }


    void filterOutIndices( const pcl::PointIndices::ConstPtr & larger,
                           const pcl::PointIndices::ConstPtr & remove,
                           pcl::PointIndices::Ptr & final ){
        int j = 0;
        for( int i = 0; i < larger->indices.size() ; i ++ ){
            if ( larger->indices[i] == remove->indices[j] ){
                j++;
            }
            else{
                final->indices.push_back( larger->indices[i] );
            }
        }
    }


    void cloudToMat(const pcl::PointIndices::Ptr & validPoints,
                          cv::Mat &mat)
    {
      //set the values of mat that correspond to being on the major
      // plane of interest.
      //These values should be 1, we will end up with a binary matrix:
      //a value of 1 is on the plane,
      //a value of 0 is off the plane.
      for (int i=0; i < validPoints->indices.size(); i++){
	    int index = validPoints->indices[i];
        mat.at<uint8_t>( index, 1 ) = 255;
	  }
      
    }
    
    void findLines(cv::Mat & src, LineArray & lines, cv::Mat & dst)
    {
        
        try{
            //cv::blur( src, dst, cv::Size(5,5) );
            src.copyTo(dst);
            cv::Canny(dst, dst, 25, 230, 3);
        }
        catch ( exception & e ){
            cout << "Error with canny edge detector" << e.what() << "\n";
            return;
        }
        
        int size = 4 ;
        cv::Mat kernel = cv::Mat::ones( size, size, CV_64F ); 
        cv::dilate( dst, dst, kernel);

        //dst, lines, rho_resolution, theta_resolution, threshold,
        //minLinLength, maxLineGap
        cv::HoughLinesP(dst, lines, 8, CV_PI/180, 200, 100, 4 );
        
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
        
        imageViewer->showRGBImage( cdst.data, cdst.cols, cdst.rows );
        //cv::imshow("foo", cdst);
        //cv::waitKey(30);
                
    }
    void convertColor(cv::Mat & mat, pcl::PointIndices::Ptr inliers)
>>>>>>> parent of e59f50f... added functionality to save/read pcd files
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

  SimpleOpenNIViewer v;

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
