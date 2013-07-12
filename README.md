
Planar Edge Segmentation and Detection.
=======================================

Team:
    Swarthmore College DRC - Door Opening

Who:
    Will Schneider and Temple Price, with supervision from Matt Zucker
    ==============     ============                        ===========

What:
    Planar Edge Detection
        This code segments out planes from a scene, and detects the straight edges
        along those planes. In addition, lines on each of these planes are found 
        using the intensity image.
    DoorDetection
        The edge and plane info is sent to the doordetector.
        In the current interation, door detection does not happen automatically,
        The user must click on the four corners of the door.
        By selecting the four corners of the door, the program can tell the pose of
        the door.

How:
      Right now, the main functionality of the program resides in two classes:
      EdgeDetector and PlaneSegmenter. PlaneSegmenter is the most general purpose
      of the two. It segments planes and finds the edges. EdgeDetector (which needs 
      renaming) has a segmenter it uses the edge and plane info from the segmenter to
      enable the user to select a door. Once the four door edges have been selected,
      the door information ( height, width, and pose{XYZ and roll, pitch, yaw} ) can be
      given using a function call (getDoorInfo).

Notes:
    Setting Parameters for the PlaneSegmenter:
        The plane segmenter has a boat-load-and-a-half parameters, so the easiest way 
        to set them all is with the configuration file. There is a constructor for the
        PlaneSegmenter that takes a string as input and this constructs the PlaneSegmenter
        with a config file.

    EdgeDetector vs PlaneSegmenter:
        The PlaneSegmenter is useful for a variety of tasks that need plane segementation 
        and line detection in those planes ie: wall detection, ladder detection, stair
        detection, ground detection, data compression (compresses point cloud into several
        lines), 
        The edgeDetector is not particularly for general use. It is meant to find doors
        using the planeSegementer data. It is highly formatted for our data, and so not
        all that useful to other teams.

Future Releases:
    Turn these classes into stand-alone ros nodes.

Dependencies:
    eigen >= 3
    pcl >= 1.6
        Handles point cloud algorithms
    openCV >= 2.3
        Handles edge detection and line detection.

