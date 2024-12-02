# Changes to ICG:
- manual_detector.cpp: 
    - Changed needed points from 4 to 6, since cv::solvePnP needs 6 points for non-planar objects
    - PointDetector::DetectPoints(), added text to the image so the user knows which buttons to press
- viewer.cpp
    - Viewer::DisplayAndSaveImage, added text to the image so the user knows which buttons to press
