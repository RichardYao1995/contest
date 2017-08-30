#include <ARToolKitPlus/TrackerSingleMarker.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
//#include <qt4/QtGui/QMessageBox>

/* by Danping, 2017,08.14*/

void cvMatToRawData(const cv::Mat& img, std::vector<unsigned char>& rawData){
    /* convert the image into a gray level image*/
    cv::Mat gray;
    if( img.channels() > 1)
        cv::cvtColor(img, gray, CV_RGB2GRAY);
    else
        gray = img;    
    rawData.resize(gray.rows*gray.cols);
    for( size_t i = 0; i < gray.rows; i++){
        for( size_t j = 0; j < gray.cols; j++)
            rawData[i*gray.cols+j] = gray.at<unsigned char>(i,j);
    }
}

int main(int argc, char** argv){
    using namespace std;
    using ARToolKitPlus::TrackerSingleMarker;
    
    //QMessageBox::about(NULL,"About","About the application");

    cv::Mat img = cv::imread("/home/dayan/catkin_ws/src/artoolkitplus/sample_data/u1.png",
                             cv::IMREAD_GRAYSCALE);
    
    /* convert the opencv matrix into a raw camera buffer*/
    std::vector<unsigned char> cameraBuffer;
    cvMatToRawData(img, cameraBuffer);
    
    // create a tracker that does:
    //  - 6x6 sized marker images (required for binary markers)
    //  - samples at a maximum of 6x6
    //  - works with luminance (gray) images
    //  - can load a maximum of 0 non-binary pattern
    //  - can detect a maximum of 8 patterns in one imagege
    TrackerSingleMarker tracker(img.cols, img.rows, 8, 6, 6, 6, 0);
    tracker.setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
    
    // load a camera calibration file.
    if (!tracker.init("/home/dayan/catkin_ws/src/artoolkitplus/sample_data/tango_camera_no_distortion.cal",
                      1.0f, 1000.0f)) // load MATLAB file
    {
        printf("ERROR: init() failed\n");
        return -1;
    }
    
    double pattern_width = 100.0;
    bool useBCH = true;
    tracker.setPatternWidth(pattern_width);
    tracker.setBorderWidth(useBCH ? 0.125 : 0.25);
    
    int thresholds[12] = {20,40,60,80,100,120,140,160,180,200,220,240};
    
    std::vector<int> markerId_tmp;
    int id_best = -1;
    float conf_best = 0;
    ARFloat T_best[16];
    ARFloat corners_best[4][2];
    for( size_t i = 0; i < 12; i++){
        tracker.setThreshold(thresholds[i]);
        
        // let's use lookup-table undistortion for high-speed
        // note: LUT only works with images up to 1024x1024
        tracker.setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
    
        // switch to simple ID based markers
        // use the tool in tools/IdPatGen to generate markers
        tracker.setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);
        
        ARToolKitPlus::ARMarkerInfo* marker_info = 0;
        markerId_tmp = tracker.calc(&cameraBuffer[0], &marker_info);
        if( markerId_tmp.size() > 0){
            id_best = tracker.selectBestMarkerByCf(); /*choose the marker with highest confidence*/
            float conf = tracker.getConfidence();
        
            if( conf > conf_best){
                conf_best = conf;
                std::copy(tracker.getModelViewMatrix(),tracker.getModelViewMatrix()+16,T_best);
                const ARToolKitPlus::ARMarkerInfo* marker_info = tracker.getMarkerInfoById(id_best);
                for( size_t s = 0; s < 4; s++){
                    for( size_t t = 0; t < 2; t++){
                        corners_best[s][t] = marker_info->vertex[s][t];
                    }
                }
            }
            // use the result of calc() to setup the OpenGL transformation
            // glMatrixMode(GL_MODELVIEW)
            // glLoadMatrixf(tracker.getModelViewMatrix());
        
            printf("\n\nFound marker %d  (confidence %d%%)\n\nPose-Matrix:\n  ", id_best, (int(conf * 100.0f)));
            for (int i = 0; i < 16; i++)
                printf("%.2f  %s", tracker.getModelViewMatrix()[i], (i % 4 == 3) ? "\n  " : "");
        }
    }
    
    cv::Mat todraw;
    cv::cvtColor(img,todraw,CV_GRAY2RGB);
    
    if( id_best >= 0){
        /*visualize those markers*/
        double R[9],t[3];
        const ARFloat* T = T_best;
        
        /**1.draw the detected corners*/
        for( size_t i = 0; i < 4; i++){
            cv::circle(todraw,cv::Point2f(corners_best[i][0],
                       corners_best[i][1]),3,cv::Scalar(0,255,255),1,CV_AA);
        }
        
        /**2.draw the reprojected corners (to check the camera pose) */
        /* T = [R  t
         *      0  1]
         */
        
        /* convert 4x4 column-major matrix into 3x3 row-major matrix*/
        R[0] = T[0];
        R[1] = T[4];
        R[2] = T[8];
        
        R[3] = T[1];
        R[4] = T[5];
        R[5] = T[9];
        
        R[6] = T[2];
        R[7] = T[6];
        R[8] = T[10];
        
        /* copy the translation vector from T*/
        t[0] = T[12];
        t[1] = T[13];
        t[2] = T[14];
        
        double corner[4][2] = {{-0.5,-0.5},{0.5,-0.5},{0.5,0.5},{-0.5,0.5}};
        for( size_t i = 0; i < 4; i++){
            double x = corner[i][0]*pattern_width;
            double y = corner[i][1]*pattern_width;
            double z = 0;
            
            double xc = R[0]*x+R[1]*y+R[2]*z +t[0];
            double yc = R[3]*x+R[4]*y+R[5]*z +t[1];
            double zc = R[6]*x+R[7]*y+R[8]*z +t[2];
            
            double xn = xc/zc;
            double yn = yc/zc;
                       
            /* project to the image*/
            double u = tracker.getCamera()->mat[0][0]*xn + tracker.getCamera()->mat[0][2];
            double v = tracker.getCamera()->mat[1][1]*yn + tracker.getCamera()->mat[1][2];
            
            cv::circle(todraw,cv::Point2f(u,v),3,cv::Scalar(255,0,0),1,CV_AA);            
        }
    }
    
    cv::imshow("image", todraw);
    cv::waitKey(-1);
  
    return 0;
}
