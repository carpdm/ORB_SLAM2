

#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>


#include"Map.h"
#include"LocalMapping.h"
#include"Frame.h"


#include"ORBextractor.h"
#include "System.h"

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    Tracking(System* pSys, Map* pMap,
            const std::string &strSettingPath);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetViewer(Viewer* pViewer);

public:

    // Tracking states
    enum eTrackingState{
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;


    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniMatches;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    std::list<cv::Mat> mlRelativeFramePoses;
    std::list<KeyFrame*> mlpReferences;
    std::list<double> mlFrameTimes;
    std::list<bool> mlbLost;



    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;


    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    

    //Map
    Map* mpMap;

    //Current matches in frame{
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    std::list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM
#endif // TRACKING_H
