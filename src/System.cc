
#include "System.h"
#include "Utility.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

namespace ORB_SLAM2
{

System::System(const std::string &strVocFile, const std::string &strSettingsFile,
               const bool bUseViewer)
{
    using namespace std;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    Config::LoadConfig(strSettingsFile.c_str());

    if (!fsSettings.isOpened()) {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
    ORBVocabulary * pvoc = SingleInstance::GetVoc();
    bool bVocLoad = pvoc->loadFromTextFile(strVocFile);
    if (!bVocLoad) {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }

    cout << "Vocabulary loaded: " << SingleInstance::GetVoc() << endl << endl;

    //Create the Map
    mpMap = new Map();


    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpMap, strSettingsFile);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, false);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);


    mpTracker->SetLocalMapper(mpLocalMapper);
    mpLocalMapper->SetTracker(mpTracker);

}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    using namespace std;
    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp);
    return Tcw;
}


void System::Shutdown()
{
    mpLocalMapper->RequestFinish();


    // Wait until all thread have effectively stopped
    while (!mpLocalMapper->isFinished()) {
        usleep(5000);
    }


}


void System::SaveTrajectoryKITTI(const std::string &filename)
{
    using namespace std;
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;


    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(), lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++) {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

        while (pKF->isBad()) {
            //  cout << "bad parent" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Two;

        cv::Mat Tcw = (*lit) * Trw;
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

        f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1)  << " " << Rwc.at<float>(0, 2) << " "  << twc.at<float>(0) << " " <<
          Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1)  << " " << Rwc.at<float>(1, 2) << " "  << twc.at<float>(1) << " " <<
          Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1)  << " " << Rwc.at<float>(2, 2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::GetAllPoses(std::vector<std::pair<cv::Mat, double>> &result_vector)
{
    result_vector.clear();
    vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();

    int pose_num = 0;
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame *pKF = vpKFs[i];

        cv::Mat Tcw = pKF->GetPose();

        result_vector.push_back(std::make_pair(Tcw, pKF->mTimeStamp));
        pose_num++;
    }

    std::sort(result_vector.begin(), result_vector.end(), [](
            const std::pair<cv::Mat, double> &x, const std::pair<cv::Mat, double> &y)
    {
        return y.second > x.second;
    });

    // cv::Mat first_cRw = result_vector[0].first.inv();

    for (int i = 0; i < result_vector.size(); i++)
    {
        // result_vector[i].first = result_vector[i].first * first_cRw;
        result_vector[i].first = result_vector[i].first;
    }
}

double System::GetRelativePose()
{
    return mpTracker->mpReferenceKF->mTimeStamp;
}


} //namespace ORB_SLAM
