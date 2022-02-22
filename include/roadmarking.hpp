// roadmarking.hpp
// reconstructs road markings from stereo
#ifndef ROADMARKING_HPP
#define ROADMARKING_HPP

#include <vector>
#include <mutex>

#include <opencv2/opencv.hpp>

namespace RM_SLAM
{

class KeyFrame;
class Map;
class Frame;

class RMPoint
{
public:
    RMPoint(const cv::Mat &pos, KeyFrame* pRefKF, Map* pMap);
    RMPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    void SetRMId(const unsigned int &id);
    unsigned int GetRMId();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(RMPoint* pMP);    
    RMPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    // void ComputeDistinctiveDescriptors();

    // cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     RMPoint* mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

    //  Road marking
    long unsigned int mnRMId;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
}; // class RMPoint

class RoadMarking
{
public:
    RoadMarking();

    bool runELAS(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat& maskLeft, const cv::Mat& maskRight, const cv::Mat& Q);

    std::vector<cv::Point3d *> getAllPoints() const;
    long unsigned int RMPionts() const;

    std::vector<unsigned int> getAllRMIds() const;

protected:
    bool convertTo3DPoints(const cv::Mat &imLeft, const cv::Mat &maskLeft, const cv::Mat imLeft32f, const cv::Mat& Q);

    std::vector<cv::Point3d *> mvpPts3D;
    std::vector<unsigned int> mvnId;
}; // class RoadMarking
} // namespace RM_SLAM


#endif // ROADMARKING_HPP