// roadmarking.cpp
#include "roadmarking.hpp"

#include "Map.h"
#include "KeyFrame.h"

#include <thirdparty/libelas/src/elas.h>
#include <thirdparty/libelas/src/image.h>

using namespace std;

namespace RM_SLAM
{

long unsigned int RMPoint::nNextId=0;
mutex RMPoint::mGlobalMutex;

RMPoint::RMPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap, const cv::Mat &refPos):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<RMPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    refPos.copyTo(mCameraPos);
    
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // RMPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

RMPoint::RMPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // RMPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

void RMPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat RMPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat RMPoint::GetCameraPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mCameraPos.clone();
}

void RMPoint::SetRMId(const unsigned int &id)
{
    mnRMId = id;
}

unsigned int RMPoint::GetRMId()
{
    return mnRMId;
}

cv::Mat RMPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* RMPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void RMPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

    if(pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}

void RMPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)
                nObs-=2;
            else
                nObs--;

            mObservations.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> RMPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int RMPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void RMPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        // pKF->EraseRMPointMatch(mit->second);
    }

    // mpMap->EraseRMPoint(this);
}

RMPoint* RMPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void RMPoint::Replace(RMPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            // pKF->ReplaceRMPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
            // pKF->EraseRMPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    // pMP->ComputeDistinctiveDescriptors();

    // mpMap->EraseRMPoint(this);
}

bool RMPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void RMPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void RMPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float RMPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

// void RMPoint::ComputeDistinctiveDescriptors()
// {
//     // Retrieve all observed descriptors
//     vector<cv::Mat> vDescriptors;

//     map<KeyFrame*,size_t> observations;

//     {
//         unique_lock<mutex> lock1(mMutexFeatures);
//         if(mbBad)
//             return;
//         observations=mObservations;
//     }

//     if(observations.empty())
//         return;

//     vDescriptors.reserve(observations.size());

//     for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
//     {
//         KeyFrame* pKF = mit->first;

//         if(!pKF->isBad())
//             vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
//     }

//     if(vDescriptors.empty())
//         return;

//     // Compute distances between them
//     const size_t N = vDescriptors.size();

//     float Distances[N][N];
//     for(size_t i=0;i<N;i++)
//     {
//         Distances[i][i]=0;
//         for(size_t j=i+1;j<N;j++)
//         {
//             int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
//             Distances[i][j]=distij;
//             Distances[j][i]=distij;
//         }
//     }

//     // Take the descriptor with least median distance to the rest
//     int BestMedian = INT_MAX;
//     int BestIdx = 0;
//     for(size_t i=0;i<N;i++)
//     {
//         vector<int> vDists(Distances[i],Distances[i]+N);
//         sort(vDists.begin(),vDists.end());
//         int median = vDists[0.5*(N-1)];

//         if(median<BestMedian)
//         {
//             BestMedian = median;
//             BestIdx = i;
//         }
//     }

//     {
//         unique_lock<mutex> lock(mMutexFeatures);
//         mDescriptor = vDescriptors[BestIdx].clone();
//     }
// }

// cv::Mat RMPoint::GetDescriptor()
// {
//     unique_lock<mutex> lock(mMutexFeatures);
//     return mDescriptor.clone();
// }

int RMPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool RMPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void RMPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float RMPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float RMPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

RoadMarking::RoadMarking()
{

}

bool RoadMarking::runELAS(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat& maskLeft, const cv::Mat& maskRight, const cv::Mat& Q, double dth)
{
    std::cout << "image size: " << imLeft.size() << std::endl;
    // check for correct size
    if (imLeft.cols <= 0 || imLeft.rows <= 0 || imRight.cols <= 0 || imRight.rows <= 0 ||
        imLeft.cols != imRight.cols || imLeft.rows != imRight.rows)
    {
        std::cout << "ERROR: Images must be of same size, but" << std::endl;
        std::cout << "       I1: " << imLeft.cols <<  " x " << imLeft.rows << 
                          ", I2: " << imRight.cols <<  " x " << imRight.rows << std::endl;
        return false;
    }

    // get image width and height
    int32_t width  = imLeft.cols;
    int32_t height = imLeft.rows;
    cv::Size imsize(width, height);

    cv::Mat imLeft32f(imsize, CV_32F);
    cv::Mat imRight32f(imsize, CV_32F);

    // allocate memory for disparity images
    const int32_t dims[3] = {width,height,width}; // bytes per line = width

    // process
    Elas::parameters param;
    param.postprocess_only_left = false;
    Elas elas(param);
    elas.process(imLeft.data, imRight.data, imLeft32f.ptr<float>(0), imRight32f.ptr<float>(0), dims, maskLeft.data, maskRight.data);

    // find maximum disparity for scaling output disparity images to [0..255]
    // float disp_max = 0;
    // for(int row = 0; row < height; ++row)
    // {
    //     for(int col = 0; col < width; ++col)
    //     {
    //         if(imLeft32f.at<float>(row,col) > disp_max)
    //             disp_max = imLeft32f.at<float>(row,col);
    //         if(imRight32f.at<float>(row,col) > disp_max)
    //             disp_max = imRight32f.at<float>(row,col);
    //     }
    // }

    // // copy float to uchar
    // for(int row = 0; row < height; ++row)
    // {
    //     for(int col = 0; col < width; ++col)
    //     {
    //         leftDM->at<uchar>(row,col)  = (uint8_t)std::max(255.0*left32F->at<float>(row,col)/disp_max, 0.0);
    //         rightDM->at<uchar>(row,col) = (uint8_t)std::max(255.0*right32F->at<float>(row,col)/disp_max, 0.0);
    //     }
    // }
    
    return convertTo3DPoints(imLeft, maskLeft, imLeft32f, Q, dth);
}

std::vector<cv::Point3f *> RoadMarking::getAllPoints() const
{
    return std::vector<cv::Point3f*>(mvpPts3D.begin(), mvpPts3D.end());
}

long unsigned int RoadMarking::RMPionts() const
{
    return mvpPts3D.size();
}

std::vector<unsigned int> RoadMarking::getAllRMIds() const
{
    return std::vector<unsigned int>(mvnId.begin(), mvnId.end());
}

bool RoadMarking::convertTo3DPoints(const cv::Mat &imLeft, const cv::Mat &maskLeft, const cv::Mat imLeft32f, const cv::Mat& Q, double dth)
{
    int32_t width  = imLeft.cols;
    int32_t height = imLeft.rows;
    cv::Size imsize(width, height);

    mvpPts3D.clear();
    mvnId.clear();

    cv::Mat V = cv::Mat(4,1, CV_64FC1);
    cv::Mat pos = cv::Mat(4,1, CV_64FC1);

    for(int v = 0; v < height; ++v)
    {
        for(int u = 0; u < width; ++u)
        {
            // Road marking id
            unsigned int id = maskLeft.at<uchar>(v, u);
            if(id < 200) continue; // ignore if background

            float d = imLeft32f.at<float>(v, u);
            if(d < 10.) continue;

            V.at<double>(0,0) = double(u);
            V.at<double>(1,0) = double(v);
            V.at<double>(2,0) = double(d);
            V.at<double>(3,0) = 1.;
 
            pos = Q*V;

            double X = pos.at<double>(0,0) / pos.at<double>(3,0);
            double Y = pos.at<double>(1,0) / pos.at<double>(3,0);
            double Z = pos.at<double>(2,0) / pos.at<double>(3,0);

            if(Z > dth) continue;

            cv::Point3f* point = new cv::Point3f();
            point->x = float(X);
            point->y = float(Y);
            point->z = float(Z);

            mvpPts3D.push_back(point);
            mvnId.push_back(id);
        }
    }

    if(mvpPts3D.empty())
    {
        std::cout << "convertTo3DPoints: no points" << std::endl;
        // return false;
    }
    return true;
}
} // namespace RM_SLAM
