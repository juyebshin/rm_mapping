// stereo_apollo.cpp
#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <cstring>
#include <sstream>
#include <locale>

#include <opencv2/opencv.hpp>

#include "system.hpp"

using namespace std;

void loadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps, const string &strStart);
void loadLabels(const string &strPathToSequence, vector<string> &vstrLabelLeft,
                vector<string> &vstrLabelRight, const string &strStart);
vector<string> split(string str, char Delimiter);

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./stereo_apollo path_to_settings path_to_color path_to_label starting_record_number" << endl;
        return 1;
    }

    stringstream ssStart;
    ssStart << std::setfill('0') << std::setw(3) << argv[4];

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    loadImages(string(argv[2]), vstrImageLeft, vstrImageRight, vTimestamps, ssStart.str());

    vector<string> vstrLabelLeft;
    vector<string> vstrLabelRight;
    loadLabels(string(argv[3]), vstrLabelLeft, vstrLabelRight, ssStart.str());

    cout << "image left size: " << vstrImageLeft.size() << ", label left size: " << vstrLabelLeft.size() << endl;

    if(vstrImageLeft.size() != vstrImageRight.size())
    {
        cerr << "number of left and right images does not match" 
             << endl << "left: " << vstrImageLeft.size() << ", right: " << vstrImageRight.size() << endl;
        for(int i = 0; i < (vstrImageLeft.size() <= vstrImageRight.size() ? vstrImageLeft.size() : vstrImageRight.size()); i++)
        {
            cout << "left: " << vstrImageLeft[i] << endl << "right: " << vstrImageRight[i] << endl << endl;
        }
        exit(-1);
    }
    if(vstrLabelLeft.size() != vstrLabelRight.size())
    {
        cerr << "number of left and right labels does not match" 
             << endl << "left: " << vstrLabelLeft.size() << ", right: " << vstrLabelRight.size() << endl;
        // for(int i = 0; i < (vstrLabelLeft.size() <= vstrLabelRight.size() ? vstrLabelLeft.size() : vstrLabelRight.size()); i++)
        // {
        //     cout << "left: " << vstrLabelLeft[i] << endl << "right: " << vstrLabelRight[i] << endl << endl;
        // }
        exit(-1);
    }

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    // fsSettings["LEFT.P"] >> P_l;
    // fsSettings["RIGHT.P"] >> P_r;

    // fsSettings["LEFT.R"] >> R_l;
    // fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    cv::Mat R12, t12;
    fsSettings["Stereo.R"] >> R12;
    fsSettings["Stereo.t"] >> t12;

    cout << "R12\n" << R12 << endl;
    cout << "t12\n" << t12 << endl;

    if(K_l.empty() || K_r.empty() /*|| P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty()*/ || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0 || R12.empty() || t12.empty())
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r,Q;
    cv::stereoRectify(K_l, D_l, K_r, D_r, cv::Size(cols_l,rows_l), R12, t12, R_l, R_r, P_l, P_r, Q, cv::CALIB_ZERO_DISPARITY, 0);
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    RM_SLAM::System SLAM(argv[1],RM_SLAM::System::STEREO,true, Q);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat imLeft, imRight, imLeftRect, imRightRect;
    cv::Mat labelLeft, labelRight, labelLeftRect, labelRightRect;
    for (int ni = 0; ni < nImages; ni++)
    {
        /* code */
        imLeft = cv::imread(vstrImageLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni], CV_LOAD_IMAGE_UNCHANGED);
        labelLeft = cv::imread(vstrLabelLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
        labelRight = cv::imread(vstrLabelRight[ni], CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageLeft[ni] << endl;
            return 1;
        }
        if(imRight.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageRight[ni] << endl;
            return 1;
        }
        if(labelLeft.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageLeft[ni] << endl;
            return 1;
        }
        if(labelRight.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageRight[ni] << endl;
            return 1;
        }
        cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);
        cv::remap(labelLeft,labelLeftRect,M1l,M2l,cv::INTER_NEAREST);
        cv::remap(labelRight,labelRightRect,M1r,M2r,cv::INTER_NEAREST);
        cout << endl << "Image index: " << ni << " / " << nImages << endl;
        cout << "File: " << vstrImageLeft[ni] << endl << endl;

        #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
        #endif

        // Pass image to SLAM system and track
        SLAM.trackStereo(imLeftRect, imRightRect, labelLeftRect, labelRightRect, tframe);

        #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
        #endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        cout << "track time: " << ttrack << " T: " << T << endl;
        if(T-ttrack > 90.0) // more then 1.5 minutes
            break;
        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    SLAM.shutdown();
    
    return 0;
}

void loadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps, const string &strStart)
{
    // strPathToSequence: ~/VDC/Dataset/ApolloScape/LaneSegmentation/Colorimage_road02/ColorImage/
    DIR *dir; struct dirent *diread;
    vector<string> files;

    string strStartPath = string("Record") + strStart;

    if ((dir = opendir(strPathToSequence.c_str())) != nullptr) {
        while ((diread = readdir(dir)) != nullptr) {
            if ( !strncmp(diread->d_name, ".", (size_t)1) )
                continue; // ignore directory . or ..
            files.push_back(string(diread->d_name));
            cout << "file: " << diread->d_name << endl;
        }
        closedir (dir);
    } else {
        perror ("opendir");
        return;
    }

    sort(files.begin(), files.end());
    vector<string>::iterator itr = find(files.begin(), files.end(), strStartPath);
    if(itr == files.end())
    {
        perror ("Starting record number not found");
        return;
    }

    files.erase(files.begin(), itr);

    for (auto file : files)
    {
        cout << "file: " << file << endl;
        // if (!strcmp(file, "")) continue;
        string strPrefixLeft = strPathToSequence + "/" + (file) + "/Camera 5/";
        cout << "strPrefixLeft: " << strPrefixLeft << endl;
        if ((dir = opendir(strPrefixLeft.c_str())) != nullptr) {
            while ((diread = readdir(dir)) != nullptr) {
                if ( !strncmp(diread->d_name, ".", (size_t)1) )
                    continue; // ignore directory . or ..
                string imageName(diread->d_name);
                vstrImageLeft.push_back(strPrefixLeft + imageName);
                // cout << "image: " << vstrImageFilenames.back() << endl;
                // split image name with token "_" and obtain timestamp
                // format: 00 00 00 000: hour min sec
                imageName.insert(0, "20");
                vector<string> timestamp = split(imageName, '_');
                // timestanp[0]: yyyymmdd, timestamp[1]: hhmmssmss
                std::tm t = {};
                string strmsec = timestamp[1].substr(6, 3);
                timestamp[0].insert(4, "-");
                timestamp[0].insert(7, "-");
                timestamp[1].insert(2, ":");
                timestamp[1].insert(5, ":");
                timestamp[1].erase(8, 3);
                istringstream ss(timestamp[0] + "T" + timestamp[1]);
                // cout << "imageName: " << timestamp[0] + "T" + timestamp[1] << endl;

                if (ss >> std::get_time(&t, "%Y-%m-%dT%H:%M:%S"))
                {
                    // cout << std::put_time(&t, "%c") << "\n"
                    //      << std::mktime(&t) << endl;
                    double time = (double)std::mktime(&t);
                    stringstream ssmsec(strmsec);
                    double dmsec = 0.0;
                    ssmsec >> dmsec;
                    dmsec = dmsec * 0.001;
                    cout.precision(3);
                    // cout << "dmsec: " << dmsec << endl;
                    time = time + dmsec;
                    vTimestamps.push_back(time);
                    cout << fixed << "timestamp: " << vTimestamps.back() << endl;
                }
            }
            closedir (dir);
        } else {
            perror ("opendir");
            return;
            // continue;
        }

        string strPrefixRight = strPathToSequence + "/" + (file) + "/Camera 6/";
        if ((dir = opendir(strPrefixRight.c_str())) != nullptr) {
            while ((diread = readdir(dir)) != nullptr) {
                if ( !strncmp(diread->d_name, ".", (size_t)1) )
                    continue; // ignore directory . or ..
                string imageName(diread->d_name);
                vstrImageRight.push_back(strPrefixRight + imageName);
            }
            closedir (dir);
        } else {
            perror ("opendir");
            return;
            // continue;
        }
    }
    cout << endl;
    sort(vstrImageLeft.begin(), vstrImageLeft.end());
    sort(vstrImageRight.begin(), vstrImageRight.end());
    sort(vTimestamps.begin(), vTimestamps.end());
}

void loadLabels(const string &strPathToSequence, vector<string> &vstrLabelLeft,
                vector<string> &vstrLabelRight, const string &strStart)
{
    // strPathToSequence: ~/VDC/Dataset/ApolloScape/LaneSegmentation/Labels_road02/Label/
    DIR *dir; struct dirent *diread;
    vector<string> files;

    string strStartPath = string("Record") + strStart;

    if ((dir = opendir(strPathToSequence.c_str())) != nullptr) {
        while ((diread = readdir(dir)) != nullptr) {
            if ( !strncmp(diread->d_name, ".", (size_t)1) )
                continue; // ignore directory . or ..
            files.push_back(string(diread->d_name));
            cout << "file: " << diread->d_name << endl;
        }
        closedir (dir);
    } else {
        perror ("opendir");
        return;
    }

    sort(files.begin(), files.end());
    vector<string>::iterator itr = find(files.begin(), files.end(), strStartPath);
    if(itr == files.end())
    {
        perror ("Starting record number not found");
        return;
    }

    files.erase(files.begin(), itr);

    for (auto file : files)
    {
        cout << "file: " << file << endl;
        // if (!strcmp(file, "")) continue;
        string strPrefixLeft = strPathToSequence + "/" + (file) + "/Camera 5/";
        cout << "strPrefixLeft: " << strPrefixLeft << endl;
        if ((dir = opendir(strPrefixLeft.c_str())) != nullptr) {
            while ((diread = readdir(dir)) != nullptr) {
                if ( !strncmp(diread->d_name, ".", (size_t)1) )
                    continue; // ignore directory . or ..
                string imageName(diread->d_name);
                vstrLabelLeft.push_back(strPrefixLeft + imageName);
            }
            closedir (dir);
        } else {
            perror ("opendir");
            return;
            // continue;
        }

        string strPrefixRight = strPathToSequence + "/" + (file) + "/Camera 6/";
        if ((dir = opendir(strPrefixRight.c_str())) != nullptr) {
            while ((diread = readdir(dir)) != nullptr) {
                if ( !strncmp(diread->d_name, ".", (size_t)1) )
                    continue; // ignore directory . or ..
                string imageName(diread->d_name);
                vstrLabelRight.push_back(strPrefixRight + imageName);
            }
            closedir (dir);
        } else {
            perror ("opendir");
            return;
            // continue;
        }
    }
    cout << endl;
    sort(vstrLabelLeft.begin(), vstrLabelLeft.end());
    sort(vstrLabelRight.begin(), vstrLabelRight.end());
}

vector<string> split(string str, char Delimiter)
{
    istringstream iss(str);             // istringstream에 str을 담는다.
    string buffer;                      // 구분자를 기준으로 절삭된 문자열이 담겨지는 버퍼
 
    vector<string> result;
 
    // istringstream은 istream을 상속받으므로 getline을 사용할 수 있다.
    while (getline(iss, buffer, Delimiter)) {
        result.push_back(buffer);               // 절삭된 문자열을 vector에 저장
    }
 
    return result;
}