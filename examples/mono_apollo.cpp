// mono_apollo.cpp
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

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps, const string &strStart);
vector<string> split(string str, char Delimiter);

int main(int argc, char **argv)
{
    cout << "Road Marking SLAM mono_apollo" << endl;
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_apollo path_to_settings path_to_sequence starting_record_number" << endl;
        return 1;
    }

    stringstream ssStart;
    ssStart << std::setfill('0') << std::setw(3) << argv[3];

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[2]), vstrImageFilenames, vTimestamps, ssStart.str());

    int nImages = vstrImageFilenames.size();

    RM_SLAM::System SLAM(argv[1], RM_SLAM::System::MONOCULAR, false);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cv::Mat im;
    for (int ni = 0; ni < nImages; ni++)
    {
        /* code */
        im = cv::imread(vstrImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }
        cout << endl << "Image index: " << ni << " / " << nImages << endl;
        cout << "File: " << vstrImageFilenames[ni] << endl << endl;

        #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
        #endif

        // Pass image to SLAM system and track
        SLAM.trackMonocular(im, tframe);

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

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps, const string &strStart)
{
    // strPathToSequence: ~/VDC/Dataset/ApolloScape/LaneSegmentation/Colorimage_road02/ColorImage/
    DIR *dir; struct dirent *diread;
    vector<string> files;

    string strStartPath = string("Record") + strStart;
    cout << "starting record: " << strStartPath << endl;

    if ((dir = opendir(strPathToSequence.c_str())) != nullptr) {
        while ((diread = readdir(dir)) != nullptr) {
            if ( !strncmp(diread->d_name, ".", (size_t)1) )
                continue; // ignore directory . or ..
            files.push_back(string(diread->d_name));
            // cout << "file: " << diread->d_name << endl;
        }
        closedir (dir);
    } else {
        perror ("opendir");
        return;
    }

    std::sort(files.begin(), files.end());
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
        // cout << "strPrefixLeft: " << strPrefixLeft << endl;
        if ((dir = opendir(strPrefixLeft.c_str())) != nullptr) {
            while ((diread = readdir(dir)) != nullptr) {
                if ( !strncmp(diread->d_name, ".", (size_t)1) )
                    continue; // ignore directory . or ..
                string imageName(diread->d_name);
                vstrImageFilenames.push_back(strPrefixLeft + imageName);
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
                    // cout << fixed << "timestamp: " << vTimestamps.back() << endl;
                }
            }
            closedir (dir);
        } else {
            perror ("opendir");
            return;
            // continue;
        }
    }
    cout << endl;
    sort(vstrImageFilenames.begin(), vstrImageFilenames.end());
    sort(vTimestamps.begin(), vTimestamps.end());
    // ifstream fTimes;
    // string strPathTimeFile = strPathToSequence + "/times.txt";
    // fTimes.open(strPathTimeFile.c_str());
    // while(!fTimes.eof())
    // {
    //     string s;
    //     getline(fTimes,s);
    //     if(!s.empty())
    //     {
    //         stringstream ss;
    //         ss << s;
    //         double t;
    //         ss >> t;
    //         vTimestamps.push_back(t);
    //     }
    // }

    // string strPrefixLeft = strPathToSequence + "/image_0/";

    // const int nTimes = vTimestamps.size();
    // vstrImageFilenames.resize(nTimes);

    // for(int i=0; i<nTimes; i++)
    // {
    //     stringstream ss;
    //     ss << setfill('0') << setw(6) << i;
    //     vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    // }
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