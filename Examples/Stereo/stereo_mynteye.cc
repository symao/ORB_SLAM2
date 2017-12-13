/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

int main(int argc, char **argv)
{
    std::string strVocFile, strSettingsFile, video_file, video_ts_file;
    if(argc <= 1)
    {
        // strVocFile = "/home/symao/workspace/ORB_SLAM2/Vocabulary/ORBvoc.txt";
        strVocFile = "/home/symao/workspace/ORB_SLAM2/Vocabulary/ORBvoc.bin";
        strSettingsFile = "/home/symao/workspace/ORB_SLAM2/Examples/Stereo/mynteye.yaml";
        video_file = "/home/symao/data/mynteye/20171107vins_outside/1/img.avi";
        video_ts_file = "/home/symao/data/mynteye/20171107vins_outside/1/imgts.txt";
    }
    else if(argc == 5)
    {
        strVocFile = std::string(argv[1]);
        strSettingsFile = std::string(argv[2]);
        video_file = std::string(argv[3]);
        video_ts_file = std::string(argv[4]);
    }
    else
    {
        cerr << endl << "Usage: ./stereo_mynteye path_to_vocabulary path_to_settings video video_ts" << endl;
        return 1;
    }

    cv::VideoCapture cap(video_file);
    if(!cap.isOpened())
    {
        printf("[ERROR] stereo_video_play: cannot open video %s\n", video_file.c_str());
        return -1;
    }
    std::ifstream fin_imgts(video_ts_file);
    if(!fin_imgts.is_open())
    {
        printf("[ERROR] stereo_video_play: cannot open file %s\n", video_ts_file.c_str());
        return -1;
    }

    const int nImages = cap.get(CV_CAP_PROP_FRAME_COUNT);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(strVocFile,strSettingsFile,ORB_SLAM2::System::STEREO,0);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat imLeft, imRight;
    for(int epoch=0; epoch<2; epoch++)
    {
        for(int ni=0; ni<nImages; ni++)
        {
            printf("==============%d\n", ni);
            cv::Mat img;
            cap>>img;
            int rows = img.rows/2;
            cv::cvtColor(img.rowRange(0,rows),imLeft,cv::COLOR_BGR2GRAY);
            cv::cvtColor(img.rowRange(rows,rows*2),imRight,cv::COLOR_BGR2GRAY);
            double tframe;
            fin_imgts>>tframe;

            if(imLeft.empty())
            {
                cerr << endl << "Failed to load image." << endl;
                return 1;
            }

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the images to the SLAM system
            SLAM.TrackStereo(imLeft,imRight,tframe);

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        #if 0
            printf("%f\n", ttrack);
            std::ofstream fout("time.txt", std::ios::app);
            fout<<ttrack<<std::endl;
            fout.close();
        #endif
            vTimesTrack[ni]=ttrack;
            usleep(20000);
        }
        printf("Press any key to continue.\n");
        getchar();
        cap.open(video_file);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    // SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
    getchar();
    return 0;
}