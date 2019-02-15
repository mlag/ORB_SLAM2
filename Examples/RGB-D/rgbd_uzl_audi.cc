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


#include <algorithm>
#include <chrono>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>

#include <opencv2/core/core.hpp>

#include "System.h"

using namespace std;

namespace fs = std::experimental::filesystem;

//void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
//                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

int main(int argc, char **argv) {
    if (argc != 5) {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association"
             << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestampsRGB;
    vector<double> vTimestampsD;
//    string strAssociationFilename = string(argv[4]);
    const auto dataset_dir = std::string{argv[3]};

    LoadImages(dataset_dir + "/rgb", vstrImageFilenamesRGB, vTimestampsRGB);
    LoadImages(dataset_dir + "/depth", vstrImageFilenamesD, vTimestampsD);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if (vstrImageFilenamesRGB.empty()) {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    } else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size()) {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
//    cv::Mat imRGB, imD;
    for (int i = 0; i < nImages; i++) {
        // Read image and depthmap from file

        const auto imRGB = cv::imread(vstrImageFilenamesRGB.at(i), cv::IMREAD_UNCHANGED);
        const auto imD = cv::imread(vstrImageFilenamesD.at(i), cv::IMREAD_UNCHANGED);
        double tframe = vTimestampsRGB[i];

        if (imRGB.empty()) {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[i] << endl;
            return 1;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB, imD, tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[i] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (i < nImages - 1)
            T = vTimestampsRGB[i + 1] - tframe;
        else if (i > 0)
            T = tframe - vTimestampsRGB[i - 1];

        if (ttrack < T)
            usleep((T - ttrack) * 1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++) {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

//void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
//                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps) {
//    ifstream fAssociation;
//    fAssociation.open(strAssociationFilename.c_str());
//    while (!fAssociation.eof()) {
//        string s;
//        getline(fAssociation, s);
//        if (!s.empty()) {
//            stringstream ss;
//            ss << s;
//            double t;
//            string sRGB, sD;
//            ss >> t;
//            vTimestamps.push_back(t);
//            ss >> sRGB;
//            vstrImageFilenamesRGB.push_back(sRGB);
//            ss >> t;
//            ss >> sD;
//            vstrImageFilenamesD.push_back(sD);
//
//        }
//    }
//}
//
//

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps) {
    std::vector<fs::path> image_paths;
    for (const auto &p : fs::directory_iterator(strPathToSequence)) {
        image_paths.push_back(p.path());
    }
    std::sort(begin(image_paths), end(image_paths), [](const auto &p1, const auto &p2) {
        return p1.filename() < p2.filename();
    });

    for (const auto &p : image_paths) {
        vstrImageFilenames.push_back(p.string());
        auto t = std::stol(p.stem()) / 1000.0;
        vTimestamps.push_back(t);
    }
}