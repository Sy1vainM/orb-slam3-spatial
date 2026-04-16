/**
 * main_tum: External wrapper replacing ORB_SLAM3/Examples/Monocular/mono_tum.
 *
 * Responsibilities previously embedded inside ORB_SLAM3::System/LoopClosing
 * are moved here:
 *   - Spawn and drive the SocketPublisher (UDS server for the Spatial Service)
 *   - Publish per-frame MapPoints + KeyFrame poses after each Track call
 *   - Detect loop closures via System::MapChanged() polling
 *   - Detect map merges via Atlas::GetCurrentMap()->GetId() transitions
 *   - Export the offline perframe_points.bin used by precision tests
 *
 * ORB-SLAM3 itself (the submodule) is built from the upstream sources with only
 * the minimal patches in patches/ applied (binary vocab loader, Atlas/Map
 * combine API, System::GetAtlas getter).
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>
#include <Atlas.h>
#include <Map.h>
#include <MapPoint.h>

#include "socket_publisher.h"

using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;

namespace {

// Parse a minimal subset of the ORB-SLAM3 YAML settings to retrieve camera
// intrinsics + image size. This avoids pulling OpenCV's FileStorage into the
// shim by piggy-backing on the same parser ORB-SLAM3 already uses internally.
struct CameraInfo {
    double fx{0}, fy{0}, cx{0}, cy{0};
    int width{0}, height{0};
    bool valid() const { return fx > 0 && fy > 0 && width > 0 && height > 0; }
};

CameraInfo LoadCameraInfo(const string& yamlPath)
{
    CameraInfo info;
    cv::FileStorage fs(yamlPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "WARN: cannot open settings " << yamlPath
             << " for camera intrinsics; skipping MSG_CAMERA_INFO." << endl;
        return info;
    }
    // The standard ORB-SLAM3 mono YAML uses Camera.fx etc. at the top level.
    cv::FileNode n;
    n = fs["Camera.fx"];           if (!n.empty()) info.fx = (double)n;
    n = fs["Camera.fy"];           if (!n.empty()) info.fy = (double)n;
    n = fs["Camera.cx"];           if (!n.empty()) info.cx = (double)n;
    n = fs["Camera.cy"];           if (!n.empty()) info.cy = (double)n;
    n = fs["Camera.width"];        if (!n.empty()) info.width  = (int)n;
    n = fs["Camera.height"];       if (!n.empty()) info.height = (int)n;
    // Newer multi-camera YAML keys (Camera1.fx, ...) fall back.
    if (info.fx == 0) { n = fs["Camera1.fx"]; if (!n.empty()) info.fx = (double)n; }
    if (info.fy == 0) { n = fs["Camera1.fy"]; if (!n.empty()) info.fy = (double)n; }
    if (info.cx == 0) { n = fs["Camera1.cx"]; if (!n.empty()) info.cx = (double)n; }
    if (info.cy == 0) { n = fs["Camera1.cy"]; if (!n.empty()) info.cy = (double)n; }
    return info;
}

void LoadImages(const string& strFile,
                vector<string>& vstrImageFilenames,
                vector<double>& vTimestamps)
{
    std::ifstream f(strFile);
    if (!f.is_open()) {
        cerr << "FATAL: cannot open " << strFile << endl;
        std::exit(1);
    }
    cout << "Load images from " << strFile << endl;
    string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        string sRGB;
        double t;
        ss >> sRGB >> t;
        vstrImageFilenames.push_back("images/" + sRGB + ".jpg");
        vTimestamps.push_back(t);
    }
}

// Exports the same binary layout as the fork's mono_tum.cc so downstream
// precision-diff tools produce identical byte streams on identical inputs.
//
//   timestamp(f64) n_points(i32) [mp_id(i64) u(f32) v(f32) X(f32) Y(f32) Z(f32)] x n
void WritePerframeRecord(std::ofstream& out,
                         double timestamp,
                         const vector<ORB_SLAM3::MapPoint*>& mapPoints,
                         const vector<cv::KeyPoint>& keyPoints)
{
    struct PointRecord { int64_t id; float u, v, x, y, z; };
    vector<PointRecord> records;
    records.reserve(mapPoints.size());
    const size_t limit = std::min(mapPoints.size(), keyPoints.size());
    for (size_t k = 0; k < limit; ++k) {
        ORB_SLAM3::MapPoint* mp = mapPoints[k];
        if (mp && !mp->isBad()) {
            auto pos = mp->GetWorldPos();
            records.push_back({
                static_cast<int64_t>(mp->mnId),
                keyPoints[k].pt.x, keyPoints[k].pt.y,
                pos(0), pos(1), pos(2)
            });
        }
    }
    out.write(reinterpret_cast<const char*>(&timestamp), sizeof(double));
    int32_t nv = static_cast<int32_t>(records.size());
    out.write(reinterpret_cast<const char*>(&nv), sizeof(int32_t));
    for (const auto& rec : records) {
        out.write(reinterpret_cast<const char*>(&rec.id), sizeof(int64_t));
        float data[5] = {rec.u, rec.v, rec.x, rec.y, rec.z};
        out.write(reinterpret_cast<const char*>(data), sizeof(data));
    }
    out.flush();
}

std::atomic<bool> g_shutdown{false};
void HandleSignal(int) { g_shutdown.store(true); }

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 4) {
        cerr << "Usage: " << argv[0]
             << " <vocab> <settings.yaml> <sequence_dir> "
                "[--socket /tmp/orbslam3.sock] [--camera-id N] "
                "[--no-viewer] [--no-publish]" << endl;
        return 1;
    }
    const string vocab     = argv[1];
    const string settings  = argv[2];
    const string sequence  = argv[3];

    string socketPath = "/tmp/orbslam3.sock";
    uint16_t cameraId = 0;
    bool useViewer = true;
    bool doPublish = true;
    for (int i = 4; i < argc; ++i) {
        string a = argv[i];
        if (a == "--socket" && i + 1 < argc)        socketPath = argv[++i];
        else if (a == "--camera-id" && i + 1 < argc) cameraId = static_cast<uint16_t>(std::stoi(argv[++i]));
        else if (a == "--no-viewer") useViewer = false;
        else if (a == "--no-publish") doPublish = false;
        else {
            cerr << "Unknown flag: " << a << endl;
            return 1;
        }
    }

    std::signal(SIGINT,  HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    vector<string> imageFiles;
    vector<double> timestamps;
    LoadImages(sequence + "/times.txt", imageFiles, timestamps);
    const int nImages = static_cast<int>(imageFiles.size());
    cout << "Images in the sequence: " << nImages << endl << endl;

    ORB_SLAM3::System SLAM(vocab, settings,
                           ORB_SLAM3::System::MONOCULAR,
                           useViewer);
    const float imageScale = SLAM.GetImageScale();

    // Bring up the UDS publisher once the system is ready; fetch current Atlas
    // via the patch-exposed getter so the publisher can read active_map_id.
    std::unique_ptr<ORB_SLAM3::SocketPublisher> publisher;
    if (doPublish) {
        publisher.reset(new ORB_SLAM3::SocketPublisher(
            &SLAM, SLAM.GetAtlas(), socketPath, cameraId));
        CameraInfo cam = LoadCameraInfo(settings);
        if (cam.valid()) {
            publisher->SetCameraIntrinsics(cam.fx, cam.fy, cam.cx, cam.cy,
                                           cam.width, cam.height);
        }
        publisher->Start();
    }

    const string perframePath = sequence + "/perframe_points.bin";
    std::ofstream fperframe(perframePath, std::ios::binary);

    vector<float> vTimesTrack(nImages, 0.f);
    unsigned long prevMapId = 0;
    bool haveMapId = false;

    std::thread worker([&]() {
        for (int ni = 0; ni < nImages && !g_shutdown.load(); ++ni) {
            cv::Mat im = cv::imread(sequence + "/" + imageFiles[ni],
                                    cv::IMREAD_UNCHANGED);
            if (im.empty()) {
                cerr << "Failed to load image at: " << sequence << "/"
                     << imageFiles[ni] << endl;
                break;
            }
            const double tframe = timestamps[ni];

            if (imageScale != 1.f) {
                cv::resize(im, im,
                           cv::Size(int(im.cols * imageScale),
                                    int(im.rows * imageScale)));
            }

            auto t1 = std::chrono::steady_clock::now();
            Sophus::SE3f Tcw = SLAM.TrackMonocular(im, tframe);
            auto t2 = std::chrono::steady_clock::now();
            vTimesTrack[ni] = std::chrono::duration_cast<
                std::chrono::duration<float>>(t2 - t1).count();

            const int state = SLAM.GetTrackingState();
            const auto mapPoints = SLAM.GetTrackedMapPoints();
            const auto keyPoints = SLAM.GetTrackedKeyPointsUn();

            if (state == 2 /*OK*/ || state == 5 /*RECENTLY_LOST pass-through*/) {
                WritePerframeRecord(fperframe, tframe, mapPoints, keyPoints);
            }

            if (publisher) {
                // Map merge detection: the active map id changes whenever
                // LoopClosing finishes MergeLocal. We fire MSG_MAP_MERGE
                // before PublishKeyframePose so the Python side can update
                // its map-to-scene table before processing the new pose.
                ORB_SLAM3::Atlas* atlas = SLAM.GetAtlas();
                if (atlas && atlas->GetCurrentMap()) {
                    unsigned long curMapId = atlas->GetCurrentMap()->GetId();
                    if (haveMapId && curMapId != prevMapId) {
                        publisher->SetMapMerge(prevMapId, curMapId);
                    }
                    prevMapId = curMapId;
                    haveMapId = true;
                }

                publisher->PublishFrame(mapPoints, keyPoints, tframe, state);

                if (state == 2 /*OK*/) {
                    // Loop closure polling: System::MapChanged() returns true
                    // once per big change (loop close / full BA / merge). It
                    // auto-resets after query, so we read it at most once per
                    // frame.
                    const bool loopClosed = SLAM.MapChanged();
                    publisher->PublishKeyframePose(
                        Tcw.matrix().cast<double>(), tframe, loopClosed, state);
                }
            }
        }
    });

    SLAM.RunViewer();
    worker.join();

    SLAM.Shutdown();
    if (publisher) publisher->Stop();

    std::sort(vTimesTrack.begin(), vTimesTrack.end());
    float totalTime = 0.f;
    for (float t : vTimesTrack) totalTime += t;
    cout << "-------" << endl
         << "median tracking time: " << vTimesTrack[nImages / 2] << endl
         << "mean tracking time: "   << totalTime / nImages       << endl;

    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    fperframe.close();
    cout << "Saved per-frame map points to " << perframePath << endl;
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    return 0;
}
