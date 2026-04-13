/**
 * mono_stream: ORB-SLAM3 monocular with live camera/RTSP input.
 *
 * Usage: mono_stream <vocab> <yaml> <stream_url>
 *   stream_url examples:
 *     /dev/video0                          (local webcam)
 *     rtsp://host:port/stream              (RTSP)
 *     http://host:port/stream.m3u8         (HLS)
 */

#include <iostream>
#include <chrono>
#include <csignal>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>

using namespace std;

volatile sig_atomic_t g_shutdown = 0;

void signal_handler(int sig) { g_shutdown = 1; }

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        cerr << "Usage: mono_stream <vocab> <yaml> <stream_url>" << endl;
        return 1;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    string vocabPath  = argv[1];
    string yamlPath   = argv[2];
    string streamUrl  = argv[3];

    cout << "Opening stream: " << streamUrl << endl;
    cv::VideoCapture cap(streamUrl);
    if (!cap.isOpened())
    {
        cerr << "ERROR: Cannot open stream " << streamUrl << endl;
        return 1;
    }

    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0) fps = 30.0;
    cout << "Stream FPS: " << fps << endl;

    // Create SLAM system. Viewer disabled — use RunViewer() pattern from
    // mono_tum so the viewer thread is handled by the SLAM system itself.
    ORB_SLAM3::System SLAM(vocabPath, yamlPath, ORB_SLAM3::System::MONOCULAR, false);
    float imageScale = SLAM.GetImageScale();

    cout << endl << "-------" << endl;
    cout << "SLAM system ready. Processing live stream..." << endl;

    int frame_count = 0;

    // Run capture + tracking in a dedicated thread (matches mono_tum pattern)
    thread th{[&]() {
        cv::Mat im;

        while (!g_shutdown)
        {
            if (!cap.read(im) || im.empty())
            {
                cerr << "Stream ended or error reading frame" << endl;
                break;
            }

            // Use stream media timestamp (ms -> seconds) for alignment with
            // Spatial Service's media_timestamp-based feature lookup.
            double timestamp = cap.get(cv::CAP_PROP_POS_MSEC) / 1000.0;
            if (timestamp <= 0)
            {
                // Fallback for sources that don't provide media time
                timestamp = static_cast<double>(frame_count) / fps;
            }

            // Resize if the YAML specifies a different image scale
            if (imageScale != 1.f)
            {
                int width  = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height));
            }

            // Track
            SLAM.TrackMonocular(im, timestamp);
            frame_count++;

            if (frame_count % 300 == 0)
            {
                cout << "Processed " << frame_count << " frames (t="
                     << timestamp << "s)" << endl;
            }
        }
    }};

    SLAM.RunViewer();
    th.join();

    cout << "Shutting down after " << frame_count << " frames..." << endl;
    SLAM.Shutdown();

    // Save trajectories
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

    return 0;
}
