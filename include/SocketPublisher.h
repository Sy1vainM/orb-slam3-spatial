/**
 * SocketPublisher: Unix Domain Socket server that pushes MapPoints and
 * KeyframePoses to the Spatial Service using a compact binary protocol.
 *
 * Wire format (little-endian):
 *   Header (16 bytes): magic(4) | msg_type(1) | flags(1) | payload_len(4) | camera_id(2) | reserved(4)
 *   Payload: variable length, format depends on msg_type
 *
 * See docs/specs/2026-04-06-spatial-memory-integration-design.md Section 1.
 */

#ifndef SOCKETPUBLISHER_H
#define SOCKETPUBLISHER_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/core.hpp>
#include <Eigen/Core>

namespace ORB_SLAM3
{

class Atlas;
class System;
class MapPoint;
class KeyFrame;

class SocketPublisher
{
public:
    // Protocol constants
    static constexpr uint32_t MAGIC         = 0x53504D50; // "SPMP"
    static constexpr uint8_t  MSG_MAPPOINTS        = 0x01;
    static constexpr uint8_t  MSG_KEYFRAME_POSE    = 0x02;
    static constexpr uint8_t  MSG_COVISIBILITY     = 0x03;
    static constexpr uint8_t  MSG_CAMERA_INFO      = 0x04;
    static constexpr uint8_t  MSG_MAP_MERGE        = 0x06;
    static constexpr uint8_t  CMD_LOAD_ATLAS       = 0x10;
    static constexpr uint8_t  CMD_SAVE_ATLAS       = 0x11;
    static constexpr uint8_t  CMD_GET_COVISIBILITY = 0x12;
    static constexpr size_t   HEADER_SIZE          = 16;

    /**
     * Construct a SocketPublisher.
     * @param pSystem   Owning System pointer (for atlas save/load commands).
     * @param pAtlas    Atlas pointer (for covisibility queries).
     * @param socketPath  Unix socket path (default /tmp/orbslam3.sock).
     * @param cameraId    Camera identifier for the protocol header.
     */
    SocketPublisher(System* pSystem, Atlas* pAtlas,
                    const std::string& socketPath = "/tmp/orbslam3.sock",
                    uint16_t cameraId = 0);

    ~SocketPublisher();

    // Lifecycle
    void Start();
    void Stop();

    /**
     * Publish the current frame's visible MapPoints.
     * Called from System::Track* after each frame is processed.
     *
     * @param mapPoints  Vector of MapPoint pointers (may contain nulls).
     * @param keypoints  Corresponding undistorted keypoints (same indexing).
     * @param timestamp  Frame timestamp (seconds, from RTSP PTS / video).
     * @param trackingState  Current tracking state (OK=2, RECENTLY_LOST=3, LOST=4).
     */
    void PublishFrame(const std::vector<MapPoint*>& mapPoints,
                      const std::vector<cv::KeyPoint>& keypoints,
                      double timestamp,
                      int trackingState);

    /**
     * Publish a KeyframePose message.
     * Called from System::Track* when pose is available.
     *
     * @param Tcw_SE3  Camera-from-world transform (Sophus::SE3f stored as 4x4).
     * @param timestamp  Frame timestamp.
     * @param isLoopClosure  True if this frame triggered a loop closure.
     * @param trackingState  Current tracking state.
     */
    void PublishKeyframePose(const Eigen::Matrix4f& Tcw,
                             double timestamp,
                             bool isLoopClosure,
                             int trackingState);

    /**
     * Send camera intrinsics (MSG_CAMERA_INFO). Sent once per connection.
     */
    void SetCameraIntrinsics(double fx, double fy, double cx, double cy,
                             uint32_t width, uint32_t height);

    /**
     * Mark that a loop closure was detected.
     * Thread-safe; read by PublishKeyframePose.
     */
    void SetLoopClosure();

    /**
     * Mark that a map merge occurred.
     * Thread-safe; the merge event is published to the client asynchronously.
     *
     * @param sourceMapId  Map being absorbed (will be marked bad).
     * @param targetMapId  Map that absorbs the source.
     */
    void SetMapMerge(unsigned long int sourceMapId, unsigned long int targetMapId);

private:
    // Internal message buffer entry
    struct Message {
        std::vector<uint8_t> data; // header + payload, ready to send
    };

    // Build protocol messages
    static std::vector<uint8_t> BuildHeader(uint8_t msgType, uint32_t payloadLen,
                                            uint16_t cameraId);
    Message BuildMapPointsMsg(const std::vector<MapPoint*>& mapPoints,
                              const std::vector<cv::KeyPoint>& keypoints,
                              double timestamp) const;
    Message BuildKeyframePoseMsg(const Eigen::Matrix4f& Tcw,
                                double timestamp,
                                bool isLoopClosure,
                                int trackingState) const;
    Message BuildCameraInfoMsg() const;
    Message BuildCovisibilityMsg(uint16_t minShared) const;

    // Thread entry points
    void AcceptLoop();   // Listens for incoming connections
    void WriteLoop();    // Drains the write queue to the client
    void ReadLoop();     // Reads commands from the client

    // Command handling
    void HandleCommand(const std::vector<uint8_t>& headerBuf);

    // Enqueue a message for sending
    void Enqueue(Message&& msg);

    // Socket helpers
    void CloseClient();

    // System / Atlas references
    System* mpSystem;
    Atlas*  mpAtlas;

    // Configuration
    std::string mSocketPath;
    uint16_t    mCameraId;

    // Camera intrinsics (set once)
    double mFx{0}, mFy{0}, mCx{0}, mCy{0};
    uint32_t mWidth{0}, mHeight{0};
    bool mbIntrinsicsSet{false};

    // Socket file descriptors
    int mListenFd{-1};
    int mClientFd{-1};
    std::mutex mMutexClient;

    // Write queue
    std::deque<Message> mWriteQueue;
    std::mutex mMutexQueue;
    std::condition_variable mCondQueue;

    // Threads
    std::thread mAcceptThread;
    std::thread mWriteThread;
    std::thread mReadThread;
    std::atomic<bool> mbRunning{false};
    std::atomic<bool> mbClientConnected{false};

    // Loop closure flag (set by LoopClosing thread, consumed by publish)
    std::atomic<bool> mbLoopClosure{false};

    // Map merge state (set by LoopClosing thread, consumed by publish)
    std::atomic<bool> mbMapMerge{false};
    unsigned long int mnMergeSourceMapId{0};
    unsigned long int mnMergeTargetMapId{0};

    // Max queue size to avoid unbounded memory growth
    static constexpr size_t MAX_QUEUE_SIZE = 300;
};

} // namespace ORB_SLAM3

#endif // SOCKETPUBLISHER_H
