/**
 * SocketPublisher implementation.
 *
 * UDS server that pushes MapPoints + KeyframePoses to the Spatial Service
 * and listens for atlas/covisibility commands.
 *
 * All multi-byte values are little-endian, matching Python struct '<' format.
 */

#include "socket_publisher.h"
#include "Atlas.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "System.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <iostream>

namespace ORB_SLAM3
{

// ---------- Construction / Destruction ----------

SocketPublisher::SocketPublisher(System* pSystem, Atlas* pAtlas,
                                 const std::string& socketPath,
                                 uint16_t cameraId)
    : mpSystem(pSystem), mpAtlas(pAtlas),
      mSocketPath(socketPath), mCameraId(cameraId)
{
}

SocketPublisher::~SocketPublisher()
{
    Stop();
}

// ---------- Lifecycle ----------

void SocketPublisher::Start()
{
    if (mbRunning.load())
        return;

    // Remove stale socket file
    ::unlink(mSocketPath.c_str());

    // Create listening socket
    mListenFd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (mListenFd < 0) {
        std::cerr << "SocketPublisher: socket() failed: " << strerror(errno) << std::endl;
        return;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, mSocketPath.c_str(), sizeof(addr.sun_path) - 1);

    if (::bind(mListenFd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "SocketPublisher: bind() failed: " << strerror(errno) << std::endl;
        ::close(mListenFd);
        mListenFd = -1;
        return;
    }

    if (::listen(mListenFd, 1) < 0) {
        std::cerr << "SocketPublisher: listen() failed: " << strerror(errno) << std::endl;
        ::close(mListenFd);
        mListenFd = -1;
        return;
    }

    mbRunning.store(true);

    mAcceptThread = std::thread(&SocketPublisher::AcceptLoop, this);
    mWriteThread  = std::thread(&SocketPublisher::WriteLoop, this);

    std::cout << "SocketPublisher: listening on " << mSocketPath << std::endl;
}

void SocketPublisher::Stop()
{
    if (!mbRunning.load())
        return;

    mbRunning.store(false);

    // Wake up the write thread
    mCondQueue.notify_all();

    // Close sockets to unblock accept/read
    CloseClient();
    if (mListenFd >= 0) {
        ::shutdown(mListenFd, SHUT_RDWR);
        ::close(mListenFd);
        mListenFd = -1;
    }

    if (mAcceptThread.joinable()) mAcceptThread.join();
    if (mWriteThread.joinable())  mWriteThread.join();
    if (mReadThread.joinable())   mReadThread.join();

    // Clean up socket file
    ::unlink(mSocketPath.c_str());

    std::cout << "SocketPublisher: stopped" << std::endl;
}

// ---------- Public API ----------

void SocketPublisher::SetCameraIntrinsics(double fx, double fy, double cx, double cy,
                                          uint32_t width, uint32_t height)
{
    mFx = fx; mFy = fy; mCx = cx; mCy = cy;
    mWidth = width; mHeight = height;
    mbIntrinsicsSet = true;
}

void SocketPublisher::SetLoopClosure()
{
    mbLoopClosure.store(true);
}

void SocketPublisher::SetMapMerge(unsigned long int sourceMapId, unsigned long int targetMapId)
{
    mnMergeSourceMapId = sourceMapId;
    mnMergeTargetMapId = targetMapId;
    mbMapMerge.store(true);
}

void SocketPublisher::PublishFrame(const std::vector<MapPoint*>& mapPoints,
                                   const std::vector<cv::KeyPoint>& keypoints,
                                   double timestamp,
                                   int trackingState)
{
    if (!mbClientConnected.load())
        return;

    // Build and enqueue MapPoints message
    Message msg = BuildMapPointsMsg(mapPoints, keypoints, timestamp);
    if (!msg.data.empty())
        Enqueue(std::move(msg));
}

void SocketPublisher::PublishKeyframePose(const Eigen::Matrix4f& Tcw,
                                          double timestamp,
                                          bool isLoopClosure,
                                          int trackingState)
{
    if (!mbClientConnected.load())
        return;

    // Consume the loop closure flag if set
    bool loopFlag = isLoopClosure || mbLoopClosure.exchange(false);

    Message msg = BuildKeyframePoseMsg(Tcw, timestamp, loopFlag, trackingState);
    Enqueue(std::move(msg));

    // Send MSG_MAP_MERGE if a merge event was flagged
    if (mbMapMerge.exchange(false))
    {
        // Payload: source_map_id(4) + target_map_id(4) = 8 bytes
        uint32_t mergePayloadLen = 8;
        std::vector<uint8_t> header = BuildHeader(MSG_MAP_MERGE, mergePayloadLen, mCameraId);

        std::vector<uint8_t> payload(mergePayloadLen);
        uint8_t* mp = payload.data();
        uint32_t srcId = static_cast<uint32_t>(mnMergeSourceMapId);
        memcpy(mp, &srcId, 4); mp += 4;
        uint32_t tgtId = static_cast<uint32_t>(mnMergeTargetMapId);
        memcpy(mp, &tgtId, 4); mp += 4;

        Message mergeMsg;
        mergeMsg.data.reserve(header.size() + payload.size());
        mergeMsg.data.insert(mergeMsg.data.end(), header.begin(), header.end());
        mergeMsg.data.insert(mergeMsg.data.end(), payload.begin(), payload.end());
        Enqueue(std::move(mergeMsg));
    }
}

// ---------- Protocol Message Builders ----------

std::vector<uint8_t> SocketPublisher::BuildHeader(uint8_t msgType, uint32_t payloadLen,
                                                   uint16_t cameraId)
{
    // Header layout (16 bytes, little-endian):
    //   offset 0:  magic        uint32   0x53504D50
    //   offset 4:  msg_type     uint8
    //   offset 5:  flags        uint8    0x00
    //   offset 6:  payload_len  uint32
    //   offset 10: camera_id    uint16
    //   offset 12: reserved     uint32   0x00000000
    std::vector<uint8_t> hdr(HEADER_SIZE, 0);
    auto* p = hdr.data();

    // magic (LE)
    p[0] = static_cast<uint8_t>(MAGIC & 0xFF);
    p[1] = static_cast<uint8_t>((MAGIC >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((MAGIC >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((MAGIC >> 24) & 0xFF);
    // msg_type
    p[4] = msgType;
    // flags
    p[5] = 0;
    // payload_len (LE)
    p[6]  = static_cast<uint8_t>(payloadLen & 0xFF);
    p[7]  = static_cast<uint8_t>((payloadLen >> 8) & 0xFF);
    p[8]  = static_cast<uint8_t>((payloadLen >> 16) & 0xFF);
    p[9]  = static_cast<uint8_t>((payloadLen >> 24) & 0xFF);
    // camera_id (LE)
    p[10] = static_cast<uint8_t>(cameraId & 0xFF);
    p[11] = static_cast<uint8_t>((cameraId >> 8) & 0xFF);
    // reserved (4 bytes, already zeroed)

    return hdr;
}

SocketPublisher::Message SocketPublisher::BuildMapPointsMsg(
    const std::vector<MapPoint*>& mapPoints,
    const std::vector<cv::KeyPoint>& keypoints,
    double timestamp) const
{
    // Count valid (non-null, non-bad) MapPoints
    uint32_t nValid = 0;
    for (size_t i = 0; i < mapPoints.size(); ++i) {
        MapPoint* pMP = mapPoints[i];
        if (pMP && !pMP->isBad())
            ++nValid;
    }

    if (nValid == 0)
        return Message{};

    // Payload: timestamp(8) + n_points(4) + per_point(28) * nValid + active_map_id(4)
    // Per point: mp_id(8, int64) + u(4) + v(4) + X(4) + Y(4) + Z(4) = 28
    uint32_t payloadLen = 8 + 4 + 28 * nValid + 4;
    std::vector<uint8_t> header = BuildHeader(MSG_MAPPOINTS, payloadLen, mCameraId);

    std::vector<uint8_t> payload(payloadLen);
    uint8_t* p = payload.data();

    // timestamp (double, LE)
    memcpy(p, &timestamp, 8);
    p += 8;

    // n_points (uint32, LE)
    memcpy(p, &nValid, 4);
    p += 4;

    // Per-point data
    for (size_t i = 0; i < mapPoints.size(); ++i) {
        MapPoint* pMP = mapPoints[i];
        if (!pMP || pMP->isBad())
            continue;

        // mp_id: int64 (MapPoint::mnId is long unsigned int)
        int64_t mpId = static_cast<int64_t>(pMP->mnId);
        memcpy(p, &mpId, 8);
        p += 8;

        // u, v from keypoint (float32)
        float u = 0.0f, v = 0.0f;
        if (i < keypoints.size()) {
            u = keypoints[i].pt.x;
            v = keypoints[i].pt.y;
        }
        memcpy(p, &u, 4); p += 4;
        memcpy(p, &v, 4); p += 4;

        // X, Y, Z world position (float32)
        Eigen::Vector3f pos = pMP->GetWorldPos();
        float x = pos(0), y = pos(1), z = pos(2);
        memcpy(p, &x, 4); p += 4;
        memcpy(p, &y, 4); p += 4;
        memcpy(p, &z, 4); p += 4;
    }

    // active_map_id (uint32) — which Map this frame belongs to
    uint32_t activeMapId = 0;
    if (mpAtlas && mpAtlas->GetCurrentMap())
        activeMapId = static_cast<uint32_t>(mpAtlas->GetCurrentMap()->GetId());
    memcpy(p, &activeMapId, 4); p += 4;

    // Concatenate header + payload
    Message msg;
    msg.data.reserve(header.size() + payload.size());
    msg.data.insert(msg.data.end(), header.begin(), header.end());
    msg.data.insert(msg.data.end(), payload.begin(), payload.end());
    return msg;
}

SocketPublisher::Message SocketPublisher::BuildKeyframePoseMsg(
    const Eigen::Matrix4f& Tcw,
    double timestamp,
    bool isLoopClosure,
    int trackingState) const
{
    // The spec says pose is "R 3x3 + t 3x1, row-major, w2c" as float64[12].
    // Tcw is camera-from-world (world-to-camera), which matches "w2c".
    // We transmit R(3x3) row-major then t(3x1).

    // Payload: timestamp(8) + pose(12*8=96) + is_loop(1) + tracking_state(1) + active_map_id(4) = 110
    uint32_t payloadLen = 8 + 96 + 1 + 1 + 4;
    std::vector<uint8_t> header = BuildHeader(MSG_KEYFRAME_POSE, payloadLen, mCameraId);

    std::vector<uint8_t> payload(payloadLen);
    uint8_t* p = payload.data();

    // timestamp (double, LE)
    memcpy(p, &timestamp, 8);
    p += 8;

    // R 3x3 row-major as double
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double val = static_cast<double>(Tcw(r, c));
            memcpy(p, &val, 8);
            p += 8;
        }
    }
    // t 3x1 as double
    for (int r = 0; r < 3; ++r) {
        double val = static_cast<double>(Tcw(r, 3));
        memcpy(p, &val, 8);
        p += 8;
    }

    // is_loop_closure (uint8)
    *p++ = isLoopClosure ? 1 : 0;

    // tracking_state (uint8): map ORB-SLAM3 states to protocol
    // Protocol: 0=OK, 1=RECENTLY_LOST, 2=LOST
    // ORB-SLAM3: OK=2, RECENTLY_LOST=3, LOST=4
    uint8_t protoState = 0;
    if (trackingState == 3) protoState = 1;      // RECENTLY_LOST
    else if (trackingState == 4) protoState = 2;  // LOST
    *p++ = protoState;

    // active_map_id (uint32) — which Map this frame belongs to
    uint32_t activeMapId = 0;
    if (mpAtlas && mpAtlas->GetCurrentMap())
        activeMapId = static_cast<uint32_t>(mpAtlas->GetCurrentMap()->GetId());
    memcpy(p, &activeMapId, 4); p += 4;

    Message msg;
    msg.data.reserve(header.size() + payload.size());
    msg.data.insert(msg.data.end(), header.begin(), header.end());
    msg.data.insert(msg.data.end(), payload.begin(), payload.end());
    return msg;
}

SocketPublisher::Message SocketPublisher::BuildCameraInfoMsg() const
{
    // Payload: fx(8) + fy(8) + cx(8) + cy(8) + width(4) + height(4) = 40
    uint32_t payloadLen = 40;
    std::vector<uint8_t> header = BuildHeader(MSG_CAMERA_INFO, payloadLen, mCameraId);

    std::vector<uint8_t> payload(payloadLen);
    uint8_t* p = payload.data();

    memcpy(p, &mFx, 8); p += 8;
    memcpy(p, &mFy, 8); p += 8;
    memcpy(p, &mCx, 8); p += 8;
    memcpy(p, &mCy, 8); p += 8;
    memcpy(p, &mWidth, 4); p += 4;
    memcpy(p, &mHeight, 4); p += 4;

    Message msg;
    msg.data.reserve(header.size() + payload.size());
    msg.data.insert(msg.data.end(), header.begin(), header.end());
    msg.data.insert(msg.data.end(), payload.begin(), payload.end());
    return msg;
}

SocketPublisher::Message SocketPublisher::BuildCovisibilityMsg(uint16_t minShared) const
{
    // Collect all keyframes from the current map
    Map* pMap = mpAtlas->GetCurrentMap();
    if (!pMap)
        return Message{};

    std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

    // Build edge list: for each KF, iterate its connected KFs with weight >= minShared.
    // To avoid duplicates, only include edge (a,b) where a->mnId < b->mnId.
    struct Edge {
        uint32_t kfA, kfB, shared;
    };
    std::vector<Edge> edges;

    for (KeyFrame* pKF : vpKFs) {
        if (pKF->isBad())
            continue;
        std::vector<KeyFrame*> vCovis = pKF->GetCovisiblesByWeight(static_cast<int>(minShared));
        for (KeyFrame* pCovis : vCovis) {
            if (pCovis->isBad())
                continue;
            if (pKF->mnId < pCovis->mnId) {
                int w = pKF->GetWeight(pCovis);
                edges.push_back({static_cast<uint32_t>(pKF->mnId),
                                 static_cast<uint32_t>(pCovis->mnId),
                                 static_cast<uint32_t>(w)});
            }
        }
    }

    // Payload: n_edges(4) + per_edge(12) * n
    uint32_t nEdges = static_cast<uint32_t>(edges.size());
    uint32_t payloadLen = 4 + 12 * nEdges;
    std::vector<uint8_t> header = BuildHeader(MSG_COVISIBILITY, payloadLen, mCameraId);

    std::vector<uint8_t> payload(payloadLen);
    uint8_t* p = payload.data();

    memcpy(p, &nEdges, 4); p += 4;
    for (const auto& e : edges) {
        memcpy(p, &e.kfA, 4);    p += 4;
        memcpy(p, &e.kfB, 4);    p += 4;
        memcpy(p, &e.shared, 4); p += 4;
    }

    Message msg;
    msg.data.reserve(header.size() + payload.size());
    msg.data.insert(msg.data.end(), header.begin(), header.end());
    msg.data.insert(msg.data.end(), payload.begin(), payload.end());
    return msg;
}

// ---------- Thread Loops ----------

void SocketPublisher::AcceptLoop()
{
    while (mbRunning.load()) {
        // Accept one client at a time
        struct sockaddr_un clientAddr;
        socklen_t addrLen = sizeof(clientAddr);
        int clientFd = ::accept(mListenFd,
                                reinterpret_cast<struct sockaddr*>(&clientAddr),
                                &addrLen);
        if (clientFd < 0) {
            if (mbRunning.load())
                std::cerr << "SocketPublisher: accept() failed: " << strerror(errno) << std::endl;
            continue;
        }

        // Close any existing client
        CloseClient();

        {
            std::lock_guard<std::mutex> lock(mMutexClient);
            mClientFd = clientFd;
        }
        mbClientConnected.store(true);
        std::cout << "SocketPublisher: client connected" << std::endl;

        // Send camera info on connection
        if (mbIntrinsicsSet) {
            Message camMsg = BuildCameraInfoMsg();
            Enqueue(std::move(camMsg));
        }

        // Start read thread for this client (join previous if still active)
        if (mReadThread.joinable())
            mReadThread.join();
        mReadThread = std::thread(&SocketPublisher::ReadLoop, this);
    }
}

void SocketPublisher::WriteLoop()
{
    while (mbRunning.load()) {
        Message msg;
        {
            std::unique_lock<std::mutex> lock(mMutexQueue);
            mCondQueue.wait(lock, [this] {
                return !mWriteQueue.empty() || !mbRunning.load();
            });
            if (!mbRunning.load())
                break;
            msg = std::move(mWriteQueue.front());
            mWriteQueue.pop_front();
        }

        if (!mbClientConnected.load())
            continue;

        // Send all bytes
        int fd;
        {
            std::lock_guard<std::mutex> lock(mMutexClient);
            fd = mClientFd;
        }
        if (fd < 0)
            continue;

        const uint8_t* data = msg.data.data();
        size_t remaining = msg.data.size();
        while (remaining > 0) {
            ssize_t sent = ::send(fd, data, remaining, 0);
            if (sent <= 0) {
                if (errno == EINTR)
                    continue;
                std::cerr << "SocketPublisher: send() failed: " << strerror(errno) << std::endl;
                CloseClient();
                break;
            }
            data += sent;
            remaining -= static_cast<size_t>(sent);
        }
    }
}

void SocketPublisher::ReadLoop()
{
    // Read commands from the client
    std::vector<uint8_t> headerBuf(HEADER_SIZE);

    while (mbRunning.load() && mbClientConnected.load()) {
        int fd;
        {
            std::lock_guard<std::mutex> lock(mMutexClient);
            fd = mClientFd;
        }
        if (fd < 0)
            break;

        // Read header
        size_t bytesRead = 0;
        while (bytesRead < HEADER_SIZE) {
            ssize_t n = ::recv(fd, headerBuf.data() + bytesRead,
                               HEADER_SIZE - bytesRead, 0);
            if (n <= 0) {
                if (n == 0) {
                    std::cout << "SocketPublisher: client disconnected" << std::endl;
                } else if (errno != EINTR) {
                    std::cerr << "SocketPublisher: recv() header failed: "
                              << strerror(errno) << std::endl;
                }
                CloseClient();
                return;
            }
            bytesRead += static_cast<size_t>(n);
        }

        HandleCommand(headerBuf);
    }
}

// ---------- Command Handling ----------

void SocketPublisher::HandleCommand(const std::vector<uint8_t>& headerBuf)
{
    const uint8_t* h = headerBuf.data();

    // Verify magic
    uint32_t magic = 0;
    memcpy(&magic, h, 4);
    if (magic != MAGIC) {
        std::cerr << "SocketPublisher: invalid magic from client: 0x"
                  << std::hex << magic << std::dec << std::endl;
        CloseClient();
        return;
    }

    uint8_t msgType = h[4];
    uint32_t payloadLen = 0;
    memcpy(&payloadLen, h + 6, 4);

    // Read payload
    std::vector<uint8_t> payload(payloadLen);
    if (payloadLen > 0) {
        int fd;
        {
            std::lock_guard<std::mutex> lock(mMutexClient);
            fd = mClientFd;
        }
        size_t bytesRead = 0;
        while (bytesRead < payloadLen) {
            ssize_t n = ::recv(fd, payload.data() + bytesRead,
                               payloadLen - bytesRead, 0);
            if (n <= 0) {
                if (n == 0)
                    std::cout << "SocketPublisher: client disconnected during payload" << std::endl;
                else if (errno != EINTR)
                    std::cerr << "SocketPublisher: recv() payload failed" << std::endl;
                CloseClient();
                return;
            }
            bytesRead += static_cast<size_t>(n);
        }
    }

    switch (msgType) {
        case CMD_LOAD_ATLAS: {
            if (payloadLen < 2) break;
            uint16_t pathLen = 0;
            memcpy(&pathLen, payload.data(), 2);
            if (payloadLen < 2u + pathLen) break;
            std::string atlasPath(reinterpret_cast<char*>(payload.data() + 2), pathLen);
            std::cout << "SocketPublisher: CMD_LOAD_ATLAS path=" << atlasPath << std::endl;
            // The actual atlas loading is delegated to System (requires careful thread coordination).
            // For now we log it; full integration needs System to expose a thread-safe LoadAtlas.
            break;
        }
        case CMD_SAVE_ATLAS: {
            if (payloadLen < 2) break;
            uint16_t pathLen = 0;
            memcpy(&pathLen, payload.data(), 2);
            if (payloadLen < 2u + pathLen) break;
            std::string atlasPath(reinterpret_cast<char*>(payload.data() + 2), pathLen);
            std::cout << "SocketPublisher: CMD_SAVE_ATLAS path=" << atlasPath << std::endl;
            // Delegate to System (same as above).
            break;
        }
        case CMD_GET_COVISIBILITY: {
            if (payloadLen < 2) break;
            uint16_t minShared = 0;
            memcpy(&minShared, payload.data(), 2);
            std::cout << "SocketPublisher: CMD_GET_COVISIBILITY min_shared=" << minShared << std::endl;
            Message response = BuildCovisibilityMsg(minShared);
            if (!response.data.empty())
                Enqueue(std::move(response));
            break;
        }
        default:
            std::cerr << "SocketPublisher: unknown command 0x"
                      << std::hex << static_cast<int>(msgType) << std::dec << std::endl;
            break;
    }
}

// ---------- Queue / Socket Helpers ----------

void SocketPublisher::Enqueue(Message&& msg)
{
    std::lock_guard<std::mutex> lock(mMutexQueue);
    // Drop oldest messages if queue is full (backpressure)
    while (mWriteQueue.size() >= MAX_QUEUE_SIZE)
        mWriteQueue.pop_front();
    mWriteQueue.push_back(std::move(msg));
    mCondQueue.notify_one();
}

void SocketPublisher::CloseClient()
{
    mbClientConnected.store(false);
    std::lock_guard<std::mutex> lock(mMutexClient);
    if (mClientFd >= 0) {
        ::shutdown(mClientFd, SHUT_RDWR);
        ::close(mClientFd);
        mClientFd = -1;
    }
    // Drain write queue on disconnect
    {
        std::lock_guard<std::mutex> lockQ(mMutexQueue);
        mWriteQueue.clear();
    }
}

} // namespace ORB_SLAM3
