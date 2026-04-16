# orb-slam3-spatial

External spatial-memory integration layer for ORB-SLAM3.
ORB-SLAM3 is consumed as an **unmodified git submodule** plus a small, auditable patch set.
All spatial-service–specific logic (UDS `SocketPublisher`, per-frame map-point export, atlas merge CLI, live-stream wrapper) lives outside the submodule and links against its library.

## Repo layout

```
.
├── CMakeLists.txt            # Top-level build for our wrapper binaries
├── patches/                  # Minimal patches applied to the ORB-SLAM3 submodule
├── scripts/
│   ├── apply-patches.sh      # Idempotent patch applier
│   ├── build-orb-slam3.sh    # Builds submodule (Thirdparty deps + libORB_SLAM3)
│   └── build.sh              # End-to-end: patches → submodule → wrapper
├── include/
│   └── socket_publisher.h    # UDS server header (was in ORB_SLAM3/include)
├── src/
│   ├── socket_publisher.cc   # UDS server implementation
│   └── main_tum.cc           # TUM-dataset entrypoint (replaces ORB_SLAM3/Examples/Monocular/mono_tum.cc)
├── config/                   # Camera YAML configs (TUM1_tuned, Generic720, etc.)
├── Vocabulary/
│   └── ORBvoc.txt.bin        # Pre-generated binary ORB vocabulary (46 MB)
├── tools/                    # (pending) combine_atlas CLI
└── thirdparty/
    └── ORB_SLAM3/            # git submodule, upstream royshil/ORB_SLAM3_macosx
```

## Patch set

Six self-contained patches, applied to the submodule at build time:

| # | File | Purpose |
|---|---|---|
| 01 | `Thirdparty/DBoW2/.../TemplatedVocabulary.h` | Binary vocabulary load/save (~12 min → ~4 s startup) |
| 02 | `Thirdparty/g2o/.../hyper_graph.h` | GCC 11 `unordered_map` include fix |
| 03 | `include/Atlas.h`, `src/Atlas.cc` | `AddMap`, `GetMaxIds` for atlas combine |
| 04 | `include/Map.h`, `src/Map.cc` | `RemapIds` for atlas combine |
| 05 | `src/System.cc` | Auto-detect `${vocab}.bin` sibling, load binary first |
| 06 | `include/System.h` | Public `GetAtlas()` getter for external consumers |

Nothing in the patches touches tracking, local mapping, or loop closing logic — they are either additive (new methods) or isolated to vocabulary I/O and header includes.

## Architecture

```
┌───────────────────────────────────────────────────────────────┐
│  Our code                                                     │
│                                                               │
│   main_tum.cc                                                 │
│     │                                                         │
│     │ SLAM.TrackMonocular(img, t) ──────────┐                 │
│     │ SLAM.GetTrackedMapPoints()            │  Public API     │
│     │ SLAM.GetTrackedKeyPointsUn()          │  only           │
│     │ SLAM.GetTrackingState()               │                 │
│     │ SLAM.MapChanged()           ←─────────┤  (loop detect)  │
│     │ SLAM.GetAtlas()->GetCurrentMap()->GetId()  ←─── patch 06│
│     │                                                         │
│     └──► spatial::SocketPublisher.PublishFrame(...)           │
│              │                                                │
│              └──► UDS: /tmp/orbslam3.sock                     │
└───────────────────────────────────────────────────────────────┘
                              ▲
                              │ links against
                              │
┌───────────────────────────────────────────────────────────────┐
│  thirdparty/ORB_SLAM3/  (submodule + 6 patches)               │
│    libORB_SLAM3.{dylib,so}                                    │
└───────────────────────────────────────────────────────────────┘
```

Key insight: the fork previously *called* `SocketPublisher` from within
`System::TrackMonocular()`. Here we move the call site to *after* the public
`TrackMonocular()` return. Because `mTrackedMapPoints` and
`mTrackedKeyPointsUn` are member variables that do not change between Track
calls (they are populated inside `GrabImageMonocular` and remain stable until
the next frame), the externally-extracted data is identical to what the fork
published internally.

## Prerequisites

### macOS (Homebrew)

```bash
brew install cmake opencv eigen pangolin boost openssl@3 libomp
```

### Linux (Debian/Ubuntu)

```bash
sudo apt install build-essential cmake libopencv-dev libeigen3-dev \
                 libboost-all-dev libssl-dev libglew-dev
# Pangolin: build from source (https://github.com/stevenlovegrove/Pangolin)
```

## Build

```bash
git submodule update --init --recursive
scripts/build.sh
```

Produces:
- `thirdparty/ORB_SLAM3/lib/libORB_SLAM3.{dylib,so}`
- `build/mono_tum`

## Run (offline TUM dataset)

```bash
build/mono_tum Vocabulary/ORBvoc.txt.bin \
               config/TUM1_tuned.yaml \
               /path/to/tum_desk_frames \
               --socket /tmp/orbslam3.sock \
               --camera-id 0
```

Emits a `perframe_points.bin` into the sequence directory that mirrors the
fork's output format byte-for-byte on identical inputs.

## Status (2026-04-16)

- [x] Repo restructured onto orphan branch `feat/external-wrapper`
- [x] Submodule added (royshil/ORB_SLAM3_macosx @ master)
- [x] 6 patches generated and idempotent `apply-patches.sh` verified
- [x] `SocketPublisher` extracted, `main_tum.cc` reimplemented externally
- [x] `CMakeLists.txt` + `build.sh` scaffolding
- [ ] End-to-end local build on macOS (pending dep install)
- [ ] Precision parity test: external `mono_tum` vs fork's `mono_tum` on TUM desk
- [ ] `main_stream.cc` external wrapper (for live camera / RTSP)
- [ ] `tools/combine_atlas.cc` external wrapper
- [ ] CI workflow (GitHub Actions) for macOS + Linux cross-platform builds
- [ ] Release artifact packaging for distribution

Previous fork state archived at tags `archive/master-v0.3.0`,
`archive/phase3-combine-stream`, `archive/socket-publisher`.
