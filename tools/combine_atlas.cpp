/**
 * combine_atlas — Merge two ORB-SLAM3 .osa Atlas files into one.
 *
 * Usage:
 *   combine_atlas <vocab_path> <atlas_a.osa> <atlas_b.osa> <output.osa>
 *                 [--manifest path.json]
 *
 * Loads Atlas A and Atlas B using the same binary deserialization as
 * System::LoadAtlas, remaps all IDs in B to avoid collisions with A,
 * merges B's maps into A, validates uniqueness, serializes the combined
 * atlas, and optionally writes a manifest JSON.
 *
 * Copyright (C) 2024 orb-slam3-spatial contributors
 * License: GPLv3
 */

#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include <vector>
#include <algorithm>
#include <cstdlib>

#include <openssl/md5.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include "Atlas.h"
#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

using namespace std;
using namespace ORB_SLAM3;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/**
 * Compute MD5 checksum of a file (same logic as System::CalculateCheckSum).
 */
static string CalculateCheckSum(const string& filename)
{
    ifstream f(filename.c_str(), ios::in);
    if (!f.is_open())
    {
        cerr << "[combine_atlas] WARNING: Cannot open " << filename
             << " for MD5 hash." << endl;
        return "";
    }

    unsigned char c[MD5_DIGEST_LENGTH];
    MD5_CTX md5Context;
    char buffer[1024];

    MD5_Init(&md5Context);
    while (int count = f.readsome(buffer, sizeof(buffer)))
    {
        MD5_Update(&md5Context, buffer, count);
    }
    f.close();
    MD5_Final(c, &md5Context);

    string checksum;
    for (int i = 0; i < MD5_DIGEST_LENGTH; i++)
    {
        char aux[10];
        sprintf(aux, "%02x", c[i]);
        checksum += aux;
    }
    return checksum;
}

static void PrintUsage(const char* prog)
{
    cerr << "Usage: " << prog
         << " <vocab_path> <atlas_a.osa> <atlas_b.osa> <output.osa>"
         << " [--manifest path.json]" << endl;
}

static ORBVocabulary* LoadVocabulary(const string& vocPath)
{
    ORBVocabulary* pVoc = new ORBVocabulary();

    // Try binary first (append .bin if not already present)
    string binPath = vocPath;
    if (binPath.size() < 4 || binPath.substr(binPath.size() - 4) != ".bin")
        binPath = vocPath + ".bin";

    bool loaded = false;
    {
        ifstream test(binPath, ios::binary);
        if (test.good())
        {
            test.close();
            cout << "[combine_atlas] Loading binary vocabulary: " << binPath << endl;
            loaded = pVoc->loadFromBinaryFile(binPath);
            if (loaded)
                cout << "[combine_atlas] Binary vocabulary loaded." << endl;
        }
    }

    if (!loaded)
    {
        cout << "[combine_atlas] Loading text vocabulary: " << vocPath << endl;
        loaded = pVoc->loadFromTextFile(vocPath);
        if (loaded)
            cout << "[combine_atlas] Text vocabulary loaded." << endl;
    }

    if (!loaded)
    {
        cerr << "[combine_atlas] ERROR: Failed to load vocabulary from "
             << vocPath << endl;
        delete pVoc;
        return nullptr;
    }

    return pVoc;
}

/**
 * Load an .osa atlas using the exact same binary format as System::LoadAtlas.
 * Wire up KeyFrameDatabase and ORBVocabulary, then call PostLoad().
 *
 * Returns nullptr on failure.
 */
static Atlas* LoadAtlas(const string& osaPath,
                        ORBVocabulary* pVoc,
                        KeyFrameDatabase*& pKFDB_out,
                        const string& expectedVocChecksum)
{
    ifstream ifs(osaPath, ios::binary);
    if (!ifs.good())
    {
        cerr << "[combine_atlas] ERROR: Cannot open " << osaPath << endl;
        return nullptr;
    }

    Atlas* pAtlas = nullptr;
    string strVocName, strVocChecksum;

    try
    {
        boost::archive::binary_iarchive ia(ifs);
        ia >> strVocName;
        ia >> strVocChecksum;
        ia >> pAtlas;
    }
    catch (const boost::archive::archive_exception& e)
    {
        cerr << "[combine_atlas] ERROR: Boost deserialization failed for "
             << osaPath << ": " << e.what() << endl;
        return nullptr;
    }
    ifs.close();

    if (!pAtlas)
    {
        cerr << "[combine_atlas] ERROR: Deserialized null atlas from "
             << osaPath << endl;
        return nullptr;
    }

    // Validate vocabulary checksum matches the supplied vocabulary
    if (!expectedVocChecksum.empty() && !strVocChecksum.empty()
        && expectedVocChecksum != strVocChecksum)
    {
        cerr << "[combine_atlas] ERROR: Vocabulary checksum mismatch for "
             << osaPath << endl;
        cerr << "  Atlas vocab: " << strVocName
             << " (checksum: " << strVocChecksum << ")" << endl;
        cerr << "  Supplied vocab checksum: " << expectedVocChecksum << endl;
        delete pAtlas;
        return nullptr;
    }

    cout << "[combine_atlas] Loaded atlas from " << osaPath
         << " (vocab: " << strVocName << ")" << endl;

    // Create KeyFrameDatabase and wire up, same as System::LoadAtlas
    KeyFrameDatabase* pKFDB = new KeyFrameDatabase(*pVoc);
    pAtlas->SetKeyFrameDababase(pKFDB);
    pAtlas->SetORBVocabulary(pVoc);
    pAtlas->PostLoad();

    pKFDB_out = pKFDB;
    return pAtlas;
}

/**
 * Serialize an Atlas to .osa using the same binary format as System::SaveAtlas.
 */
static bool SaveAtlas(Atlas* pAtlas, const string& osaPath,
                      const string& vocName, const string& vocChecksum)
{
    pAtlas->PreSave();

    ofstream ofs(osaPath, ios::binary);
    if (!ofs.good())
    {
        cerr << "[combine_atlas] ERROR: Cannot create output file " << osaPath << endl;
        return false;
    }

    try
    {
        boost::archive::binary_oarchive oa(ofs);
        oa << vocName;
        oa << vocChecksum;
        oa << pAtlas;
    }
    catch (const boost::archive::archive_exception& e)
    {
        cerr << "[combine_atlas] ERROR: Boost serialization failed: "
             << e.what() << endl;
        return false;
    }
    ofs.close();

    cout << "[combine_atlas] Saved combined atlas to " << osaPath << endl;
    return true;
}

/**
 * Validate that no KeyFrame or MapPoint IDs collide across all maps.
 * Returns true if all IDs are unique.
 */
static bool ValidateNoCollisions(Atlas* pAtlas)
{
    vector<Map*> allMaps = pAtlas->GetAllMaps();
    set<long unsigned int> kfIds;
    set<long unsigned int> mpIds;
    bool ok = true;

    for (Map* pMap : allMaps)
    {
        if (!pMap || pMap->IsBad()) continue;

        for (KeyFrame* pKF : pMap->GetAllKeyFrames())
        {
            if (!pKF) continue;
            if (!kfIds.insert(pKF->mnId).second)
            {
                cerr << "[combine_atlas] COLLISION: KeyFrame ID " << pKF->mnId
                     << " in map " << pMap->GetId() << endl;
                ok = false;
            }
        }

        for (MapPoint* pMP : pMap->GetAllMapPoints())
        {
            if (!pMP) continue;
            if (!mpIds.insert(pMP->mnId).second)
            {
                cerr << "[combine_atlas] COLLISION: MapPoint ID " << pMP->mnId
                     << " in map " << pMap->GetId() << endl;
                ok = false;
            }
        }
    }

    cout << "[combine_atlas] Validation: " << kfIds.size() << " unique KF IDs, "
         << mpIds.size() << " unique MP IDs — "
         << (ok ? "PASS" : "FAIL") << endl;
    return ok;
}

/**
 * Write a simple manifest JSON (no external dependency).
 */
static bool WriteManifest(const string& path,
                          const vector<long unsigned int>& sourceMapIds,
                          const vector<long unsigned int>& targetMapIds,
                          long unsigned int kfOffset,
                          long unsigned int mpOffset)
{
    ofstream ofs(path);
    if (!ofs.good())
    {
        cerr << "[combine_atlas] ERROR: Cannot write manifest to " << path << endl;
        return false;
    }

    ofs << "{\n";

    // source_maps
    ofs << "  \"source_maps\": [";
    for (size_t i = 0; i < sourceMapIds.size(); ++i)
    {
        if (i > 0) ofs << ", ";
        ofs << "{\"map_id\": " << sourceMapIds[i] << "}";
    }
    ofs << "],\n";

    // target_maps
    ofs << "  \"target_maps\": [";
    for (size_t i = 0; i < targetMapIds.size(); ++i)
    {
        if (i > 0) ofs << ", ";
        ofs << "{\"map_id\": " << targetMapIds[i] << "}";
    }
    ofs << "],\n";

    // remapped_ids
    ofs << "  \"remapped_ids\": {\"kf_offset\": " << kfOffset
        << ", \"mp_offset\": " << mpOffset << "}\n";

    ofs << "}\n";
    ofs.close();

    cout << "[combine_atlas] Manifest written to " << path << endl;
    return true;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    if (argc < 5)
    {
        PrintUsage(argv[0]);
        return 1;
    }

    string vocPath   = argv[1];
    string atlasAPath = argv[2];
    string atlasBPath = argv[3];
    string outputPath = argv[4];
    string manifestPath;

    // Parse optional --manifest flag
    for (int i = 5; i < argc; ++i)
    {
        string arg(argv[i]);
        if (arg == "--manifest" && i + 1 < argc)
        {
            manifestPath = argv[++i];
        }
        else
        {
            cerr << "[combine_atlas] Unknown argument: " << arg << endl;
            PrintUsage(argv[0]);
            return 1;
        }
    }

    // -----------------------------------------------------------------------
    // Step 1: Load vocabulary
    // -----------------------------------------------------------------------
    ORBVocabulary* pVoc = LoadVocabulary(vocPath);
    if (!pVoc)
        return 1;

    // -----------------------------------------------------------------------
    // Step 1b: Compute vocabulary checksum for validation + output
    // -----------------------------------------------------------------------
    string vocChecksum = CalculateCheckSum(vocPath);
    if (vocChecksum.empty())
        cout << "[combine_atlas] WARNING: Could not compute vocab checksum (text file missing?)" << endl;
    else
        cout << "[combine_atlas] Vocabulary checksum: " << vocChecksum << endl;

    // -----------------------------------------------------------------------
    // Step 2: Load Atlas A
    // -----------------------------------------------------------------------
    KeyFrameDatabase* pKFDB_A = nullptr;
    Atlas* pAtlasA = LoadAtlas(atlasAPath, pVoc, pKFDB_A, vocChecksum);
    if (!pAtlasA)
    {
        delete pVoc;
        return 1;
    }

    // Capture A's max IDs (live objects, after PostLoad)
    Atlas::IdCounters idsA = pAtlasA->GetMaxIds();

    // Also capture the static counters after loading A
    long unsigned int a_mapNextId  = Map::nNextId;
    long unsigned int a_kfNextId   = KeyFrame::nNextId;
    long unsigned int a_mpNextId   = MapPoint::nNextId;
    long unsigned int a_frameNextId = Frame::nNextId;
    long unsigned int a_camNextId  = GeometricCamera::nNextId;

    cout << "[combine_atlas] Atlas A max IDs — map: " << idsA.mapId
         << ", kf: " << idsA.kfId << ", mp: " << idsA.mpId << endl;
    cout << "[combine_atlas] Static counters after A — Map::nNextId: " << a_mapNextId
         << ", KF::nNextId: " << a_kfNextId
         << ", MP::nNextId: " << a_mpNextId << endl;

    // Collect source map IDs (A's maps, original IDs)
    vector<long unsigned int> sourceMapIds;
    for (Map* pMap : pAtlasA->GetAllMaps())
    {
        if (pMap && !pMap->IsBad())
            sourceMapIds.push_back(pMap->GetId());
    }

    // -----------------------------------------------------------------------
    // Step 3: Load Atlas B (this overwrites static counters!)
    // -----------------------------------------------------------------------
    KeyFrameDatabase* pKFDB_B = nullptr;
    Atlas* pAtlasB = LoadAtlas(atlasBPath, pVoc, pKFDB_B, vocChecksum);
    if (!pAtlasB)
    {
        delete pKFDB_A;
        delete pAtlasA;
        delete pVoc;
        return 1;
    }

    Atlas::IdCounters idsB = pAtlasB->GetMaxIds();

    cout << "[combine_atlas] Atlas B max IDs — map: " << idsB.mapId
         << ", kf: " << idsB.kfId << ", mp: " << idsB.mpId << endl;

    // -----------------------------------------------------------------------
    // Step 4: Calculate offsets with 1000-unit buffer
    // -----------------------------------------------------------------------
    static const long unsigned int kBuffer = 1000;

    long unsigned int kfOffset = max(idsA.kfId, a_kfNextId) + kBuffer;
    long unsigned int mpOffset = max(idsA.mpId, a_mpNextId) + kBuffer;
    long unsigned int mapOffset = max(idsA.mapId, a_mapNextId) + kBuffer;

    cout << "[combine_atlas] Offsets — kf: " << kfOffset
         << ", mp: " << mpOffset
         << ", map: " << mapOffset << endl;

    // -----------------------------------------------------------------------
    // Step 5: PreSave B (populate backup IDs), then remap
    // -----------------------------------------------------------------------
    // RemapIds works on backup ID fields (populated by PreSave) and live
    // mnId fields on KF/MP objects.
    pAtlasB->PreSave();

    vector<Map*> bMaps = pAtlasB->GetAllMaps();
    for (Map* pMap : bMaps)
    {
        if (!pMap || pMap->IsBad()) continue;

        // Remap map ID to avoid collision with A's maps
        pMap->ChangeId(pMap->GetId() + mapOffset);

        // Remap KF/MP IDs within this map (mapOffset for mnOriginMapId)
        pMap->RemapIds(kfOffset, mpOffset, mapOffset);
    }

    // Collect target map IDs (B's maps, after remapping)
    vector<long unsigned int> targetMapIds;
    for (Map* pMap : bMaps)
    {
        if (pMap && !pMap->IsBad())
            targetMapIds.push_back(pMap->GetId());
    }

    // -----------------------------------------------------------------------
    // Step 6: Merge B's maps into A
    // -----------------------------------------------------------------------
    // We need to call PostLoad on B's maps so they rebuild live pointer
    // structures from the remapped backup IDs, before adding to A.
    // But first, wire B's KFDB to A's so the merged KFs register there.
    // Actually — after PreSave + RemapIds, the backup IDs are remapped.
    // We add the maps to A, then let PostLoad rebuild when we save/reload.
    // For the merge we add maps directly to A's map set.
    for (Map* pMap : bMaps)
    {
        if (!pMap || pMap->IsBad()) continue;
        pAtlasA->AddMap(pMap);
    }

    // Merge B's camera registry into A (deduplication via AddCamera).
    // Without this, Atlas::PreSave builds spCams from A's mvpCameras only,
    // and B's KeyFrames write mnBackupIdCamera = -1 → broken on reload.
    vector<GeometricCamera*> bCams = pAtlasB->GetAllCameras();
    int newCams = 0;
    for (GeometricCamera* pCam : bCams)
    {
        if (!pCam) continue;
        GeometricCamera* pExisting = pAtlasA->AddCamera(pCam);
        if (pExisting != pCam)
        {
            // Camera was deduplicated — update B's KFs to point to A's copy.
            // This ensures PreSave finds them in spCams.
            for (Map* pMap : bMaps)
            {
                if (!pMap || pMap->IsBad()) continue;
                for (KeyFrame* pKF : pMap->GetAllKeyFrames())
                {
                    if (pKF && pKF->mpCamera == pCam)
                        pKF->mpCamera = pExisting;
                    if (pKF && pKF->mpCamera2 == pCam)
                        pKF->mpCamera2 = pExisting;
                }
            }
        }
        else
        {
            newCams++;
        }
    }
    cout << "[combine_atlas] Camera merge: " << bCams.size()
         << " from B, " << newCams << " new, "
         << pAtlasA->GetAllCameras().size() << " total in A" << endl;

    cout << "[combine_atlas] Merged " << targetMapIds.size()
         << " maps from B into A. Total maps: "
         << pAtlasA->GetAllMaps().size() << endl;

    // -----------------------------------------------------------------------
    // Step 7: Set static counters to safe max values
    // -----------------------------------------------------------------------
    // After remapping, B's max IDs have shifted. Compute new maxima.
    long unsigned int finalMaxKF = max(idsA.kfId, idsB.kfId + kfOffset);
    long unsigned int finalMaxMP = max(idsA.mpId, idsB.mpId + mpOffset);
    long unsigned int finalMaxMap = max(idsA.mapId, idsB.mapId + mapOffset);

    Map::nNextId          = finalMaxMap + kBuffer;
    KeyFrame::nNextId     = finalMaxKF + kBuffer;
    MapPoint::nNextId     = finalMaxMP + kBuffer;
    Frame::nNextId        = max(a_frameNextId, Frame::nNextId) + kBuffer;
    GeometricCamera::nNextId = max(a_camNextId, GeometricCamera::nNextId) + kBuffer;

    cout << "[combine_atlas] Final static counters — Map::nNextId: " << Map::nNextId
         << ", KF::nNextId: " << KeyFrame::nNextId
         << ", MP::nNextId: " << MapPoint::nNextId << endl;

    // -----------------------------------------------------------------------
    // Step 8: Validate no ID collisions
    // -----------------------------------------------------------------------
    if (!ValidateNoCollisions(pAtlasA))
    {
        cerr << "[combine_atlas] ERROR: ID collision detected! Aborting." << endl;
        delete pKFDB_B;
        delete pAtlasB;
        delete pKFDB_A;
        delete pAtlasA;
        delete pVoc;
        return 1;
    }

    // -----------------------------------------------------------------------
    // Step 9: Serialize combined atlas
    // -----------------------------------------------------------------------
    // Extract vocab name from path (same logic as System::SaveAtlas)
    size_t slashPos = vocPath.find_last_of("/\\");
    string vocName = (slashPos != string::npos) ? vocPath.substr(slashPos + 1) : vocPath;

    if (!SaveAtlas(pAtlasA, outputPath, vocName, vocChecksum))
    {
        delete pKFDB_B;
        delete pKFDB_A;
        delete pVoc;
        return 1;
    }

    // -----------------------------------------------------------------------
    // Step 10: Write manifest JSON (optional)
    // -----------------------------------------------------------------------
    if (!manifestPath.empty())
    {
        WriteManifest(manifestPath, sourceMapIds, targetMapIds, kfOffset, mpOffset);
    }

    // -----------------------------------------------------------------------
    // Summary
    // -----------------------------------------------------------------------
    cout << "\n[combine_atlas] DONE." << endl;
    cout << "  Source maps (A): " << sourceMapIds.size() << endl;
    cout << "  Target maps (B): " << targetMapIds.size() << endl;
    cout << "  Total maps:      " << pAtlasA->GetAllMaps().size() << endl;
    cout << "  Output:          " << outputPath << endl;

    // Cleanup: B's maps are now owned by A. Atlas destructor deletes maps,
    // so we must clear B's map set to prevent double-free.
    pAtlasB->clearAtlas();
    delete pKFDB_B;
    delete pKFDB_A;
    delete pAtlasB;
    delete pAtlasA;
    delete pVoc;

    return 0;
}
