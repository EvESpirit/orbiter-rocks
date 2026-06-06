#include "D3D9Client.h"
extern class D3D9Client* g_client;
// ==============================================================
// RockScatter.cpp
// Part of the ORBITER VISUALISATION PROJECT (OVP)
// Dual licensed under GPL v3 and LGPL v3
// Copyright (c) EvESpirit
// ==============================================================

#include "RockScatter.h"
#include "D3D9Config.h"
#include "Log.h"
#include "OapiExtension.h"
#include "Scene.h"
#include "VPlanet.h"
#include "DebugControls.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

namespace oapi { class D3D9Client; }
extern oapi::D3D9Client *g_client;
// Quality multipliers for rock density: Off / Low / Medium / High
static const float g_qualityMult[] = {0.0f, 0.25f, 0.6f, 1.0f};

// Draw distance multipliers (config)
static inline float GetLodCull(int sizeClass) {
  switch (sizeClass) {
  case 0:
    return Config->fRockDistSmall;
  case 1:
    return Config->fRockDistMedium;
  case 2:
    return Config->fRockDistLarge;
  default:
    return 1.0f;
  }
}

// PRNG helpers

uint32_t RockScatter::HashTile(uint32_t seed, int lvl, int ilat, int ilng) {
  uint32_t h = seed;
  h ^= (uint32_t)lvl * 2654435761u;
  h ^= (uint32_t)ilat * 2246822519u;
  h ^= (uint32_t)ilng * 3266489917u;
  h ^= h >> 16;
  h *= 0x85ebca6bu;
  h ^= h >> 13;
  h *= 0xc2b2ae35u;
  h ^= h >> 16;
  return h ? h : 1u;
}

uint32_t RockScatter::XorShift32(uint32_t &state) {
  state ^= state << 13;
  state ^= state >> 17;
  state ^= state << 5;
  return state;
}

float RockScatter::RandFloat(uint32_t &state) {
  return float(XorShift32(state) & 0x00FFFFFFu) / float(0x01000000u);
}

float RockScatter::RandRange(uint32_t &state, float lo, float hi) {
  return lo + RandFloat(state) * (hi - lo);
}

// Basic meshgen using vertex displacement (icosphere)

// normalise a D3DXVECTOR3
static D3DXVECTOR3 Norm3(const D3DXVECTOR3 &v) {
  D3DXVECTOR3 o;
  D3DXVec3Normalize(&o, &v);
  return o;
}

// subdivide one triangle of an icosphere
static void SubdivideIco(std::vector<NTVERTEX> &verts, std::vector<WORD> &idxs,
                         std::map<uint64_t, WORD> &midCache, WORD i0, WORD i1,
                         WORD i2, int depth) {
  if (depth == 0) {
    idxs.push_back(i0);
    idxs.push_back(i1);
    idxs.push_back(i2);
    return;
  }
  // get or create midpoint vertex
  auto getMid = [&](WORD a, WORD b) -> WORD {
    uint64_t key = (uint64_t)min(a, b) << 32 | (uint64_t)max(a, b);
    auto it = midCache.find(key);
    if (it != midCache.end())
      return it->second;
    D3DXVECTOR3 p = Norm3(D3DXVECTOR3((verts[a].x + verts[b].x) * 0.5f,
                                      (verts[a].y + verts[b].y) * 0.5f,
                                      (verts[a].z + verts[b].z) * 0.5f));
    NTVERTEX nv;
    memset(&nv, 0, sizeof(nv));
    nv.x = p.x;
    nv.y = p.y;
    nv.z = p.z;
    nv.nx = p.x;
    nv.ny = p.y;
    nv.nz = p.z;
    WORD idx = (WORD)verts.size();
    verts.push_back(nv);
    midCache[key] = idx;
    return idx;
  };
  WORD m01 = getMid(i0, i1), m12 = getMid(i1, i2), m02 = getMid(i0, i2);
  SubdivideIco(verts, idxs, midCache, i0, m01, m02, depth - 1);
  SubdivideIco(verts, idxs, midCache, i1, m12, m01, depth - 1);
  SubdivideIco(verts, idxs, midCache, i2, m02, m12, depth - 1);
  SubdivideIco(verts, idxs, midCache, m01, m12, m02, depth - 1);
}

void RockScatter::CreateIcosphereMesh(int subdivisions, UINT seed,
                                      float baseScale,
                                      std::vector<NTVERTEX> &outVerts,
                                      std::vector<WORD> &outIdxs) {
  // Assembly
  const float t = (1.0f + sqrtf(5.0f)) * 0.5f;
  D3DXVECTOR3 baseVerts[] = {
      Norm3({-1, t, 0}),  Norm3({1, t, 0}),   Norm3({-1, -t, 0}),
      Norm3({1, -t, 0}),  Norm3({0, -1, t}),  Norm3({0, 1, t}),
      Norm3({0, -1, -t}), Norm3({0, 1, -t}),  Norm3({t, 0, -1}),
      Norm3({t, 0, 1}),   Norm3({-t, 0, -1}), Norm3({-t, 0, 1})};
  static const WORD baseTris[] = {
      0, 11, 5,  0, 5,  1, 0, 1, 7, 0, 7,  10, 0, 10, 11, 1, 5, 9, 5, 11,
      4, 11, 10, 2, 10, 7, 6, 7, 1, 8, 3,  9,  4, 3,  4,  2, 3, 2, 6, 3,
      6, 8,  3,  8, 9,  4, 9, 5, 2, 4, 11, 6,  2, 10, 8,  6, 7, 9, 8, 1};

  outVerts.resize(12);
  for (int i = 0; i < 12; i++) {
    memset(&outVerts[i], 0, sizeof(NTVERTEX));
    outVerts[i].x = baseVerts[i].x;
    outVerts[i].y = baseVerts[i].y;
    outVerts[i].z = baseVerts[i].z;
    outVerts[i].nx = baseVerts[i].x;
    outVerts[i].ny = baseVerts[i].y;
    outVerts[i].nz = baseVerts[i].z;
  }

  outIdxs.clear();
  std::map<uint64_t, WORD> midCache;
  for (int i = 0; i < 20; i++)
    SubdivideIco(outVerts, outIdxs, midCache, baseTris[i * 3],
                 baseTris[i * 3 + 1], baseTris[i * 3 + 2], subdivisions);

  // displace vertices using seeded noise to make a rock shape
  uint32_t rng = seed ? seed : 42u;
  for (auto &v : outVerts) {
    float disp = 1.0f + (RandFloat(rng) - 0.5f) * 0.6f; // +-30%
    v.x *= disp * baseScale;
    v.y *= disp * baseScale * 0.7f; // flatten vertically
    v.z *= disp * baseScale;

    // without this they're kinda sunken in
    v.y += baseScale * 0.5f;
  }

  // recompute normals
  for (auto &v : outVerts) {
    v.nx = 0;
    v.ny = 0;
    v.nz = 0;
  }
  for (size_t i = 0; i + 2 < outIdxs.size(); i += 3) {
    auto &a = outVerts[outIdxs[i]], &b = outVerts[outIdxs[i + 1]],
         &c = outVerts[outIdxs[i + 2]];
    D3DXVECTOR3 e1(b.x - a.x, b.y - a.y, b.z - a.z),
        e2(c.x - a.x, c.y - a.y, c.z - a.z), n;
    D3DXVec3Cross(&n, &e1, &e2);
    D3DXVec3Normalize(&n, &n);
    a.nx += n.x;
    a.ny += n.y;
    a.nz += n.z;
    b.nx += n.x;
    b.ny += n.y;
    b.nz += n.z;
    c.nx += n.x;
    c.ny += n.y;
    c.nz += n.z;
  }
  for (auto &v : outVerts) {
    D3DXVECTOR3 n = Norm3({v.nx, v.ny, v.nz});
    v.nx = n.x;
    v.ny = n.y;
    v.nz = n.z;
  }

  // reverse winding order for DX
  // The icosphere meshgen produces CCW faces, which are culled by
  // D3DCULL_CCW. Swapping the indices after normal computation makes them CW
  // (visible from outside). We fix that here.
  for (size_t i = 0; i + 2 < outIdxs.size(); i += 3) {
    std::swap(outIdxs[i + 1], outIdxs[i + 2]);
  }
}

void RockScatter::CreateRockMeshes() {
  const ::RockScatterCfg *pCfg = oapiGetRockScatterCfg(m_planet->Object());
  if (!pCfg) return;
  const ::RockScatterCfg &cfg = *pCfg;

  if (cfg.sMeshPrefix[0] != '\0') {
    WIN32_FIND_DATAA fd;
    char searchPath[MAX_PATH];

    // absolute path to orbiter root dir
    char exePath[MAX_PATH];
    GetModuleFileNameA(NULL, exePath, MAX_PATH);
    // strip filename to get directory
    char *lastSlash = strrchr(exePath, '\\');
    if (lastSlash)
      *(lastSlash + 1) = '\0';

    sprintf_s(searchPath, sizeof(searchPath), "%sMeshes\\%s*.msh", exePath,
              cfg.sMeshPrefix);
    LogAlw("RockScatter: Mesh prefix = '%s'", cfg.sMeshPrefix);
    LogAlw("RockScatter: Searching for meshes with pattern '%s'", searchPath);

    std::vector<std::string> foundFiles;
    HANDLE hFind = FindFirstFileA(searchPath, &fd);
    if (hFind != INVALID_HANDLE_VALUE) {
      do {
        foundFiles.push_back(fd.cFileName);
      } while (FindNextFileA(hFind, &fd));
      FindClose(hFind);
    }

    // sort to guarantee determinism
    std::sort(foundFiles.begin(), foundFiles.end());

    std::string dirPath = "";
    std::string meshPrefixStr(cfg.sMeshPrefix);
    size_t slashPos = meshPrefixStr.find_last_of("\\/");
    if (slashPos != std::string::npos) {
      dirPath = meshPrefixStr.substr(0, slashPos + 1);
    }

    for (const auto &fname : foundFiles) {
      std::string base = fname;
      if (base.length() > 4)
        base.erase(base.length() - 4);

      std::string lowerBase = base;
      std::transform(lowerBase.begin(), lowerBase.end(), lowerBase.begin(),
                     ::tolower);

      std::string relativePath = dirPath + base;
      MESHHANDLE hMesh = oapiLoadMeshGlobal(relativePath.c_str());
      if (hMesh) {
        // bottom extent
        float minY = 0.0f;
        DWORD nGrp = oapiMeshGroupCount(hMesh);
        for (DWORD g = 0; g < nGrp; g++) {
          MESHGROUPEX *grp = oapiMeshGroupEx(hMesh, g);
          if (grp && grp->Vtx) {
            for (DWORD v = 0; v < grp->nVtx; v++) {
              if (grp->Vtx[v].y < minY)
                minY = grp->Vtx[v].y;
            }
          }
        }
        float bottomExtent = -minY; // positive value = distance below origin

        D3D9Mesh *d9m = new D3D9Mesh(hMesh, true);
        if (d9m) {
          // Fallback: If Orbiter's mesh parser failed to parse the TEXTURES block (TextureCount <= 1), manually bind it
          if (d9m->GetTextureCount() <= 1 && d9m->GetGroupCount() > 0) {
            std::string texName = fname;
            if (texName.length() > 4) texName.erase(texName.length() - 4);
            texName += ".dds";
            
            SURFHANDLE hTex = oapiLoadTexture(texName.c_str());
            if (hTex) {
              for (DWORD i = 0; i < d9m->GetGroupCount(); i++) {
                D3D9Mesh::GROUPREC *grp = const_cast<D3D9Mesh::GROUPREC*>(d9m->GetGroup(i));
                if (grp) grp->TexIdx = 0;
              }
              d9m->SetTexture(0, hTex);
            }
          }
          bool isSmall = (lowerBase.find("small") != std::string::npos);
          bool isMedium = (lowerBase.find("medium") != std::string::npos);
          bool isLarge = (lowerBase.find("large") != std::string::npos);

          if (!isSmall && !isMedium && !isLarge) {
            m_meshPool[0].push_back(d9m);
            m_meshBottomExtent[0].push_back(bottomExtent);
            m_meshPool[1].push_back(d9m);
            m_meshBottomExtent[1].push_back(bottomExtent);
            m_meshPool[2].push_back(d9m);
            m_meshBottomExtent[2].push_back(bottomExtent);
          } else {
            if (isSmall) {
              m_meshPool[0].push_back(d9m);
              m_meshBottomExtent[0].push_back(bottomExtent);
            }
            if (isMedium) {
              m_meshPool[1].push_back(d9m);
              m_meshBottomExtent[1].push_back(bottomExtent);
            }
            if (isLarge) {
              m_meshPool[2].push_back(d9m);
              m_meshBottomExtent[2].push_back(bottomExtent);
            }
          }
          LogAlw("RockScatter: Mesh '%s' bottomExtent=%.2f", fname.c_str(),
                 bottomExtent);
        }
      }
    }

    if (!m_meshPool[0].empty() || !m_meshPool[1].empty() ||
        !m_meshPool[2].empty()) {
      // we have custom meshes

      // If any pool is completely empty due to naming, copy from another pool,
      // else we crash
      for (int i = 0; i < 3; i++) {
        if (m_meshPool[i].empty()) {
          for (int j = 0; j < 3; j++) {
            if (!m_meshPool[j].empty()) {
              m_meshPool[i] = m_meshPool[j];
              m_meshBottomExtent[i] = m_meshBottomExtent[j];
              break;
            }
          }
        }
      }
      LogAlw("RockScatter: Loaded %u custom meshes with prefix '%s'",
             foundFiles.size(), cfg.sMeshPrefix);
      LogAlw(
          "RockScatter: m_meshPool sizes -> Small: %u, Medium: %u, Large: %u",
          m_meshPool[0].size(), m_meshPool[1].size(), m_meshPool[2].size());
      return; // Skip procedural generation
    }
  }

  // Small (icosahedron, 0 subdiv), Medium (1 subdiv), Large (2 subdiv)
  int subdiv[] = {0, 1, 2};
  float scales[] = {0.5f, 1.0f, 1.0f};

  // Default material
  MATERIAL mtrl;
  memset(&mtrl, 0, sizeof(mtrl));
  mtrl.diffuse = {0.45f, 0.42f, 0.40f, 1.0f};
  mtrl.ambient = {0.15f, 0.14f, 0.13f, 1.0f};
  mtrl.specular = {0.05f, 0.05f, 0.05f, 1.0f};
  mtrl.emissive = {0.0f, 0.0f, 0.0f, 1.0f};
  mtrl.power = 5.0f;

  for (int i = 0; i < 3; i++) {
    std::vector<NTVERTEX> verts;
    std::vector<WORD> idxs;
    CreateIcosphereMesh(subdiv[i], m_seed + (i + 1) * 7919u, scales[i], verts,
                        idxs);

    if (verts.empty() || idxs.empty())
      continue;

    MESHGROUPEX grpex;
    memset(&grpex, 0, sizeof(grpex));
    grpex.Vtx = verts.data();
    grpex.Idx = idxs.data();
    grpex.nVtx = (DWORD)verts.size();
    grpex.nIdx = (DWORD)idxs.size();
    grpex.MtrlIdx = 0;
    grpex.TexIdx = 0;
    grpex.Flags = 0;
    grpex.UsrFlag = 0;
    grpex.zBias = 0;

    D3D9Mesh *procMesh = new D3D9Mesh(&grpex, &mtrl, NULL);
    m_meshPool[i].push_back(procMesh);
  }
}

RockScatter::RockScatter(vPlanet *planet, LPDIRECT3DDEVICE9 pDev)
    : m_planet(planet), m_pDev(pDev), m_seed(0),
      m_lastDensityMult(*(float*)g_client->GetConfigParam(CFGPRM_ROCKDENSITYMULT)) {
  const char *name = planet->GetName();
  uint32_t h = 5381u;
  if (name)
    for (const char *p = name; *p; p++)
      h = h * 33u + (uint32_t)*p;
  const ::RockScatterCfg *pCfgInit = oapiGetRockScatterCfg(planet->Object());
  h ^= pCfgInit ? pCfgInit->uSeed : 0u;
  m_seed = h ? h : 1u;

  CreateRockMeshes();
  LoadBaseClearZones();
  LogAlw("RockScatter: Initialised for '%s' (seed=%u, drawDist=%.0f m, "
         "density=%.4f, clearZones=%u)",
         name ? name : "?", m_seed,
         pCfgInit ? pCfgInit->fDrawDist : 0.0f,
         pCfgInit ? pCfgInit->fDensity : 0.0f,
         (unsigned)m_clearZones.size());
}

RockScatter::~RockScatter() {
  std::vector<D3D9Mesh *> deleted;
  for (int i = 0; i < 3; i++) {
    for (auto mesh : m_meshPool[i]) {
      if (std::find(deleted.begin(), deleted.end(), mesh) == deleted.end()) {
        delete mesh;
        deleted.push_back(mesh);
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(m_cacheMutex);
    m_cache.clear();
  }
}

// Load clear zones from base config files

void RockScatter::LoadBaseClearZones() {
  m_clearZones.clear();

  OBJHANDLE hPlanet = m_planet->Object();
  if (!hPlanet)
    return;

  DWORD nBases = oapiGetBaseCount(hPlanet);
  if (nBases == 0)
    return;

  double planetRad = m_planet->GetSize();

  for (DWORD b = 0; b < nBases; b++) {
    OBJHANDLE hBase = oapiGetBaseByIndex(hPlanet, b);
    if (!hBase)
      continue;

    // Get base position in radians
    double baseLng = 0.0, baseLat = 0.0;
    oapiGetBaseEquPos(hBase, &baseLng, &baseLat);

    // Open base config file
    const char *cfgFile = oapiGetObjectFileName(hBase);
    if (!cfgFile || !cfgFile[0])
      continue;

    std::ifstream fs(cfgFile);
    if (fs.fail())
      continue;

    int zonesFound = 0;
    std::string line;

    while (std::getline(fs, line) && zonesFound < MAX_CLEAR_AREAS_PER_BASE) {
      // Look for AREA_TO_CLEAR_N = X, Y
      size_t pos = line.find("AREA_TO_CLEAR_");
      if (pos == std::string::npos)
        continue;

      // Find the '=' sign
      size_t eq = line.find('=', pos);
      if (eq == std::string::npos)
        continue;

      // Parse the two values after '='
      std::string values = line.substr(eq + 1);
      // Replace commas with spaces for easier parsing
      for (char &c : values) {
        if (c == ',')
          c = ' ';
      }

      float x = 0.0f, y = 0.0f;
      std::istringstream iss(values);
      if (!(iss >> x >> y))
        continue;

      // Both extents must be positive and non-zero
      if (x <= 0.0f || y <= 0.0f)
        continue;

      ClearZone cz;
      cz.baseLng = baseLng;
      cz.baseLat = baseLat;
      cz.halfExtX = x;
      cz.halfExtY = y;
      m_clearZones.push_back(cz);
      zonesFound++;

      char baseName[64];
      oapiGetObjectName(hBase, baseName, 64);
      LogAlw("RockScatter: Base '%s' clear zone #%d: halfExtX=%.1f m, "
             "halfExtY=%.1f m",
             baseName, zonesFound, x, y);
    }
  }

  if (!m_clearZones.empty()) {
    LogAlw("RockScatter: Loaded %u total clear zones from %u bases on '%s'",
           (unsigned)m_clearZones.size(), nBases, m_planet->GetName());
  }
}

// Check if a surface position falls inside any base clear zone

bool RockScatter::IsInClearZone(double lng, double lat) const {
  if (m_clearZones.empty())
    return false;

  double planetRad = m_planet->GetSize();

  for (const auto &cz : m_clearZones) {
    // Quick angular rejection before expensive trig
    // Max extent in radians (generous upper bound)
    double maxArc = (double)max(cz.halfExtX, cz.halfExtY) / planetRad * 1.5;

    double dLat = lat - cz.baseLat;
    if (fabs(dLat) > maxArc)
      continue;

    double dLng = lng - cz.baseLng;
    while (dLng > PI)
      dLng -= 2.0 * PI;
    while (dLng < -PI)
      dLng += 2.0 * PI;
    if (fabs(dLng) > maxArc)
      continue;

    // Convert angular deltas to surface metres
    double dx = dLng * cos(lat) * planetRad; // east-west (metres)
    double dy = dLat * planetRad;            // north-south (metres)

    // Rectangle test: [-halfExtX, -halfExtY] to [+halfExtX, +halfExtY]
    if (fabs(dx) <= (double)cz.halfExtX && fabs(dy) <= (double)cz.halfExtY) {
      return true;
    }
  }
  return false;
}

// Fetch rock instances from the core engine via OrbiterAPI.
// The core owns the authoritative rock generation; we just convert to D3D9 format.

const std::vector<RockScatter::RockInstance> &
RockScatter::GetRocksForTile(int lvl, int ilat, int ilng) const {
  TileKey key = {lvl, ilat, ilng};
  std::lock_guard<std::mutex> lock(m_cacheMutex);
  auto it = m_cache.find(key);
  if (it != m_cache.end())
    return it->second;

  auto &rocks = m_cache[key];

  // Query the core engine for this tile's rock data
  int nRocks = 0;
  const ::RockInstance *coreRocks =
      oapiGetRockScatterTiles(m_planet->Object(), lvl, ilat, ilng, &nRocks);

  if (!coreRocks || nRocks <= 0)
    return rocks;

  rocks.reserve(nRocks);

  for (int i = 0; i < nRocks; i++) {
    const auto &src = coreRocks[i];
    RockInstance rock;
    rock.localPos =
        D3DXVECTOR3((float)src.localPos.x, (float)src.localPos.y,
                    (float)src.localPos.z);
    rock.elevation = src.elevation;
    rock.scale = src.scale;
    rock.rotY = src.rotY;
    rock.sizeClass = src.sizeClass;

    // Remap meshIndex to the D3D9 mesh pool size (core may have different
    // pool sizes). Clamp to valid range.
    int poolSize = (int)m_meshPool[rock.sizeClass].size();
    if (poolSize > 0)
      rock.meshIndex = src.meshIndex % (uint8_t)poolSize;
    else
      rock.meshIndex = 0;

    rocks.push_back(rock);
  }

  return rocks;
}


// Frame

void RockScatter::Render(LPDIRECT3DDEVICE9 pDev) {
  if (*(bool*)g_client->GetConfigParam(CFGPRM_SURFACEROCKS) == 0)
    return;
  if (m_meshPool[0].empty() && m_meshPool[1].empty() && m_meshPool[2].empty())
    return;

  // Invalidate cache dynamically if density multiplier changed
  if (m_lastDensityMult != *(float*)g_client->GetConfigParam(CFGPRM_ROCKDENSITYMULT)) {
    std::lock_guard<std::mutex> lock(m_cacheMutex);
    m_cache.clear();
    m_lastDensityMult = *(float*)g_client->GetConfigParam(CFGPRM_ROCKDENSITYMULT);
  }

  const ::RockScatterCfg *pCfg = oapiGetRockScatterCfg(m_planet->Object());
  if (!pCfg) return;
  const ::RockScatterCfg &cfg = *pCfg;
  const Scene *scn = m_planet->GetScene();
  if (!scn)
    return;

  // Only render when this planet is the camera proxy
  if (scn->GetCameraProxyVisual() != m_planet)
    return;

  float activeDrawDist = *(float*)g_client->GetConfigParam(CFGPRM_ROCKMAXDIST);
  float fov = (float)scn->GetCameraAperture();

  // Camera and vessel positions
  VECTOR3 camGlob = scn->GetCameraGPos();
  VECTOR3 planGlob = m_planet->GlobalPos();
  VECTOR3 camRel = camGlob - planGlob;

  OBJHANDLE hFocus = oapiGetFocusObject();
  VECTOR3 vesselRel = camRel;
  if (hFocus) {
    VECTOR3 vesselGlob;
    oapiGetGlobalPos(hFocus, &vesselGlob);
    vesselRel = vesselGlob - planGlob;
  }

  // Rotate into planet frame
  MATRIX3 grot;
  oapiGetRotationMatrix(m_planet->Object(), &grot);
  MATRIX3 igrot = {grot.m11, grot.m21, grot.m31, grot.m12, grot.m22,
                   grot.m32, grot.m13, grot.m23, grot.m33};
  VECTOR3 vesselLocal = mul(igrot, vesselRel);

  double planetRad = m_planet->GetSize();
  double vesselAlt = length(vesselLocal) - planetRad;
  if (vesselAlt > activeDrawDist * 3.0f)
    return;

  // If the camera is extremely far away, stop rendering completely. Otherwise
  // we crash.
  double camAlt = length(camRel) - planetRad;
  if (camAlt > activeDrawDist * 10.0f)
    return;

  double vLen = length(vesselLocal);
  double y_norm = vLen > 0.0 ? (vesselLocal.y / vLen) : 0.0;
  if (y_norm > 1.0)
    y_norm = 1.0;
  if (y_norm < -1.0)
    y_norm = -1.0;

  double vesselLat = asin(y_norm);
  double vesselLng = atan2(vesselLocal.z, vesselLocal.x);

  // get tile level
  double tgtTileSize = cfg.fDrawDist * 2.0 / planetRad;
  int lvl = 1;
  while ((PI / double(1 << lvl)) > tgtTileSize && lvl < 15)
    lvl++;
  if (lvl < 4)
    lvl = 4;

  double tileSize = PI / double(1 << lvl);
  int vesselIlat = (int)((PI * 0.5 - vesselLat) / tileSize);
  int vesselIlng = (int)((vesselLng + PI) / tileSize);
  int nLngBands = 1 << (lvl + 1);
  int searchR = max(2, min(25, (int)ceil(*(float*)g_client->GetConfigParam(CFGPRM_ROCKMAXDIST) / cfg.fDrawDist * 0.5)));

  D3D9Sun sunParams = m_planet->GetObjectAtmoParams(camRel);
  float drawDist2 = activeDrawDist * activeDrawDist;

  // per-frame constants
  D3DXVECTOR3 vesselLocalDX((float)vesselLocal.x, (float)vesselLocal.y,
                            (float)vesselLocal.z);

  // Camera forward vector for frustum culling (planet-local frame)
  VECTOR3 camLocalV = mul(igrot, camRel);
  double camLen = length(camLocalV);
  D3DXVECTOR3 camFwd;
  if (camLen > 1.0) {
    // Camera looks toward planet center from orbit, but for surface-level
    // the camera forward is roughly -(camera position normalized)
    // We use the actual view direction from the oapi
    VECTOR3 camDir;
    oapiCameraGlobalDir(&camDir);
    VECTOR3 camDirLocal = mul(igrot, camDir);
    camFwd = D3DXVECTOR3((float)camDirLocal.x, (float)camDirLocal.y,
                         (float)camDirLocal.z);
    D3DXVec3Normalize(&camFwd, &camFwd);
  } else {
    camFwd = D3DXVECTOR3(0, 1, 0);
  }

  // Set cull mode ONCE before the entire rock loop
  m_pDev->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);

  // Collect and batch rocks by (sizeClass, meshIndex)
  //  We use a simple approach: iterate all mesh variants, for each one
  //  set up the batch, then iterate tiles and draw matching rocks.
  //  This minimises state changes to one Begin/End per mesh variant.

  struct BatchKey {
    uint8_t sizeClass;
    uint8_t meshIndex;
  };

  // Build a flat list of all unique mesh variants in use
  // (at most 3 size classes × N meshes per class, typically < 10 total)
  struct MeshBatch {
    uint8_t sizeClass;
    uint8_t meshIndex;
    D3D9Mesh *mesh;
  };

  std::vector<MeshBatch> batches;
  batches.reserve(16);
  for (int sc = 0; sc < 3; sc++) {
    for (uint8_t mi = 0; mi < (uint8_t)m_meshPool[sc].size(); mi++) {
      MeshBatch b;
      b.sizeClass = (uint8_t)sc;
      b.meshIndex = mi;
      b.mesh = m_meshPool[sc][mi];
      batches.push_back(b);
    }
  }

  int renderedRocks = 0;

  for (const auto &batch : batches) {
    if (!batch.mesh)
      continue;

    // Begin batch for this mesh variant - sets up ALL shader state once
    batch.mesh->RenderBatchBegin(&sunParams);

    std::vector<D3DXMATRIX> debugMatrices;
    bool showColliders = (Config->bShowRockColliders == 1) || (DebugControls::IsActive() && (*(DWORD*)g_client->GetConfigParam(CFGPRM_GETDEBUGFLAGS) & 0x0004));

    for (int dlat = -searchR; dlat <= searchR; dlat++) {
      int tilat = vesselIlat + dlat;
      if (tilat < 0 || tilat >= (1 << lvl))
        continue;

      for (int dlng = -searchR; dlng <= searchR; dlng++) {
        int tilng = (vesselIlng + dlng) % nLngBands;
        if (tilng < 0)
          tilng += nLngBands;

        const auto &rocks = GetRocksForTile(lvl, tilat, tilng);

        for (const auto &rock : rocks) {
          // Skip rocks not belonging to this batch
          if (rock.sizeClass != batch.sizeClass ||
              rock.meshIndex != batch.meshIndex)
            continue;

          // World position
          float bottomOfs = 0.0f;
          if (rock.meshIndex < m_meshBottomExtent[rock.sizeClass].size())
            bottomOfs = m_meshBottomExtent[rock.sizeClass][rock.meshIndex];
            
          double rockAlt = planetRad + (double)rock.elevation + (double)bottomOfs * rock.scale;
          VECTOR3 rGeo = _V(rock.localPos.x * rockAlt, rock.localPos.y * rockAlt, rock.localPos.z * rockAlt);

          // Distance from vessel (not camera) for culling
          D3DXVECTOR3 rockGeoFloat((float)rGeo.x, (float)rGeo.y, (float)rGeo.z);
          D3DXVECTOR3 diff = rockGeoFloat - vesselLocalDX;
          float dist2 = D3DXVec3LengthSq(&diff);

          // LOD sub-cull by size class
          float lodCull = GetLodCull(rock.sizeClass);
          float classDist2 = drawDist2 * lodCull * lodCull;
          if (dist2 > classDist2)
            continue;

          // skip rocks behind camera
          D3DXVECTOR3 rockCamLocal =
              rockGeoFloat - D3DXVECTOR3((float)camLocalV.x, (float)camLocalV.y,
                                    (float)camLocalV.z);
          float dotFwd = D3DXVec3Dot(&rockCamLocal, &camFwd);
          // Skip rocks more than 10m behind the camera (generous margin)
          if (dotFwd < -10.0f)
            continue;

          // culling
          float cdist = D3DXVec3Length(&rockCamLocal);
          if (cdist > activeDrawDist * 2.5f)
            continue;

          float scr_size = (rock.scale * 5.0f / max(1.0f, cdist)) / fov *
                           1080.0f; // assuming 5m base mesh size
          if (scr_size < 1.0f)
            continue;

          // Build world matrix - localPos is already unit-length, skip Norm3
          D3DXVECTOR3 up = rock.localPos;
          D3DXVECTOR3 right, fwd;

          if (fabsf(up.y) < 0.99f)
            right = D3DXVECTOR3(-up.z, 0, up.x);
          else
            right = D3DXVECTOR3(1, 0, 0);
          // Normalize right (was implicitly done by Norm3 before but whatever)
          D3DXVec3Normalize(&right, &right);
          D3DXVec3Cross(&fwd, &up, &right);

          // Apply Y rotation
          float cy = cosf(rock.rotY), sy = sinf(rock.rotY);
          D3DXVECTOR3 rr = right * cy + fwd * sy;
          D3DXVECTOR3 ff = fwd * cy - right * sy;

          // Rotate axes into world frame
          VECTOR3 wRight = mul(grot, _V(rr.x, rr.y, rr.z));
          VECTOR3 wUp = mul(grot, _V(up.x, up.y, up.z));
          VECTOR3 wFwd = mul(grot, _V(ff.x, ff.y, ff.z));

          // Camera-relative position, double precision
          VECTOR3 rGeoRot = mul(grot, rGeo);
          VECTOR3 rockCam = rGeoRot - camRel;

          float s = rock.scale;
          D3DXMATRIX mWorld(
              (float)wRight.x * s, (float)wRight.y * s, (float)wRight.z * s, 0,
              (float)wUp.x * s, (float)wUp.y * s, (float)wUp.z * s, 0,
              (float)wFwd.x * s, (float)wFwd.y * s, (float)wFwd.z * s, 0,
              (float)rockCam.x, (float)rockCam.y, (float)rockCam.z, 1);

          if (std::isnan(mWorld._41) || std::isnan(mWorld._42) ||
              std::isnan(mWorld._43) || std::isinf(mWorld._41) ||
              std::isinf(mWorld._42) || std::isinf(mWorld._43))
            continue;

          // Only update world matrix + commit + draw
          batch.mesh->RenderBatchInstance(&mWorld);
          renderedRocks++;

          if (showColliders) {
            debugMatrices.push_back(mWorld);
          }
        }
      }
    }

    batch.mesh->RenderBatchEnd();

    if (!debugMatrices.empty()) {
      for (auto &m : debugMatrices) {
        batch.mesh->RenderWireframe(&m, D3DCOLOR_RGBA(50, 255, 50, 100));
      }
    }
  }
}

void RockScatter::RenderShadows(LPDIRECT3DDEVICE9 pDev, float alpha) {
  if (*(bool*)g_client->GetConfigParam(CFGPRM_SURFACEROCKS) == 0)
    return;
  if (m_meshPool[0].empty() && m_meshPool[1].empty() && m_meshPool[2].empty())
    return;

  const ::RockScatterCfg *pCfg = oapiGetRockScatterCfg(m_planet->Object());
  if (!pCfg) return;
  const ::RockScatterCfg &cfg = *pCfg;
  const Scene *scn = m_planet->GetScene();
  if (!scn)
    return;

  if (scn->GetCameraProxyVisual() != m_planet)
    return;

  float activeDrawDist = *(float*)g_client->GetConfigParam(CFGPRM_ROCKMAXDIST);
  float fov = (float)scn->GetCameraAperture();

  VECTOR3 camGlob = scn->GetCameraGPos();
  VECTOR3 planGlob = m_planet->GlobalPos();
  VECTOR3 camRel = camGlob - planGlob;

  OBJHANDLE hFocus = oapiGetFocusObject();
  VECTOR3 vesselRel = camRel;
  if (hFocus) {
    VECTOR3 vesselGlob;
    oapiGetGlobalPos(hFocus, &vesselGlob);
    vesselRel = vesselGlob - planGlob;
  }

  MATRIX3 grot;
  oapiGetRotationMatrix(m_planet->Object(), &grot);
  MATRIX3 igrot = {grot.m11, grot.m21, grot.m31, grot.m12, grot.m22,
                   grot.m32, grot.m13, grot.m23, grot.m33};
  VECTOR3 vesselLocal = mul(igrot, vesselRel);

  VECTOR3 sd;
  oapiGetGlobalPos(m_planet->Object(), &sd);
  normalise(sd);
  D3DXVECTOR3 lsun = D3DXVEC(tmul(grot, sd));

  double planetRad = m_planet->GetSize();
  double vesselAlt = length(vesselLocal) - planetRad;
  if (vesselAlt > activeDrawDist * 3.0f)
    return;

  VECTOR3 camLocalV = mul(igrot, camRel);
  double camAlt = length(camLocalV) - planetRad;
  if (camAlt > activeDrawDist * 20.0f)
    return;

  double vLen = length(vesselLocal);
  double y_norm = vLen > 0.0 ? (vesselLocal.y / vLen) : 0.0;
  if (y_norm > 1.0)
    y_norm = 1.0;
  if (y_norm < -1.0)
    y_norm = -1.0;

  double vesselLat = asin(y_norm);
  double vesselLng = atan2(vesselLocal.z, vesselLocal.x);

  double tgtTileSize = cfg.fDrawDist * 2.0 / planetRad;
  int lvl = 1;
  while ((PI / double(1 << lvl)) > tgtTileSize && lvl < 15)
    lvl++;
  if (lvl < 4)
    lvl = 4;
  double tileSize = PI / double(1 << lvl);

  int vesselIlat = (int)((PI * 0.5 - vesselLat) / tileSize);
  int vesselIlng = (int)((vesselLng + PI) / tileSize);
  int nLngBands = 1 << (lvl + 1);
  int searchR = max(2, min(25, (int)ceil(*(float*)g_client->GetConfigParam(CFGPRM_ROCKMAXDIST) / cfg.fDrawDist * 0.5)));

  float drawDist2 = activeDrawDist * activeDrawDist;
  D3DXVECTOR4 param = D9OffsetRange(planetRad, 30e3);

  // Hoisted per-frame constant
  D3DXVECTOR3 vesselLocalDX((float)vesselLocal.x, (float)vesselLocal.y,
                            (float)vesselLocal.z);

  pDev->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);

  // batch by mesh variant
  struct MeshBatch {
    uint8_t sizeClass;
    uint8_t meshIndex;
    D3D9Mesh *mesh;
  };

  std::vector<MeshBatch> batches;
  batches.reserve(16);
  for (int sc = 0; sc < 3; sc++) {
    for (uint8_t mi = 0; mi < (uint8_t)m_meshPool[sc].size(); mi++) {
      MeshBatch b;
      b.sizeClass = (uint8_t)sc;
      b.meshIndex = mi;
      b.mesh = m_meshPool[sc][mi];
      batches.push_back(b);
    }
  }

  int renderedRocks = 0;

  for (const auto &batch : batches) {
    if (!batch.mesh)
      continue;

    batch.mesh->RenderShadowBatchBegin(&param);

    for (int dlat = -searchR; dlat <= searchR; dlat++) {
      int tilat = vesselIlat + dlat;
      if (tilat < 0 || tilat >= (1 << lvl))
        continue;

      for (int dlng = -searchR; dlng <= searchR; dlng++) {
        int tilng = (vesselIlng + dlng) % nLngBands;
        if (tilng < 0)
          tilng += nLngBands;
        const auto &rocks = GetRocksForTile(lvl, tilat, tilng);

        for (const auto &rock : rocks) {
          if (rock.sizeClass != batch.sizeClass ||
              rock.meshIndex != batch.meshIndex)
            continue;

          float bottomOfs = 0.0f;
          if (rock.meshIndex < m_meshBottomExtent[rock.sizeClass].size())
            bottomOfs = m_meshBottomExtent[rock.sizeClass][rock.meshIndex];

          double rockAlt = planetRad + (double)rock.elevation + (double)bottomOfs * rock.scale;
          VECTOR3 rGeo = _V(rock.localPos.x * rockAlt, rock.localPos.y * rockAlt, rock.localPos.z * rockAlt);

          // Distance cull (hoisted vesselLocalDX)
          D3DXVECTOR3 rockGeoFloat((float)rGeo.x, (float)rGeo.y, (float)rGeo.z);
          D3DXVECTOR3 diff = rockGeoFloat - vesselLocalDX;
          if (D3DXVec3LengthSq(&diff) > drawDist2 * GetLodCull(rock.sizeClass) *
                                            GetLodCull(rock.sizeClass))
            continue;

          // localPos is already unit-length - skip Norm3
          D3DXVECTOR3 up = rock.localPos;

          float nd = D3DXVec3Dot(&up, &lsun);
          if (nd > -0.01f)
            continue;

          // sub-pixel and max distance culling for shadows
          D3DXVECTOR3 rockCamLocal =
              rockGeoFloat - D3DXVECTOR3((float)camLocalV.x, (float)camLocalV.y,
                                    (float)camLocalV.z);
          float cdist = D3DXVec3Length(&rockCamLocal);
          if (cdist > activeDrawDist * 2.5f)
            continue;

          float scr_size =
              (rock.scale * 5.0f / max(1.0f, cdist)) / fov * 1080.0f;
          if (scr_size < 1.0f)
            continue;

          D3DXVECTOR3 right, fwd;
          if (fabsf(up.y) < 0.99f)
            right = D3DXVECTOR3(-up.z, 0, up.x);
          else
            right = D3DXVECTOR3(1, 0, 0);
          D3DXVec3Normalize(&right, &right);
          D3DXVec3Cross(&fwd, &up, &right);

          float cy = cosf(rock.rotY), sy = sinf(rock.rotY);
          D3DXVECTOR3 rr = right * cy + fwd * sy;
          D3DXVECTOR3 ff = fwd * cy - right * sy;

          VECTOR3 wRight = mul(grot, _V(rr.x, rr.y, rr.z));
          VECTOR3 wUp = mul(grot, _V(up.x, up.y, up.z));
          VECTOR3 wFwd = mul(grot, _V(ff.x, ff.y, ff.z));

          VECTOR3 rGeoRot = mul(grot, rGeo);
          VECTOR3 rockCam = rGeoRot - camRel;

          float s = rock.scale;
          D3DXMATRIX mWorld(
              (float)wRight.x * s, (float)wRight.y * s, (float)wRight.z * s, 0,
              (float)wUp.x * s, (float)wUp.y * s, (float)wUp.z * s, 0,
              (float)wFwd.x * s, (float)wFwd.y * s, (float)wFwd.z * s, 0,
              (float)rockCam.x, (float)rockCam.y, (float)rockCam.z, 1);

          if (std::isnan(mWorld._41) || std::isnan(mWorld._42) ||
              std::isnan(mWorld._43) || std::isinf(mWorld._41) ||
              std::isinf(mWorld._42) || std::isinf(mWorld._43))
            continue;

          D3DXVECTOR3 rockLsun;
          rockLsun.x = D3DXVec3Dot(&lsun, &rr);
          rockLsun.y = nd;
          rockLsun.z = D3DXVec3Dot(&lsun, &ff);

          float zo = bottomOfs;
          float ofs = zo / nd;

          D3DXMATRIX mProj;
          D3DXMatrixIdentity(&mProj);
          mProj._21 = -rockLsun.x / nd;
          mProj._22 = 0.0f;
          mProj._23 = -rockLsun.z / nd;
          mProj._41 = -rockLsun.x * ofs;
          mProj._42 = -bottomOfs;
          mProj._43 = -rockLsun.z * ofs;

          D3DXVECTOR4 nrml(0, 0, 0, 1.0f);

          float scale = (-nd - 0.07f) * 25.0f;
          scale = (1.0f - alpha) * max(0.0f, min(1.0f, scale));

          batch.mesh->RenderShadowBatchInstance(scale, &mProj, &mWorld, &nrml,
                                                &param);
          renderedRocks++;
        }
      }
    }

    batch.mesh->RenderShadowBatchEnd();
  }
}

