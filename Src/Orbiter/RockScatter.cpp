// ==============================================================
// RockScatter.cpp - Core engine rock scatter (data + collision only)
// ==============================================================

#include "RockScatter.h"
#include "Planet.h"
#include "Orbiter.h"
#include "Log.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <map>

extern Orbiter *g_pOrbiter;

// ---- PRNG helpers ----

uint32_t RockScatter::HashTile(uint32_t seed, int lvl, int ilat, int ilng) {
  uint32_t h = seed;
  h ^= (uint32_t)lvl * 2654435761u;
  h ^= (uint32_t)ilat * 2246822519u;
  h ^= (uint32_t)ilng * 3266489917u;
  h ^= h >> 16; h *= 0x85ebca6bu;
  h ^= h >> 13; h *= 0xc2b2ae35u;
  h ^= h >> 16;
  return h ? h : 1u;
}

uint32_t RockScatter::XorShift32(uint32_t &state) {
  state ^= state << 13; state ^= state >> 17; state ^= state << 5;
  return state;
}

float RockScatter::RandFloat(uint32_t &state) {
  return float(XorShift32(state) & 0x00FFFFFFu) / float(0x01000000u);
}

float RockScatter::RandRange(uint32_t &state, float lo, float hi) {
  return lo + RandFloat(state) * (hi - lo);
}

VECTOR3 RockScatter::Norm3(const VECTOR3 &v) {
  double len = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
  if (len < 1e-12) return _V(0,1,0);
  return _V(v.x/len, v.y/len, v.z/len);
}

// Collision geometry from loaded meshes

void RockScatter::BuildCollisionGeometry() {
  for (int i = 0; i < 3; i++) {
    m_collGeom[i].clear();
    m_meshBottomExtent[i].clear();
    m_meshCount[i] = 0;
  }

  if (m_cfg.sMeshPrefix[0] == '\0') {
    // Build simple icosphere collision geometry procedurally for each size class
    for (int sc = 0; sc < 3; sc++) {
      CollisionGeom cg;
      cg.maxRadius = 0.0f;

      // Simple 8-triangle octahedron as collision hull
      float s = (sc == 0) ? 0.5f : 1.0f;
      VECTOR3 top = _V(0, s*1.2, 0), bot = _V(0, -s*0.2, 0);
      VECTOR3 pts[4] = {_V(s,s*0.5,0), _V(0,s*0.5,s), _V(-s,s*0.5,0), _V(0,s*0.5,-s)};

      for (int i = 0; i < 4; i++) {
        int j = (i+1)%4;
        CollisionGeom::Tri t1 = {top, pts[i], pts[j]};
        CollisionGeom::Tri t2 = {bot, pts[j], pts[i]};
        cg.tris.push_back(t1);
        cg.tris.push_back(t2);
        for (auto *p : {&pts[i], &pts[j], &top, &bot}) {
          float r = (float)sqrt(p->x*p->x + p->y*p->y + p->z*p->z);
          if (r > cg.maxRadius) cg.maxRadius = r;
        }
      }

      m_collGeom[sc].push_back(cg);
      m_meshBottomExtent[sc].push_back(s * 0.2f);
      m_meshCount[sc] = 1;
    }
    return;
  }

  // Load collision geometry from mesh files
  char exePath[MAX_PATH];
  GetModuleFileNameA(NULL, exePath, MAX_PATH);
  char *lastSlash = strrchr(exePath, '\\');
  if (lastSlash) *(lastSlash + 1) = '\0';

  char searchPath[MAX_PATH];
  sprintf_s(searchPath, sizeof(searchPath), "%sMeshes\\%s*.msh", exePath, m_cfg.sMeshPrefix);

  WIN32_FIND_DATAA fd;
  std::vector<std::string> foundFiles;
  HANDLE hFind = FindFirstFileA(searchPath, &fd);
  if (hFind != INVALID_HANDLE_VALUE) {
    do { foundFiles.push_back(fd.cFileName); } while (FindNextFileA(hFind, &fd));
    FindClose(hFind);
  }
  std::sort(foundFiles.begin(), foundFiles.end());

  std::string dirPath = "";
  std::string prefix(m_cfg.sMeshPrefix);
  size_t slashPos = prefix.find_last_of("\\/");
  if (slashPos != std::string::npos) dirPath = prefix.substr(0, slashPos + 1);

  for (const auto &fname : foundFiles) {
    std::string base = fname;
    if (base.length() > 4) base.erase(base.length() - 4);

    std::string lowerBase = base;
    std::transform(lowerBase.begin(), lowerBase.end(), lowerBase.begin(), ::tolower);

    std::string relativePath = dirPath + base;
    MESHHANDLE hMesh = oapiLoadMeshGlobal(relativePath.c_str());
    if (!hMesh) continue;

    float minY = 0.0f;
    DWORD nGrp = oapiMeshGroupCount(hMesh);
    for (DWORD g = 0; g < nGrp; g++) {
      MESHGROUPEX *grp = oapiMeshGroupEx(hMesh, g);
      if (grp && grp->Vtx) {
        for (DWORD v = 0; v < grp->nVtx; v++)
          if (grp->Vtx[v].y < minY) minY = grp->Vtx[v].y;
      }
    }
    float bottomExtent = -minY;

    CollisionGeom cg;
    cg.maxRadius = 0.0f;
    for (DWORD gi = 0; gi < nGrp; gi++) {
      MESHGROUPEX *grp = oapiMeshGroupEx(hMesh, gi);
      if (!grp || !grp->Vtx || !grp->Idx) continue;
      for (DWORD t = 0; t + 2 < grp->nIdx; t += 3) {
        CollisionGeom::Tri tri;
        WORD i0 = grp->Idx[t], i1 = grp->Idx[t+1], i2 = grp->Idx[t+2];
        tri.v0 = _V(grp->Vtx[i0].x, grp->Vtx[i0].y, grp->Vtx[i0].z);
        tri.v1 = _V(grp->Vtx[i1].x, grp->Vtx[i1].y, grp->Vtx[i1].z);
        tri.v2 = _V(grp->Vtx[i2].x, grp->Vtx[i2].y, grp->Vtx[i2].z);
        cg.tris.push_back(tri);
        for (auto *vv : {&tri.v0, &tri.v1, &tri.v2}) {
          float r = (float)length(*vv);
          if (r > cg.maxRadius) cg.maxRadius = r;
        }
      }
    }

    bool isSmall  = (lowerBase.find("small")  != std::string::npos);
    bool isMedium = (lowerBase.find("medium") != std::string::npos);
    bool isLarge  = (lowerBase.find("large")  != std::string::npos);

    if (!isSmall && !isMedium && !isLarge) {
      for (int i = 0; i < 3; i++) {
        m_collGeom[i].push_back(cg);
        m_meshBottomExtent[i].push_back(bottomExtent);
      }
    } else {
      if (isSmall)  { m_collGeom[0].push_back(cg); m_meshBottomExtent[0].push_back(bottomExtent); }
      if (isMedium) { m_collGeom[1].push_back(cg); m_meshBottomExtent[1].push_back(bottomExtent); }
      if (isLarge)  { m_collGeom[2].push_back(cg); m_meshBottomExtent[2].push_back(bottomExtent); }
    }
  }

  // Fill empty pools from non-empty ones
  for (int i = 0; i < 3; i++) {
    if (m_collGeom[i].empty()) {
      for (int j = 0; j < 3; j++) {
        if (!m_collGeom[j].empty()) {
          m_collGeom[i] = m_collGeom[j];
          m_meshBottomExtent[i] = m_meshBottomExtent[j];
          break;
        }
      }
    }
    m_meshCount[i] = (int)m_collGeom[i].size();
  }
}

// Constructor / Destructor

RockScatter::RockScatter(Planet *planet)
    : m_planet(planet), m_seed(0) {
  memset(&m_cfg, 0, sizeof(m_cfg));
  m_meshCount[0] = m_meshCount[1] = m_meshCount[2] = 0;

  // Read config from planet - it must have been parsed already
  // (Planet stores it in RockCfg)
  memcpy(&m_cfg, &planet->RockCfg, sizeof(RockScatterCfg));

  const char *name = planet->Name();
  uint32_t h = 5381u;
  if (name)
    for (const char *p = name; *p; p++)
      h = h * 33u + (uint32_t)*p;
  h ^= m_cfg.uSeed;
  m_seed = h ? h : 1u;

  BuildCollisionGeometry();
  LoadBaseClearZones();
}

RockScatter::~RockScatter() {
  std::lock_guard<std::mutex> lock(m_cacheMutex);
  m_cache.clear();
}

// Base clear zones

void RockScatter::LoadBaseClearZones() {
  m_clearZones.clear();
  OBJHANDLE hPlanet = (OBJHANDLE)m_planet;
  if (!hPlanet) return;

  DWORD nBases = oapiGetBaseCount(hPlanet);
  if (nBases == 0) return;

  for (DWORD b = 0; b < nBases; b++) {
    OBJHANDLE hBase = oapiGetBaseByIndex(hPlanet, b);
    if (!hBase) continue;

    double baseLng = 0.0, baseLat = 0.0;
    oapiGetBaseEquPos(hBase, &baseLng, &baseLat);

    const char *cfgFile = oapiGetObjectFileName(hBase);
    if (!cfgFile || !cfgFile[0]) continue;

    std::ifstream fs(cfgFile);
    if (fs.fail()) continue;

    int zonesFound = 0;
    std::string line;
    while (std::getline(fs, line) && zonesFound < MAX_CLEAR_AREAS_PER_BASE) {
      if (line.find("AREA_TO_CLEAR_") == std::string::npos) continue;
      size_t eq = line.find('=');
      if (eq == std::string::npos) continue;
      std::string values = line.substr(eq + 1);
      for (char &c : values) if (c == ',') c = ' ';

      float x = 0.0f, y = 0.0f;
      std::istringstream iss(values);
      if (!(iss >> x >> y)) continue;
      if (x <= 0.0f || y <= 0.0f) continue;

      ClearZone cz;
      cz.baseLng = baseLng; cz.baseLat = baseLat;
      cz.halfExtX = x; cz.halfExtY = y;
      m_clearZones.push_back(cz);
      zonesFound++;
    }
  }
}

bool RockScatter::IsInClearZone(double lng, double lat) const {
  if (m_clearZones.empty()) return false;
  double planetRad = m_planet->Size();

  for (const auto &cz : m_clearZones) {
    double maxArc = (double)std::max(cz.halfExtX, cz.halfExtY) / planetRad * 1.5;
    double dLat = lat - cz.baseLat;
    if (fabs(dLat) > maxArc) continue;
    double dLng = lng - cz.baseLng;
    while (dLng > PI) dLng -= 2.0 * PI;
    while (dLng < -PI) dLng += 2.0 * PI;
    if (fabs(dLng) > maxArc) continue;
    double dx = dLng * cos(lat) * planetRad;
    double dy = dLat * planetRad;
    if (fabs(dx) <= (double)cz.halfExtX && fabs(dy) <= (double)cz.halfExtY)
      return true;
  }
  return false;
}

// Rock generation

const std::vector<RockInstance>&
RockScatter::GetRocksForTile(int lvl, int ilat, int ilng) const {
  TileKey key = {lvl, ilat, ilng};
  std::lock_guard<std::mutex> lock(m_cacheMutex);
  auto it = m_cache.find(key);
  if (it != m_cache.end()) return it->second;

  auto &rocks = m_cache[key];

  double tileSize = PI / double(1 << lvl);
  double latMin = PI * 0.5 - tileSize * (ilat + 1);
  double latMax = PI * 0.5 - tileSize * ilat;
  double lngMin = -PI + tileSize * ilng;

  double planetRad = m_planet->Size();
  double midLat = (latMin + latMax) * 0.5;
  double areaM2 = (tileSize * planetRad) * (tileSize * cos(midLat) * planetRad);
  if (areaM2 < 1.0) return rocks;

  if (!m_cfg.bEnabled) return rocks;
  if (!g_pOrbiter->Cfg()->CfgVisualPrm.bSurfaceRocks) return rocks;
  
  float densityMult = g_pOrbiter->Cfg()->CfgVisualPrm.fRockDensityMult;
  if (densityMult <= 0.0f) return rocks;
  
  float density = m_cfg.fDensity * densityMult;

  int nRocks = (int)(density * areaM2);
  if (nRocks > 2000) nRocks = 2000;
  if (nRocks <= 0) return rocks;

  uint32_t rng = HashTile(m_seed, lvl, ilat, ilng);
  rocks.reserve(nRocks);

  for (int i = 0; i < nRocks; i++) {
    RockInstance rock;
    double lat = latMin + RandFloat(rng) * (latMax - latMin);
    double lng = lngMin + RandFloat(rng) * tileSize;

    double clat = cos(lat), slat = sin(lat), clng = cos(lng), slng = sin(lng);
    rock.localPos = _V(clat * clng, slat, clat * slng);
    rock.elevation = (float)oapiSurfaceElevation((OBJHANDLE)m_planet, lng, lat);

    float r = RandFloat(rng);
    if (r < m_cfg.fRatioSmall) {
      rock.sizeClass = 0;
      rock.scale = RandRange(rng, m_cfg.fSizeSmall[0], m_cfg.fSizeSmall[1]);
    } else if (r < m_cfg.fRatioSmall + m_cfg.fRatioMedium) {
      rock.sizeClass = 1;
      rock.scale = RandRange(rng, m_cfg.fSizeMedium[0], m_cfg.fSizeMedium[1]);
    } else {
      rock.sizeClass = 2;
      rock.scale = RandRange(rng, m_cfg.fSizeLarge[0], m_cfg.fSizeLarge[1]);
    }

    int poolSize = m_meshCount[rock.sizeClass];
    if (poolSize <= 0) poolSize = 1;
    rock.meshIndex = (uint8_t)(RandFloat(rng) * poolSize);
    if (rock.meshIndex >= poolSize) rock.meshIndex = poolSize - 1;
    rock.rotY = RandFloat(rng) * 6.283185f;

    if (!m_clearZones.empty()) {
      double rockLat = asin(rock.localPos.y);
      double rockLng = atan2(rock.localPos.z, rock.localPos.x);
      if (IsInClearZone(rockLng, rockLat)) continue;
    }
    rocks.push_back(rock);
  }
  return rocks;
}

// Raycast

float RockScatter::RaycastMeshY(const CollisionGeom &geom, float lx, float lz) {
  float bestY = -1e30f;
  for (const auto &tri : geom.tris) {
    float ax = (float)tri.v0.x, az = (float)tri.v0.z;
    float bx = (float)tri.v1.x, bz = (float)tri.v1.z;
    float cx = (float)tri.v2.x, cz = (float)tri.v2.z;
    float d00 = (bx-ax)*(bx-ax) + (bz-az)*(bz-az);
    float d01 = (bx-ax)*(cx-ax) + (bz-az)*(cz-az);
    float d11 = (cx-ax)*(cx-ax) + (cz-az)*(cz-az);
    float d20 = (lx-ax)*(bx-ax) + (lz-az)*(bz-az);
    float d21 = (lx-ax)*(cx-ax) + (lz-az)*(cz-az);
    float denom = d00*d11 - d01*d01;
    if (fabs(denom) < 1e-12f) continue;
    float v = (d11*d20 - d01*d21) / denom;
    float w = (d00*d21 - d01*d20) / denom;
    float u = 1.0f - v - w;
    if (u >= -0.01f && v >= -0.01f && w >= -0.01f) {
      float y = (float)(u*tri.v0.y + v*tri.v1.y + w*tri.v2.y);
      if (y > bestY) bestY = y;
    }
  }
  return bestY;
}

// Elevation modifier

double RockScatter::GetElevationModifier(double lng, double lat) const {
  if (!m_cfg.bEnabled) return 0.0;

  double planetRad = m_planet->Size();
  double tgtTileSize = m_cfg.fDrawDist * 2.0 / planetRad;
  int lvl = 1;
  while ((PI / double(1 << lvl)) > tgtTileSize && lvl < 15) lvl++;
  if (lvl < 4) lvl = 4;

  double tileSize = PI / double(1 << lvl);
  int centerIlat = (int)((PI * 0.5 - lat) / tileSize);
  int centerIlng = (int)((lng + PI) / tileSize);
  int nLngBands = 1 << (lvl + 1);

  double maxElev = 0.0;
  for (int dlat = -1; dlat <= 1; dlat++) {
    int tilat = centerIlat + dlat;
    if (tilat < 0 || tilat >= (1 << lvl)) continue;
    for (int dlng = -1; dlng <= 1; dlng++) {
      int tilng = (centerIlng + dlng + nLngBands) % nLngBands;
      const auto &rocks = GetRocksForTile(lvl, tilat, tilng);
      for (const auto &rock : rocks) {
        if (rock.meshIndex >= (int)m_collGeom[rock.sizeClass].size()) continue;
        const CollisionGeom &cg = m_collGeom[rock.sizeClass][rock.meshIndex];
        double boundingR = (double)rock.scale * cg.maxRadius;

        float bottomOfs = 0.0f;
        if (rock.meshIndex < (int)m_meshBottomExtent[rock.sizeClass].size())
          bottomOfs = m_meshBottomExtent[rock.sizeClass][rock.meshIndex];

        double rockLat = asin(rock.localPos.y);
        double rockLng = atan2(rock.localPos.z, rock.localPos.x);
        double dLat = lat - rockLat, dLng = lng - rockLng;
        while (dLng > PI) dLng -= 2.0*PI;
        while (dLng < -PI) dLng += 2.0*PI;
        double dx_m = dLng * cos(lat) * planetRad;
        double dy_m = dLat * planetRad;
        if (dx_m*dx_m + dy_m*dy_m > boundingR*boundingR) continue;

        float cosR = cosf(-rock.rotY), sinR = sinf(-rock.rotY);
        float localX = (float)((dx_m*cosR + dy_m*sinR) / rock.scale);
        float localZ = (float)((-dx_m*sinR + dy_m*cosR) / rock.scale);
        float meshY = RaycastMeshY(cg, localX, localZ);
        if (meshY > -1e20f) {
          double h = ((double)meshY + (double)bottomOfs) * rock.scale;
          if (h > 0.0 && h > maxElev) maxElev = h;
        }
      }
    }
  }
  return maxElev;
}

// Collisions

RockScatter::CollisionResult
RockScatter::CheckCollision(const VECTOR3 *hullPts, int nPts,
                            const VECTOR3 &vesselPosLocal, double vesselRadius,
                            float maxCollisionDist) const {
  CollisionResult result = {false, {0,0,0}, 0.0, {0,0,0}};
  if (nPts <= 0 || !hullPts || !m_cfg.bEnabled) return result;

  double planetRad = oapiGetSize((OBJHANDLE)m_planet);
  if (planetRad < 1.0) return result;

  double vLen = length(vesselPosLocal);
  if (vLen < 1.0) return result;
  double vAlt = vLen - planetRad;
  if (vAlt > 500.0) return result;

  double vLat = asin(vesselPosLocal.y / vLen);
  double vLng = atan2(vesselPosLocal.z, vesselPosLocal.x);

  float drawDist = m_cfg.fDrawDist;
  int lvl = (int)(log2(PI / (drawDist / planetRad)));
  if (lvl < 1) lvl = 1;
  if (lvl > 19) lvl = 19;

  int nLat = 1 << lvl, nLng = 2 * nLat;
  int centerIlat = (int)((PI*0.5 - vLat) * nLat / PI);
  int centerIlng = (int)((vLng + PI) * nLng / (2.0*PI));
  centerIlat = std::max(0, std::min(centerIlat, nLat-1));
  centerIlng = ((centerIlng % nLng) + nLng) % nLng;

  double maxCollDist2 = (double)maxCollisionDist * (double)maxCollisionDist;
  double deepestPen = 0.0;
  int searchR = 2;

  for (int dLat = -searchR; dLat <= searchR; dLat++) {
    int tilat = centerIlat + dLat;
    if (tilat < 0 || tilat >= nLat) continue;
    for (int dLng = -searchR; dLng <= searchR; dLng++) {
      int tilng = ((centerIlng + dLng) % nLng + nLng) % nLng;
      const auto &rocks = GetRocksForTile(lvl, tilat, tilng);

      for (const auto &rock : rocks) {
        if (rock.meshIndex >= (int)m_collGeom[rock.sizeClass].size()) continue;
        const CollisionGeom &cg = m_collGeom[rock.sizeClass][rock.meshIndex];

        float bottomOfs = 0.0f;
        if (rock.meshIndex < (int)m_meshBottomExtent[rock.sizeClass].size())
          bottomOfs = m_meshBottomExtent[rock.sizeClass][rock.meshIndex];

        double rockAlt = planetRad + (double)rock.elevation + (double)bottomOfs * rock.scale;
        double rockWX = rock.localPos.x * rockAlt;
        double rockWY = rock.localPos.y * rockAlt;
        double rockWZ = rock.localPos.z * rockAlt;

        double dx = vesselPosLocal.x - rockWX;
        double dy = vesselPosLocal.y - rockWY;
        double dz = vesselPosLocal.z - rockWZ;
        double dist2Center = dx*dx + dy*dy + dz*dz;
        if (dist2Center > maxCollDist2) continue;

        double distCenter = sqrt(dist2Center);
        double rockBoundR = (double)rock.scale * cg.maxRadius;
        if (distCenter > vesselRadius + rockBoundR + 5.0) continue;

        VECTOR3 up = Norm3(rock.localPos);
        VECTOR3 right, fwd;
        if (fabs(up.y) < 0.99)
          right = Norm3(_V(-up.z, 0, up.x));
        else
          right = _V(1, 0, 0);
        fwd = crossp(up, right);

        float cy = cosf(rock.rotY), sy = sinf(rock.rotY);
        VECTOR3 rr = right*cy + fwd*sy;
        VECTOR3 ff = fwd*cy - right*sy;
        float invScale = 1.0f / rock.scale;

        for (int p = 0; p < nPts; p++) {
          double hx = hullPts[p].x - rockWX;
          double hy = hullPts[p].y - rockWY;
          double hz = hullPts[p].z - rockWZ;
          double ptDist2 = hx*hx + hy*hy + hz*hz;
          if (ptDist2 > rockBoundR*rockBoundR) continue;

          float localX = (float)(hx*rr.x + hy*rr.y + hz*rr.z) * invScale;
          float localY = (float)(hx*up.x + hy*up.y + hz*up.z) * invScale;
          float localZ = (float)(hx*ff.x + hy*ff.y + hz*ff.z) * invScale;

          float meshY = RaycastMeshY(cg, localX, localZ);
          if (meshY <= -1e20f) continue;
          if (localY < -bottomOfs) continue;

          if (localY < meshY) {
            double pen = ((double)meshY - (double)localY) * rock.scale;
            if (pen > deepestPen) {
              deepestPen = pen;
              result.hit = true;
              result.contactPtLocal = hullPts[p];
              if (distCenter > 1e-6) {
                result.normal = _V(dx/distCenter, dy/distCenter, dz/distCenter);
              } else {
                result.normal = up;
              }
              double sphereOverlap = rockBoundR - sqrt(ptDist2);
              result.depth = (sphereOverlap > 0) ? std::min(pen, sphereOverlap) : std::min(pen, 0.01);
            }
          }
        }
      }
    }
  }
  return result;
}
