// ==============================================================
// RockScatter.h
// Part of the ORBITER VISUALISATION PROJECT (OVP)
// Dual licensed under GPL v3 and LGPL v3
// ==============================================================

// ==============================================================
// Procedural surface rock scatter system.
// Generates deterministic fields of small, medium and large rocks
// on planetary surfaces that support the feature.  Every tile
// always produces the exact same rock field given the same seed.
// Potentially also works for vegetation and shrubbery.
// ==============================================================

#ifndef __ROCKSCATTER_H
#define __ROCKSCATTER_H

#include "D3D9Client.h"
#include "D3D9Util.h"
#include "Mesh.h"
#include <mutex>
#include <unordered_map>
#include <vector>

// Maximum number of AREA_TO_CLEAR entries per base
#define MAX_CLEAR_AREAS_PER_BASE 32

class vPlanet;

// RockScatterCfg is now defined in OrbiterAPI.h (core engine).
// The D3D9 client uses the core's definition via oapiGetRockScatterCfg().

// Scatter renderer

class RockScatter {
public:
  RockScatter(vPlanet *planet, LPDIRECT3DDEVICE9 pDev);
  ~RockScatter();

  /// Called once per frame from vPlanet::Render().
  void Render(LPDIRECT3DDEVICE9 pDev);

  /// Called from vPlanet::RenderBaseShadows().
  void RenderShadows(LPDIRECT3DDEVICE9 pDev, float alpha);



private:
  // One rock instance inside a tile
  struct RockInstance {
    D3DXVECTOR3 localPos; // Position on unit sphere (geocentric direction)
    float elevation;      // Surface elevation at the point (metres)
    float scale;          // Size multiplier
    float rotY;           // Y-axis rotation in radians
    uint8_t sizeClass;    // 0 = small, 1 = medium, 2 = large
    uint8_t
        meshIndex; // Index of the specific mesh inside m_meshPool[sizeClass]
  };

  // Tile key for the instance cache
  struct TileKey {
    int lvl, ilat, ilng;
    bool operator==(const TileKey &o) const {
      return lvl == o.lvl && ilat == o.ilat && ilng == o.ilng;
    }
  };

  // Hash functor for TileKey (used by unordered_map)
  struct TileKeyHash {
    size_t operator()(const TileKey &k) const {
      size_t h = (size_t)k.lvl * 73856093u ^ (size_t)k.ilat * 19349663u ^
                 (size_t)k.ilng * 83492791u;
      return h;
    }
  };

  // Rectangular exclusion zone around a surface base
  struct ClearZone {
    double baseLng; // Base longitude (radians)
    double baseLat; // Base latitude (radians)
    float halfExtX; // Half-extent in east-west direction (metres)
    float halfExtY; // Half-extent in north-south direction (metres)
  };

  // Generate (or look up cached) rocks for one tile
  const std::vector<RockInstance> &GetRocksForTile(int lvl, int ilat,
                                                   int ilng) const;

  // Load AREA_TO_CLEAR definitions from all base config files on this planet
  void LoadBaseClearZones();

  // Check whether a given position (radians) falls inside any clear zone
  bool IsInClearZone(double lng, double lat) const;

  // Build procedural icosphere-based rock meshes
  void CreateRockMeshes();
  void CreateIcosphereMesh(int subdivisions, UINT seed, float baseScale,
                           std::vector<NTVERTEX> &outVerts,
                           std::vector<WORD> &outIdxs);

  // Deterministic hash / helpers
  static uint32_t HashTile(uint32_t seed, int lvl, int ilat, int ilng);
  static uint32_t XorShift32(uint32_t &state);
  static float RandFloat(uint32_t &state); // [0, 1)
  static float RandRange(uint32_t &state, float lo, float hi);

  // Owner planet
  vPlanet *m_planet;
  LPDIRECT3DDEVICE9 m_pDev;

  // Meshes used for the 3 size classes (0=Small, 1=Medium, 2=Large)
  std::vector<D3D9Mesh *> m_meshPool[3];
  std::vector<float> m_meshBottomExtent[3];

  // Cached per-tile rock lists
  mutable std::mutex m_cacheMutex;
  mutable std::unordered_map<TileKey, std::vector<RockInstance>, TileKeyHash>
      m_cache;

  // Derived seed (computed once from planet name + config seed)
  uint32_t m_seed;

  float m_lastDensityMult;

  // Base clear zones loaded from all surface base configs on this planet
  std::vector<ClearZone> m_clearZones;


};

#endif // !__ROCKSCATTER_H
