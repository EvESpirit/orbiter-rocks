// ==============================================================
// RockScatter.h
// Part of the ORBITER VISUALISATION PROJECT (OVP)
// Dual licensed under GPL v3 and LGPL v3
// Copyright (c) EvESpirit
// ==========================================================================================

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


  // Generate (or look up cached) rocks for one tile
  const std::vector<RockInstance> &GetRocksForTile(int lvl, int ilat,
                                                   int ilng) const;



  // Build procedural icosphere-based rock meshes
  void CreateRockMeshes();
  void CreateIcosphereMesh(int subdivisions, UINT seed, float baseScale,
                           std::vector<NTVERTEX> &outVerts,
                           std::vector<WORD> &outIdxs);

  // Deterministic hash / helpers

  static uint32_t XorShift32(uint32_t &state);
  static float RandFloat(uint32_t &state); // [0, 1)


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




};

#endif // !__ROCKSCATTER_H
