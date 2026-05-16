// ==============================================================
// RockScatter.h
// Part of the ORBITER core engine
// Dual licensed under GPL v3 and LGPL v3
// ==============================================================
//
// Core rock scatter system - procedurally generates deterministic
// rock fields on planetary surfaces and provides collision geometry.
// Rendering is handled separately by the graphics client.
// ==============================================================

#ifndef __ROCKSCATTER_H
#define __ROCKSCATTER_H

#include "OrbiterAPI.h"
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>
#include "ZTreeMgr.h"

#define MAX_CLEAR_AREAS_PER_BASE 32

class Planet;

class RockScatter {
public:
  RockScatter(Planet *planet);
  ~RockScatter();

  /// Get the config for this planet's rock scatter
  const RockScatterCfg& GetConfig() const { return m_cfg; }

  /// Get rocks for a specific tile (cached)
  const std::vector<RockInstance>& GetRocksForTile(int lvl, int ilat, int ilng) const;

  /// Get the additional elevation caused by rocks at a given lng/lat
  double GetElevationModifier(double lng, double lat) const;

  /// Collision result from mesh-to-mesh check
  struct CollisionResult {
    bool hit;
    VECTOR3 normal;
    double depth;
    VECTOR3 contactPtLocal;
  };

  /// Check hull points against all nearby rocks for collision
  CollisionResult CheckCollision(const VECTOR3 *hullPtsLocal, int nPts,
                                 const VECTOR3 &vesselPosLocal,
                                 double vesselRadius,
                                 float maxCollisionDist) const;

  /// Get collision geometry for a given size class and mesh index
  struct CollisionGeom {
    struct Tri {
      VECTOR3 v0, v1, v2;
    };
    std::vector<Tri> tris;
    float maxRadius;
  };

  const std::vector<CollisionGeom>& GetCollisionGeom(int sizeClass) const {
    return m_collGeom[sizeClass];
  }
  const std::vector<float>& GetBottomExtents(int sizeClass) const {
    return m_meshBottomExtent[sizeClass];
  }

public:
  struct RockTileData {
    uint8_t *data;
    uint8_t *originalData;
    int width, height;
    int bpp;
    uint32_t rMask, gMask, bMask, aMask;
    bool isDXT1;
    bool isDXT5;
    int lvl_loaded, ilat_loaded, ilng_loaded;

    RockTileData() : data(nullptr), originalData(nullptr), width(0), height(0), bpp(0), rMask(0), gMask(0), bMask(0), aMask(0), isDXT1(false), isDXT5(false), lvl_loaded(0), ilat_loaded(0), ilng_loaded(0) {}
  };

private:
  struct TileKey {
    int lvl, ilat, ilng;
    bool operator==(const TileKey &o) const {
      return lvl == o.lvl && ilat == o.ilat && ilng == o.ilng;
    }
  };



  struct TileKeyHash {
    size_t operator()(const TileKey &k) const {
      return (size_t)k.lvl * 73856093u ^ (size_t)k.ilat * 19349663u ^
             (size_t)k.ilng * 83492791u;
    }
  };

  struct ClearZone {
    double baseLng;
    double baseLat;
    float halfExtX;
    float halfExtY;
  };

  void LoadBaseClearZones();
  bool IsInClearZone(double lng, double lat) const;
  void BuildCollisionGeometry();
  void LoadRockMapTile(int lvl, int ilat, int ilng, RockTileData &tileData) const;

  static uint32_t HashTile(uint32_t seed, int lvl, int ilat, int ilng);
  static uint32_t XorShift32(uint32_t &state);
  static float RandFloat(uint32_t &state);
  static float RandRange(uint32_t &state, float lo, float hi);

  static VECTOR3 Norm3(const VECTOR3 &v);

  Planet *m_planet;
  RockScatterCfg m_cfg;

  std::vector<float> m_meshBottomExtent[3];
  std::vector<CollisionGeom> m_collGeom[3];
  int m_meshCount[3]; // number of mesh variants per size class

  mutable std::mutex m_cacheMutex;
  mutable std::unordered_map<TileKey, std::vector<RockInstance>, TileKeyHash> m_cache;
  mutable std::unordered_map<TileKey, RockTileData, TileKeyHash> m_mapCache;

  ZTreeMgr *m_rockTreeMgr;
  bool m_bRockDirExists;

  uint32_t m_seed;
  std::vector<ClearZone> m_clearZones;

  static float RaycastMeshY(const CollisionGeom &geom, float localX, float localZ);
};

#endif // !__ROCKSCATTER_H
