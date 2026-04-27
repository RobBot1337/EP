#ifndef AIR_SPACE_DATA_H
#define AIR_SPACE_DATA_H

#include <QVector>
#include <QHash>
#include <QSet>

#include "vec3D.h"

struct BlockedAirCorridor {
  std::size_t id1{0};
  std::size_t id2{0};

  bool operator==(const BlockedAirCorridor& other) const {
    return (id1 == other.id1 && id2 == other.id2) ||
           (id1 == other.id2 && id2 == other.id1);
  }
};

struct Pvo {
  std::size_t id{0};
  Vec3D position{0, 0, 0};
  unsigned int radius{0};
};

struct HighReliefZone {
  std::size_t id{0};
  QVector<Vec3D> vertices;
};

struct AirSpace {
  QVector<Vec3D> points;
  QSet<BlockedAirCorridor> air_corridors;
  QVector<Pvo> pvo_list;
  QVector<HighReliefZone> high_relief_zones;
};

inline unsigned int qHash(const BlockedAirCorridor& key, uint seed = 0) {
  std::size_t id_a = std::min(key.id1, key.id2);
  std::size_t id_b = std::max(key.id1, key.id2);
  return qHash(std::make_pair(id_a, id_b), seed);
}

#endif
