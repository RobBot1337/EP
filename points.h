#ifndef POINTS_H
#define POINTS_H

#include "air_space_data.h"
#include "vec3D.h"
#include <QVector>
#include <QSet>
#include <vector>
#include <utility>
#include <limits>

const float INFINITY_F = std::numeric_limits<float>::infinity();

double euclideanDistance(const Vec3D& a, const Vec3D& b);
bool pointOnSegment(const Vec3D& p, const Vec3D& a, const Vec3D& b, double epsilon = 1e-9);
bool segmentsIntersect(const Vec3D& a1, const Vec3D& a2, const Vec3D& b1, const Vec3D& b2, double epsilon = 1e-9);
double distanceToSegment(const Vec3D& p, const Vec3D& a, const Vec3D& b);
bool segmentIntersectsCircle(const Vec3D& p1, const Vec3D& p2, const Vec3D& center, double radius, double epsilon = 1e-9);
std::pair<Vec3D, Vec3D> getTangentPoints(const Vec3D& point, const Vec3D& center, double radius);
double calculateArcLength(const Vec3D& center, double radius, const Vec3D& point1, const Vec3D& point2);

void rightPolygon(HighReliefZone& pol);
bool segmentIntersectsPolygon(const Vec3D& p1, const Vec3D& p2, const HighReliefZone& pol, double epsilon = 1e-9);
QVector<Vec3D> grahamScan(QVector<Vec3D> points);
void grahamScanOnZone(HighReliefZone& pol);

struct IntersectionInfo {
    int pvoIndex{-1};
    const Pvo* pvo{nullptr};
    bool intersects{false};
};

struct IntersectionInfoPolygon {
    int zoneIndex;
    const HighReliefZone* zone;
    bool intersects;

    IntersectionInfoPolygon() : zoneIndex(-1), zone(nullptr), intersects(false) {}
    IntersectionInfoPolygon(int idx, const HighReliefZone* z, bool inter)
        : zoneIndex(idx), zone(z), intersects(inter) {}
};

struct BypassPath {
    QVector<Vec3D> waypoints;
    double length{0.0};
};

IntersectionInfo findIntersectingPVO(const Vec3D& p1, const Vec3D& p2, const AirSpace& airspace);
IntersectionInfoPolygon findIntersectingPolygon(const Vec3D& p1, const Vec3D& p2, const AirSpace& airspace, double epsilon = 1e-9);
BypassPath calculateCircularBypass(const Vec3D& start, const Vec3D& end, const Vec3D& center, double radius);
BypassPath calculatePolygonBypass(const Vec3D& start, const Vec3D& end, const HighReliefZone& zone);
double calculateDistanceWithPVO(const Vec3D& p1, const Vec3D& p2, const AirSpace& airspace);

float* createDistanceMatrix(const AirSpace& airspace, int numSalesmen);
void deleteDistanceMatrix(float* matrix);
QVector<QVector<int>> splitTourIntoTours(const std::vector<int>& fullTour, int originalSize, int numSalesmen);
std::vector<std::vector<int>> solveGreedyMTSP(const AirSpace& airspace, int numSalesmen);

#endif
