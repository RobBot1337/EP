#define _USE_MATH_DEFINES
#include "points.h"
#include "air_space_data.h"
#include "vec3D.h"
#include "littlealgcpy.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>
#include <iostream>

double euclideanDistance(const Vec3D& a, const Vec3D& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool pointOnSegment(const Vec3D& p, const Vec3D& a, const Vec3D& b,
                    double epsilon) {
  double cross = (p.x - a.x) * (b.y - a.y) - (p.y - a.y) * (b.x - a.x);
  if (std::abs(cross) > epsilon) return false;

  double dot = (p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y);
  if (dot < -epsilon) return false;

  double squaredLength = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
  if (dot > squaredLength + epsilon) return false;

  return true;
}

double orientation(const Vec3D& p, const Vec3D& q, const Vec3D& r) {
  return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
}

bool segmentsIntersect(const Vec3D& a1, const Vec3D& a2, const Vec3D& b1,
                       const Vec3D& b2, double epsilon) {
  double o1 = orientation(a1, a2, b1);
  double o2 = orientation(a1, a2, b2);
  double o3 = orientation(b1, b2, a1);
  double o4 = orientation(b1, b2, a2);

  if (o1 * o2 < -epsilon && o3 * o4 < -epsilon) return true;

  if (std::abs(o1) < epsilon && pointOnSegment(b1, a1, a2, epsilon)) return true;
  if (std::abs(o2) < epsilon && pointOnSegment(b2, a1, a2, epsilon)) return true;
  if (std::abs(o3) < epsilon && pointOnSegment(a1, b1, b2, epsilon)) return true;
  if (std::abs(o4) < epsilon && pointOnSegment(a2, b1, b2, epsilon)) return true;

  return false;
}

double distanceToSegment(const Vec3D& p, const Vec3D& a, const Vec3D& b) {
  double ax = p.x - a.x;
  double ay = p.y - a.y;
  double bx = b.x - a.x;
  double by = b.y - a.y;

  double dot = ax * bx + ay * by;
  double len2 = bx * bx + by * by;

  if (len2 < 1e-9) return euclideanDistance(p, a);

  double t = dot / len2;
  t = std::max(0.0, std::min(1.0, t));

  double projX = a.x + t * bx;
  double projY = a.y + t * by;

  Vec3D projPoint(projX, projY, 0);
  return euclideanDistance(p, projPoint);
}

bool segmentIntersectsCircle(const Vec3D& p1, const Vec3D& p2,
                             const Vec3D& center, double radius,
                             double epsilon) {
  double dist = distanceToSegment(center, p1, p2);

  if (dist > radius + epsilon) return false;

  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;

  double fx = center.x - p1.x;
  double fy = center.y - p1.y;

  double a = dx * dx + dy * dy;
  if (a < epsilon) return false;

  double b = 2 * (fx * dx + fy * dy);
  double c = (fx * fx + fy * fy) - radius * radius;

  double discriminant = b * b - 4 * a * c;

  if (discriminant < -epsilon) return false;
  if (std::abs(discriminant) < epsilon) discriminant = 0;

  double sqrtDisc = std::sqrt(discriminant);
  double t1 = (-b - sqrtDisc) / (2 * a);
  double t2 = (-b + sqrtDisc) / (2 * a);

  return (t1 >= -epsilon && t1 <= 1 + epsilon) ||
         (t2 >= -epsilon && t2 <= 1 + epsilon);
}

void rightPolygon(HighReliefZone& pol){
    AirSpace airspace;
    airspace.points = pol.vertices;
    float* distanceMatrix = createDistanceMatrix(airspace);
    std::vector<int> result = LittleAlg(distanceMatrix, pol.vertices.size());

    QVector<Vec3D> sortedVertices;
    sortedVertices.reserve(result.size());

    for (int index : result) {
        if (index >= 0 && index < pol.vertices.size()) {
            sortedVertices.push_back(pol.vertices[index]);
        }
    }

    pol.vertices = sortedVertices;
    delete[] distanceMatrix;
}

bool segmentIntersectsPolygon(const Vec3D& p1, const Vec3D& p2, const HighReliefZone& pol, double epsilon){
    QVector<Vec3D> points = pol.vertices;
    if(points.size() < 3) return false;

    for(int i = 0; i < points.size()-1; ++i){
        if(segmentsIntersect(p1, p2, points[i], points[i+1], epsilon)){
            return true;
        }
    }
    if(segmentsIntersect(p1, p2, points[0], points[points.size()-1], epsilon)){
        return true;
    }
    return false;
}

QVector<Vec3D> grahamScan(QVector<Vec3D> points) {
    if (points.size() < 3) return points;
    Vec3D p0 = points[0];
    for (const auto& p : points) {
        if (p.y < p0.y || (p.y == p0.y && p.x < p0.x)) {
            p0 = p;
        }
    }
    points.erase(std::remove(points.begin(), points.end(), p0), points.end());

    std::sort(points.begin(), points.end(), [p0](const Vec3D& a, const Vec3D& b) {
        double cross = (a.x - p0.x) * (b.y - p0.y) - (a.y - p0.y) * (b.x - p0.x);
        if (cross != 0) return cross > 0;
        return euclideanDistance(p0, a) < euclideanDistance(p0, b);
    });

    QVector<Vec3D> hull;
    hull.push_back(p0);
    hull.push_back(points[0]);

    for (int i = 1; i < points.size(); ++i) {
        while (hull.size() >= 2) {
            Vec3D a = hull[hull.size() - 2];
            Vec3D b = hull[hull.size() - 1];
            Vec3D c = points[i];
            double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
            if (cross <= 0)
                hull.pop_back();
            else
                break;
        }
        hull.push_back(points[i]);
    }

    return hull;
}

void grahamScanOnZone(HighReliefZone& pol) {
    QVector<Vec3D> hullVertices = grahamScan(pol.vertices);
    pol.vertices = hullVertices;
}

std::pair<Vec3D, Vec3D> getTangentPoints(const Vec3D& point,
                                         const Vec3D& center, double radius) {
  double dx = point.x - center.x;
  double dy = point.y - center.y;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist <= radius) {
    Vec3D p1(point.x, point.y, 0);
    Vec3D p2(point.x, point.y, 0);
    return {p1, p2};
  }

  double angle = std::atan2(dy, dx);
  double deltaAngle = std::acos(radius / dist);

  Vec3D tangent1, tangent2;
  tangent1.x = center.x + radius * std::cos(angle + deltaAngle);
  tangent1.y = center.y + radius * std::sin(angle + deltaAngle);
  tangent1.z = 0;

  tangent2.x = center.x + radius * std::cos(angle - deltaAngle);
  tangent2.y = center.y + radius * std::sin(angle - deltaAngle);
  tangent2.z = 0;

  return {tangent1, tangent2};
}

double calculateArcLength(const Vec3D& center, double radius,
                          const Vec3D& point1, const Vec3D& point2) {
  double angle1 = std::atan2(point1.y - center.y, point1.x - center.x);
  double angle2 = std::atan2(point2.y - center.y, point2.x - center.x);

  double deltaAngle = std::abs(angle2 - angle1);
  deltaAngle = std::min(deltaAngle, 2 * M_PI - deltaAngle);

  return radius * deltaAngle;
}

IntersectionInfo findIntersectingPVO(const Vec3D& p1, const Vec3D& p2,
                                     const AirSpace& airspace) {
  for (int i = 0; i < airspace.pvo_list.size(); ++i) {
    const auto& pvo = airspace.pvo_list[i];
    if (segmentIntersectsCircle(p1, p2, pvo.position, pvo.radius)) {
      return {i, &pvo, true};
    }
  }
  return {-1, nullptr, false};
}

IntersectionInfoPolygon findIntersectingPolygon(const Vec3D& p1, const Vec3D& p2,
                                                 const AirSpace& airspace,
                                                 double epsilon) {
    for (int i = 0; i < airspace.high_relief_zones.size(); ++i) {
        const auto& zone = airspace.high_relief_zones[i];
        if (segmentIntersectsPolygon(p1, p2, zone, epsilon)) {
            return IntersectionInfoPolygon(i, &zone, true);
        }
    }
    return IntersectionInfoPolygon(-1, nullptr, false);
}

BypassPath calculateCircularBypass(const Vec3D& start, const Vec3D& end,
                                   const Vec3D& center, double radius) {
    BypassPath result;

    auto [t1a, t1b] = getTangentPoints(start, center, radius);
    auto [t2a, t2b] = getTangentPoints(end, center, radius);

    double path1 = euclideanDistance(start, t1a) +
                   calculateArcLength(center, radius, t1a, t2a) +
                   euclideanDistance(t2a, end);

    double path2 = euclideanDistance(start, t1a) +
                   calculateArcLength(center, radius, t1a, t2b) +
                   euclideanDistance(t2b, end);

    double path3 = euclideanDistance(start, t1b) +
                   calculateArcLength(center, radius, t1b, t2a) +
                   euclideanDistance(t2a, end);

    double path4 = euclideanDistance(start, t1b) +
                   calculateArcLength(center, radius, t1b, t2b) +
                   euclideanDistance(t2b, end);

    if(path1 <= path2 && path1 <= path3 && path1 <= path4) {
        result.waypoints = {start, t1a, t2a, end};
        result.length = path1;
    } else if(path2 <= path1 && path2 <= path3 && path2 <= path4) {
        result.waypoints = {start, t1a, t2b, end};
        result.length = path2;
    } else if(path3 <= path1 && path3 <= path2 && path3 <= path4) {
        result.waypoints = {start, t1b, t2a, end};
        result.length = path3;
    } else {
        result.waypoints = {start, t1b, t2b, end};
        result.length = path4;
    }

    return result;
}

BypassPath calculatePolygonBypass(const Vec3D& start, const Vec3D& end,
                                   const HighReliefZone& zone) {
    BypassPath result;
    QVector<Vec3D> vertices = zone.vertices;
    int n = vertices.size();

    if (n < 2) {
        result.waypoints = {start, end};
        result.length = euclideanDistance(start, end);
        return result;
    }

    double bestLength = std::numeric_limits<double>::max();
    QVector<Vec3D> bestPath;

    // Пробуем все возможные комбинации точек входа и выхода
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            // Путь по часовой стрелке от i до j
            QVector<Vec3D> pathCw;
            pathCw.append(start);
            pathCw.append(vertices[i]);

            int current = i;
            while (current != j) {
                current = (current + 1) % n;
                pathCw.append(vertices[current]);
                if (current == j) break;
            }
            pathCw.append(end);

            // Путь против часовой стрелки от i до j
            QVector<Vec3D> pathCcw;
            pathCcw.append(start);
            pathCcw.append(vertices[i]);

            current = i;
            while (current != j) {
                current = (current - 1 + n) % n;
                pathCcw.append(vertices[current]);
                if (current == j) break;
            }
            pathCcw.append(end);

            // Вычисляем длины и проверяем, что путь не пересекает многоугольник
            double lenCw = 0;
            bool validCw = true;
            for (int k = 0; k < pathCw.size() - 1; ++k) {
                // Проверяем, что отрезок пути не пересекает многоугольник (кроме как в вершинах)
                for (int m = 0; m < n; ++m) {
                    int next = (m + 1) % n;
                    if (segmentsIntersect(pathCw[k], pathCw[k + 1], vertices[m], vertices[next]) &&
                        pathCw[k + 1] != vertices[m] && pathCw[k] != vertices[next] &&
                        pathCw[k + 1] != vertices[next] && pathCw[k] != vertices[m]) {
                        validCw = false;
                        break;
                    }
                }
                if (!validCw) break;
                lenCw += euclideanDistance(pathCw[k], pathCw[k + 1]);
            }

            double lenCcw = 0;
            bool validCcw = true;
            for (int k = 0; k < pathCcw.size() - 1; ++k) {
                for (int m = 0; m < n; ++m) {
                    int next = (m + 1) % n;
                    if (segmentsIntersect(pathCcw[k], pathCcw[k + 1], vertices[m], vertices[next]) &&
                        pathCcw[k + 1] != vertices[m] && pathCcw[k] != vertices[next] &&
                        pathCcw[k + 1] != vertices[next] && pathCcw[k] != vertices[m]) {
                        validCcw = false;
                        break;
                    }
                }
                if (!validCcw) break;
                lenCcw += euclideanDistance(pathCcw[k], pathCcw[k + 1]);
            }

            if (validCw && lenCw < bestLength) {
                bestLength = lenCw;
                bestPath = pathCw;
            }
            if (validCcw && lenCcw < bestLength) {
                bestLength = lenCcw;
                bestPath = pathCcw;
            }
        }
    }

    result.waypoints = bestPath;
    result.length = bestLength;
    return result;
}

double calculateDistanceWithPVO(const Vec3D& p1, const Vec3D& p2,
                                const AirSpace& airspace) {
  auto polyInfo = findIntersectingPolygon(p1, p2, airspace);

  if (polyInfo.intersects && polyInfo.zone) {
    auto bypass = calculatePolygonBypass(p1, p2, *polyInfo.zone);
    return bypass.length;
  }

  auto [pvoIndex, pvo, intersects] = findIntersectingPVO(p1, p2, airspace);

  if (intersects && pvo) {
    auto bypass = calculateCircularBypass(p1, p2, pvo->position, pvo->radius);
    return bypass.length;
  }

  return euclideanDistance(p1, p2);
}

float* createDistanceMatrix(const AirSpace& airspace) {
  int n = airspace.points.size();
  float* matrix = new float[n * n];

  for (int i{0}; i < n; ++i) {
    for (int j{0}; j < n; ++j) {
      if (i == j) {
        matrix[i * n + j] = INFINITY_F;
        continue;
      }

      double distance = calculateDistanceWithPVO(airspace.points[i],
                                                 airspace.points[j], airspace);
      matrix[i * n + j] = static_cast<float>(distance);
    }
  }

  return matrix;
}

void deleteDistanceMatrix(float* matrix) { delete[] matrix; }
