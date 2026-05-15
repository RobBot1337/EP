#ifndef AIRSPACE_LOADER_H
#define AIRSPACE_LOADER_H

#include "air_space_data.h"  // Добавь этот файл, где определены AirSpace, BlockedAirCorridor, Pvo, HighReliefZone
#include "vec3D.h"
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QDebug>
#include <string>

class AirspaceLoader {
public:
  static AirSpace LoadFrom(const std::string& path);

private:
  static AirSpace _ConstructAirspace(QJsonArray& arr);
};

#endif
