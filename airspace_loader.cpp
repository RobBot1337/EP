#include "airspace_loader.h"
#include "air_space_data.h"
#include <QDebug>

AirSpace AirspaceLoader::LoadFrom(const std::string& path) {
  QFile file(QString::fromStdString(path));
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return AirSpace{};
  }

  QByteArray json_data = file.readAll();
  file.close();

  QJsonParseError parser_error;
  QJsonDocument document = QJsonDocument::fromJson(json_data, &parser_error);

  if (parser_error.error != QJsonParseError::NoError) {
    qDebug() << "JSON parse error:" << parser_error.errorString();
    return AirSpace{};
  }

  if (!document.isArray()) {
    qDebug() << "JSON root is not an array";
    return AirSpace{};
  }

  QJsonArray rootArray = document.array();
  return _ConstructAirspace(rootArray);
}

AirSpace AirspaceLoader::_ConstructAirspace(QJsonArray& arr) {
  AirSpace result;

  if (arr.size() < 4) {
    qDebug() << "Invalid airspace data: expected 4 arrays, got" << arr.size();
    return result;
  }

  QJsonArray pointsArray = arr[0].toArray();
  for (const auto& item : pointsArray) {
    QJsonObject obj = item.toObject();
    Vec3D point;
    point.x = obj["x"].toDouble();
    point.y = obj["y"].toDouble();
    point.z = 0;
    result.points.append(point);
  }

  QJsonArray corridorsArray = arr[1].toArray();
  for (const auto& item : corridorsArray) {
    QJsonObject obj = item.toObject();
    BlockedAirCorridor corridor;
    corridor.id1 = obj["id1"].toInteger();
    corridor.id2 = obj["id2"].toInteger();
    result.air_corridors.insert(corridor);
  }

  QJsonArray pvo_array = arr[2].toArray();
  for (const auto& item : pvo_array) {
    QJsonObject obj = item.toObject();
    Pvo pvo;
    pvo.id = obj["id"].toInteger();
    pvo.position.x = obj["x"].toDouble();
    pvo.position.y = obj["y"].toDouble();
    pvo.position.z = 0;
    pvo.radius = obj["r"].toInteger();
    result.pvo_list.append(pvo);
  }

  QJsonArray mountainsArray = arr[3].toArray();
  for (const auto& mountainItem : mountainsArray) {
    QJsonArray high_relief_zone_points = mountainItem.toArray();
    HighReliefZone zone;

    for (const auto& point_item : high_relief_zone_points) {
      QJsonObject obj = point_item.toObject();
      Vec3D point;
      point.x = obj["x"].toDouble();
      point.y = obj["y"].toDouble();
      point.z = 0;
      zone.vertices.append(point);
    }

    result.high_relief_zones.append(zone);
  }

  return result;
}
