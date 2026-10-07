#include "map.hpp"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "esp_err.h"
#include "esp_log.h"

#include "context/GlobalData.hpp"
#include "data_types.hpp"
#include "env.hpp"
#include "storage/storage.hpp"
#include "tasks/cli/cli.hpp"

namespace cli_map {

namespace {
const char *TAG = "cli_map";
} // namespace

// Campos do ponto, na ordem do protocolo (map_add e map_get):
// t, encoder_left, encoder_right, v, omega, x, y, theta, sector, right_marks,
// speed
bool parseMapAddBodyFields(const wire::WireView &view, MapPoint *point) {
  if(view.payloadArgc() < 11 || point == nullptr) {
    return false;
  }
  int t          = 0;
  int left       = 0;
  int right      = 0;
  int sector     = 0;
  int rightMarks = 0;
  if(!wire::parseInt(view.arg(0), t) || !wire::parseInt(view.arg(1), left) ||
     !wire::parseInt(view.arg(2), right) ||
     !wire::parseFloat(view.arg(3), point->v) ||
     !wire::parseFloat(view.arg(4), point->omega) ||
     !wire::parseFloat(view.arg(5), point->x) ||
     !wire::parseFloat(view.arg(6), point->y) ||
     !wire::parseFloat(view.arg(7), point->theta) ||
     !wire::parseInt(view.arg(8), sector) ||
     !wire::parseInt(view.arg(9), rightMarks) ||
     !wire::parseFloat(view.arg(10), point->speed)) {
    return false;
  }
  point->t            = static_cast<uint32_t>(t);
  point->encoderLeft  = static_cast<int32_t>(left);
  point->encoderRight = static_cast<int32_t>(right);
  point->sector       = static_cast<uint16_t>(sector);
  point->rightMarks   = static_cast<uint8_t>(rightMarks);
  return true;
}

bool wireMapAddBody(const wire::WireView &view, CliProtocol &proto,
                    bool sortAfter) {
  MapPoint point;
  if(!parseMapAddBodyFields(view, &point)) {
    ESP_LOGW(TAG, "map_add body: need 11 fields after idx "
                  "(t,encoder_left,encoder_right,v,omega,x,y,theta,sector,"
                  "right_marks,speed)");
    return false;
  }
  if(globalData.mapData.size() >= static_cast<size_t>(MAP_POINT_MAX_COUNT)) {
    ESP_LOGW(TAG, "map_add: map full (%d)", MAP_POINT_MAX_COUNT);
    if(sortAfter) {
      return proto.emitSingleResponse("map_add", {"error", "map full"});
    }
    return false;
  }
  globalData.mapData.push_back(point);
  if(sortAfter) {
    std::sort(globalData.mapData.begin(), globalData.mapData.end(),
              [](const MapPoint &a, const MapPoint &b) {
                return mapPointProgress(a) < mapPointProgress(b);
              });
    return proto.emitSingleResponse("map_add", {"ok"});
  }
  return true;
}

bool wireMapClear(CliProtocol &proto) {
  globalData.mapData.clear();
  return proto.emitSingleResponse("map_clear", {"ok"});
}

bool wireMapClearStorage(CliProtocol &proto) {
  Storage              *storage = Storage::getInstance();
  std::vector<MapPoint> emptyMap;
  esp_err_t             ret = storage->write_vector(emptyMap, MAP_STORAGE_FILE);
  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "map_clear_storage failed (%s)", esp_err_to_name(ret));
    return proto.emitSingleResponse("map_clear_storage",
                                    {"error", "Failed to clear Flash"});
  }
  return proto.emitSingleResponse("map_clear_storage", {"ok"});
}

bool wireMapSave(CliProtocol &proto) {
  Storage  *storage = Storage::getInstance();
  esp_err_t ret = storage->write_vector(globalData.mapData, MAP_STORAGE_FILE);
  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "map_save failed (%s)", esp_err_to_name(ret));
    return proto.emitSingleResponse("map_save",
                                    {"error", "Failed to save to Flash"});
  }
  return proto.emitSingleResponse("map_save", {"ok"});
}

bool wireMapGet(CliProtocol &proto) {
  std::vector<std::string> bodies;
  for(size_t i = 0; i < globalData.mapData.size(); i++) {
    const MapPoint &point = globalData.mapData[i];
    char tBuf[16], leftBuf[16], rightBuf[16], vBuf[16], omegaBuf[16], xBuf[16],
        yBuf[16], thetaBuf[16], sectorBuf[8], marksBuf[8], speedBuf[16];
    snprintf(tBuf, sizeof(tBuf), "%lu", static_cast<unsigned long>(point.t));
    snprintf(leftBuf, sizeof(leftBuf), "%ld",
             static_cast<long>(point.encoderLeft));
    snprintf(rightBuf, sizeof(rightBuf), "%ld",
             static_cast<long>(point.encoderRight));
    snprintf(vBuf, sizeof(vBuf), "%.1f", static_cast<double>(point.v));
    snprintf(omegaBuf, sizeof(omegaBuf), "%.4f",
             static_cast<double>(point.omega));
    snprintf(xBuf, sizeof(xBuf), "%.1f", static_cast<double>(point.x));
    snprintf(yBuf, sizeof(yBuf), "%.1f", static_cast<double>(point.y));
    snprintf(thetaBuf, sizeof(thetaBuf), "%.4f",
             static_cast<double>(point.theta));
    snprintf(sectorBuf, sizeof(sectorBuf), "%u",
             static_cast<unsigned>(point.sector));
    snprintf(marksBuf, sizeof(marksBuf), "%u",
             static_cast<unsigned>(point.rightMarks));
    snprintf(speedBuf, sizeof(speedBuf), "%.3f",
             static_cast<double>(point.speed));
    std::string seg = proto.makeListBodySegment(
        "map_get", 's', static_cast<int>(i + 1),
        {tBuf, leftBuf, rightBuf, vBuf, omegaBuf, xBuf, yBuf, thetaBuf,
         sectorBuf, marksBuf, speedBuf});
    if(seg.empty()) {
      return false;
    }
    bodies.push_back(std::move(seg));
  }
  proto.emitListResponse("map_get", bodies);
  return true;
}

} // namespace cli_map
