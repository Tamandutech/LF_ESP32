#include "map.hpp"

#include <algorithm>
#include <cstdio>
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

MapPoint::PointType pointTypeFromInt(int value) {
  switch(value) {
  case MapPoint::AUTO_MARK:
  case MapPoint::MANUAL_MARK:
  case MapPoint::STOP_COMMAND_MARK:
  case MapPoint::UNKNOWN_MARK:
  case MapPoint::CURVE_START_MARK:
  case MapPoint::CURVE_END_MARK: return static_cast<MapPoint::PointType>(value);
  default: return MapPoint::UNKNOWN_MARK;
  }
}

/// One `map_get(b,s,idx,...)` segment. Floats are quoted, matching
/// appendWireToken (a decimal is not a plain integer). Returns the length
/// without the trailing NUL, or 0 if it does not fit.
size_t formatMapGetBody(int index, char *buf, size_t cap) {
  if(buf == nullptr || index < 0 ||
     static_cast<size_t>(index) >= globalData.mapData.size()) {
    return 0;
  }
  const MapPoint &point = globalData.mapData[static_cast<size_t>(index)];
  const int       n     = snprintf(
      buf, cap, "map_get(b,s,%d,%ld,%ld,\"%.3f\",\"%.3f\",\"%.3f\",%d);",
      index + 1, static_cast<long>(point.encoderLeft),
      static_cast<long>(point.encoderRight),
      static_cast<double>(point.encoderDerivative),
      static_cast<double>(point.encoderDerivativeAverage),
      static_cast<double>(point.speed), static_cast<int>(point.pointType));
  if(n < 0 || static_cast<size_t>(n) >= cap) {
    return 0;
  }
  return static_cast<size_t>(n);
}
} // namespace

bool parseMapAddBodyFields(const wire::WireView &view, int32_t *encoderLeft,
                           int32_t *encoderRight, float *encoderDerivative,
                           float *encoderDerivativeAverage, float *speed,
                           MapPoint::PointType *pointType) {
  if(view.payloadArgc() < 6 || encoderLeft == nullptr ||
     encoderRight == nullptr || encoderDerivative == nullptr ||
     encoderDerivativeAverage == nullptr || speed == nullptr ||
     pointType == nullptr) {
    return false;
  }
  int left  = 0;
  int right = 0;
  int type  = 0;
  if(!wire::parseInt(view.arg(0), left) ||
     !wire::parseInt(view.arg(1), right) ||
     !wire::parseFloat(view.arg(2), *encoderDerivative) ||
     !wire::parseFloat(view.arg(3), *encoderDerivativeAverage) ||
     !wire::parseFloat(view.arg(4), *speed) ||
     !wire::parseInt(view.arg(5), type)) {
    return false;
  }
  *encoderLeft  = static_cast<int32_t>(left);
  *encoderRight = static_cast<int32_t>(right);
  *pointType    = pointTypeFromInt(type);
  return true;
}

bool wireMapAddBody(const wire::WireView &view, CliProtocol &proto,
                    bool sortAfter) {
  int32_t             encoderLeft               = 0;
  int32_t             encoderRight              = 0;
  float               encoderDerivative         = 0.0F;
  float               encoderDerivativeAverage  = 0.0F;
  float               speed                     = 0.0F;
  MapPoint::PointType pointType                 = MapPoint::MANUAL_MARK;
  if(!parseMapAddBodyFields(view, &encoderLeft, &encoderRight,
                            &encoderDerivative, &encoderDerivativeAverage,
                            &speed, &pointType)) {
    ESP_LOGW(TAG, "map_add body: need 6 fields after idx "
                  "(encoder_left,encoder_right,derivative,average,speed,type)");
    return false;
  }
  if(globalData.mapData.size() >= static_cast<size_t>(MAP_POINT_MAX_COUNT)) {
    ESP_LOGW(TAG, "map_add: map full (%d)", MAP_POINT_MAX_COUNT);
    if(sortAfter) {
      return proto.emitSingleResponse("map_add", {"error", "map full"});
    }
    return false;
  }
  MapPoint point;
  point.encoderLeft              = encoderLeft;
  point.encoderRight             = encoderRight;
  point.encoderDerivative        = encoderDerivative;
  point.encoderDerivativeAverage = encoderDerivativeAverage;
  point.speed                    = speed;
  point.pointType                = pointType;
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
  // Stream each BLE packet as it fills. Holding one std::string per point
  // (and a second copy while packing) exhausts the heap once the map grows
  // past roughly a hundred points; operator new then abort()s because
  // exceptions are disabled.
  const int total = static_cast<int>(globalData.mapData.size());
  if(!proto.emitListStreaming("map_get", 's', total, formatMapGetBody)) {
    ESP_LOGE(TAG, "map_get failed to pack %d points", total);
    return false;
  }
  return true;
}

} // namespace cli_map
