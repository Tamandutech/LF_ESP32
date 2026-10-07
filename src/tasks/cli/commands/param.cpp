#include "param.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "esp_log.h"

#include "context/GlobalData.hpp"
#include "env.hpp"
#include "storage/storage.hpp"
#include "tasks/cli/cli.hpp"

namespace cli_param {

namespace {
const char *TAG = "cli_param";
} // namespace

ParseError parseClassNameParameter(const char *input, ParsedReference &result) {
  if(input == nullptr || input[0] == '\0') {
    return ParseError::EMPTY_STRING;
  }

  const char *dotPos = strchr(input, '.');
  if(dotPos == nullptr) {
    return ParseError::NO_DOT;
  }
  if(strchr(dotPos + 1, '.') != nullptr) {
    return ParseError::MULTIPLE_DOTS;
  }

  size_t classNameLen     = static_cast<size_t>(dotPos - input);
  size_t parameterNameLen = strlen(dotPos + 1);

  if(classNameLen == 0) {
    return ParseError::EMPTY_CLASS_NAME;
  }
  if(parameterNameLen == 0) {
    return ParseError::EMPTY_PARAMETER_NAME;
  }

  if(classNameLen >= sizeof(result.className) ||
     parameterNameLen >= sizeof(result.parameterName)) {
    return ParseError::INVALID_FORMAT;
  }

  memcpy(result.className, input, classNameLen);
  result.className[classNameLen] = '\0';

  memcpy(result.parameterName, dotPos + 1, parameterNameLen);
  result.parameterName[parameterNameLen] = '\0';

  return ParseError::SUCCESS;
}

bool parseCliFloat(const char *value, float *out) {
  if(value == nullptr || out == nullptr) {
    return false;
  }
  bool        isNegative  = (value[0] == '!');
  const char *actualValue = isNegative ? value + 1 : value;
  if(actualValue[0] == '\0') {
    return false;
  }
  char *end = nullptr;
  errno     = 0;
  float v   = strtof(actualValue, &end);
  if(end == actualValue || *end != '\0' || errno == ERANGE) {
    return false;
  }
  *out = isNegative ? -v : v;
  return true;
}

bool getParameterValue(const char *className, const char *parameterName,
                       char *valueBuffer, size_t bufferSize) {
  if(strcmp(className, "State") == 0) {
    if(strcmp(parameterName, "runOnMappingMode") == 0) {
      snprintf(valueBuffer, bufferSize, "%d",
               globalData.parametersConfig.runOnMappingMode ? 1 : 0);
      return true;
    }
  }

  if(strcmp(className, "Vacuum") == 0) {
    if(strcmp(parameterName, "speed") == 0) {
      snprintf(valueBuffer, bufferSize, "%ld",
               static_cast<long>(globalData.parametersConfig.vacuumPWM));
      return true;
    }
  }

  if(strcmp(className, "Calibration") == 0) {
    if(strcmp(parameterName, "hardcodedCalibration") == 0) {
      snprintf(valueBuffer, bufferSize, "%d",
               globalData.parametersConfig.hardcodedCalibration ? 1 : 0);
      return true;
    }
  }

  if(strcmp(className, "Mapping") == 0) {
    if(strcmp(parameterName, "mappingMotorPWM") == 0) {
      snprintf(valueBuffer, bufferSize, "%ld",
               static_cast<long>(globalData.parametersConfig.mappingMotorPWM));
      return true;
    }
    if(strcmp(parameterName, "mapPointSavePulseInterval") == 0) {
      snprintf(valueBuffer, bufferSize, "%ld",
               static_cast<long>(
                   globalData.parametersConfig.mapPointSavePulseInterval));
      return true;
    }
    if(strcmp(parameterName, "mapPointMovingAverageSize") == 0) {
      snprintf(valueBuffer, bufferSize, "%ld",
               static_cast<long>(
                   globalData.parametersConfig.mapPointMovingAverageSize));
      return true;
    }
    if(strcmp(parameterName, "mapPointDerivativeMargin") == 0) {
      snprintf(valueBuffer, bufferSize, "%.6f",
               static_cast<double>(
                   globalData.parametersConfig.mapPointDerivativeMargin));
      return true;
    }
  }

  if(strcmp(className, "PID") == 0) {
    if(strcmp(parameterName, "kP") == 0) {
      snprintf(valueBuffer, bufferSize, "%.6f",
               static_cast<double>(globalData.parametersConfig.pidKp));
      return true;
    }
    if(strcmp(parameterName, "kI") == 0) {
      snprintf(valueBuffer, bufferSize, "%.6f",
               static_cast<double>(globalData.parametersConfig.pidKi));
      return true;
    }
    if(strcmp(parameterName, "kD") == 0) {
      snprintf(valueBuffer, bufferSize, "%.6f",
               static_cast<double>(globalData.parametersConfig.pidKd));
      return true;
    }
  }

  return false;
}

bool setParameterValue(const char *className, const char *parameterName,
                       const char *value) {
  bool        isNegative  = (value[0] == '!');
  const char *actualValue = isNegative ? value + 1 : value;

  if(strcmp(className, "State") == 0) {
    if(strcmp(parameterName, "runOnMappingMode") == 0) {
      int val                                      = atoi(actualValue);
      globalData.parametersConfig.runOnMappingMode = (val == 1);
      return true;
    }
  }

  if(strcmp(className, "Vacuum") == 0) {
    if(strcmp(parameterName, "speed") == 0) {
      int val = atoi(actualValue);
      if(val < 0) {
        val = 0;
      }
      if(val > 100) {
        val = 100;
      }
      globalData.parametersConfig.vacuumPWM = static_cast<int32_t>(val);
      return true;
    }
  }

  if(strcmp(className, "Calibration") == 0) {
    if(strcmp(parameterName, "hardcodedCalibration") == 0) {
      int val                                          = atoi(actualValue);
      globalData.parametersConfig.hardcodedCalibration = (val == 1);
      return true;
    }
  }

  if(strcmp(className, "Mapping") == 0) {
    if(strcmp(parameterName, "mappingMotorPWM") == 0) {
      int val = atoi(actualValue);
      if(val < 0) {
        val = 0;
      }
      if(val > MAX_MOTOR_PWM) {
        val = MAX_MOTOR_PWM;
      }
      globalData.parametersConfig.mappingMotorPWM = static_cast<int32_t>(val);
      return true;
    }
    if(strcmp(parameterName, "mapPointSavePulseInterval") == 0) {
      int val = atoi(actualValue);
      if(val < 1) {
        val = 1;
      }
      globalData.parametersConfig.mapPointSavePulseInterval =
          static_cast<int32_t>(val);
      return true;
    }
    if(strcmp(parameterName, "mapPointMovingAverageSize") == 0) {
      int val = atoi(actualValue);
      if(val < 2) {
        val = 2;
      }
      if(val > 256) {
        val = 256;
      }
      globalData.parametersConfig.mapPointMovingAverageSize =
          static_cast<int32_t>(val);
      return true;
    }
    if(strcmp(parameterName, "mapPointDerivativeMargin") == 0) {
      float v = 0.0F;
      if(!parseCliFloat(value, &v)) {
        return false;
      }
      if(v < 0.0F) {
        v = 0.0F;
      }
      globalData.parametersConfig.mapPointDerivativeMargin = v;
      return true;
    }
  }

  if(strcmp(className, "PID") == 0) {
    float v;
    if(!parseCliFloat(value, &v)) {
      return false;
    }
    if(strcmp(parameterName, "kP") == 0) {
      globalData.parametersConfig.pidKp = v;
      return true;
    }
    if(strcmp(parameterName, "kI") == 0) {
      globalData.parametersConfig.pidKi = v;
      return true;
    }
    if(strcmp(parameterName, "kD") == 0) {
      globalData.parametersConfig.pidKd = v;
      return true;
    }
  }

  return false;
}

bool paramSetPersistWireError(CliProtocol &proto) {
  Storage *storage = Storage::getInstance();
  if(!storage->is_mounted()) {
    ESP_LOGW(TAG, "param_set: storage not mounted");
    proto.emitSingleResponse("param_set", {"error", "storage not mounted"});
    return false;
  }
  if(storage->write(globalData.parametersConfig, PARAMETERS_STORAGE_FILE) !=
     ESP_OK) {
    proto.emitSingleResponse("param_set",
                             {"error", "failed to save parameters"});
    return false;
  }
  return true;
}

int paramSetRamOnly(const char *refWire, const char *valueWire) {
  ParsedReference ref;
  ParseError      pe = parseClassNameParameter(refWire, ref);
  if(pe != ParseError::SUCCESS) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  if(!setParameterValue(ref.className, ref.parameterName, valueWire)) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return CLI_SUCCESS;
}

bool wireParamList(CliProtocol &proto) {
  struct Row {
    const char *nameCol;
    char        valueBuf[64];
  };
  std::vector<Row> rows;
  char             v[64];

  if(getParameterValue("State", "runOnMappingMode", v, sizeof(v))) {
    rows.push_back({"State.runOnMappingMode", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }
  if(getParameterValue("Vacuum", "speed", v, sizeof(v))) {
    rows.push_back({"Vacuum.speed", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }
  if(getParameterValue("Calibration", "hardcodedCalibration", v, sizeof(v))) {
    rows.push_back({"Calibration.hardcodedCalibration", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }
  if(getParameterValue("Mapping", "mappingMotorPWM", v, sizeof(v))) {
    rows.push_back({"Mapping.mappingMotorPWM", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }
  if(getParameterValue("Mapping", "mapPointSavePulseInterval", v, sizeof(v))) {
    rows.push_back({"Mapping.mapPointSavePulseInterval", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }
  if(getParameterValue("Mapping", "mapPointMovingAverageSize", v, sizeof(v))) {
    rows.push_back({"Mapping.mapPointMovingAverageSize", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }
  if(getParameterValue("Mapping", "mapPointDerivativeMargin", v, sizeof(v))) {
    rows.push_back({"Mapping.mapPointDerivativeMargin", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }
  if(getParameterValue("PID", "kP", v, sizeof(v))) {
    rows.push_back({"PID.kP", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }
  if(getParameterValue("PID", "kI", v, sizeof(v))) {
    rows.push_back({"PID.kI", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }
  if(getParameterValue("PID", "kD", v, sizeof(v))) {
    rows.push_back({"PID.kD", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }

  std::vector<std::string> bodies;
  bodies.reserve(rows.size());
  for(size_t i = 0; i < rows.size(); i++) {
    const Row &r = rows[i];
    std::string seg =
        proto.makeListBodySegment("param_list", 's', static_cast<int>(i + 1),
                                  {r.nameCol, r.valueBuf});
    if(seg.empty()) {
      return false;
    }
    bodies.push_back(std::move(seg));
  }
  proto.emitListResponse("param_list", bodies);
  return true;
}

bool wireParamGet(const wire::WireView &view, CliProtocol &proto) {
  if(view.payloadArgc() < 1) {
    ESP_LOGW(TAG, "param_get(s,r,ref) missing args");
    return false;
  }
  ParsedReference ref;
  ParseError      pe = parseClassNameParameter(view.arg(0), ref);
  if(pe != ParseError::SUCCESS) {
    ESP_LOGW(TAG, "param_get: bad ref");
    return false;
  }
  char value[64];
  if(!getParameterValue(ref.className, ref.parameterName, value,
                        sizeof(value))) {
    return false;
  }
  return proto.emitSingleResponse("param_get", {value});
}

bool wireParamSetSingle(const wire::WireView &view, CliProtocol &proto) {
  if(view.payloadArgc() < 2) {
    return false;
  }
  if(paramSetRamOnly(view.arg(0), view.arg(1)) != CLI_SUCCESS) {
    return false;
  }
  if(!paramSetPersistWireError(proto)) {
    return true;
  }
  return proto.emitSingleResponse("param_set", {"ok"});
}

bool wireParamSetBodyRamOnly(const wire::WireView &view) {
  if(view.payloadArgc() < 2) {
    return false;
  }
  return paramSetRamOnly(view.arg(0), view.arg(1)) == CLI_SUCCESS;
}

bool wireParamSetLoneBody(const wire::WireView &view, CliProtocol &proto) {
  if(view.payloadArgc() < 2) {
    return false;
  }
  if(paramSetRamOnly(view.arg(0), view.arg(1)) != CLI_SUCCESS) {
    return false;
  }
  if(!paramSetPersistWireError(proto)) {
    return true;
  }
  return proto.emitSingleResponse("param_set", {"ok"});
}

} // namespace cli_param
