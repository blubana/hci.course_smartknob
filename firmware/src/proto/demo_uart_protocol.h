#pragma once

#include <Arduino.h>
#include <logging.h>
#include <math.h>

#include "app_config.h"
#include "cJSON.h"

class DemoUartProtocol
{
public:
    explicit DemoUartProtocol(HardwareSerial &uart);

    void begin(uint32_t baud, int8_t rx_pin, int8_t tx_pin);
    void loop();

    bool sendCommand(const char *dev, int32_t val);
    bool sendCommandWithId(uint32_t id, const char *dev, int32_t val);
    bool sendFromEntityState(const EntityStateUpdate &state);

private:
    void handleLine(const char *line);
    void mirrorToSerial(const char *prefix, const char *payload);
    uint32_t nextId();
    void trackPending(uint32_t id);
    void clearPending(uint32_t id);

    HardwareSerial &uart_;

    static constexpr size_t kLineBufferSize = 256;
    char line_buf_[kLineBufferSize] = {};
    size_t line_len_ = 0;

    static constexpr size_t kMaxPendingIds = 8;
    uint32_t pending_ids_[kMaxPendingIds] = {};
    size_t pending_count_ = 0;

    uint32_t next_id_ = 1;
};
