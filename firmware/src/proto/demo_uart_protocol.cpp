#include "demo_uart_protocol.h"
#include "apps/app.h"

DemoUartProtocol::DemoUartProtocol(HardwareSerial &uart) : uart_(uart) {}

void DemoUartProtocol::begin(uint32_t baud, int8_t rx_pin, int8_t tx_pin)
{
    uart_.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
}

void DemoUartProtocol::loop()
{
    while (uart_.available() > 0)
    {
        char c = static_cast<char>(uart_.read());
        if (c == '\r')
        {
            continue;
        }
        if (c == '\n')
        {
            line_buf_[line_len_] = '\0';
            handleLine(line_buf_);
            line_len_ = 0;
            continue;
        }

        if (line_len_ + 1 < kLineBufferSize)
        {
            line_buf_[line_len_++] = c;
        }
        else
        {
            line_len_ = 0;
            LOGW("Demo UART line overflow, dropping buffer");
        }
    }
}

bool DemoUartProtocol::sendCommand(const char *dev, int32_t val)
{
    return sendCommandWithId(0, dev, val);
}

bool DemoUartProtocol::sendCommandWithId(uint32_t id, const char *dev, int32_t val)
{
    if (dev == nullptr || dev[0] == '\0')
    {
        return false;
    }

    uint32_t id_to_use = id == 0 ? nextId() : id;
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "id", static_cast<double>(id_to_use));
    cJSON_AddStringToObject(json, "dev", dev);
    cJSON_AddNumberToObject(json, "val", val);

    char *json_string = cJSON_PrintUnformatted(json);
    if (json_string == nullptr)
    {
        cJSON_Delete(json);
        return false;
    }

    uart_.print(json_string);
    uart_.print('\n');
    mirrorToSerial("[UART_TX] ", json_string);

    cJSON_free(json_string);
    cJSON_Delete(json);

    trackPending(id_to_use);
    return true;
}

bool DemoUartProtocol::sendFromEntityState(const EntityStateUpdate &state)
{
    if (!state.changed || state.app_slug[0] == '\0' || state.state[0] == '\0')
    {
        return false;
    }

    cJSON *json = cJSON_Parse(state.state);
    if (json == nullptr)
    {
        return false;
    }

    const char *dev = nullptr;
    int32_t val = 0;

    if (strcmp(state.app_slug, APP_SLUG_CLIMATE) == 0)
    {
        dev = "climate";
        cJSON *target_temp = cJSON_GetObjectItemCaseSensitive(json, "target_temp");
        if (!cJSON_IsNumber(target_temp))
        {
            cJSON_Delete(json);
            return false;
        }
        val = target_temp->valueint;
        if (val < 16 || val > 35)
        {
            cJSON_Delete(json);
            return false;
        }
    }
    else if (strcmp(state.app_slug, APP_SLUG_BLINDS) == 0)
    {
        dev = "blinds";
        cJSON *position = cJSON_GetObjectItemCaseSensitive(json, "position");
        if (!cJSON_IsNumber(position))
        {
            cJSON_Delete(json);
            return false;
        }
        int32_t position_pct = position->valueint;
        if (position_pct < 0 || position_pct > 100)
        {
            cJSON_Delete(json);
            return false;
        }
        val = position_pct;
    }
    else if (strcmp(state.app_slug, APP_SLUG_LIGHT_DIMMER) == 0)
    {
        dev = "workbench";
        cJSON *brightness = cJSON_GetObjectItemCaseSensitive(json, "brightness");
        if (!cJSON_IsNumber(brightness))
        {
            cJSON_Delete(json);
            return false;
        }
        double brightness_raw = brightness->valuedouble;
        int32_t brightness_pct = static_cast<int32_t>(round(brightness_raw / 2.55));
        if (brightness_pct < 0 || brightness_pct > 100)
        {
            cJSON_Delete(json);
            return false;
        }
        val = brightness_pct;
    }
    else if (strcmp(state.app_slug, APP_SLUG_SWITCH) == 0 || strcmp(state.app_slug, APP_SLUG_LIGHT_SWITCH) == 0)
    {
        dev = "ceiling";
        cJSON *on = cJSON_GetObjectItemCaseSensitive(json, "on");
        if (cJSON_IsBool(on))
        {
            val = cJSON_IsTrue(on) ? 1 : 0;
        }
        else if (cJSON_IsNumber(on))
        {
            val = on->valueint ? 1 : 0;
        }
        else
        {
            cJSON_Delete(json);
            return false;
        }
    }
    else
    {
        cJSON_Delete(json);
        return false;
    }

    cJSON_Delete(json);
    return sendCommand(dev, val);
}

void DemoUartProtocol::handleLine(const char *line)
{
    if (line == nullptr)
    {
        return;
    }

    bool has_content = false;
    for (size_t i = 0; line[i] != '\0'; i++)
    {
        if (line[i] != ' ' && line[i] != '\t')
        {
            has_content = true;
            break;
        }
    }

    if (!has_content)
    {
        return;
    }

    cJSON *json = cJSON_Parse(line);
    if (json == nullptr)
    {
        return;
    }

    cJSON *id = cJSON_GetObjectItemCaseSensitive(json, "id");
    if (!cJSON_IsNumber(id))
    {
        cJSON_Delete(json);
        return;
    }

    uint32_t id_val = static_cast<uint32_t>(id->valueint);
    clearPending(id_val);

    cJSON *ok = cJSON_GetObjectItemCaseSensitive(json, "ok");
    int ok_val = cJSON_IsNumber(ok) ? ok->valueint : 0;

    cJSON *dev = cJSON_GetObjectItemCaseSensitive(json, "dev");
    cJSON *val = cJSON_GetObjectItemCaseSensitive(json, "val");

    if (cJSON_IsString(dev) && cJSON_IsNumber(val))
    {
        (void)dev;
        (void)val;
    }
    else
    {
        (void)ok_val;
    }

    cJSON_Delete(json);
}

void DemoUartProtocol::mirrorToSerial(const char *prefix, const char *payload)
{
    if (prefix == nullptr || payload == nullptr)
    {
        return;
    }

#if DEMO_UART_MIRROR_TO_USB
    Serial.print(prefix);
    Serial.println(payload);
#else
    (void)prefix;
    (void)payload;
#endif
}

uint32_t DemoUartProtocol::nextId()
{
    uint32_t id = next_id_;
    next_id_++;
    if (next_id_ == 0)
    {
        next_id_ = 1;
    }
    return id;
}

void DemoUartProtocol::trackPending(uint32_t id)
{
    if (pending_count_ < kMaxPendingIds)
    {
        pending_ids_[pending_count_++] = id;
        return;
    }

    for (size_t i = 1; i < kMaxPendingIds; i++)
    {
        pending_ids_[i - 1] = pending_ids_[i];
    }
    pending_ids_[kMaxPendingIds - 1] = id;
}

void DemoUartProtocol::clearPending(uint32_t id)
{
    for (size_t i = 0; i < pending_count_; i++)
    {
        if (pending_ids_[i] == id)
        {
            for (size_t j = i + 1; j < pending_count_; j++)
            {
                pending_ids_[j - 1] = pending_ids_[j];
            }
            pending_count_--;
            return;
        }
    }
}
