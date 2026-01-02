#pragma once

#include "task.h"
#include "app_config.h"
#include <vector>
#if SK_PROXIMITY
#include <Adafruit_VL53L0X.h>
#endif

#if SK_STRAIN
#include <HX711.h>
#endif

#include "driver/temp_sensor.h"

const uint16_t PROXIMITY_SENSOR_OFFSET_MM = 10;

class SensorsTask : public Task<SensorsTask>
{
    friend class Task<SensorsTask>; // Allow base Task to invoke protected run()

public:
    SensorsTask(const uint8_t task_core, Configuration *configuration);
    ~SensorsTask();

    void addStateListener(QueueHandle_t queue);
    void factoryStrainCalibrationCallback(float calibration_weight);
    void weightMeasurementCallback();

    void setSharedEventsQueue(QueueHandle_t shared_event_queue);
    void publishEvent(WiFiEvent event);

    bool powerDownAllowed();

    void strainPowerDown();
    void strainPowerUp();

    bool isStrainPowered() { return strain_powered; }

protected:
    void run();

private:
    SensorsState sensors_state = {};
    QueueHandle_t sensors_state_queue_;

    bool do_strain = false;
    bool strain_powered = false;

    QueueHandle_t shared_events_queue;

    std::vector<QueueHandle_t> state_listeners_;

    SemaphoreHandle_t mutex_;
    void publishState(const SensorsState &state);
#if SK_STRAIN
    HX711 strain;
#endif
#if SK_TOUCH
    uint32_t touch_down_at_ms_ = 0;
    uint32_t last_touch_poll_ms_ = 0;
    uint32_t last_touch_error_log_ms_ = 0;
    bool touch_is_down_ = false;
    bool touch_long_sent_ = false;
    uint8_t pending_touch_release_code_ = VIRTUAL_BUTTON_IDLE;
#endif
    uint32_t button_down_at_ms_ = 0;
    uint32_t last_button_poll_ms_ = 0;
    uint32_t last_button_change_ms_ = 0;
    uint32_t last_button_log_ms_ = 0;
    bool button_is_down_ = false;
    bool button_long_sent_ = false;
    bool button_raw_state_ = false;
    bool button_debounced_state_ = false;

    Configuration *configuration_;

    uint8_t factory_strain_calibration_step_ = 0;
    uint8_t weight_measurement_step_ = 0;

    float last_press_value_ = 0;
    float last_strain_reading_raw_ = 0;

    float raw_initial_value_ = 0;

    float calibration_scale_ = 0;
};
