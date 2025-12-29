#pragma once
#include <Arduino.h>

#include "core/config.h"

namespace liftrr {
namespace core {

class RuntimeState {
public:
    RuntimeState();

    DeviceMode deviceMode() const;
    void setDeviceMode(DeviceMode mode);

    unsigned long lastScreenUpdate() const;
    void setLastScreenUpdate(unsigned long value);

    unsigned long lastLogTime() const;
    void setLastLogTime(unsigned long value);

    unsigned long lastAutoDumpTime() const;
    void setLastAutoDumpTime(unsigned long value);

    int lastBtnState() const;
    void setLastBtnState(int value);

    unsigned long btnPressTime() const;
    void setBtnPressTime(unsigned long value);

private:
    DeviceMode device_mode_;
    unsigned long last_screen_update_;
    unsigned long last_log_time_;
    unsigned long last_auto_dump_time_;
    int last_btn_state_;
    unsigned long btn_press_time_;
};

} // namespace core
} // namespace liftrr
