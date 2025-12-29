#include "core/globals.h"

namespace liftrr {
namespace core {

RuntimeState::RuntimeState()
    : device_mode_(MODE_IDLE),
      last_screen_update_(0),
      last_log_time_(0),
      last_auto_dump_time_(0),
      last_btn_state_(HIGH),
      btn_press_time_(0) {}

DeviceMode RuntimeState::deviceMode() const {
    return device_mode_;
}

void RuntimeState::setDeviceMode(DeviceMode mode) {
    device_mode_ = mode;
}

unsigned long RuntimeState::lastScreenUpdate() const {
    return last_screen_update_;
}

void RuntimeState::setLastScreenUpdate(unsigned long value) {
    last_screen_update_ = value;
}

unsigned long RuntimeState::lastLogTime() const {
    return last_log_time_;
}

void RuntimeState::setLastLogTime(unsigned long value) {
    last_log_time_ = value;
}

unsigned long RuntimeState::lastAutoDumpTime() const {
    return last_auto_dump_time_;
}

void RuntimeState::setLastAutoDumpTime(unsigned long value) {
    last_auto_dump_time_ = value;
}

int RuntimeState::lastBtnState() const {
    return last_btn_state_;
}

void RuntimeState::setLastBtnState(int value) {
    last_btn_state_ = value;
}

unsigned long RuntimeState::btnPressTime() const {
    return btn_press_time_;
}

void RuntimeState::setBtnPressTime(unsigned long value) {
    btn_press_time_ = value;
}

} // namespace core
} // namespace liftrr
