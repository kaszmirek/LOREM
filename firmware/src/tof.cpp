#include "tof.h"

bool ToFSensor::init() {
    enable();
    sleep_ms(2);  // t_boot: 1.2 ms min

    _sensor.setTimeout(500);
    if (!_sensor.init()) return false;
    if (_addr != DEFAULT_ADDR)
        _sensor.setAddress(_addr);
    _sensor.setMeasurementTimingBudget(20000);  // 20 ms (default 33 ms)
    _sensor.startContinuous();
    return true;
}

bool ToFSensor::data_ready() {
    return (_sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0;
}

int16_t ToFSensor::read_mm() {
    uint16_t dist = _sensor.readRangeContinuousMillimeters();
    if (_sensor.timeoutOccurred()) return -1;
    if (dist >= 8000) return -1;  // VL53L0X out-of-range sentinel (~8190)
    return static_cast<int16_t>(dist);
}
