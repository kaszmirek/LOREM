#include "tof.h"
#include "config.h"

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
    if (_sensor.timeoutOccurred() || dist >= TOF_MAX_MM)
        return _last_good;
    int16_t d = static_cast<int16_t>(dist);
    if (_last_good >= 0 && d < _last_good - TOF_SPIKE_MM)
        return _last_good;
    return _last_good = d;
}
