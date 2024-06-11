#include <Wire.h>
#include "Arduino.h"
#include "QMI8658A.h"

/**
 * put default values in variables.
 * real initialization of sensor happens in begin().
 */
QMI8658A::QMI8658A()
{
    this->device_addr = 0x6B; // default for SD0/SA0 low, 0x6A if high
    this->acc_scale = acc_scale_4g;
    this->gyro_scale = gyro_scale_1024dps;
    this->acc_odr = acc_odr_norm_8000;
    this->gyro_odr = gyro_odr_norm_8000;
    this->sensor_state = sensor_default;
}

/**
 * Inialize Wire and send default configs
 * @param addr I2C address of sensor, typically 0x6A or 0x6B
 */
void QMI8658A::begin(byte addr)
{
    Wire.begin();
    this->device_addr = addr;
    setState(sensor_running);
    setAccScale(this->acc_scale);
    setAccODR(this->acc_odr);
    setGyroScale(this->gyro_scale);
    setGyroODR(this->gyro_odr);
    setAccLPF(lpf_5_39);
    setGyroLPF(lpf_5_39);
}

/**
 * Inialize Wire and send default configs
 * @param addr I2C address of sensor, typically 0x6A or 0x6B
 * @param speed I2C speed, sensor supports up to 400kHz
 */
void QMI8658A::begin(byte addr, uint32_t speed)
{
    begin(addr);
    Wire.setClock(speed);
}

/**
 * Transmit one byte of data to QMI8658A.
 * @param addr address of data to be written
 * @param data the data to be written
 */
void QMI8658A::QMI8658A_transmit(byte addr, byte data)
{
    Wire.beginTransmission(this->device_addr);
    Wire.write(addr);
    Wire.write(data);
    Wire.endTransmission();
}

/**
 * Receive one byte of data from QMI8658A.
 * @param addr address of data to be read
 * @return the byte of data that was read
 */
byte QMI8658A::QMI8658A_receive(byte addr)
{
    Wire.beginTransmission(this->device_addr);
    unsigned long beforeRequest = millis();
    Wire.write(addr);
    Wire.endTransmission(false);
    Wire.requestFrom(this->device_addr, (uint8_t)1);
    while (Wire.available() == 0)
    {
        if (millis() - beforeRequest > QMI8658_COMM_TIMEOUT)
            return 0; // may need work, cleanest way I could think of
                      // to deal with this situation
    }
    byte retval = Wire.read();
    Wire.endTransmission();
    return retval;
}

/**
 * Writes data to CTRL9 (command register) and waits for ACK.
 * @param command the command to be executed
 */
void QMI8658A::QMI8658A_CTRL9_Write(byte command)
{
    // transmit command
    QMI8658A_transmit(QMI8658_CTRL9, command);

    // wait for command to be done
    while (QMI8658A_receive(QMI8658_STATUSINT)& 0x80 == 0x00);
}

/**
 * Update readings array while in sensor_locking state.
 * This guarantees all data are synchronized together.
 */
void QMI8658A::QMI8658_sensor_update()
{
    if (this->sensor_state == sensor_locking)
    {    // wait until STATUSINT shows data is ready
        while(QMI8658A_receive(QMI8658_STATUSINT) & 0x01 == 0)
            delayMicroseconds(3);
        
        // wait a resonable interval for data loading
        // these are reasonable for when gyro is enabled
        // if accel only, we need to delay longer
        if (gyro_odr <= gyro_odr_norm_4000)
            delayMicroseconds(3);
        else if (gyro_odr <= gyro_odr_norm_1000)
            delayMicroseconds(6);
        else
            delayMicroseconds(12);
    }
    // update timestamp for readings
    this->reading_timestamp_us = micros();

    // load actual data
    Wire.beginTransmission(this->device_addr);
    unsigned long beforeRequest = millis();
    Wire.write(QMI8658_AX_L);
    Wire.endTransmission(false);
    Wire.requestFrom(this->device_addr, (uint8_t)12, (uint8_t)true);
    for (int i = 0; i < 12; i++) {
        while(Wire.available() == 0)
        {
            if (millis() - beforeRequest > QMI8658_COMM_TIMEOUT)
                return;
        }
        if (i % 2 == 0)
        {
            readings[i >> 1] = (int16_t)Wire.read();
        }
        else
        {
            readings[i >> 1] |= (int16_t)Wire.read() << 8;
        }
    }
}

inline void QMI8658A::QMI8658_update_if_needed()
{
    if (sensor_state == sensor_locking
        && micros() - reading_timestamp_us > QMI8658_REFRESH_DELAY)
    {
        QMI8658_sensor_update();
    }
}

/**
 * Set output data rate (ODR) of accelerometer.
 * @param odr acc_odr_t variable representing new data rate
 */
void QMI8658A::setAccODR(acc_odr_t odr)
{
    if (this->sensor_state != sensor_default)
    {
        byte ctrl2 = QMI8658A_receive(QMI8658_CTRL2);
        ctrl2 &= ~QMI8658_AODR_MASK; // clear previous setting
        ctrl2 |= odr; // OR in new setting
        QMI8658A_transmit(QMI8658_CTRL2, ctrl2);
    }
    this->acc_odr = odr;
}

/**
 * Set output data rate (ODR) of gyro.
 * @param odr gyro_odr_t variable representing new data rate
 */
void QMI8658A::setGyroODR(gyro_odr_t odr)
{
    if (this->sensor_state != sensor_default)
    {
    byte ctrl3 = QMI8658A_receive(QMI8658_CTRL3);
    ctrl3 &= ~QMI8658_GODR_MASK; // clear previous setting
    ctrl3 |= odr; // OR in new setting
    QMI8658A_transmit(QMI8658_CTRL3, ctrl3);
    }
    this->gyro_odr = odr;
}

/**
 * Set scale of accelerometer output.
 * @param scale acc_scale_t variable representing new scale
 */
void QMI8658A::setAccScale(acc_scale_t scale)
{
    if (this->sensor_state != sensor_default)
    {
    byte ctrl2 = QMI8658A_receive(QMI8658_CTRL2);
    ctrl2 &= ~QMI8658_ASCALE_MASK; // clear previous setting
    ctrl2 |= scale << QMI8658_ASCALE_OFFSET; // OR in new setting
    QMI8658A_transmit(QMI8658_CTRL2, ctrl2);
    }
    this->acc_scale = scale;
}

/**
 * Set scale of gyro output.
 * @param scale gyro_scale_t variable representing new scale
 */
void QMI8658A::setGyroScale(gyro_scale_t scale)
{
    if (this->sensor_state != sensor_default)
    {
    byte ctrl3 = QMI8658A_receive(QMI8658_CTRL3);
    ctrl3 &= ~QMI8658_GSCALE_MASK; // clear previous setting
    ctrl3 |= scale << QMI8658_GSCALE_OFFSET; // OR in new setting
    QMI8658A_transmit(QMI8658_CTRL3, ctrl3);
    }
    this->gyro_scale = scale;
}

/**
 * Set new low-pass filter value for accelerometer
 * @param lp lpf_t variable representing new low-pass filter value
 */
void QMI8658A::setAccLPF(lpf_t lpf)
{
    if (this->sensor_state != sensor_default)
    {
    byte ctrl5 = QMI8658A_receive(QMI8658_CTRL5);
    ctrl5 &= !QMI8658_ALPF_MASK;
    ctrl5 |= lpf << QMI8658_ALPF_OFFSET;
    ctrl5 |= 0x01; // turn on acc low pass filter
    QMI8658A_transmit(QMI8658_CTRL5, ctrl5);
    }
    this->acc_lpf = lpf;
}

/**
 * Set new low-pass filter value for gyro
 * @param lp lpf_t variable representing new low-pass filter value
 */
void QMI8658A::setGyroLPF(lpf_t lpf)
{
    if (this->sensor_state != sensor_default)
    {
    byte ctrl5 = QMI8658A_receive(QMI8658_CTRL5);
    ctrl5 &= !QMI8658_GLPF_MASK;
    ctrl5 |= lpf << QMI8658_GLPF_OFFSET;
    ctrl5 |= 0x10; // turn on gyro low pass filter
    QMI8658A_transmit(QMI8658_CTRL5, ctrl5);
    }
}

/**
 * Set new state of QMI8658A.
 * @param state new state to transition to
 */
void QMI8658A::setState(sensor_state_t state)
{
    byte ctrl1;
    switch (state)
    {
    case sensor_running:
        ctrl1 = QMI8658A_receive(QMI8658_CTRL1);
        // enable 2MHz oscillator
        ctrl1 &= 0xFE;
        // enable auto address increment for fast block reads
        ctrl1 |= 0x40;
        QMI8658A_transmit(QMI8658_CTRL1, ctrl1);

        // enable high speed internal clock,
        // acc and gyro in full mode, and
        // disable syncSample mode
        QMI8658A_transmit(QMI8658_CTRL7, 0x43);

        // disable AttitudeEngine Motion On Demand
        QMI8658A_transmit(QMI8658_CTRL6, 0x00);
        break;
    case sensor_power_down:
        // disable high speed internal clock,
        // acc and gyro powered down
        QMI8658A_transmit(QMI8658_CTRL7, 0x00);

        ctrl1 = QMI8658A_receive(QMI8658_CTRL1);
        // disable 2MHz oscillator
        ctrl1|= 0x01;
        QMI8658A_transmit(QMI8658_CTRL1, ctrl1);
        break;
    case sensor_locking:
        ctrl1 = QMI8658A_receive(QMI8658_CTRL1);
        // enable 2MHz oscillator
        ctrl1 &= 0xFE;
        // enable auto address increment for fast block reads
        ctrl1 |= 0x40;
        QMI8658A_transmit(QMI8658_CTRL1, ctrl1);

        // enable high speed internal clock,
        // acc and gyro in full mode, and
        // enable syncSample mode
        QMI8658A_transmit(QMI8658_CTRL7, 0x83);

        // disable AttitudeEngine Motion On Demand
        QMI8658A_transmit(QMI8658_CTRL6, 0x00);

        // disable internal AHB clock gating:
        QMI8658A_transmit(QMI8658_CAL1_L, 0x01);
        QMI8658A_CTRL9_Write(0x12);
        // re-enable clock gating
        QMI8658A_transmit(QMI8658_CAL1_L, 0x00);
        QMI8658A_CTRL9_Write(0x12);
        break;
    default:
        break;
    }
    this->sensor_state = state;
}

/**
 * Retrieves raw readings in int16_t form and copies
 * them into the provided pointer location.
 * @param buf pointer to destination buffer
 */
void QMI8658A::getRawReadings(int16_t* buf)
{
    QMI8658A::QMI8658_sensor_update();
    memcpy((void*) buf, (void*)readings, sizeof(readings));
}

/**
 * Get X-axis acceleration in floating point g units.
 * If in locking mode, this will retrieve new data
 * as needed from the sensor, or use a local version
 * if the data is fresh enough. In running mode,
 * this gets the most recent data from the sensor.
 * @return X-axis acceleration, in g
 */
float QMI8658A::getAccX()
{
    float xval;
    // update sensor values if necessary
    QMI8658_update_if_needed();
    if (sensor_state = sensor_locking) {
        xval = (float)readings[0];
    }
    else {
        xval = (float)(
            (int16_t)QMI8658A_receive(QMI8658_AX_L)
            | ((int16_t)QMI8658A_receive(QMI8658_AX_H) << 8)
        );
    }
    xval *= (float)(2 << acc_scale) * QMI8658_SIXTEENBIT_SCALER;
    return xval;
}

/**
 * Get Y-axis acceleration in floating point g units.
 * If in locking mode, this will retrieve new data
 * as needed from the sensor, or use a local version
 * if the data is fresh enough. In running mode,
 * this gets the most recent data from the sensor.
 * @return Y-axis acceleration, in g
 */
float QMI8658A::getAccY()
{
    float yval;
    // update sensor values if necessary
    QMI8658_update_if_needed();
    if (sensor_state = sensor_locking) {
        yval = (float)readings[1];
    }
    else {
        yval = (float)(
            (int16_t)QMI8658A_receive(QMI8658_AY_L)
            | ((int16_t)QMI8658A_receive(QMI8658_AY_H) << 8)
        );
    }
    yval *= (float)(2 << acc_scale) * QMI8658_SIXTEENBIT_SCALER;
    return yval;
}

/**
 * Get Z-axis acceleration in floating point g units.
 * If in locking mode, this will retrieve new data
 * as needed from the sensor, or use a local version
 * if the data is fresh enough. In running mode,
 * this gets the most recent data from the sensor.
 * @return Z-axis acceleration, in g
 */
float QMI8658A::getAccZ()
{
    float zval;
    // update sensor values if necessary
    QMI8658_update_if_needed();
    if (sensor_state = sensor_locking) {
        zval = (float)readings[2];
    }
    else {
        zval = (float)(
            (int16_t)QMI8658A_receive(QMI8658_AZ_L)
            | ((int16_t)QMI8658A_receive(QMI8658_AZ_H) << 8)
        );
    }
    zval *= (float)(2 << acc_scale) * QMI8658_SIXTEENBIT_SCALER;
    return zval;
}

/**
 * Get X-axis angular velocity in floating point dps.
 * If in locking mode, this will retrieve new data
 * as needed from the sensor, or use a local version
 * if the data is fresh enough. In running mode,
 * this gets the most recent data from the sensor.
 * @return X-axis angular velocity, in degrees per second
 */
float QMI8658A::getGyroX()
{
    float xval;
    // update sensor values if necessary
    QMI8658_update_if_needed();
    if (sensor_state = sensor_locking) {
        xval = (float)readings[3];
    }
    else {
        xval = (float)(
            (int16_t)QMI8658A_receive(QMI8658_GX_L)
            | ((int16_t)QMI8658A_receive(QMI8658_GX_H) << 8)
        );
    }
    xval *= (float)(16 << gyro_scale) * QMI8658_SIXTEENBIT_SCALER;
    return xval;
}

/**
 * Get Y-axis angular velocity in floating point dps.
 * If in locking mode, this will retrieve new data
 * as needed from the sensor, or use a local version
 * if the data is fresh enough. In running mode,
 * this gets the most recent data from the sensor.
 * @return Y-axis angular velocity, in degrees per second
 */
float QMI8658A::getGyroY()
{
    float yval;
    // update sensor values if necessary
    QMI8658_update_if_needed();
    if (sensor_state = sensor_locking) {
        yval = (float)readings[4];
    }
    else {
        yval = (float)(
            (int16_t)QMI8658A_receive(QMI8658_GY_L)
            | ((int16_t)QMI8658A_receive(QMI8658_GY_H) << 8)
        );
    }
    yval *= (float)(16 << gyro_scale) * QMI8658_SIXTEENBIT_SCALER;
    return yval;
}

/**
 * Get Z-axis angular velocity in floating point dps.
 * If in locking mode, this will retrieve new data
 * as needed from the sensor, or use a local version
 * if the data is fresh enough. In running mode,
 * this gets the most recent data from the sensor.
 * @return Z-axis angular velocity, in degrees per second
 */
float QMI8658A::getGyroZ()
{
    float zval;
    // update sensor values if necessary
    QMI8658_update_if_needed();
    if (sensor_state = sensor_locking) {
        zval = (float)readings[5];
    }
    else {
        zval = (float)(
            (int16_t)QMI8658A_receive(QMI8658_GZ_L)
            | ((int16_t)QMI8658A_receive(QMI8658_GZ_H) << 8)
        );
    }
    zval *= (float)(16 << gyro_scale) * QMI8658_SIXTEENBIT_SCALER;
    return zval;
}

QMI8658A::~QMI8658A()
{
}