/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "sensors/mpu6886sensor.h"
#include "network/network.h"
#include "ledmgr.h"
#include "utils.h"

void MPU6886Sensor::motionSetup()
{
#ifdef FULL_DEBUG
    imu.enableDebugging(Serial);
#endif

    if(!imu.begin(addr, Wire, intPin)) {
        m_Logger.fatal("Can't connect to %s at address 0x%02x", getIMUNameByType(sensorType), addr);
        LEDManager::signalAssert();
        return;
    }

    m_Logger.info("Connected to %s on 0x%02x. ",
                  getIMUNameByType(sensorType), 
                  addr 
                );

    lastReset = 0;
    lastData = millis();
    working = true;
    configured = true;
}

void MPU6886Sensor::motionLoop()
{
    float lastv[3], v[3];

    // NOTE: There's probably a better way to do this...
    imu.getAccelData(&v[0], &v[1], &v[2]);

#if 0
    lastReset = 0;
    lastData = millis();
#endif

    if(lastv[0] != v[0] || lastv[1] != v[1] || lastv[2] != v[2])
    {
        Network::sendAccel(v, PACKET_ACCEL);
        lastv[0] = v[0];
        lastv[1] = v[1];
        lastv[2] = v[2];
    }

#if 0    
    if (lastData + 1000 < millis() && configured)
    {
        LEDManager::setLedStatus(LED_STATUS_IMU_ERROR);
        working = false;
        lastData = millis();
        uint8_t rr = imu.resetReason();
        if (rr != lastReset)
        {
            lastReset = rr;
            Network::sendError(rr, this->sensorId);
        }
        m_Logger.error("Sensor %d was reset: %d", sensorId, rr);
    }
#endif
}

uint8_t MPU6886Sensor::getSensorState() {
    return lastReset > 0 ? SensorStatus::SENSOR_ERROR : isWorking() ? SensorStatus::SENSOR_OK : SensorStatus::SENSOR_OFFLINE;
}

void MPU6886Sensor::sendData()
{
    if (newData)
    {
        newData = false;
        Network::sendRotationData(&quaternion, DATA_TYPE_NORMAL, calibrationAccuracy, sensorId);
        if (useMagnetometerAllTheTime)
            Network::sendMagnetometerAccuracy(magneticAccuracyEstimate, sensorId);

#ifdef FULL_DEBUG
        m_Logger.trace("Quaternion: %f, %f, %f, %f", UNPACK_QUATERNION(quaternion));
#endif
    }
    if (newMagData)
    {
        newMagData = false;
        Network::sendRotationData(&magQuaternion, DATA_TYPE_CORRECTION, magCalibrationAccuracy, sensorId);
        Network::sendMagnetometerAccuracy(magneticAccuracyEstimate, sensorId);
    }
    
    if (tap != 0)
    {
        Network::sendTap(tap, sensorId);
        tap = 0;
    }
}

void MPU6886Sensor::startCalibration(int calibrationType)
{
#if 0
    // TODO It only calibrates gyro, it should have multiple calibration modes, and check calibration status in motionLoop()
    LEDManager::pattern(CALIBRATING_LED, 20, 20, 10);
    LEDManager::blink(CALIBRATING_LED, 2000);
    imu.calibrateGyro();
    do
    {
        LEDManager::on(CALIBRATING_LED);
        imu.requestCalibrationStatus();
        delay(20);
        imu.getReadings();
        LEDManager::off(CALIBRATING_LED);
        delay(20);
    } while (!imu.calibrationComplete());
    imu.saveCalibration();
#endif
}
