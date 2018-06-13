/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "drivers/bus_i2c.h"
#include "pitotmeter.h"
#include "drivers/time.h"

#include "common/utils.h"

// PITOT_EXP, Standard address 0x4D
#define PITOT_EXP_ADDR                 0x4D
#define PITOT_EXP_RAW_VALUE_MAX        4048.0f //Max value we can expect from sensor
#define PITOT_EXP_RAW_VALUE_MIN        896.0f  //Min value we can expect from the sensor, anything less than this is negative preasure and useless for our needs
#define PITOT_EXP_PA_VALUE_MAX         3920.0f //Max value the sensor can read in Pascals
#define PITOT_EXP_PA_VALUE_MIN         0.0f    //Min value the sensor can read in Pascals, anything less than this is negative preasure and useless for our needs


static uint16_t pitot_exp_up;  // static result of pressure measurement
static uint8_t rxbuf[4];

static inline float pitotExpRawToKpa(uint16_t rawValue)
{
  return ((((float)constrainf(rawValue,PITOT_EXP_RAW_VALUE_MIN,PITOT_EXP_RAW_VALUE_MAX) - PITOT_EXP_RAW_VALUE_MIN) * (PITOT_EXP_PA_VALUE_MAX - PITOT_EXP_PA_VALUE_MIN)) / (PITOT_EXP_RAW_VALUE_MAX - PITOT_EXP_RAW_VALUE_MIN) ) + PITOT_EXP_PA_VALUE_MIN;
}

static void pitot_exp_start(pitotDev_t * pitot)
{
    busReadBuf( pitot->busDev, PITOT_EXP_ADDR, rxbuf, 2 );
}

static void pitot_exp_read(pitotDev_t * pitot)
{
    if (busReadBuf( pitot->busDev, PITOT_EXP_ADDR, rxbuf, 2 ))
    {
        pitot_exp_up = (rxbuf[0] << 8) | (rxbuf[1] << 0);
    }
}

static void pitot_exp_calculate(pitotDev_t * pitot, float *pressure, float *temperature)
{
    UNUSED(pitot);

    if (pressure)
        *pressure = pitotExpRawToKpa(pitot_exp_up);    // Pa
    if (temperature)
        *temperature = 288.15f;     // Temperature at standard sea level (288.15 K)
}

bool pitotExpDetect(pitotDev_t * pitot)
{
    pitot->busDev = busDeviceInit(BUSTYPE_I2C, DEVHW_PITOT_EXP, 0, OWNER_AIRSPEED);
    if (pitot->busDev == NULL) {
        return false;
    }

    bool ack = false;

    // Read twice to fix:
    // Sending a start-stop condition without any transitions on the SCL line (no clock pulses in between) creates a
    // communication error for the next communication, even if the next start condition is correct and the clock pulse is applied.
    // An additional start condition must be sent, which results in restoration of proper communication.
    ack = busReadBuf( pitot->busDev, PITOT_EXP_ADDR, rxbuf, 2 );
    ack = busReadBuf( pitot->busDev, PITOT_EXP_ADDR, rxbuf, 2 );
    if (!ack) {
        return false;
    }

    pitot->delay = 10000;
    pitot->start = pitot_exp_start;
    pitot->get = pitot_exp_read;
    pitot->calculate = pitot_exp_calculate;
    pitot_exp_read(pitot);
    return true;
}
