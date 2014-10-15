/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <hardware/sensors.h>
#include <linux/hwmsensor.h>
#include "hwmsen_custom.h"

struct sensor_t sSensorList[MAX_NUM_SENSORS] = 
{
	{ 
		.name       = "LGE Orientation Sensor",
		.vendor     = "Bosch",
		.version    = 1,
		.handle     = ID_ORIENTATION,
		.type       = SENSOR_TYPE_ORIENTATION,
		.maxRange   = 360.0f,
		.resolution = 1.0f,
		.power      = 0.53f,
		.reserved   = {}
	},

	{
		.name       = "LGE Magnetic Sensor",
		.vendor     = "Bosch",
		.version    = 1,
		.handle     = ID_MAGNETIC,
		.type       = SENSOR_TYPE_MAGNETIC_FIELD,
		.maxRange   = 1600.0f,
		.resolution = 0.3f,
		.power      = 0.5f,
		.reserved   = {}
	},

	{
		.name       = "LGE Accelerometer Sensor",
		.vendor     = "Bosch",
		.version    = 1,
		.handle     = ID_ACCELEROMETER,
		.type       = SENSOR_TYPE_ACCELEROMETER,
		.maxRange   = 4.0f*9.805,
		.resolution = 4.0f*9.805/1024.0f,
		.power      = 0.03f,
		.reserved   = {}
	},

	{
		.name       = "LGE Proximity Sensor",
		.vendor     = "Avago",
		.version    = 1,
		.handle     = ID_PROXIMITY,
		.type       = SENSOR_TYPE_PROXIMITY,
		.maxRange   = 1.00f,
		.resolution = 1.0f,
		.power      = 0.13f,
		.reserved   = {}
	},
};

struct sensor_t sSensorList_revB[MAX_NUM_SENSORS] =
{
	{
		.name       = "LGE Orientation Sensor",
		.vendor     = "Bosch",
		.version    = 1,
		.handle     = ID_ORIENTATION,
		.type       = SENSOR_TYPE_ORIENTATION,
		.maxRange   = 360.0f,
		.resolution = 1.0f,
		.power      = 0.25f,
		.reserved   = {}
	},

	{ 
		.name       = "LGE Magnetometer Sensor",
		.vendor     = "Bosch",
		.version    = 1,
		.handle     = ID_MAGNETIC,
		.type       = SENSOR_TYPE_MAGNETIC_FIELD,
		.maxRange   = 600.0f,
		.resolution = 0.0016667f,
		.power      = 0.25f,
		.reserved   = {}
	}, 

	
	{  
		.name       = "LGE Accelerometer Sensor",
		.vendor     = "Bosch",
		.version    = 1,
		.handle     = ID_ACCELEROMETER,
		.type       = SENSOR_TYPE_ACCELEROMETER,
		.maxRange   = 32.0f,
		.resolution = 4.0f/1024.0f,
		.power      =130.0f/1000.0f,
		.reserved   = {}
	},        

	{ 
		.name       = "LGE Proximity Sensor",
		.vendor     = "Avago",
		.version    = 1,
		.handle     = ID_PROXIMITY,
		.type       = SENSOR_TYPE_PROXIMITY,
		.maxRange   = 1.00f,
		.resolution = 1.0f,
		.power      = 0.13f,
		.reserved   = {}
	},
};

