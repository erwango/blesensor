#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/util.h>

//Include MotionFX (Quaternions including Compass) libraries headers
#include "../inc/STM32_MotionFX_Library/Inc/motion_fx.h"
#include "../inc/STM32_MotionFX_Library/Inc/motion_fx_cm0p.h"
#include "MotionFX_Manager.h"

#define FROM_UT50_TO_MGAUSS 500.0f
#define FROM_MGAUSS_TO_UT50 (0.1f/50.0f)

static uint32_t mag_time_stamp = 0;

//Quaternions Service and Feature UUIDs
static struct bt_uuid_128 feature_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
static struct bt_uuid_128 quaternions_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00);

// Quaternions Variable declarations
static struct bt_gatt_ccc_cfg quat_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t quat_buf[8];
static u8_t quat_ind;
static u8_t indicating;
static struct bt_gatt_indicate_params ind_params;
struct sensor_value accel_xyz[3], gyro_xyz[3], magn_xyz[3];
struct device *lis3mdl;
struct device *lsm6ds0;

//Compass Service UUIDs
static struct bt_uuid_128 compass_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x40, 0x00, 0x00, 0x00);

//Compass Variable declarations
static struct bt_gatt_ccc_cfg comp_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t comp_buf[4];
static u8_t comp_ind;
static u8_t indicating2;
static struct bt_gatt_indicate_params ind_params2;


// Library Variables declarations
MFX_MagCal_input_t mag_data_in;
MFX_MagCal_output_t magOffset;
static SensorAxes_t quat_axes;
SensorAxes_t MAG_Offset;
SensorAxesRaw_t ACC_Value_Raw;
SensorAxes_t GYR_Value;
SensorAxes_t MAG_Value;

//Quaternions bluetooth fonctions
static void quat_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	quat_ind = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

//Quaternions thread indication
static void indicate_quat(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			u8_t err)
{
	printk("Quaternions indication %s\n", err != 0U ? "fail" : "success");
	indicating = 0U;
}

//Compass bluetooth fonctions
static void comp_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	comp_ind = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

//Compass thread indication
static void indicate_comp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			u8_t err)
{
	printk("Compass indication %s\n", err != 0U ? "fail" : "success");
	indicating2 = 0U;
}

//Bluetooth characteristics declaration
BT_GATT_SERVICE_DEFINE(quat_svc,
	BT_GATT_PRIMARY_SERVICE(&feature_service_uuid),
  BT_GATT_CHARACTERISTIC(&quaternions_uuid.uuid,
						 BT_GATT_CHRC_INDICATE,
						 BT_GATT_PERM_NONE,
						 NULL, NULL, NULL),
  BT_GATT_CCC(quat_ccc_cfg, quat_ccc_cfg_changed),
  BT_GATT_CHARACTERISTIC(&compass_uuid.uuid,
						 BT_GATT_CHRC_INDICATE,
						 BT_GATT_PERM_NONE,
						 NULL, NULL, NULL),
  BT_GATT_CCC(comp_ccc_cfg, comp_ccc_cfg_changed),

);

//Initialize Quaternions Service
void init_quat(void){

	lis3mdl = device_get_binding(DT_INST_0_ST_LIS3MDL_MAGN_LABEL);
	lsm6ds0 = device_get_binding(DT_INST_0_ST_LSM6DS0_LABEL);

	if (lis3mdl == NULL) {
		printf("Could not get LIS3MDL device\n");
		return;
	}
	if (lsm6ds0 == NULL) {
		printf("Could not get LSM6DS0 device\n");
		return;
	}

	//Fonction to initialize the use of MotionFX library function
	MotionFX_manager_init();
	MotionFX_manager_start_9X();

}


//Update quaternions including Compass
static void update_quat(){

	if (sensor_sample_fetch(lis3mdl) < 0) {
  	printf("LIS3MDL Sensor sample update error\n");
  	return;
  }
  if (sensor_sample_fetch(lsm6ds0) < 0) {
  	printf("LSM6DS0 Sensor sample update error\n");
  	return;
  }

	/* Erase previous */
	printf("\0033\014");

	sensor_channel_get(lis3mdl, SENSOR_CHAN_MAGN_XYZ, magn_xyz);
	sensor_channel_get(lsm6ds0, SENSOR_CHAN_ACCEL_XYZ, accel_xyz);
	sensor_channel_get(lsm6ds0, SENSOR_CHAN_GYRO_XYZ, gyro_xyz);

	float m_x = sensor_value_to_double(&magn_xyz[0]);
	float m_y = sensor_value_to_double(&magn_xyz[1]);
	float m_z = sensor_value_to_double(&magn_xyz[2]);

	float a_x = sensor_value_to_double(&accel_xyz[0]);
	float a_y = sensor_value_to_double(&accel_xyz[1]);
	float a_z = sensor_value_to_double(&accel_xyz[2]);

	float g_x = sensor_value_to_double(&gyro_xyz[0]);
	float g_y = sensor_value_to_double(&gyro_xyz[1]);
	float g_z = sensor_value_to_double(&gyro_xyz[2]);

	/* magneto data */
	printf(
		"LIS3MDL: Magnetic field (gauss): x: %.1f, y: %.1f, z: %.1f\n",
		m_x, m_y, m_z);

	/* acceleration */
	printf(
		"LSM6DS0: Acceleration (m.s-2): x: %.1f, y: %.1f, z: %.1f\n",
		a_x, a_y, a_z);

	/* gyroscope */
	printf(
		"LSM6DS0: Gyroscope (dps): x: %.3f, y: %.3f, z: %.3f\n",
		g_x, g_y, g_z);

	// set to mG
	m_x = m_x * 1000.0;
	m_y = m_y * 1000.0;
	m_z = m_z * 1000.0;

	//Load the mag data to calibrate
	mag_data_in.mag[0] = m_x * FROM_MGAUSS_TO_UT50;
	mag_data_in.mag[1] = m_y * FROM_MGAUSS_TO_UT50;
	mag_data_in.mag[2] = m_z * FROM_MGAUSS_TO_UT50;
	mag_data_in.time_stamp = mag_time_stamp;
	mag_time_stamp += SAMPLE_PERIOD;

	//Calibrate the magnetometer
	MotionFX_manager_MagCal_run(&mag_data_in, &magOffset);

	/* Control the calibration status */
	if( (magOffset.cal_quality == MFX_MAGCALOK) ||
			(magOffset.cal_quality == MFX_MAGCALGOOD) )
	{

		MAG_Offset.AXIS_X= (int32_t)(magOffset.hi_bias[0] * FROM_UT50_TO_MGAUSS);
		MAG_Offset.AXIS_Y= (int32_t)(magOffset.hi_bias[1] * FROM_UT50_TO_MGAUSS);
		MAG_Offset.AXIS_Z= (int32_t)(magOffset.hi_bias[2] * FROM_UT50_TO_MGAUSS);

		/* Disable magnetometer calibration */
		MotionFX_manager_MagCal_stop(SAMPLE_PERIOD);
	}

	//Parse mag acc gyr values
  GYR_Value.AXIS_X = (int16_t)sys_cpu_to_le16(g_x);
  GYR_Value.AXIS_Y = (int16_t)sys_cpu_to_le16(g_y);
  GYR_Value.AXIS_Z = (int16_t)sys_cpu_to_le16(g_z);

	ACC_Value_Raw.AXIS_X = (int32_t)sys_cpu_to_le32(a_x);
  ACC_Value_Raw.AXIS_Y = (int32_t)sys_cpu_to_le32(a_y);
	ACC_Value_Raw.AXIS_Z = (int32_t)sys_cpu_to_le32(a_z);

	MAG_Value.AXIS_X = (int16_t)sys_cpu_to_le16(m_x);
	MAG_Value.AXIS_Y  = (int16_t)sys_cpu_to_le16(m_x);
	MAG_Value.AXIS_Z = (int16_t)sys_cpu_to_le16(m_x);

	//Transform acc gyr mag values to quaternions values
	MotionFX_manager_run(ACC_Value_Raw, GYR_Value, MAG_Value);

  /* Read the quaternions */
  MFX_output_t *MotionFX_Engine_Out = MotionFX_manager_getDataOUT();

	// Set the quaternions data in format
	if(MotionFX_Engine_Out->quaternion_9X[3] < 0){
		quat_axes.AXIS_X = (int32_t)(MotionFX_Engine_Out->quaternion_9X[0] * (-10000));
		quat_axes.AXIS_Y = (int32_t)(MotionFX_Engine_Out->quaternion_9X[1] * (-10000));
		quat_axes.AXIS_Z = (int32_t)(MotionFX_Engine_Out->quaternion_9X[2] * (-10000));
	} else {
		quat_axes.AXIS_X = (int32_t)(MotionFX_Engine_Out->quaternion_9X[0] * 10000);
		quat_axes.AXIS_Y = (int32_t)(MotionFX_Engine_Out->quaternion_9X[1] * 10000);
		quat_axes.AXIS_Z = (int32_t)(MotionFX_Engine_Out->quaternion_9X[2] * 10000);
	}

	//Save Quaternions values
	u8_t buf_pos;
	memcpy(quat_buf, quat_axes.AXIS_X, 2); //init buf value
	buf_pos = 2U;
	memcpy(quat_buf+buf_pos, quat_axes.AXIS_X, 2);
	buf_pos += 2U;
	memcpy(quat_buf+buf_pos, quat_axes.AXIS_Y, 2);
	buf_pos += 2U;
	memcpy(quat_buf+buf_pos, quat_axes.AXIS_Z, 2);


	//Get Compass Value
	uint16_t Angle = (uint16_t)trunc(100*MotionFX_Engine_Out->heading_9X);
	//Save Compass value
	memcpy(comp_buf, Angle, 2);
	memcpy(comp_buf+2U, Angle, 2);

	//Send Quaternions buffer
	ind_params.attr = &quat_svc.attrs[2];
	ind_params.func = indicate_quat;
	ind_params.data = &quat_buf;
	ind_params.len = sizeof(quat_buf);
	if (bt_gatt_indicate(NULL, &ind_params) == 0) {
		indicating = 1U;
	}

	//Send Compass buffer
	ind_params2.attr = &quat_svc.attrs[4];
	ind_params2.func = indicate_comp;
	ind_params2.data = &comp_buf;
	ind_params2.len = sizeof(comp_buf);
	if (bt_gatt_indicate(NULL, &ind_params2) == 0) {
		indicating2 = 1U;
	}

}

// Quaternions thread
void quat_indicate(void){
	if (quat_ind){
		if(indicating){
			return;
		}
		k_sleep(MSEC_PER_SEC);
		update_quat();
		k_sleep(100);
	}
}


// Compass thread
void comp_indicate(void){
	if (comp_ind){
		if(indicating2){
			return;
		}
		k_sleep(MSEC_PER_SEC);
		update_quat();
		k_sleep(100);
	}
}
