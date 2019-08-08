
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

#include <gatt/hrs.h>
#include <gatt/bas.h>
#include <gatt/cts.h>

#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <sys/util.h>

#include "main.h"

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
/*#define FROM_MDPS_TO_DPS    0.001
#define FROM_MGAUSS_TO_UT50 (0.1f/50.0f)
#define SEND_N_QUATERNIONS 3*/
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
//#define W2ST_CONSOLE_MAX_CHAR_LEN 20

static uint8_t manuf_data[] = {
	0x01, 0x80, 0x5C, 0xFD, 0x01, 0x5A,
	0xC0, 0x86, 0x50, 0x35, 0x31, 0x42};

/*static uint8_t manuf_data[] = {
	2, 0x0A, 0x00, // Trasmission Power 0 dBm
	8, 0x09, 0x41, 0x4D, 0x31 ,0x56, 0x33 ,0x34, 0x30, //Node name
	13, 0xFF, 0x01, //SDK Version
	0x80, //stm32 nucleo
	0x04, // AudioSync+AudioData
	0xEF, //ACC + Gyro + Mag + Environmental + Battery Info
	0x0F, //Hardware Events + MotionFX + SD Card Logging
  0x00, //
	0xC0, 0x86, 0x50, 0x35, 0x31, 0x42}; //Mac address


static uint8_t LastStderrBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastStderrLen;
static uint8_t LastTermBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastTermLen;

int32_t bytesToWrite;
uint8_t bufferToWrite[256];

typedef struct
{
  int32_t AXIS_X;
  int32_t AXIS_Y;
  int32_t AXIS_Z;
} SensorAxes_t;

static SensorAxes_t quaternions_value = {.AXIS_X = 1, .AXIS_Y = 9, .AXIS_Z = 19};

//MFX_MagCal_input_t mag_data_in;
//MFX_MagCal_output_t magOffset;
SensorAxes_t MAG_Offset;
SensorAxes_t ACC_Value_Raw;
SensorAxes_t GYR_Value;
SensorAxes_t MAG_Value;

*/

/*
** Feature services
*/
/*static struct bt_uuid_128 feature_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);

/* Base feature */

/*static struct bt_uuid_128 environmental_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x10, 0x00);

static struct bt_uuid_128 acc_gyro_mag_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0xE0, 0x00);

static struct bt_uuid_128 audio_ADPCM_Sync_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x40);

static struct bt_uuid_128 switch_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x20);

static struct bt_uuid_128 audio_source_localization_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10);


static struct bt_uuid_128 audio_ADPCM_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x08);

static struct bt_uuid_128 mic_level_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x04);

static struct bt_uuid_128 proximity_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02);

static struct bt_uuid_128 luminosity_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01);


static struct bt_uuid_128 acceleration_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x80, 0x00);

static struct bt_uuid_128 gyroscope_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x40, 0x00);

static struct bt_uuid_128 magnetometer_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x20, 0x00);

static struct bt_uuid_128 pressure_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x10, 0x00);


static struct bt_uuid_128 humidity_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x08, 0x00);

static struct bt_uuid_128 temperature_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00);

static struct bt_uuid_128 battery_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00);

static struct bt_uuid_128 temperature_bis_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00);


static struct bt_uuid_128 co_sensor_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x80, 0x00, 0x00);

static struct bt_uuid_128 sd_logging_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x10, 0x00, 0x00);


static struct bt_uuid_128 beam_forming_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x08, 0x00, 0x00);

static struct bt_uuid_128 acceleration_event_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x04, 0x00, 0x00);

static struct bt_uuid_128 free_fall_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00);

static struct bt_uuid_128 quaternions_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00);


static struct bt_uuid_128 mems_sensor_fusion_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x80, 0x00, 0x00, 0x00);

static struct bt_uuid_128 compass_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x40, 0x00, 0x00, 0x00);

static struct bt_uuid_128 motion_intensity_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x20, 0x00, 0x00, 0x00);

static struct bt_uuid_128 activity_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x10, 0x00, 0x00, 0x00);


static struct bt_uuid_128 carry_position_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x08, 0x00, 0x00, 0x00);

static struct bt_uuid_128 proximity_gesture_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00);

static struct bt_uuid_128 mems_gesture_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00);

static struct bt_uuid_128 pedometer_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x10, 0x00, 0x00, 0x00);


/*
** Debug service
*/
/*static struct bt_uuid_128 debug_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00);

/* Read and write outpout commands characteristic */
/*static struct bt_uuid_128 debug_term_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x0e, 0x00, 0x01, 0x00, 0x00, 0x00);

/* Error message characteristic */
/*static struct bt_uuid_128 debug_stderr_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x0e, 0x00, 0x02, 0x00, 0x00, 0x00);

/*
** Common config control service
*/
/*static struct bt_uuid_128 config_control_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00);

/* Manage register characteristic */
/*static struct bt_uuid_128 register_access_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x0f, 0x00, 0x01, 0x00, 0x00, 0x00);

/* send command to feature characteristc */
/*static struct bt_uuid_128 feature_command_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x0f, 0x00, 0x02, 0x00, 0x00, 0x00);*



static u8_t quat_value[3];
static u8_t env_value[12];
static u8_t hum_value;
static u8_t env_update;
static u8_t hum_update;
static u8_t quat_update;
static u8_t term_value;
static u8_t term_update;



//hum_value = env_value[6];
static ssize_t read_quat(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, u16_t len, u16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(quat_value));
}

static ssize_t read_env(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, u16_t len, u16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(env_value));
}

static ssize_t write_env(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, u16_t len, u16_t offset)
{
	struct sensor_value *value = attr->user_data;

	if (offset + len > sizeof(env_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	env_update = 1U;

	return len;
}

static ssize_t read_hum(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, u16_t len, u16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(hum_value));
}

static ssize_t write_hum(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, u16_t len, u16_t offset)
{
	struct sensor_value *value = attr->user_data;

	if (offset + len > sizeof(hum_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	hum_update = 1U;

	return len;
}

static ssize_t write_quat(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, u16_t len, u16_t offset)
{
	struct sensor_value *value = attr->user_data;

	if (offset + len > sizeof(env_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	quat_update = 1U;

	return len;
}

static struct bt_gatt_ccc_cfg env_ccc_cfg[BT_GATT_CCC_MAX] = {};
static struct bt_gatt_ccc_cfg hum_ccc_cfg[BT_GATT_CCC_MAX] = {};

static void env_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
}
static void hum_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
}

static ssize_t write_without_rsp_term(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr,
				     const void *buf, u16_t len, u16_t offset,
				     u8_t flags)
{
	u8_t *value = attr->user_data;

	/* Write request received. Reject it since this char only accepts
	 * Write Commands.
	 */
	/*if (!(flags & BT_GATT_WRITE_FLAG_CMD)) {
		return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
	}

	if (offset + len > sizeof(term_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}


/* App Primary Service Declaration */
/*BT_GATT_SERVICE_DEFINE(vnd_svc,
	BT_GATT_PRIMARY_SERVICE(&feature_service_uuid),

	BT_GATT_CHARACTERISTIC(&environmental_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_READ,
						 read_env, write_env,
						 env_value),
	BT_GATT_CCC(env_ccc_cfg, env_ccc_cfg_changed),

	/*BT_GATT_CHARACTERISTIC(&acc_gyro_mag_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_term,
						 &acc_gyro_mag_handle),

	BT_GATT_CHARACTERISTIC(&audio_ADPCM_Sync_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_term,
						 &audio_ADPCM_Sync_handle),
	/*BT_GATT_CHARACTERISTIC(&switch_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_term, &vnd_value),*/
	/*BT_GATT_CHARACTERISTIC(&audio_source_localization_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_term,
						 &audio_source_localization_handle),

	BT_GATT_CHARACTERISTIC(&audio_ADPCM_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_term,
						 &audio_ADPCM_handle),
	BT_GATT_CHARACTERISTIC(&mic_level_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_term,
						 &mic_level_handle),
	/*BT_GATT_CHARACTERISTIC(&proximity_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_term, &vnd_value),
  BT_GATT_CHARACTERISTIC(&luminosity_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_term, &vnd_value),

	BT_GATT_CHARACTERISTIC(&acceleration_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_term, &vnd_value),
	BT_GATT_CHARACTERISTIC(&gyroscope_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_term, &vnd_value),
	BT_GATT_CHARACTERISTIC(&magnetometer_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_term, &vnd_value),*/
	/*BT_GATT_CHARACTERISTIC(&pressure_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_READ,
						 read_env, write_env,
						 env_value),*/

	/*BT_GATT_CHARACTERISTIC(&humidity_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_READ,
						 read_hum, write_hum,
						 env_value),
	BT_GATT_CCC(hum_ccc_cfg, hum_ccc_cfg_changed),
  /*BT_GATT_CHARACTERISTIC(&temperature_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_READ,
						 read_env, write_env,
						 env_value),
	/*BT_GATT_CHARACTERISTIC(&battery_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_READ,
						 read_env, write_env,
						 &environmental_value),*/
	/*BT_GATT_CHARACTERISTIC(&temperature_bis_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_READ,
						 read_env, write_env,
						 env_value),

	/*BT_GATT_CHARACTERISTIC(&co_sensor_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_term, &vnd_value),*/
	/*BT_GATT_CHARACTERISTIC(&sd_logging_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE |
						 BT_GATT_CHRC_READ,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_term,
						 &sd_logging_handle),

	BT_GATT_CHARACTERISTIC(&beam_forming_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_term,
						 &beam_forming_handle),
	BT_GATT_CHARACTERISTIC(&acceleration_event_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_term,
						 &acceleration_event_handle),
  /*BT_GATT_CHARACTERISTIC(&free_fall_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_term, &vnd_value),*/
	/*BT_GATT_CHARACTERISTIC(&quaternions_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY,
						 BT_GATT_PERM_READ,
						 read_quat, write_quat,
						 quat_value),

  /*BT_GATT_CHARACTERISTIC(&mems_sensor_fusion_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_term, &vnd_value),*/
	/*BT_GATT_CHARACTERISTIC(&compass_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_term,
						 &compass_handle),
  /*BT_GATT_CHARACTERISTIC(&motion_intensity_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_term, &vnd_value),*/
	/*BT_GATT_CHARACTERISTIC(&activity_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_term,
						 &activity_handle),

	BT_GATT_CHARACTERISTIC(&carry_position_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_term,
						 &carry_position_handle),
  /*BT_GATT_CHARACTERISTIC(&proximity_gesture_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_term, &vnd_value),
  BT_GATT_CHARACTERISTIC(&mems_gesture_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_NONE,
						 NULL,
						 &mems_gesture_handle),
  BT_GATT_CHARACTERISTIC(&pedometer_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_term, &vnd_value), */


	/*BT_GATT_PRIMARY_SERVICE(&debug_service_uuid),
	BT_GATT_CHARACTERISTIC(&debug_term_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE_WITHOUT_RESP |
						 BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_term,
						 term_value),
	BT_GATT_CHARACTERISTIC(&debug_stderr_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_WRITE,
						 NULL, write_without_rsp_term,
						 stderr_value),


	BT_GATT_PRIMARY_SERVICE(&config_control_service_uuid),
	BT_GATT_CHARACTERISTIC(&register_access_uuid.uuid,
						 BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE |
						 BT_GATT_CHRC_INDICATE,
						 BT_GATT_PERM_READ_ENCRYPT |
						 BT_GATT_PERM_WRITE_ENCRYPT,
						 read_env, write_env, env_value),
	BT_GATT_CHARACTERISTIC(&feature_command_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_term,
						 env_value),

);*/


static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	//BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x0d, 0x18, 0x0f, 0x18, ),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA_BYTES(BT_DATA_TX_POWER, 0x00),
	//BT_DATA(BT_DATA_NAME_SHORTENED, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, 12),
	/*BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA,
		0x01, //SDK Version
		0x80, //stm32 nucleo
		0x04, // AudioSync+AudioData
		0xEF, //ACC + Gyro + Mag + Environmental + Battery Info
		0x0F, //Hardware Events + MotionFX + SD Card Logging
	  0x00, //
		0xC0, 0x86, 0x50, 0x35, 0x31, 0x42), //Mac address)*/
};

static void connected(struct bt_conn *conn, u8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
	} else {
		printk("\n------------- Connected -------------\n");
	}
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	printk("\n-------------Disconnected (reason %u)-------------\n", reason);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};


static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	hrs_init(0x01);
	cts_init();


	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
};


/*void quat_update(SensorAxes_t *quat_data)
{
  uint8_t buff[2+ 6*SEND_N_QUATERNIONS];
	STORE_LE_16(buff, (0>>3));

  STORE_LE_16(buff+2,quat_data[0].AXIS_X);
  STORE_LE_16(buff+4,quat_data[0].AXIS_Y);
  STORE_LE_16(buff+6,quat_data[0].AXIS_Z);

  STORE_LE_16(buff+8 ,quat_data[1].AXIS_X);
  STORE_LE_16(buff+10,quat_data[1].AXIS_Y);
  STORE_LE_16(buff+12,quat_data[1].AXIS_Z);

  STORE_LE_16(buff+14,quat_data[2].AXIS_X);
  STORE_LE_16(buff+16,quat_data[2].AXIS_Y);
  STORE_LE_16(buff+18,quat_data[2].AXIS_Z);

}



}

/*static void generate_current_env(u8_t *buf)
{
	buf[0] = 5U;
	buf[2] = 5U;
	buf[6] = 5U;
	buf[8] = 5U;
	buf[10] = 5U;

	printk("JEEE RENTTTTTTTTTTTTTTTTRE");

	bytesToWrite = sprintf((char *)bufferToWrite,"Sending: ");
	Term_Update(bufferToWrite,bytesToWrite);

	bytesToWrite = sprintf((char *)bufferToWrite,"Press=%ld ",buf[2]);
	Term_Update(bufferToWrite,bytesToWrite);

	bytesToWrite = sprintf((char *)bufferToWrite,"Hum=%d ",buf[6]);
	Term_Update(bufferToWrite,bytesToWrite);

	bytesToWrite = sprintf((char *)bufferToWrite,"Temp=%d ",buf[8]);
	Term_Update(bufferToWrite,bytesToWrite);

	bytesToWrite = sprintf((char *)bufferToWrite,"Temp2=%d ",buf[10]);
	Term_Update(bufferToWrite,bytesToWrite);

}



void env_init(void){

	generate_current_env(env_value);

}


static void update_environment(double pr, double hu, double tp2, double tp1)
{
	static u16_t tick;
	u32_t press = sys_cpu_to_le32(pr);
	u16_t hum = sys_cpu_to_le16(hu);
	u16_t temp2 = sys_cpu_to_le16(tp2);
	u16_t temp1 = sys_cpu_to_le16(tp1);

	memcpy(hum_value, &hum, 2);





	/*memcpy(env_value, &tick, 2);
	memcpy(env_value[2], &press, 4);
  memcpy(env_value[6], &hum, 2);
  memcpy(env_value[8], &temp2, 2);
  memcpy(env_value[10], &temp1, 2);

}



void env_notify(void){
	if (!env_update){
		return;
	}
	env_update = 0U;
	printk("JE RENTRE MAIS");
	bt_gatt_notify(NULL, &vnd_svc.attrs[1], &env_value, sizeof(env_value));

}

void hum_notify(void){
	if (!hum_update){
		return;
	}
	hum_update = 0U;
	bt_gatt_notify(NULL, &vnd_svc.attrs[2], &hum_value, sizeof(hum_value));

}*/



void main(void)
{

	int err;
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_conn_cb_register(&conn_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);

	/* Implement notification. At the moment there is no suitable way
	 * of starting delayed work so we do it here
	 */

	while(1){

		k_sleep(MSEC_PER_SEC);

		env_indicate();
		stderr_notify();
		cmd_notify();
		reg_notify();
		bat_notify();
		quat_notify();

		k_sleep(1000);

	}
	//term_notify();



}
