
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

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static uint8_t manuf_data[] = {
	0x01, 0x80, 0x5C, 0xFD, 0x01, 0x5A,
	0xC0, 0x86, 0x50, 0x35, 0x31, 0x42};
static uint16_t environmental_handle;
static uint16_t acc_gyro_mag_handle;
static uint16_t acceleration_event_handle;
static uint16_t mic_level_handle;
static uint16_t battery_handle;
static uint16_t mems_sensor_fusion_compact_handle;
static uint16_t compass_handle;
static uint16_t activity_handle;
static uint16_t carry_position_handle;
static uint16_t mems_gesture_handle;
uint16_t audio_source_localization_handle;
uint16_t beam_forming_handle;
static uint16_t sd_logging_handle;
static uint16_t feature_command_handle;
static uint16_t debug_term_handle;
static uint16_t debug_stderr_handle;
static uint16_t audio_ADPCM_handle;
static uint16_t audio_ADPCM_Sync_handle;


/*
** Feature services
*/
static struct bt_uuid_128 feature_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);

/* Base feature */

static struct bt_uuid_128 environmental_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);

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

static struct bt_uuid_128 mems_sensor_fusion_compact_uuid = BT_UUID_INIT_128(
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
static struct bt_uuid_128 debug_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00);

/* Read and write outpout commands characteristic */
static struct bt_uuid_128 debug_term_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x0e, 0x00, 0x01, 0x00, 0x00, 0x00);

/* Error message characteristic */
static struct bt_uuid_128 debug_stderr_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x0e, 0x00, 0x02, 0x00, 0x00, 0x00);

/*
** Common config control service
*/
static struct bt_uuid_128 config_control_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00);

/* Manage register characteristic */
static struct bt_uuid_128 register_access_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x0f, 0x00, 0x01, 0x00, 0x00, 0x00);

/* send command to feature characteristc */
static struct bt_uuid_128 feature_command_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x0f, 0x00, 0x02, 0x00, 0x00, 0x00);



static u8_t vnd_value[] = { 'V', 'e', 'n', 'd', 'o', 'r' };

static ssize_t read_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, u16_t len, u16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

static ssize_t write_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, u16_t len, u16_t offset,
			 u8_t flags)
{
	u8_t *value = attr->user_data;

	if (offset + len > sizeof(vnd_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

static struct bt_gatt_ccc_cfg vnd_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t simulate_vnd;
static u8_t indicating;
static struct bt_gatt_indicate_params ind_params;

static void vnd_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	simulate_vnd = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

static void indicate_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			u8_t err)
{
	printk("Indication %s\n", err != 0U ? "fail" : "success");
	indicating = 0U;
}


static ssize_t write_without_rsp_vnd(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr,
				     const void *buf, u16_t len, u16_t offset,
				     u8_t flags)
{
	u8_t *value = attr->user_data;

	/* Write request received. Reject it since this char only accepts
	 * Write Commands.
	 */
	if (!(flags & BT_GATT_WRITE_FLAG_CMD)) {
		return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
	}

	if (offset + len > sizeof(vnd_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

/* Vendor Primary Service Declaration */
BT_GATT_SERVICE_DEFINE(vnd_svc,
	BT_GATT_PRIMARY_SERVICE(&feature_service_uuid),

	BT_GATT_CHARACTERISTIC(&environmental_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_vnd,
						 &environmental_handle),

	BT_GATT_CCC(vnd_ccc_cfg, vnd_ccc_cfg_changed),

	BT_GATT_CHARACTERISTIC(&acc_gyro_mag_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY, BT_GATT_CHRC_NOTIFY,
						 NULL, write_without_rsp_vnd,
						 &acc_gyro_mag_handle),

	BT_GATT_CHARACTERISTIC(&audio_ADPCM_Sync_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_vnd,
						 &audio_ADPCM_Sync_handle),
	/*BT_GATT_CHARACTERISTIC(&switch_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value),*/
	BT_GATT_CHARACTERISTIC(&audio_source_localization_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_vnd,
						 &audio_source_localization_handle),

	BT_GATT_CHARACTERISTIC(&audio_ADPCM_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_vnd,
						 &audio_ADPCM_handle),
	BT_GATT_CHARACTERISTIC(&mic_level_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_vnd,
						 &mic_level_handle),
	/*BT_GATT_CHARACTERISTIC(&proximity_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value),
  BT_GATT_CHARACTERISTIC(&luminosity_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value),

	BT_GATT_CHARACTERISTIC(&acceleration_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value),
	BT_GATT_CHARACTERISTIC(&gyroscope_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value),
	BT_GATT_CHARACTERISTIC(&magnetometer_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value),
	BT_GATT_CHARACTERISTIC(&pressure_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value),

	BT_GATT_CHARACTERISTIC(&humidity_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value),
  BT_GATT_CHARACTERISTIC(&temperature_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value),*/
	BT_GATT_CHARACTERISTIC(&battery_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_vnd,
						 &battery_handle),
	/*BT_GATT_CHARACTERISTIC(&temperature_bis_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value),

	BT_GATT_CHARACTERISTIC(&co_sensor_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value),*/
	BT_GATT_CHARACTERISTIC(&sd_logging_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE |
						 BT_GATT_CHRC_READ,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_vnd,
						 &sd_logging_handle),

	BT_GATT_CHARACTERISTIC(&beam_forming_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_vnd,
						 &beam_forming_handle),
	BT_GATT_CHARACTERISTIC(&acceleration_event_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_vnd,
						 &acceleration_event_handle),
  /*BT_GATT_CHARACTERISTIC(&free_fall_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value),*/
	BT_GATT_CHARACTERISTIC(&mems_sensor_fusion_compact_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_vnd,
						 &mems_sensor_fusion_compact_handle),

  /*BT_GATT_CHARACTERISTIC(&mems_sensor_fusion_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value),*/
	BT_GATT_CHARACTERISTIC(&compass_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_vnd,
						 &compass_handle),
  /*BT_GATT_CHARACTERISTIC(&motion_intensity_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value),*/
	BT_GATT_CHARACTERISTIC(&activity_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_vnd,
						 &activity_handle),

	BT_GATT_CHARACTERISTIC(&carry_position_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_vnd,
						 &carry_position_handle),
  /*BT_GATT_CHARACTERISTIC(&proximity_gesture_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value),
  BT_GATT_CHARACTERISTIC(&mems_gesture_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_NONE,
						 NULL,
						 &mems_gesture_handle),
  BT_GATT_CHARACTERISTIC(&pedometer_uuid.uuid,
						 BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_vnd, &vnd_value), */


	BT_GATT_PRIMARY_SERVICE(&debug_service_uuid),
	BT_GATT_CHARACTERISTIC(&debug_term_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE_WITHOUT_RESP |
						 BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_NONE, NULL,
						 write_without_rsp_vnd,
						 &debug_term_handle),
	BT_GATT_CHARACTERISTIC(&debug_stderr_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_NONE,
						 NULL, write_without_rsp_vnd,
						 &debug_stderr_handle),


	BT_GATT_PRIMARY_SERVICE(&config_control_service_uuid),
	/*BT_GATT_CHARACTERISTIC(&register_access_uuid.uuid,
						 BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE |
						 BT_GATT_CHRC_INDICATE,
						 BT_GATT_PERM_READ_ENCRYPT |
						 BT_GATT_PERM_WRITE_ENCRYPT,
						 read_vnd, write_vnd, vnd_value), */
	BT_GATT_CHARACTERISTIC(&feature_command_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
						 BT_GATT_PERM_NONE, NULL,
						 write_without_rsp_vnd,
						 &feature_command_handle),

);


static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA_BYTES(BT_DATA_TX_POWER, 0x00),
	//BT_DATA(BT_DATA_NAME_SHORTENED, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, 12),
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

void main(void)
{

	struct sensor_value temp, hum, press;
	struct sensor_value magn_xyz[3], accel_xyz[3];
	struct device *hts221 = device_get_binding(DT_INST_0_ST_HTS221_LABEL);
	struct device *lis3mdl = device_get_binding(DT_INST_0_ST_LIS3MDL_MAGN_LABEL);
	struct device *lsm6ds0 = device_get_binding(DT_INST_0_ST_LSM6DS0_LABEL);
	struct device *lps25hb = device_get_binding(DT_INST_0_ST_LPS25HB_PRESS_LABEL);
	int err;

	if (hts221 == NULL) {
		printf("Could not get HTS221 device\n");
		return;
	}
	if (lis3mdl == NULL) {
		printf("Could not get LIS3MDL device\n");
		return;
	}
	if (lsm6ds0 == NULL) {
		printf("Could not get LSM6DS0 device\n");
		return;
	}
	if (lps25hb == NULL) {
		printf("Could not get LPS25HB device\n");
		return;
	}

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
	while (1) {
		k_sleep(MSEC_PER_SEC);

		/* Get sensor samples */

		if (sensor_sample_fetch(hts221) < 0) {
			printf("HTS221 Sensor sample update error\n");
			return;
		}
		if (sensor_sample_fetch(lps25hb) < 0) {
			printf("LPS25HB Sensor sample update error\n");
			return;
		}
		if (sensor_sample_fetch(lis3mdl) < 0) {
			printf("LIS3MDL Sensor sample update error\n");
			return;
		}
		if (sensor_sample_fetch(lsm6ds0) < 0) {
			printf("LSM6DS0 Sensor sample update error\n");
			return;
		}

		/* Get sensor data */

		sensor_channel_get(hts221, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(hts221, SENSOR_CHAN_HUMIDITY, &hum);
		sensor_channel_get(lps25hb, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(lis3mdl, SENSOR_CHAN_MAGN_XYZ, magn_xyz);
		sensor_channel_get(lsm6ds0, SENSOR_CHAN_ACCEL_XYZ, accel_xyz);

		/* Display sensor data */

		/* Erase previous */
		printf("\0033\014");

		printf("X-NUCLEO-IKS01A1 sensor dashboard\n\n");

		/* temperature */
		printf("HTS221: Temperature: %.1f C\n",
					 sensor_value_to_double(&temp));

		/* humidity */
		printf("HTS221: Relative Humidity: %.1f%%\n",
					 sensor_value_to_double(&hum));

		/* pressure */
		printf("LPS25HB: Pressure:%.1f kpa\n",
					 sensor_value_to_double(&press));

		/* magneto data */
		printf(
		 "LIS3MDL: Magnetic field (gauss): x: %.1f, y: %.1f, z: %.1f\n",
		 sensor_value_to_double(&magn_xyz[0]),
		 sensor_value_to_double(&magn_xyz[1]),
		 sensor_value_to_double(&magn_xyz[2]));

		/* acceleration */
		printf(
			 "LSM6DS0: Acceleration (m.s-2): x: %.1f, y: %.1f, z: %.1f\n",
			 sensor_value_to_double(&accel_xyz[0]),
			 sensor_value_to_double(&accel_xyz[1]),
			 sensor_value_to_double(&accel_xyz[2]));

		k_sleep(10000);
	}

}
