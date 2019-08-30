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

//Environmental Service AND Feature UUIDs
static struct bt_uuid_128 feature_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
static struct bt_uuid_128 env_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x1c, 0x00);

//BT variable declarations
static struct bt_gatt_ccc_cfg env_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t env_update;
static u8_t indicating;
static struct bt_gatt_indicate_params ind_params;


static void env_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	env_update = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

//BT thread indication
static void indicate_env(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			u8_t err)
{
	printk("Environmental indication %s\n", err != 0U ? "fail" : "success");
	indicating = 0U;
}

//Bluetooth characteristics declaration
BT_GATT_SERVICE_DEFINE(env_svc,
	BT_GATT_PRIMARY_SERVICE(&feature_service_uuid),
  BT_GATT_CHARACTERISTIC(&env_uuid.uuid,
						 BT_GATT_CHRC_INDICATE,
						 BT_GATT_PERM_NONE,
						 NULL, NULL, NULL),
  BT_GATT_CCC(env_ccc_cfg, env_ccc_cfg_changed),

);


static void update_env(void)
{
	struct sensor_value temp, hum, press;
	struct device *hts221 = device_get_binding(DT_INST_0_ST_HTS221_LABEL);
	struct device *lps25hb = device_get_binding(DT_INST_0_ST_LPS25HB_PRESS_LABEL);

	static u8_t env_buf[10];

	if (hts221 == NULL) {
		printf("Could not get HTS221 device\n");
		return;
	}

	if (lps25hb == NULL) {
		printf("Could not get LPS25HB device\n");
		return;
	}


	k_sleep(MSEC_PER_SEC);

	if (sensor_sample_fetch(hts221) < 0) {
		printf("HTS221 Sensor sample update error\n");
		return;
	}
	if (sensor_sample_fetch(lps25hb) < 0) {
		printf("LPS25HB Sensor sample update error\n");
		return;
  }

	/* Get sensor data */

	sensor_channel_get(hts221, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	sensor_channel_get(hts221, SENSOR_CHAN_HUMIDITY, &hum);
	sensor_channel_get(lps25hb, SENSOR_CHAN_PRESS, &press);

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


	/*Convert to little endian and app format data*/
	u32_t pr = sys_cpu_to_le32(sensor_value_to_double(&press)*1000);
	u16_t hu = sys_cpu_to_le16(sensor_value_to_double(&hum)*10);
	u16_t tp = sys_cpu_to_le16(sensor_value_to_double(&temp)*10);
	u8_t buf_pos;


	memcpy(env_buf, &hu, 2);
	buf_pos = 2U;
	memcpy(env_buf+buf_pos, &pr, 4);
	buf_pos += 4U;
	memcpy(env_buf+buf_pos, &hu, 2);
	buf_pos += 2U;
	memcpy(env_buf+buf_pos, &tp, 2);

	ind_params.attr = &env_svc.attrs[2];
	ind_params.func = indicate_env;
	ind_params.data = &env_buf;
	ind_params.len = sizeof(env_buf);
	if (bt_gatt_indicate(NULL, &ind_params) == 0) {
		indicating = 1U;
	}

}

void env_indicate(void){
	if (env_update){
		if(indicating){
			return;
		}
		k_sleep(MSEC_PER_SEC);
		update_env();
		k_sleep(2000);
	}
}
