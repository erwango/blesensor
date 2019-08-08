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

static struct bt_uuid_128 feature_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
static struct bt_uuid_128 env_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x1d, 0x00);
static struct bt_gatt_ccc_cfg env_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t env_update;
static u8_t indicating;
u8_t buffer_to_write[256];
u32_t bytes_to_write;
static struct bt_gatt_indicate_params ind_params;


static void env_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	printf("JE SUIS RENTREEEEEE\n");
	env_update = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

static void indicate_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			u8_t err)
{
	printk("Indication %s\n", err != 0U ? "fail" : "success");
	indicating = 0U;
}


BT_GATT_SERVICE_DEFINE(env_svc,
	BT_GATT_PRIMARY_SERVICE(&feature_service_uuid),
  BT_GATT_CHARACTERISTIC(&env_uuid.uuid,
						 BT_GATT_CHRC_INDICATE,
						 BT_GATT_PERM_NONE,
						 NULL, NULL,
						 NULL),
  BT_GATT_CCC(env_ccc_cfg, env_ccc_cfg_changed),

);


//static void update_env(struct bt_conn *conn, const struct bt_gatt_attr *chrc,u8_t *env_buf)
static void update_env(void)
{
	struct sensor_value temp, hum, press;
	struct device *hts221 = device_get_binding(DT_INST_0_ST_HTS221_LABEL);
	struct device *lis3mdl = device_get_binding(DT_INST_0_ST_LIS3MDL_MAGN_LABEL);
	struct device *lsm6ds0 = device_get_binding(DT_INST_0_ST_LSM6DS0_LABEL);
	struct device *lps25hb = device_get_binding(DT_INST_0_ST_LPS25HB_PRESS_LABEL);

	static u8_t env_buf[12];

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


  k_sleep(MSEC_PER_SEC);

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
  /*sensor_channel_get(lis3mdl, SENSOR_CHAN_MAGN_XYZ, magn_xyz);
  sensor_channel_get(lsm6ds0, SENSOR_CHAN_ACCEL_XYZ, accel_xyz);
  sensor_channel_get(lsm6ds0, SENSOR_CHAN_GYRO_XYZ, gyro_xyz);*/

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
  /*printf(
    "LIS3MDL: Magnetic field (gauss): x: %.1f, y: %.1f, z: %.1f\n",
    sensor_value_to_double(&magn_xyz[0]),
    sensor_value_to_double(&magn_xyz[1]),
    sensor_value_to_double(&magn_xyz[2]));*/

  /* acceleration */
  /*printf(
    "LSM6DS0: Acceleration (m.s-2): x: %.1f, y: %.1f, z: %.1f\n",
    sensor_value_to_double(&accel_xyz[0]),
    sensor_value_to_double(&accel_xyz[1]),
    sensor_value_to_double(&accel_xyz[2]));

  /* gyroscope */
  /*printf(
    "LSM6DS0: Gyroscope: x: %.1f, y: %.1f, z: %.1f\n",
    sensor_value_to_double(&gyro_xyz[0]),
    sensor_value_to_double(&gyro_xyz[1]),
    sensor_value_to_double(&gyro_xyz[2]));*/
	/*int intPart, decPart;
	double press_to_send;
	int32_t decPart, intPart;
	intPart = (int) sensor_value_to_double(&press), intPart, decPart);
	decPart = sensor_value_to_double(&press)-intPart;
	double press_to_send = intPart*100U+decPart;
	printf("LPS25HB: Pressure to send:%d kpa\n",
  			press_to_send);
	convert_to_app_1D_format(sensor_value_to_double(&hum), intPart, decPart);
	u16_t hum_to_send = intPart*10U+decPart;
	 printf("HTS221: Relative Humidity: %d%%\n",
	  			hum_to_send);
	convert_to_app_1D_format(sensor_value_to_double(&temp), intPart, decPart);
	u16_t temp2_to_send = intPart*10U+decPart;
	printf("HTS221: Temperature to send: %d C\n",
  			temp2_to_send);
	convert_to_app_1D_format(sensor_value_to_double(&temp), intPart, decPart);
	u16_t temp1_to_send = intPart*10U+decPart;*/


	/*Convert to little endian */
	u32_t pr = sys_cpu_to_le32(sensor_value_to_double(&press)*100);
	u16_t hu = sys_cpu_to_le16(sensor_value_to_double(&hum)*10);
	u16_t tp2 = sys_cpu_to_le16(sensor_value_to_double(&temp)*10);
	u16_t tp1 = sys_cpu_to_le16(sensor_value_to_double(&temp)*10);
	u8_t buf_pos;


  /* temperature */
  printf("HTS221: Temperature: %d C\n",
  			tp2);

  /* humidity */
  printf("HTS221: Relative Humidity: %d%%\n",
  			hu);

  /* pressure */
  printf("LPS25HB: Pressure:%d kpa\n",
  			pr);

  /*bytes_to_write = sprintf((char *)buffer_to_write, "Sending: ");
  term_notify(buffer_to_write, bytes_to_write);*/
	memcpy(env_buf, &hu, 2);
	//env_buf[0] = hu;
	buf_pos = 2U;
  /*bytes_to_write = sprintf((char *)buffer_to_write, "Cal=%ld ",env_buf);
  term_notify(buffer_to_write, bytes_to_write);*/
	memcpy(env_buf+buf_pos, &pr, 4);
	//env_buf[2] = pr;
	buf_pos += 4U;
  /*bytes_to_write = sprintf((char *)buffer_to_write, "Press=%ld ",env_buf[2]);
  term_notify(buffer_to_write, bytes_to_write);*/
  memcpy(env_buf+buf_pos, &hu, 2);
	//env_buf[6] = hu;
	buf_pos += 2U;
  /*bytes_to_write = sprintf((char *)buffer_to_write, "Hum=%ld ",env_buf[6]);
  term_notify(buffer_to_write, bytes_to_write);*/
  memcpy(env_buf+buf_pos, &tp2, 2);
	//env_buf[8] = tp2;
	buf_pos += 2U;
  /*bytes_to_write = sprintf((char *)buffer_to_write, "Temp2=%ld ",env_buf[8]);
  term_notify(buffer_to_write, bytes_to_write);*/
  memcpy(env_buf+buf_pos, &tp1, 2);
	//env_buf[10] = tp1;
  /*bytes_to_write = sprintf((char *)buffer_to_write, "Temp1=%ld ",env_buf[10]);
  term_notify(buffer_to_write, bytes_to_write);*/

	//bt_gatt_notify(conn, chrc, &env_buf, sizeof(env_buf));
	ind_params.attr = &env_svc.attrs[2];
	ind_params.func = indicate_cb;
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
		update_env();
	}
}






/*void env_notify(void)
{	//Current Time Service updates only when time is changed
	printf("%d\n", env_update);
	if (env_update){
		update_env(NULL, &env_svc.attrs[1], env);
	}
}*/
