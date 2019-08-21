#include <errno.h>
#include <init.h>
#include <misc/__assert.h>
#include <stdbool.h>
#include <zephyr/types.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/uuid.h>

#define LOG_LEVEL CONFIG_BT_GATT_bat_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(blvl);

static u8_t blvl_update;
static u8_t indicating;
static struct bt_gatt_indicate_params ind_params;

static struct bt_uuid_128 feature_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
static struct bt_uuid_128 battery_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00);

static struct bt_gatt_ccc_cfg  blvl_ccc_cfg[BT_GATT_CCC_MAX] = {};

static void blvl_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	blvl_update = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

static void indicate_blvl(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			u8_t err)
{
	printk("Compass indication %s\n", err != 0U ? "fail" : "success");
	indicating = 0U;
}


BT_GATT_SERVICE_DEFINE(blvl_svc,
	BT_GATT_PRIMARY_SERVICE(&feature_service_uuid),
	BT_GATT_CHARACTERISTIC(&battery_uuid.uuid,
						 BT_GATT_CHRC_INDICATE,
						 BT_GATT_PERM_NONE,
						 NULL, NULL, NULL),
	BT_GATT_CCC(blvl_ccc_cfg, blvl_ccc_cfg_changed),
);

static void update_blvl(void){

	static u8_t blvl_buf[9];
	u32_t soc = 0x00U;
	u32_t voltage = 0x00U;
	u32_t current = 0x00U;
	u32_t status = 0x03U;

	memcpy(blvl_buf, &soc, 2);
	memcpy(blvl_buf+2U, &soc, 2);
	memcpy(blvl_buf+2U, &voltage, 2);
	memcpy(blvl_buf+2U, &current, 2);
	memcpy(blvl_buf+2U, &status, 2);

	ind_params.attr = &blvl_svc.attrs[2];
	ind_params.func = indicate_blvl;
	ind_params.data = &blvl_buf;
	ind_params.len = sizeof(blvl_buf);
	if (bt_gatt_indicate(NULL, &ind_params) == 0) {
		indicating = 1U;
	}

}

/*static int bas_init(struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

u8_t bt_gatt_bas_get_battery_level(void)
{
	return battery_level;
}*/


void blvl_indicate(void){
	if (blvl_update){
		if(indicating){
			return;
		}
		k_sleep(MSEC_PER_SEC);
		update_blvl();
		k_sleep(2000);
	}
}
