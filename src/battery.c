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
LOG_MODULE_REGISTER(bat);

static struct bt_uuid_128 feature_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
static struct bt_uuid_128 battery_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00);

static struct bt_gatt_ccc_cfg  blvl_ccc_cfg[BT_GATT_CCC_MAX] = {};

static u8_t battery_level[] = {1, 1, 1, 1, 1, 1, 1, 1, 1};

static void blvl_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				       u16_t value)
{
	ARG_UNUSED(attr);

	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	LOG_INF("bat Notifications %s", notif_enabled ? "enabled" : "disabled");
}

static ssize_t read_blvl(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr, void *buf,
			       u16_t len, u16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(battery_level));
}

BT_GATT_SERVICE_DEFINE(bat,
	BT_GATT_PRIMARY_SERVICE(&feature_service_uuid),
	BT_GATT_CHARACTERISTIC(&battery_uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_blvl, NULL,
			       battery_level),
	BT_GATT_CCC(blvl_ccc_cfg, blvl_ccc_cfg_changed),
);

static void update_bat(void){

}

void bat_notify(void){

}

/*static int bat_init(struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

u8_t bt_gatt_bat_get_battery_level(void)
{
	return battery_level;
}

int bt_gatt_bat_set_battery_level(u8_t level)
{
	int rc;

	if (level > 100U) {
		return -EINVAL;
	}

	battery_level = level;

	rc = bt_gatt_notify(NULL, &bat.attrs[1], &level, sizeof(level));

	return rc == -ENOTCONN ? 0 : rc;
}*/



/*static void bat_notify(void)
{
	u8_t battery_level = bt_gatt_bat_get_battery_level();

	battery_level--;

	if (!battery_level) {
		battery_level = 100U;
	}

	bt_gatt_bat_set_battery_level(battery_level);
  printk("\nBattery is : %ld", battery_level);
}*/

//SYS_INIT(bat_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
