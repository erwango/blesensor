#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>


static struct bt_uuid_128 config_control_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00);

static struct bt_uuid_128 register_access_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x0f, 0x00, 0x01, 0x00, 0x00, 0x00);
static struct bt_gatt_ccc_cfg reg_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t reg;
static u8_t reg_update;


static void reg_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	/* TODO: Handle value */
}

static ssize_t read_reg(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, u16_t len, u16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(reg));
}

static ssize_t write_reg(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	u8_t *value = attr->user_data;

	if (offset + len > sizeof(reg)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	reg_update = 1U;

	return len;
}

BT_GATT_SERVICE_DEFINE(reg_svc,
	BT_GATT_PRIMARY_SERVICE(&config_control_service_uuid),
	BT_GATT_CHARACTERISTIC(&register_access_uuid.uuid,
						 BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE |
						 BT_GATT_CHRC_INDICATE,
						 BT_GATT_PERM_READ_ENCRYPT |
						 BT_GATT_PERM_WRITE_ENCRYPT,
						 read_reg, write_reg, &reg),
  BT_GATT_CCC(reg_ccc_cfg, reg_ccc_cfg_changed),
);

static void update_register()
{

}



void reg_notify(void)
{	/* Current Time Service updates only when time is changed */
	if (!reg_update) {
		return;
	}

	reg_update = 0U;
	bt_gatt_notify(NULL, &reg_svc.attrs[1], &reg, sizeof(reg));
}
