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


static struct bt_uuid_128 feature_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
static struct bt_uuid_128 quaternions_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00);
static struct bt_gatt_ccc_cfg quat_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t quat[] = {1, 1, 1, 1, 5, 5, 9, 9};
static u8_t quat_update;;




static void quat_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	printf("JE SUIS RENTREEEEEE\n");
	quat_update = 1U;
}

static ssize_t read_quat(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, u16_t len, u16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(quat));
}

static ssize_t write_quat(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	u8_t *value = attr->user_data;

	if (offset + len > sizeof(quat)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	printf("JE SUIS GENIEEEEEEEEEEEEEEE\n");
	return len;
}

BT_GATT_SERVICE_DEFINE(quat_svc,
	BT_GATT_PRIMARY_SERVICE(&feature_service_uuid),
  BT_GATT_CHARACTERISTIC(&quaternions_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
						 BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
						 read_quat, write_quat,
						 quat),
  BT_GATT_CCC(quat_ccc_cfg, quat_ccc_cfg_changed),

);

static void update_quat(){

}


void quat_notify(void)
{	/* Current Time Service updates only when time is changed */
	printf("%d\n", quat_update);
	if (quat_update){
		update_quat(NULL, &quat_svc.attrs[1], quat);
	}


}
