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


static struct bt_uuid_128 debug_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00);
static struct bt_uuid_128 stderr_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x0e, 0x00, 0x02, 0x00, 0x00, 0x00);
static struct bt_gatt_ccc_cfg stderr_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t stderr;
static u8_t stderr_update;


static void stderr_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	/* TODO: Handle value */
}

static ssize_t write_without_rsp_stderr(struct bt_conn *conn,
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

	if (offset + len > sizeof(stderr)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	stderr_update = 1U;

	return len;
}

BT_GATT_SERVICE_DEFINE(stderr_svc,
	BT_GATT_PRIMARY_SERVICE(&debug_service_uuid),
	BT_GATT_CHARACTERISTIC(&stderr_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_WRITE,
						 NULL, write_without_rsp_stderr,
						 &stderr),
  BT_GATT_CCC(stderr_ccc_cfg, stderr_ccc_cfg_changed),
);

static void update_stderr()
{

}



void stderr_notify(void)
{	/* Current Time Service updates only when time is changed */
	if (!stderr_update) {
		return;
	}

	stderr_update = 0U;
	bt_gatt_notify(NULL, &stderr_svc.attrs[1], &stderr, sizeof(stderr));
}
