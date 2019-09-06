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

/*It is this service that notifies the user how to use certain serives
 (quaternions, compass, ...). We implemented it just for that*/

static struct bt_uuid_128 config_control_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00);
static struct bt_uuid_128 feature_command_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x0f, 0x00, 0x02, 0x00, 0x00, 0x00);
static u8_t cmd;
static u8_t cmd_update;


static void cmd_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	/* TODO: Handle value */
}

static ssize_t write_without_rsp_cmd(struct bt_conn *conn,
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

	if (offset + len > sizeof(cmd)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	cmd_update = 1U;

	return len;
}

BT_GATT_SERVICE_DEFINE(cmd_svc,
	BT_GATT_PRIMARY_SERVICE(&config_control_service_uuid),
  BT_GATT_CHARACTERISTIC(&feature_command_uuid.uuid,
		BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
		BT_GATT_PERM_WRITE, NULL,
		write_without_rsp_cmd,
		&cmd),
  BT_GATT_CCC(cmd_ccc_cfg_changed),
);



void cmd_notify(void)
{	/* Current Time Service updates only when time is changed */
	if (!cmd_update) {
		return;
	}

	cmd_update = 0U;
	bt_gatt_notify(NULL, &cmd_svc.attrs[1], &cmd, sizeof(cmd));
}
