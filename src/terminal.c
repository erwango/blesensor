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

#define W2ST_CONSOLE_MAX_CHAR_LEN 20

//static uint8_t last_term_buffer[W2ST_CONSOLE_MAX_CHAR_LEN];
//static uint8_t last_term_len;


static struct bt_uuid_128 debug_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00);
static struct bt_uuid_128 term_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x0e, 0x00, 0x01, 0x00, 0x00, 0x00);
static struct bt_gatt_ccc_cfg term_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t term[W2ST_CONSOLE_MAX_CHAR_LEN];
static u8_t term_len;


static void term_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	/* TODO: Handle value */
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
	if (!(flags & BT_GATT_WRITE_FLAG_CMD)) {
		return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
	}

	if (offset + len > sizeof(term)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	term_len = len;

	return len;
}

BT_GATT_SERVICE_DEFINE(term_svc,
	BT_GATT_PRIMARY_SERVICE(&debug_service_uuid),
	BT_GATT_CHARACTERISTIC(&term_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE_WITHOUT_RESP |
						 BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ,
						 BT_GATT_PERM_WRITE, NULL,
						 write_without_rsp_term,
						 term),
	BT_GATT_CCC(term_ccc_cfg, term_ccc_cfg_changed),
);


void term_notify(u8_t *data, u8_t length)
{	/* Current Time Service updates only when term is changed */
  u8_t offset;
  u8_t data_to_send;

  /* Split the code in packages */
  for(offset =0U; offset<length; offset +=W2ST_CONSOLE_MAX_CHAR_LEN){
    data_to_send = (length-offset);
    data_to_send = (data_to_send>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : data_to_send;

    /* keep a copy */
    memcpy(term, data+offset, data_to_send);
    term_len = data_to_send;

		bt_gatt_notify(NULL, &term_svc.attrs[1], term, sizeof(term));
	}
}
