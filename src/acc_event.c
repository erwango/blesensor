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
static struct bt_uuid_128 acc_event_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x04, 0x00, 0x00);
static struct bt_gatt_ccc_cfg acc_event_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t acc_event[] = {1, 1, 1, 1, 1};
static u8_t acc_event_update;;




static void acc_event_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	printf("JE SUIS RENTREEEEEE\n");
	acc_event_update = 1U;
}

static ssize_t read_acc_event(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, u16_t len, u16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(acc_event));
}

static ssize_t write_acc_event(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	u8_t *value = attr->user_data;

	if (offset + len > sizeof(acc_event)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	printf("JE SUIS GENIEEEEEEEEEEEEEEE\n");
	return len;
}

BT_GATT_SERVICE_DEFINE(acc_event_svc,
	BT_GATT_PRIMARY_SERVICE(&feature_service_uuid),
  BT_GATT_CHARACTERISTIC(&acc_event_uuid.uuid,
						 BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
						 BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
						 read_acc_event, write_acc_event,
						 acc_event),
  BT_GATT_CCC(acc_event_ccc_cfg, acc_event_ccc_cfg_changed),

);

static void update_acc_event(){

}


void acc_event_notify(void)
{	/* Current Time Service updates only when time is changed */
	printf("%d\n", acc_event_update);
	if (acc_event_update){
		update_acc_event(NULL, &acc_event_svc.attrs[1], acc_event);
	}


}
