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

#include "main.h"

extern float q3;

static struct bt_uuid_128 feature_service_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
	0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
static struct bt_uuid_128 compass_uuid = BT_UUID_INIT_128(
	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
	0xe1, 0x11, 0x01, 0x00, 0x40, 0x00, 0x00, 0x00);

static struct bt_gatt_ccc_cfg comp_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t comp_buf[4];
static u8_t comp_ind;
static u8_t indicating;
static struct bt_gatt_indicate_params ind_params;

static void comp_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	comp_ind = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

static void indicate_comp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			u8_t err)
{
	printk("Compass indication %s\n", err != 0U ? "fail" : "success");
	indicating = 0U;
}

BT_GATT_SERVICE_DEFINE(comp_svc,
	BT_GATT_PRIMARY_SERVICE(&feature_service_uuid),
  BT_GATT_CHARACTERISTIC(&compass_uuid.uuid,
						 BT_GATT_CHRC_INDICATE,
						 BT_GATT_PERM_NONE,
						 NULL, NULL, NULL),
  BT_GATT_CCC(comp_ccc_cfg, comp_ccc_cfg_changed),

);

static void update_comp(void){

	printf("w: %.1f\n", &q3);
	memcpy(comp_buf, &q3, 2);
	memcpy(comp_buf, &q3, 2);

	ind_params.attr = &comp_svc.attrs[2];
	ind_params.func = indicate_comp;
	ind_params.data = &comp_buf;
	ind_params.len = sizeof(comp_buf);
	if (bt_gatt_indicate(NULL, &ind_params) == 0) {
		indicating = 1U;
	}
}

void comp_indicate(void){
	if (comp_ind){
		if(indicating){
			return;
		}
		k_sleep(MSEC_PER_SEC);
		update_comp();
		k_sleep(100);
	}
}
