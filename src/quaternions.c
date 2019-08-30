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

	#include <device.h>
	#include <drivers/sensor.h>
	#include <stdio.h>
	#include <stdint.h>
	#include <sys/util.h>

	// Update quaternions variables
	#define sampleFreq	512.0f			// sample frequency in Hz
	#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
	#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

	static float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
	static float twoKi = twoKiDef;											// 2 * integral gain (Ki)
	static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f; 					// quaternion of sensor
	float q3 = 0.0f; //Euler angle
	static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

	//Function declaration
	static float invSqrt(float x);
	static void genius(float ax, float ay, float az, float mx, float my,
										 float mz, float gx, float gy, float gz);
	static void error(float ax, float ay, float az, float gx, float gy, float gz);
	static float sqrt(float value);

	//Quaternions Service and Feature UUIDs
	static struct bt_uuid_128 feature_service_uuid = BT_UUID_INIT_128(
		0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a,
		0xe1, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
	static struct bt_uuid_128 quaternions_uuid = BT_UUID_INIT_128(
		0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
		0xe1, 0x11, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00);

	// Quaternions Variable declarations
	static struct bt_gatt_ccc_cfg quat_ccc_cfg[BT_GATT_CCC_MAX] = {};
	static u8_t quat_buf[8];
	static u8_t quat_ind;
	static u8_t indicating;
	static struct bt_gatt_indicate_params ind_params;
	struct sensor_value accel_xyz[3], gyro_xyz[3], magn_xyz[3];
	struct device *lis3mdl;
	struct device *lsm6ds0;


	//Compass Service UUIDs
	static struct bt_uuid_128 compass_uuid = BT_UUID_INIT_128(
		0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac,
		0xe1, 0x11, 0x01, 0x00, 0x40, 0x00, 0x00, 0x00);

	//Compass Variable declarations
	static struct bt_gatt_ccc_cfg comp_ccc_cfg[BT_GATT_CCC_MAX] = {};
	static u8_t comp_buf[4];
	static u8_t comp_ind;
	static u8_t indicating2;
	static struct bt_gatt_indicate_params ind_params2;

	//Quaternions bluetooth fonctions
	static void quat_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
	{
		quat_ind = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
	}

	//Quaternions thread indication
	static void indicate_quat(struct bt_conn *conn, const struct bt_gatt_attr *attr,
														u8_t err)
	{
		printk("Quaternions indication %s\n", err != 0U ? "fail" : "success");
		indicating = 0U;
	}

	//Compass bluetooth fonctions
	static void comp_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
	{
		comp_ind = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
	}

	//Compass thread indication
	static void indicate_comp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
														u8_t err)
	{
		printk("Compass indication %s\n", err != 0U ? "fail" : "success");
		indicating2 = 0U;
	}

	//Bluetooth characteristics declaration
	BT_GATT_SERVICE_DEFINE(quat_svc,
		BT_GATT_PRIMARY_SERVICE(&feature_service_uuid),
	  BT_GATT_CHARACTERISTIC(&quaternions_uuid.uuid,
							 BT_GATT_CHRC_INDICATE,
							 BT_GATT_PERM_NONE,
							 NULL, NULL, NULL),
	  BT_GATT_CCC(quat_ccc_cfg, quat_ccc_cfg_changed),
	  BT_GATT_CHARACTERISTIC(&compass_uuid.uuid,
							 BT_GATT_CHRC_INDICATE,
							 BT_GATT_PERM_NONE,
							 NULL, NULL, NULL),
	  BT_GATT_CCC(comp_ccc_cfg, comp_ccc_cfg_changed),

	);

	//Initialize Quaternions Service
	void init_quat(void){

		lis3mdl = device_get_binding(DT_INST_0_ST_LIS3MDL_MAGN_LABEL);
		lsm6ds0 = device_get_binding(DT_INST_0_ST_LSM6DS0_LABEL);

		if (lis3mdl == NULL) {
			printf("Could not get LIS3MDL device\n");
			return;
		}
		if (lsm6ds0 == NULL) {
			printf("Could not get LSM6DS0 device\n");
			return;
		}

	}

	static float sqrt(float value)
	{
		int i;
		float sqrt = value / 3;

		if (value <= 0) {
			return 0;
		}

		for (i = 0; i < 6; i++) {
			sqrt = (sqrt + value / sqrt) / 2;
		}

		return sqrt;
	}

	static void genius(float ax, float ay, float az, float mx, float my,
										 float mz, float gx, float gy, float gz){

		float recipNorm;
		float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
		float hx, hy, bx, bz;
		float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
		float halfex, halfey, halfez;
		float qa, qb, qc;

	// Use error algorithm (avoids NaN in magnetometer normalisation)
		if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
			error(gx, gy, gz, ax, ay, az);
			return;
		}

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;

			// Normalise magnetometer measurement
			recipNorm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

			// Auxiliary variables to avoid repeated arithmetic
			q0q0 = q0 * q0;
			q0q1 = q0 * q1;
			q0q2 = q0 * q2;
			q0q3 = q0 * q3;
			q1q1 = q1 * q1;
			q1q2 = q1 * q2;
			q1q3 = q1 * q3;
			q2q2 = q2 * q2;
			q2q3 = q2 * q3;
			q3q3 = q3 * q3;

			// Reference direction of Earth's magnetic field
			hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
			hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
			bx = sqrt(hx * hx + hy * hy);
			bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

			// Estimated direction of gravity and magnetic field
			halfvx = q1q3 - q0q2;
			halfvy = q0q1 + q2q3;
			halfvz = q0q0 - 0.5f + q3q3;
			halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
			halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
			halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

			// Error is sum of cross product between estimated direction and measured direction of field vectors
			halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
			halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
			halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

			// Compute and apply integral feedback if enabled
			if(twoKi > 0.0f) {
				integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
				integralFBy += twoKi * halfey * (1.0f / sampleFreq);
				integralFBz += twoKi * halfez * (1.0f / sampleFreq);
				gx += integralFBx;	// apply integral feedback
				gy += integralFBy;
				gz += integralFBz;
			}
			else {
				integralFBx = 0.0f;	// prevent integral windup
				integralFBy = 0.0f;
				integralFBz = 0.0f;
			}

			// Apply proportional feedback
			gx += twoKp * halfex;
			gy += twoKp * halfey;
			gz += twoKp * halfez;
		}

		// Integrate rate of change of quaternion
		gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
		gy *= (0.5f * (1.0f / sampleFreq));
		gz *= (0.5f * (1.0f / sampleFreq));
		qa = q0;
		qb = q1;
		qc = q2;
		q0 += (-qb * gx - qc * gy - q3 * gz);
		q1 += (qa * gx + qc * gz - q3 * gy);
		q2 += (qa * gy - qb * gz + q3 * gx);
		q3 += (qa * gz + qb * gy - qc * gx);

		// Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;

	}

	static void error(float ax, float ay, float az, float gx, float gy, float gz){

		float recipNorm;
		float halfvx, halfvy, halfvz;
		float halfex, halfey, halfez;
		float qa, qb, qc;

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;

			// Estimated direction of gravity and vector perpendicular to magnetic flux
			halfvx = q1 * q3 - q0 * q2;
			halfvy = q0 * q1 + q2 * q3;
			halfvz = q0 * q0 - 0.5f + q3 * q3;

			// Error is sum of cross product between estimated and measured direction of gravity
			halfex = (ay * halfvz - az * halfvy);
			halfey = (az * halfvx - ax * halfvz);
			halfez = (ax * halfvy - ay * halfvx);

			// Compute and apply integral feedback if enabled
			if(twoKi > 0.0f) {
				integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
				integralFBy += twoKi * halfey * (1.0f / sampleFreq);
				integralFBz += twoKi * halfez * (1.0f / sampleFreq);
				gx += integralFBx;	// apply integral feedback
				gy += integralFBy;
				gz += integralFBz;
			}
			else {
				integralFBx = 0.0f;	// prevent integral windup
				integralFBy = 0.0f;
				integralFBz = 0.0f;
			}

			// Apply proportional feedback
			gx += twoKp * halfex;
			gy += twoKp * halfey;
			gz += twoKp * halfez;
		}

		// Integrate rate of change of quaternion
		gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
		gy *= (0.5f * (1.0f / sampleFreq));
		gz *= (0.5f * (1.0f / sampleFreq));
		qa = q0;
		qb = q1;
		qc = q2;
		q0 += (-qb * gx - qc * gy - q3 * gz);
		q1 += (qa * gx + qc * gz - q3 * gy);
		q2 += (qa * gy - qb * gz + q3 * gx);
		q3 += (qa * gz + qb * gy - qc * gx);

		// Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;

	}

	static float invSqrt(float x) {
		float halfx = 0.5f * x;
		float y = x;
		long i = *(long*)&y;
		i = 0x5f3759df - (i>>1);
		y = *(float*)&i;
		y = y * (1.5f - (halfx * y * y));
		return y;
	}



	//Update quaternions including Compass
	static void update_quat(){

		if (sensor_sample_fetch(lis3mdl) < 0) {
	  	printf("LIS3MDL Sensor sample update error\n");
	  	return;
	  }
	  if (sensor_sample_fetch(lsm6ds0) < 0) {
	  	printf("LSM6DS0 Sensor sample update error\n");
	  	return;
	  }

		/* Erase previous */
		printf("\0033\014");

		sensor_channel_get(lis3mdl, SENSOR_CHAN_MAGN_XYZ, magn_xyz);
		sensor_channel_get(lsm6ds0, SENSOR_CHAN_ACCEL_XYZ, accel_xyz);
		sensor_channel_get(lsm6ds0, SENSOR_CHAN_GYRO_XYZ, gyro_xyz);

		float m_x = sensor_value_to_double(&magn_xyz[0]);
		float m_y = sensor_value_to_double(&magn_xyz[1]);
		float m_z = sensor_value_to_double(&magn_xyz[2]);

		float a_x = sensor_value_to_double(&accel_xyz[0]);
		float a_y = sensor_value_to_double(&accel_xyz[1]);
		float a_z = sensor_value_to_double(&accel_xyz[2]);

		float g_x = sensor_value_to_double(&gyro_xyz[0]);
		float g_y = sensor_value_to_double(&gyro_xyz[1]);
		float g_z = sensor_value_to_double(&gyro_xyz[2]);

		/* magneto data */
		printf(
			"LIS3MDL: Magnetic field (gauss): x: %.1f, y: %.1f, z: %.1f\n",
			m_x, m_y, m_z);

		/* acceleration */
		printf(
			"LSM6DS0: Acceleration (m.s-2): x: %.1f, y: %.1f, z: %.1f\n",
			a_x, a_y, a_z);

		/* gyroscope */
		printf(
			"LSM6DS0: Gyroscope (dps): x: %.3f, y: %.3f, z: %.3f\n",
			g_x, g_y, g_z);

		genius(a_x, a_y, a_z, m_x, m_y, m_z, g_x, g_y, g_z);
		printf("Quaternions: x: %.1f, y: %.1f, z: %.1f, w: %.1f\n", q0, q1, q2, q3);


		u8_t buf_pos;

		static u16_t q_x, q_y, q_z, w;
		q_x = sys_cpu_to_le16(q0 * 10000);
		q_y = sys_cpu_to_le16(q1 * 10000);
		q_z = sys_cpu_to_le16(q2 * 10000);

		if (q3<0){
			w = sys_cpu_to_le16((1 - q3) * 180 * 100);//Euler angle
		}
		else{
			w = sys_cpu_to_le16(q3 * 180 * 100);//Euler angle
		}


		memcpy(quat_buf, &q_x , 2); //init buf value
		buf_pos = 2U;
		memcpy(quat_buf+buf_pos, &q_x, 2);
		buf_pos += 2U;
		memcpy(quat_buf+buf_pos, &q_y, 2);
		buf_pos += 2U;
		memcpy(quat_buf+buf_pos, &q_z, 2);

		//Compass part
		memcpy(comp_buf, &w, 2);
		memcpy(comp_buf+2U, &w, 2);

		//Send Quaternions buffer
		ind_params.attr = &quat_svc.attrs[2];
		ind_params.func = indicate_quat;
		ind_params.data = &quat_buf;
		ind_params.len = sizeof(quat_buf);
		if (bt_gatt_indicate(NULL, &ind_params) == 0) {
			indicating = 1U;
		}

		//Send Compass buffer
		ind_params2.attr = &quat_svc.attrs[4];
		ind_params2.func = indicate_comp;
		ind_params2.data = &comp_buf;
		ind_params2.len = sizeof(comp_buf);
		if (bt_gatt_indicate(NULL, &ind_params2) == 0) {
			indicating2 = 1U;
		}

	}

	// Quaternions thread
	void quat_indicate(void){
		if (quat_ind){
			if(indicating){
				return;
			}
			k_sleep(MSEC_PER_SEC);
			update_quat();
			k_sleep(100);
		}
	}


	// Compass thread
	void comp_indicate(void){
		if (comp_ind){
			if(indicating2){
				return;
			}
			k_sleep(MSEC_PER_SEC);
			update_quat();
			k_sleep(100);
		}
	}
