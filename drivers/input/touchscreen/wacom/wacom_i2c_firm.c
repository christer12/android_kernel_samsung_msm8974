/*
 *  wacom_i2c_firm.c - Wacom G5 Digitizer Controller (I2C bus)
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/kernel.h>
#include <linux/wacom_i2c.h>

unsigned char *Binary;
bool ums_binary;

const unsigned int Binary_nLength = 0x0000;
const unsigned char Mpu_type = 0x28;
#ifdef CONFIG_SEC_H_PROJECT
unsigned int Firmware_version_of_file = 0x70;
#else
unsigned int Firmware_version_of_file = 0x104;
#endif

unsigned char *firmware_name = "epen/W9001_B911.bin";

char Firmware_checksum[] = { 0x1F, 0xAB, 0x8E, 0x93, 0x33, };
/*ver 1320*/
const char B887_checksum[] = { 0x1F, 0x6E, 0x2A, 0x13, 0x71, };

void wacom_i2c_set_firm_data(unsigned char *Binary_new)
{
	if (Binary_new == NULL) {
		Binary = NULL;
		return;
	}

	Binary = (unsigned char *)Binary_new;
	ums_binary = true;
}

/*Return digitizer type according to board rev*/
int wacom_i2c_get_digitizer_type(void)
{
	if (system_rev >= 2)
		return EPEN_DTYPE_B911;
	else
		return EPEN_DTYPE_B887;
}

void wacom_i2c_init_firm_data(void)
{
	int type;
	type = wacom_i2c_get_digitizer_type();

	if (type == EPEN_DTYPE_B911) {
		printk(KERN_INFO
			"%s: Digitizer type is B911\n",
			__func__);
	} else if (type == EPEN_DTYPE_B887) {
		printk(KERN_INFO
			"%s: Digitizer type is B887\n",
			__func__);
		firmware_name = "epen/W9001_B887.bin";
		Firmware_version_of_file = 0x1320;
		memcpy(Firmware_checksum, B887_checksum,
			sizeof(Firmware_checksum));
	}
}
