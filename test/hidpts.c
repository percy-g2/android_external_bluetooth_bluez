/*
 *  Simple HID test application
 *
 *  Copyright (c) 2010 Alan Ott <alan@signal11.us>
 *  Copyright (c) 2010 Signal 11 Software
 *  Copyright (C) 2011  ST-Ericsson SA. All rights reserved.
 *
 *  This code is ST-Ericsson proprietary and confidential.
 *  Any use of the code for whatever purpose is subject to
 *  specific written permission of ST-Ericsson SA.
 *
 */

/*
 *  Based on 'Hidraw Userspace Example' from Linux samples/hidraw/hid-example.c
 */

/* Linux */
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>

/* Unix */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* C */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

/*
 * Ugly hack to work around failing compilation on systems that don't
 * yet populate new version of hidraw.h to userspace.
 *
 * If you need this, please have your distro update the kernel headers.
 */
#ifndef HIDIOCSFEATURE
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

static void usage(void)
{
	printf("Simple HID test application\n");
	printf("-d dev      -- HIDRAW device to use\n");
	printf("-c cmd_num  -- number of command to send\n");
	printf("\tavailable commands:\n");
	printf("\t1 - Get_Report\n");
	printf("\t2 - Set_Report\n");
}

int main(int argc, char **argv)
{
	int fd;
	int i, c, res, desc_size = 0;
	char buf[256];
	int opt;
	char *device = NULL;


	while ((opt=getopt(argc,argv,"c:d:")) != EOF) {
		switch(opt) {
		case 'c':
			c = atoi(optarg);
			break;
		case 'd':
			device = (optarg);
			break;
		default:
			usage();
			return 1;
		}
	}

	if (!device) {
		usage();
		return 1;
	}

	fd = open(device, O_RDWR|O_NONBLOCK);
	if (fd < 0) {
		perror("Unable to open device");
		return 1;
	}

	switch(c) {
	case 1:	/* Get Feature */
		buf[0] = 0x9; /* Report Number */
		res = ioctl(fd, HIDIOCGFEATURE(256), buf);
		if (res < 0) {
			perror("HIDIOCGFEATURE");
		} else {
			printf("ioctl HIDIOCGFEATURE returned: %d\n", res);
			printf("Report data (not containing the report number):\n\t");
			for (i = 0; i < res; i++)
				printf("%hhx ", buf[i]);
			puts("\n");
		}
		break;
	case 2:	/* Set Feature */
		buf[0] = 0x9; /* Report Number */
		buf[1] = 0xff;
		buf[2] = 0xff;
		buf[3] = 0xff;
		res = ioctl(fd, HIDIOCSFEATURE(4), buf);
		if (res < 0)
			perror("HIDIOCSFEATURE");
		else
			printf("ioctl HIDIOCGFEATURE returned: %d\n", res);

		break;
	default:
		printf("Unknown command\n");
		break;
	}

	close(fd);
	return 0;
}
