/*
The MIT License (MIT)

Copyright (c) 2014-2015 CSAIL, MIT

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <linux/init.h>
#include <linux/module.h>

#include "bdbm_drv.h"
#include "debug.h"
#include "blkio.h"
#include "devices.h"

bdbm_drv_info_t* _bdi = NULL;

// start hoon
typedef unsigned long long ext4_fsblk_t;
void ext4_extent_bdbm_register_communication(
        struct bdbm_comm_operations* comm);

int recieve_double_lpa(ext4_fsblk_t o_lpa, ext4_fsblk_t d_lpa, int len){
    //printk("o_lpa = %llu, d_lpa = %llu\n\n", o_lpa, d_lpa);
    _bdi->ptr_ftl_inf->remap(_bdi, (int64_t)o_lpa, (int64_t)d_lpa, len);
}
struct bdbm_comm_operations {
	int (*send_double_lpa) (ext4_fsblk_t o_lpa, ext4_fsblk_t d_lpa, int len);
};
static struct bdbm_comm_operations bdbm_comm = {
    .send_double_lpa = recieve_double_lpa,
};
// end hoon


static int __init bdbm_drv_init (void)
{
	/* create bdi with default parameters */
	if ((_bdi = bdbm_drv_create ()) == NULL) {
		bdbm_error ("[kmain] bdbm_drv_create () failed");
		return -ENXIO;
	}

	/* open the device */
	if (bdbm_dm_init (_bdi) != 0) {
		bdbm_error ("[kmain] bdbm_dm_init () failed");
		return -ENXIO;
	}

	/* attach the host & the device interface to the bdbm */
	if (bdbm_drv_setup (_bdi, &_blkio_inf, bdbm_dm_get_inf (_bdi)) != 0) {
		bdbm_error ("[kmain] bdbm_drv_setup () failed");
		return -ENXIO;
	}

	/* run it */
	if (bdbm_drv_run (_bdi) != 0) {
		bdbm_error ("[kmain] bdbm_drv_run () failed");
		return -ENXIO;
	}

    //for communication with ext4
    ext4_extent_bdbm_register_communication(&bdbm_comm);

	return 0;
}

static void __exit bdbm_drv_exit(void)
{
	/* stop running layers */
	bdbm_drv_close (_bdi);

	/* close the device */
	bdbm_dm_exit (_bdi);

	/* remove bdbm_drv */
	bdbm_drv_destroy (_bdi);
}

MODULE_AUTHOR ("Sungjin Lee <chamdoo@csail.mit.edu>");
MODULE_DESCRIPTION ("BlueDBM Device Driver");
MODULE_LICENSE ("GPL");

module_init (bdbm_drv_init);
module_exit (bdbm_drv_exit);
