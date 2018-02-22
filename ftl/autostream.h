#include "bdbm_drv.h"
#include "hlm_nobuf.h"
#include "hlm_reqs_pool.h"

uint32_t autostream_create_queues(bdbm_drv_info_t* bdi);
uint32_t autostream_destroy_queues(void);
int32_t get_streamid(uint64_t sLBA);
