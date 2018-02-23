#if defined(KERNEL_MODE)
#include <linux/module.h>
#include <linux/blkdev.h>

#elif defined(USER_MODE)
#include <stdio.h>
#include <stdint.h>

#else
#error Invalid Platform (KERNEL_MODE or USER_MODE)
#endif

#include "debug.h"
#include "params.h"
#include "bdbm_drv.h"
#include "hlm_nobuf.h"
#include "hlm_reqs_pool.h"
#include "utime.h"
#include "umemory.h"
#include "queue/queue.h"

#include "algo/no_ftl.h"
#include "algo/block_ftl.h"
#include "algo/page_ftl.h"

#define CHUNKSIZE 1048576
#define NUM_SECTORS_PER_CHUNK (CHUNKSIZE/512)

typedef struct {
	struct list_head list;
	int64_t cid;
	int8_t qid;
	uint64_t ref_cnt;
	uint64_t last_time;
	uint64_t exp_time;
}autostream_chunk_t;

struct list_head* autostream_queue = NULL;
autostream_chunk_t* autostream_chunks;

uint64_t dev_total_ref_cnt = 0;
uint64_t dev_max_ref_cnt = 0;
uint64_t dev_lifetime = 0;
uint64_t max_chunks = 0;

uint32_t autostream_create_queues(bdbm_drv_info_t* bdi) {
    bdbm_device_params_t* np = BDBM_GET_DEVICE_PARAMS(bdi);
	uint64_t num_chunks = np->nr_subpages_per_ssd / NUM_SECTORS_PER_CHUNK;
	int64_t cid;
	int8_t qid;

	if((autostream_queue = bdbm_malloc(sizeof(struct list_head) * (BDBM_STREAM_NUM+1))) == NULL) {
		bdbm_error("malloc for %d queues failed", BDBM_STREAM_NUM+1);
		return 1;
	}

	for(qid = 0; qid < BDBM_STREAM_NUM+1; qid++) {
		INIT_LIST_HEAD(&autostream_queue[qid]);
	}

	if((autostream_chunks = bdbm_zmalloc(sizeof(autostream_chunk_t) * num_chunks)) == NULL) {
		bdbm_error("malloc for chunks failed");
		return 1;
	}

	for(cid = 0; cid < num_chunks; cid++) {
		autostream_chunks[cid].cid = cid;
		autostream_chunks[cid].qid = 1;
		autostream_chunks[cid].ref_cnt = 0;
		autostream_chunks[cid].last_time = 0;
		autostream_chunks[cid].exp_time = 0;
		list_add_tail(&(autostream_chunks[cid].list), &autostream_queue[1]);
	}

	max_chunks = np->nr_subpages_per_ssd;

	return 0;
}

uint32_t autostream_destroy_queues(void) {

	bdbm_free(autostream_chunks);
	bdbm_free(autostream_queue);
	return 0;
}

autostream_chunk_t* hottest_chunk = NULL;

autostream_chunk_t* find_next_hottest_chunk(uint64_t cur_hottest_cid) {
	int8_t qid;
	struct list_head* pos = NULL;
	autostream_chunk_t* chunk = NULL;
	autostream_chunk_t* second_hottest_chunk = NULL;
	uint64_t max_ref_cnt = 0;

	for(qid = BDBM_STREAM_NUM; qid > 1; qid--) {
		list_for_each(pos, &autostream_queue[qid]) {
			chunk = list_entry(pos, autostream_chunk_t, list);
			if(chunk != NULL) {
				if(chunk->cid != cur_hottest_cid && chunk->ref_cnt >= max_ref_cnt) {
					max_ref_cnt = chunk->ref_cnt;
					second_hottest_chunk = chunk;
				}
			}
		}
		if(second_hottest_chunk != NULL) break;
	}

	return (second_hottest_chunk == NULL)?&(autostream_chunks[cur_hottest_cid]):second_hottest_chunk;
}


void autostream_demotion(void) {
	int8_t qid;
	struct list_head* pos = NULL;
	autostream_chunk_t* chunk = NULL;
	autostream_chunk_t* sec_hottest_chunk = NULL;

	for(qid = BDBM_STREAM_NUM; qid > 1; qid--) {
		list_for_each(pos, &autostream_queue[qid]) {
			chunk = list_entry(pos, autostream_chunk_t, list);
			break;
		}
		if(chunk != NULL){
			if(chunk->exp_time < dev_total_ref_cnt) {
				if(chunk->cid == hottest_chunk->cid) {
					// find new hottest chunk
					sec_hottest_chunk = find_next_hottest_chunk(hottest_chunk->cid);
					if(sec_hottest_chunk == NULL) {
						bdbm_msg("second hottest chunk is NULL"); }
					else {
						hottest_chunk = sec_hottest_chunk;
						dev_max_ref_cnt = hottest_chunk->ref_cnt;
						//bdbm_msg("second hottest_chunk: %lld(%lld)", hottest_chunk->cid, hottest_chunk->ref_cnt);
					}
				}
				chunk->exp_time = dev_total_ref_cnt + dev_lifetime;
				list_del(&chunk->list);
				chunk->qid = qid-1;
				bdbm_bug_on(chunk->qid == 0 || chunk->qid > BDBM_STREAM_NUM);
				list_add_tail(&chunk->list, &autostream_queue[chunk->qid]);
				/*
				bdbm_msg("autostream: chunk %lld(%lld) demoted to %d, chunk exp_time: %lld, total_ref_cnt: %lld", 
						chunk->cid, chunk->ref_cnt, chunk->qid, chunk->exp_time, dev_total_ref_cnt);
				*/
			}
		}
	}
}

int32_t log10_t(int v) {
	return (v >= 1000000000u) ? 9 : (v >= 100000000u) ? 8 : 
		(v >= 10000000u) ? 7 : (v >= 1000000u) ? 6 : 
		(v >= 100000u) ? 5 : (v >= 10000u) ? 4 :
		(v >= 1000u) ? 3 : (v >= 100u) ? 2 : (v >= 10u) ? 1u : 0u; 
}

void autostream_promotion(autostream_chunk_t* chunk) {
	int8_t qid;

	dev_total_ref_cnt++;
	chunk->ref_cnt++;
	if(chunk->ref_cnt > dev_max_ref_cnt) {
		int64_t old_hottest_cid = (hottest_chunk==NULL)?-1:hottest_chunk->cid;
		dev_lifetime = dev_total_ref_cnt - chunk->last_time;
		dev_max_ref_cnt = chunk->ref_cnt;
		hottest_chunk = chunk;
		if(old_hottest_cid != hottest_chunk->cid) {
			/*
			bdbm_msg("autostream: new hottest chunk: %lld(%lld), max_ref_cnt: %lld, lifetime: %lld", 
					chunk->cid, chunk->ref_cnt, dev_max_ref_cnt, dev_lifetime);
			*/
		}
	}
	chunk->last_time = dev_total_ref_cnt;
	chunk->exp_time = dev_total_ref_cnt + dev_lifetime;
	qid = log10_t(chunk->ref_cnt) + 1;
	bdbm_bug_on(qid == 0 || qid < chunk->qid);
	if(chunk->qid != qid && qid <= BDBM_STREAM_NUM) {
		list_del(&chunk->list);
		chunk->qid = qid;
		list_add_tail(&chunk->list, &autostream_queue[qid]);
		//bdbm_msg("autostream: chunk %lld(%lld) promoted to %d", chunk->cid, chunk->ref_cnt, chunk->qid);
	}

	autostream_demotion();
}

autostream_chunk_t* get_chunk(uint64_t sLBA) {
	int64_t cid = sLBA / NUM_SECTORS_PER_CHUNK;
	if(cid >= max_chunks) {
		bdbm_error("chunk index is beyond logical space(%lld)", sLBA);
		return NULL;
	}
	return &(autostream_chunks[cid]);
}

int8_t get_queue_index(uint64_t sLBA) {
	autostream_chunk_t* cur_chunk = get_chunk(sLBA); 
	int8_t ret;

	if(cur_chunk == NULL) {
		bdbm_msg("autostream: cur_chunk NULL");
		return 0;
	}

	ret = cur_chunk->qid;

	autostream_promotion(cur_chunk);

	//bdbm_msg("lpa: %lld, cid: %lld, sID: %d", sLBA, cur_chunk->cid, ret);

	return ret;	
}

// we check only the first address, the sID of LBAs in sequential request are the same
int8_t get_streamid(uint64_t sLBA) {
	int8_t sID;

	sID = get_queue_index(sLBA);

	if(sID == 0) bdbm_msg("autostream: sID is 0 for LBA: %lld", sLBA);
	return sID;
}
