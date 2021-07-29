#ifndef LINUX_DSM_BLOCK_H
#define LINUX_DSM_BLOCK_H

#include <linux/hisi/hisi_bbox_diaginfo.h>
#include <linux/kthread.h>
#include <linux/time.h>

#define CP_TIMEOUT_THRESHOLD 1
#define NORMAL_TIMEOUT_THRESHOLD 1
#define ONE_DAY_JIFFIES (24 * 3600 * HZ)

enum DSM_BLOCK_TYPE {
	DSM_BLOCK_CP,
	DSM_BLOCK_NORMAL,
	DSM_BLOCK_MAX,
};

struct dsm_block_type {
	unsigned int threshold;
	unsigned int errno;
};

/* if anyone of them overflow, then notify */
struct dsm_record {
	/* cp IO timeout */
	unsigned int cp_timeout_cnt;
	/* normal priority IO timeout */
	unsigned int normal_timeout_cnt;
};

union block_dsm_union {
	unsigned int buff[DSM_BLOCK_MAX];
	struct dsm_record member;
};

struct block_dsm_body {
	union block_dsm_union rec;
	/* disk space_left */
	unsigned int space_left;
	struct delayed_work dsm_work;
};

extern unsigned int f2fs_get_free_size(void);

#ifdef CONFIG_HUAWEI_DSM
#define dsm_block_info_init()                                                  \
	memset((void *)block_dsm.rec.buff, 0, sizeof(block_dsm.rec.buff));

#define BLOCK_DSM_FORMAT                                                       \
	"CP_TIMEOUT_CNT: %d, NORMAL_TIMEOUT_CNT: %d, SPACE_LEFT: %dMB\n",      \
		block_dsm.rec.member.cp_timeout_cnt,                           \
		block_dsm.rec.member.normal_timeout_cnt, block_dsm.space_left

/* record only two err type, cp io and normal, if more, change it */
#define dsm_block_log(no) bbox_diaginfo_record(no, NULL, BLOCK_DSM_FORMAT);
inline void dsm_block_record(enum DSM_BLOCK_TYPE type);
void hisi_blk_dsm_init(void);
#else
inline void dsm_block_record(enum DSM_BLOCK_TYPE type)
{
	return 0;
}
void hisi_blk_dsm_init(void)
{
	return 0;
}
#define dsm_block_log(no)
#endif /* CONFIG_HUAWEI_DSM */
#endif /* LINUX_DSM_BLOCK_H */
