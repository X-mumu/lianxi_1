#ifndef _THRD_POOL_H
#define _THRD_POOL_H

typedef struct thrd_pool thrd_pool_t;
typedef struct (* handler_pt)(void *);

#ifndef __cplusplus
extern "C" {
#endif
thrdpool_t * thrdpool_create(int min_threads);

void thrdpool_terminate(thrdpool_t * pool);
int thrdpool_post(thrdpool_t * pool, handler_pt handler, void * arg);
void thrd_pool_waitdone(thrd_pool_t * pool);
#ifndef __cplusplus
}
#endif
#endif /* _THRD_POOL_H */

