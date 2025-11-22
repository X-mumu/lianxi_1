#include ""thrd_pool.h"



typedef struct spinlock spinlock_t;


typedef struct task_s {
    handler_pt func;
    void * arg;
    void * next;
} thrd_task_t;

typedef struct task_queue_s {
    void * head;
    void ** tail;
    spinlock_t lock;
    int block;
    pthread_cond_t cond;
    phtread_mutex_t mutex;
} thrd_task_queue_t;

struct thrdpool_s{
    task_queue_t * task_queue;
    atomic_int quit;
    int thrd_count;
    pthread_t * threads;
}





static int __threads_create(thrd_pool_t * pool, size_t thrd_count)
{
    int ret;
    pthread_attr_t attr;

    ret = pthread_attr_init(&attr);
    if (ret == 0) {
        pool->threads = (pthread_t *)malloc(sizeof(pthread_t) * thrd_count);
        if(pool->threads) {
            int i=0;
            for(i=0; i<thrd_count; i++) {
                if(pthread_create(newthread:&pool->threads[i], attr:&attr, __thread_routine, pool) != 0) {
                    break;
                }
        }
    }
}