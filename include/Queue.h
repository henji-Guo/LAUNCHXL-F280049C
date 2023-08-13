/*
 * Queue.h
 *
 *  Created on: 2023年8月11日
 *      Author: GHJ
 */

#ifndef INCLUDE_QUEUE_H_
#define INCLUDE_QUEUE_H_

#include <stdint.h>

struct foc_frame{
    uint16_t data[4*16];
};

// typedef int QueueElemType;
typedef struct foc_frame QueueElemType;

struct SequenceQueue {
    QueueElemType data[100];
    int head;
    int tail;
    int length;
    int allSize;
};

extern struct SequenceQueue bufQueue;

int SequenceQueue_Init(struct SequenceQueue *p,int len);
int SequenceQueue_Push(struct SequenceQueue *p,QueueElemType data);
QueueElemType SequenceQueue_Pop(struct SequenceQueue *p);
int SequenceQueue_isEmpty(struct SequenceQueue *p);
int SequenceQueue_isFull(struct SequenceQueue *p);

#endif /* INCLUDE_QUEUE_H_ */
