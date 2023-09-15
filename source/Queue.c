/*
 * Queue.c
 *
 *  Created on: 2023年8月11日
 *      Author: GHJ
 */
#include "Queue.h"
#include <stdlib.h>
#include <assert.h>

struct SequenceQueue bufQueue;

static QueueElemType valNULL;

int SequenceQueue_Init(struct SequenceQueue *p,int len)
{
    assert(p != NULL);

    // p->data = (QueueElemType*)malloc(sizeof(QueueElemType)*len);

    p->head = 0;
    p->tail = 0;
    p->length = 0;
    p->allSize = len;

    return 0;
}

int SequenceQueue_Push(struct SequenceQueue *p,QueueElemType data)
{
    assert(p != NULL);

    /* check Queue full */
    if( p->length == p->allSize )
        return -1;

    /* add data */
    p->data[p->tail] = data;
    p->tail = (p->tail + 1) % p->allSize;
    p->length++;

    return 0;
}

QueueElemType SequenceQueue_Pop(struct SequenceQueue *p)
{
    assert(p != NULL);

    QueueElemType data;

    /* check Queue empty */
    if( p->length <= 0 )
        return valNULL;

    /* remove data */
    data = p->data[p->head];
    p->head = (p->head + 1) % p->allSize;
    p->length--;

    return data;
}

int SequenceQueue_isEmpty(struct SequenceQueue *p)
{
    assert(p != NULL);

    /* check Queue empty */
    if( p->length <= 0)
        return 0;

    return -1;
}

int SequenceQueue_isFull(struct SequenceQueue *p)
{
    assert(p != NULL);

    /* check Queue full */
    if( p->length == p->allSize)
        return 0;

    return -1;
}
