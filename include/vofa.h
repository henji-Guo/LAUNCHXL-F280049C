/*
 * vofa.h
 *
 *  Created on: 2023年7月27日
 *      Author: GHJ
 */

#ifndef INCLUDE_VOFA_H_
#define INCLUDE_VOFA_H_

/* The vofa+ data channel number */
#define VOFA_CH_COUNT  16

/**
 *  The vofa+ JustFloat data frame format 
 *  note that : tail[4] = {0x00,0x00,0x80,0x7F}
*/
struct JustFloat {
  float data[VOFA_CH_COUNT];
  unsigned char tail[4];
};

struct vofa {
    struct JustFloat frame;
    int (*setData)(struct vofa *vofa,float data,int chCnt);
    void (*toBareFrame)(struct vofa *vofa);
    void (*print)(struct vofa *vofa);
};

extern struct vofa vofa;

void vofa_init(struct vofa *vofa);

#endif /* INCLUDE_VOFA_H_ */
