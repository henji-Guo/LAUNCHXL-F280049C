/*
 * vofa.c
 *
 *  Created on: 2023年7月27日
 *      Author: GHJ
 */

#include "vofa.h"
#include "string.h"

/**
 * @brief vofa init
 * 
 * @param frame vofa JustFloat frame struct
 */
static void vofa_frame_init(struct JustFloat* frame)
{
    /* clear */
    memset(frame, 0, sizeof(frame));

    /* add tail[4] = {0x00,0x00,0x80,0x7F} */
    frame->tail[0] = 0x00;
    frame->tail[1] = 0x00;
    frame->tail[2] = 0x80;
    frame->tail[3] = 0x7F;
}

/**
 * @brief vofa set channel data
 * 
 * @param frame vofa JustFloat frame struct
 * @param data The number of users in the channel is according to
 * @param chCnt Which channel(0 ~ VOFA_CH_COUNT)
 */
static int vofa_setData(struct vofa *vofa,float data,int chCnt)
{
    if(chCnt < 0 || chCnt > VOFA_CH_COUNT)
        return -1;

    vofa->frame.data[chCnt] = data;

    return 0;
}

/**
 * @brief Vofa Print Data Message
 * 
 */
__attribute__((__weak__))
void vofa_print(struct vofa* vofa)
{
    //TODO
    // Like that 
    // Serial.write((char*)&vofa->frame,sizeof(vofa->frame));
}


/**
 * @brief Initialize vofa handle
 * 
 * @param vofa pointer to vofa struct
 */
void vofa_init(struct vofa* vofa)
{
    /* init vofa JustFloat frame */
    vofa_frame_init(&vofa->frame);

    /* init vofa set data function */
    vofa->setData = &vofa_setData;

    /* init vofa print function */
    vofa->print = &vofa_print;
    
}
