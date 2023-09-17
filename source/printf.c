#include "main.h"

// To use printf
int fputc(int _c, register FILE *_fp)
{
    SCI_writeCharBlockingFIFO(SCIA_BASE, _c);
    return _c;
}
int putc(int _c, register FILE *_fp)
{
    SCI_writeCharBlockingFIFO(SCIA_BASE, _c);
    return _c;
}
int putchar(int data)
{
    SCI_writeCharBlockingFIFO(SCIA_BASE, data);
    return data;
}
int fputs(const char *_ptr, register FILE *_fp)
{
    unsigned int i, len;
    len = strlen(_ptr);
    for (i = 0; i < len; i++)
    {
        SCI_writeCharBlockingFIFO(SCIA_BASE, _ptr[i]);
    }
    return len;
}

/**
 * @brief Vofa Print Data Message
 *
 */
void vofa_print(struct vofa *vofa)
{
#ifndef LOG_USE_VOFA_QUEUE
    uint16_t *tx_buf = (uint16_t*)&vofa->frame;
    int i;
    for (i = 0; i < VOFA_CH_COUNT * 2; i++){
        while(SciaRegs.SCIFFTX.bit.TXFFST >= 14);
        SciaRegs.SCITXBUF.bit.TXDT = tx_buf[i];
        SciaRegs.SCITXBUF.bit.TXDT = tx_buf[i] >> 8;
    }
    while(SciaRegs.SCIFFTX.bit.TXFFST >= 12);
    SciaRegs.SCITXBUF.bit.TXDT = (uint16_t)vofa->frame.tail[0];
    SciaRegs.SCITXBUF.bit.TXDT = (uint16_t)vofa->frame.tail[1];
    SciaRegs.SCITXBUF.bit.TXDT = (uint16_t)vofa->frame.tail[2];
    SciaRegs.SCITXBUF.bit.TXDT = (uint16_t)vofa->frame.tail[3];
#else
    struct foc_frame logFrame;
    uint32_t *tx_buf = (uint32_t *)&vofa->frame;
    int i;

    for (i = 0; i < VOFA_CH_COUNT; i++)
    {
        logFrame.data[4 * i] = tx_buf[i] & 0xFF;
        logFrame.data[4 * i + 1] = (tx_buf[i] & 0xFF00) >> 8;
        logFrame.data[4 * i + 2] = (tx_buf[i] & 0xFF0000) >> 16;
        logFrame.data[4 * i + 3] = (tx_buf[i] & 0xFF000000) >> 24;
    }
    logFrame.data[4 * i] = (uint16_t)vofa->frame.tail[0];
    logFrame.data[4 * i + 1] = (uint16_t)vofa->frame.tail[1];
    logFrame.data[4 * i + 2] = (uint16_t)vofa->frame.tail[2];
    logFrame.data[4 * i + 3] = (uint16_t)vofa->frame.tail[3];
    if (SequenceQueue_isFull(&bufQueue) != 0)
        SequenceQueue_Push(&bufQueue, logFrame);
#endif
}
