#ifndef __FIFO_H
#define __FIFO_H
#include "main.h"
#include "stdbool.h"
#include "string.h"

#define FIFO_SIZE 128   // FIFO队列大小

typedef struct {
    uint8_t buffer[FIFO_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t head_LAST;
} FIFOQueue;
extern FIFOQueue rs485_master_fifo;
extern FIFOQueue rs485_slaver_fifo;
extern FIFOQueue NUC_fifo;
bool FIFO_IsFull(FIFOQueue *fifo);
bool FIFO_IsEmpty(FIFOQueue *fifo);
bool FIFO_Write(FIFOQueue *fifo, const uint8_t *data, uint32_t len);
bool WritePacketToFIFO(FIFOQueue *fifo, const uint8_t *packet,uint8_t PACKET_SIZE);
bool FIFO_Read(FIFOQueue *fifo, uint8_t *data, uint32_t len,uint8_t head,uint8_t tail);
bool FIFO_Read_chassis_ctrl(FIFOQueue *fifo, uint8_t *data, uint32_t len,uint8_t head,uint8_t add);
#endif
