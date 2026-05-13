#include "fifo.h"

FIFOQueue rs485_master_fifo= {{0}, 0, 0};
FIFOQueue rs485_slaver_fifo= {{0}, 0, 0};
FIFOQueue NUC_fifo = {{0}, 0, 0}; // 初始化FIFO队列
// 判断 FIFO 队列是否满
bool FIFO_IsFull(FIFOQueue *fifo) {
    return ((fifo->head + 1) % FIFO_SIZE) == fifo->tail;
}

// 判断 FIFO 队列是否为空
bool FIFO_IsEmpty(FIFOQueue *fifo) {
    return fifo->head == fifo->tail;
}

// 写入数据到 FIFO 队列
bool FIFO_Write(FIFOQueue *fifo, const uint8_t *data, uint32_t len) {
    if (len > FIFO_SIZE) {
        return false;  // 数据包太大，无法写入队列
    }
    for (uint32_t i = 0; i < len; i++) {
        fifo->buffer[fifo->head] = *(data+i);  // 写入数据
        fifo->head = (fifo->head + 1) % FIFO_SIZE;  // 更新头指针
    }
    return true;
}

bool WritePacketToFIFO(FIFOQueue *fifo, const uint8_t *packet,uint8_t PACKET_SIZE) {

    return FIFO_Write(fifo, packet, PACKET_SIZE);
}
// 校验和计算函数
uint8_t calculate_checksum(uint8_t *data, uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}
// 从 FIFO 队列读取数据
bool FIFO_Read(FIFOQueue *fifo, uint8_t *data, uint32_t len,uint8_t head,uint8_t tail) {
    uint8_t data_check_SUM[len];
    if (len > FIFO_SIZE || FIFO_IsEmpty(fifo)) {
        return false;  // 数据包太大或 FIFO 队列为空，无法读取
    }
    if(fifo->head_LAST==fifo->head)return false; // 下一次还没写入，不读
    for (uint32_t i = 0; i < FIFO_SIZE; i++) {
        if(fifo->buffer[fifo->tail]==head&&((tail==0x00)||fifo->buffer[(fifo->tail+len-1)%FIFO_SIZE]==tail)){
            
            if(fifo->tail+len<=FIFO_SIZE){
                __disable_irq();
                memcpy(data,&fifo->buffer[fifo->tail],len);
                __enable_irq();
                // break;
            }
            else {
            uint8_t index=FIFO_SIZE-fifo->tail;
            __disable_irq();
            memcpy(data,&fifo->buffer[fifo->tail],index);
            memcpy(data+index,&fifo->buffer[0],len-index);
            __enable_irq();
            // break;
            }
            // if(calculate_checksum(data_check_SUM,len-2)==data_check_SUM[len-2]){
            //     __disable_irq();
            //     memcpy(data,data_check_SUM,len);
            //     __enable_irq();
            //     break;
            // }
            // else fifo->tail = (fifo->tail + 1) % FIFO_SIZE;  // 更新尾指针
        }
        else fifo->tail = (fifo->tail + 1) % FIFO_SIZE;  // 更新尾指针
        
    }
    fifo->head_LAST=fifo->head;
    return true;
}
bool FIFO_READ_POWER(FIFOQueue *fifo, uint8_t *data, uint32_t len){
    if (len > FIFO_SIZE || FIFO_IsEmpty(fifo)) {
        return false;  // 数据包太大或 FIFO 队列为空，无法读取
    }
    if(fifo->head_LAST==fifo->head)return false; // 下一次还没写入，不读
    for (uint32_t i = 0; i < FIFO_SIZE; i++) {
        if(fifo->buffer[fifo->tail]==0x56&&fifo->buffer[(fifo->tail+1)%FIFO_SIZE]==0x6f){
            
            if(fifo->tail+len<=FIFO_SIZE){
                __disable_irq();
                memcpy(data,&fifo->buffer[fifo->tail],len);
                __enable_irq();
                break;
            }
            else {
            uint8_t index=FIFO_SIZE-fifo->tail;
            __disable_irq();
            memcpy(data,&fifo->buffer[fifo->tail],index);
            memcpy(data+index,&fifo->buffer[0],len-index);
            __enable_irq();
            break;
            } 
        }
        else fifo->tail = (fifo->tail + 1) % FIFO_SIZE;  // 更新尾指针
        
    }
    fifo->head_LAST=fifo->head;
    return true;
}
bool FIFO_Read_chassis_ctrl(FIFOQueue *fifo, uint8_t *data, uint32_t len,uint8_t head,uint8_t add) {
    uint8_t data_check_SUM[len];
    if (len > FIFO_SIZE || FIFO_IsEmpty(fifo)) {
        return false;  // 数据包太大或 FIFO 队列为空，无法读取
    }
    if(fifo->head_LAST==fifo->head)return false; // 下一次还没写入，不读
    for (uint32_t i = 0; i < FIFO_SIZE; i++) {
        if(fifo->buffer[fifo->tail]==head&&fifo->buffer[(fifo->tail+1)%FIFO_SIZE]==add){
            
            if(fifo->tail+len<=FIFO_SIZE){
                __disable_irq();
                memcpy(data_check_SUM,&fifo->buffer[fifo->tail],len);
                __enable_irq();
                //break;
            }
            else {
            uint8_t index=FIFO_SIZE-fifo->tail;
            __disable_irq();
            memcpy(data_check_SUM,&fifo->buffer[fifo->tail],index);
            memcpy(data_check_SUM+index,&fifo->buffer[0],len-index);
            __enable_irq();
            //break;
            }
            if(calculate_checksum(data_check_SUM,len-1)==data_check_SUM[len-1]){
                __disable_irq();
                memcpy(data,data_check_SUM,len);
                __enable_irq();
                break;
            }
            else fifo->tail = (fifo->tail + 1) % FIFO_SIZE;  // 更新尾指针
        }
        else fifo->tail = (fifo->tail + 1) % FIFO_SIZE;  // 更新尾指针
        
    }
    fifo->tail = (fifo->tail + len) % FIFO_SIZE;  // 更新尾指针
    fifo->head_LAST=fifo->head;
    return true;
}
