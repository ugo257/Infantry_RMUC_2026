#include "hwt606.h"
#include "bsp_usart.h"
#include "daemon.h"
float hwt606_dt;
hwt606_info_t *hwt606_info_t_ptr;
hwt606_info_t HWT606_info;
//hwt606_info_t *HWT606_data;
static USARTInstance *hwt606_usart_instance; // 裁判系统串口实例
static DaemonInstance *hwt606_daemon;        // 裁判系统守护进程
uint8_t data[64];
hwt606_info_t *HWT606_HardwareInit(UART_HandleTypeDef *hwt606_usart_handle){
    hwt606_info_t_ptr=HWT606_Init(hwt606_usart_handle);
    //DWT_GetDeltaT(&hwt606_info_t_ptr->DWT_CNT);
    hwt606_info_t_ptr->init_flag=1;
    return hwt606_info_t_ptr;
}
uint8_t hwt606_check_sum(uint8_t *data){
    uint8_t sum=0;
    for(int i=0;i<10;i++)sum+=data[i];
    return sum==data[10];
}
void HWT606Decode(uint8_t *data){
    
    if(data[0]==0x55&&data[1]==0x51){
        if(hwt606_check_sum(data))for(int k=0;k<3;k++)HWT606_info.accel[k]=(short)((short)data[2*k+3]<<8|data[2*k+2])*16*9.8/32768;
    }   
    if(data[11]==0x55&&data[12]==0x52){
        if(hwt606_check_sum(data+11))for(int k=0;k<3;k++)HWT606_info.gyro[k]=(short)((short)data[2*k+14]<<8|data[2*k+13])*2000/32768;
    }
    if(data[22]==0x55&&data[23]==0x53){
        if(hwt606_check_sum(data+22))for(int k=0;k<3;k++)HWT606_info.angle[k]=(short)((short)data[2*k+25]<<8|data[2*k+3+24])*180/32768;
    }
}
void Hwt606RxCallback(){
    //hwt606_info_t_ptr->dt=DWT_GetDeltaT(&hwt606_info_t_ptr->DWT_CNT);
    DaemonReload(hwt606_daemon);
    HWT606Decode(hwt606_usart_instance->recv_buff);
    memcpy(data,hwt606_usart_instance->recv_buff,16);
}
static void Hwt606LostCallback(void *arg)
{
    USARTServiceInit(hwt606_usart_instance);
}
hwt606_info_t *HWT606_Init(UART_HandleTypeDef *hwt606_usart_handle){
    USART_Init_Config_s conf;
    conf.module_callback   = Hwt606RxCallback;
    conf.usart_handle      = hwt606_usart_handle;
    conf.recv_buff_size    = HWT606_recv_size; // mx 255(u8)
    hwt606_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback     = Hwt606LostCallback,
        .owner_id     = hwt606_usart_instance,
        .reload_count = 2, // 2ms没有收到数据,则认为丢失,重启串口接收
    };
    hwt606_daemon = DaemonRegister(&daemon_conf);

    return &HWT606_info;
}
