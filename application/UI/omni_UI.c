// app
#include "robot_board.h"
#include "robot_types.h"
#include "omni_UI.h"
// module
#include "rm_referee.h"
#include "referee_protocol.h"
#include "referee_UI.h"
#include "string.h"
#include "crc_ref.h"
#include "stdio.h"
#include "rm_referee.h"
#include "message_center.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "UI_interface.h"

#define Super_Cap_Charge_Voltage_MAX  23.0f // 电容最大电压

static Publisher_t *ui_pub;
static Subscriber_t *ui_sub;
static UI_Cmd_s ui_cmd_recv;
static UI_Upload_Data_s ui_feedback_data;

float cap_debug;
float heat_debug;
extern float diff2;
referee_info_t *referee_data_for_ui;

uint8_t UI_rune; // 自瞄打符标志位


uint8_t UI_Seq;                      // 包序号，供整个referee文件使用
static Graph_Data_t shoot_line[7];   // 射击准线
static Graph_Data_t crosshairs[4];   // 射击准心
static Graph_Data_t benchmark[5];   // 基准线
static Graph_Data_t state_circle[4]; // 圆形
static String_Data_t Char_State[6];  // 字符串
static Graph_Data_t Cap_voltage_arc;  
static Graph_Data_t Deviation_arc;
static Graph_Data_t Allow_heat_arc; 
static Graph_Data_t Cap_voltage;      // 电容电压
static Graph_Data_t total_voltage;      // 电容电压
static Graph_Data_t Shoot_Local_Heat; // 射击本地热量
int linex,liney,line,linex1,liney1,width,size;
//查表计算总允许发射热量
static float getTotal_heat(uint8_t level){
    switch(level){
        case 1:
        return 100;break;
        case 2:
        return 140;break;
        case 3:
        return 180;break;
        case 4:
        return 220;break;
        case 5:
        return 260;break;
        case 6:
        return 300;break;
        case 7:
        return 340;break;
        case 8:
        return 380;break;
        case 9:
        return 420;break;
        case 10:
        return 500;break;
    }
}
//获得剩余热量
static float allow_shoot_heat(float total,uint16_t temp){
    float res;
    res=total-(float)temp;
    // res*=1000;
    return res;
}

static void UI_StaticInit()
{
    // 清空UI
    UIDelete(&referee_data_for_ui->referee_id, UI_Data_Del_ALL, 0);



   // 射击准心
    //UILineDraw(&crosshairs[0], "crosshairs0", UI_Graph_ADD, 9, UI_Color_Main, 6, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 25, SCREEN_LENGTH / 2 + 25, SCREEN_WIDTH / 2 + 25);
    // UILineDraw(&shoot_line[0], "ol0", UI_Graph_ADD, 9, UI_Color_Green, 1, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 + 5, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 44);
    // UILineDraw(&shoot_line[1], "ol1", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 - 75, SCREEN_WIDTH / 2 - 50, SCREEN_LENGTH / 2 - 30, SCREEN_WIDTH / 2 - 50);
    // UILineDraw(&shoot_line[2], "ol2", UI_Graph_ADD, 9, UI_Color_Green, 1, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 56, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 105);
    // UILineDraw(&shoot_line[3], "ol3", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 - 20, SCREEN_WIDTH / 2 - 50, SCREEN_LENGTH / 2 + 25, SCREEN_WIDTH / 2 - 50);

    // UILineDraw(&shoot_line[4], "ol4", UI_Graph_ADD, 9, UI_Color_Green, 1, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 49, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 51);

    // UILineDraw(&shoot_line[5], "ol5", UI_Graph_ADD, 9, UI_Color_White, 2, 540, 150, 700, 300);
    // UILineDraw(&shoot_line[6], "ol6", UI_Graph_ADD, 9, UI_Color_White, 2, 1380, 150, 1220, 300);
    //基准线
    UILineDraw(&benchmark[0], "benchmark0", UI_Graph_ADD, 8, UI_Color_White, 10, 575, 530, 575, 550);
    UILineDraw(&benchmark[1], "benchmark1", UI_Graph_ADD, 9, UI_Color_White, 10,1345, 530, 1345, 550); 
    UIArcDraw(&benchmark[2], "benchmark2", UI_Graph_ADD, 7, UI_Color_White, 313, 316, 10, 956, 542, 383, 386);
    UIArcDraw(&benchmark[3], "benchmark3", UI_Graph_ADD, 6, UI_Color_White, 223, 226, 10, 955, 541, 383, 386);
    UIArcDraw(&benchmark[4], "benchmark4", UI_Graph_ADD, 5, UI_Color_White, 44, 47, 10, 961, 541, 383, 386);
    // 小陀螺
    sprintf(Char_State[0].show_Data, "Rotate");
    UICharDraw(&Char_State[0], "sc0", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 580, 125, "Rotate");
    UICharRefresh(&referee_data_for_ui->referee_id, Char_State[0]);

    UICircleDraw(&state_circle[0], "oc0", UI_Graph_ADD, 9, UI_Color_White, 10, 620, 160, 10);
    // 打符(单发)
    // sprintf(Char_State[1].show_Data, "Rune");
    // UICharDraw(&Char_State[1], "sc1", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 780, 125, "Rune");
    // UICharRefresh(&referee_data_for_ui->referee_id, Char_State[1]);

    // UICircleDraw(&state_circle[1], "oc1", UI_Graph_ADD, 9, UI_Color_White, 10, 800, 160, 10);
    // 摩擦轮
    sprintf(Char_State[2].show_Data, "Friction");
    UICharDraw(&Char_State[2], "sc2", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 880, 125, "Friction");
    UICharRefresh(&referee_data_for_ui->referee_id, Char_State[2]);

    UICircleDraw(&state_circle[2], "oc2", UI_Graph_ADD, 9, UI_Color_White, 10, 960, 160, 10);
    // 电容
    sprintf(Char_State[3].show_Data, "Cap");
    UICharDraw(&Char_State[3], "sc3", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 1270, 125, "Cap");
    UICharRefresh(&referee_data_for_ui->referee_id, Char_State[3]);
    UICircleDraw(&state_circle[3], "oc3", UI_Graph_ADD, 9, UI_Color_White, 10, 1300, 160, 10);

    UIArcDraw(&Cap_voltage_arc, "powerline", UI_Graph_ADD, 9, UI_Color_Green, 271, 273, 7, 956, 542, 383, 386);

    UIFloatDraw(&total_voltage, "of0", UI_Graph_ADD, 9, UI_Color_Green, 15, 3, 3, 740, 910, 24000);
    UIFloatDraw(&Cap_voltage, "of5", UI_Graph_ADD, 9, UI_Color_Pink, 15, 3, 3, 555, 550, (int32_t)(ui_cmd_recv.supercap_voltage * 1000));

    //底盘功率

    UIFloatDraw(&total_voltage, "of0", UI_Graph_ADD, 9, UI_Color_Green, 10, 0, 4, 860, 660, 24000);
    // UIFloatDraw(&Cap_voltage, "of1", UI_Graph_ADD, 9, UI_Color_Pink, 12, 1, 4, 860, 640, allow_shoot_heat(getTotal_heat()));
    // **参数：*graph Graph_Data类型变量指针，用于存放图形数据
    // graphname[3]   图片名称，用于标识更改
    // Graph_Operate   图片操作，见头文件
    // Graph_Layer    图层0-9
    // Graph_Color    图形颜色
    // Graph_Size     字号
    // Graph_Width    图形线宽
    // Start_x、Start_y    开始坐标
    // 枪口热量
    // sprintf(Char_State[5].show_Data, "ALLOW_Shoot");
    // UICharDraw(&Char_State[5], "sc5", UI_Graph_ADD, 7, UI_Color_White, 20, 3, 25+1000, 780-140, "Heat");
    // UICharRefresh(&referee_data_for_ui->referee_id, Char_State[5]);
    UIArcDraw(&Allow_heat_arc, "heatline", UI_Graph_ADD, 9, UI_Color_Green, 273, 300, 7, SCREEN_WIDTH/2, SCREEN_LENGTH/2, 383, 386);
    UIFloatDraw(&Shoot_Local_Heat, "of1", UI_Graph_ADD, 7, UI_Color_Green, 20, 3, 3, 1100, 600, (ui_cmd_recv.Shooter_heat) * 1000);
    //偏差
    UIArcDraw(&Deviation_arc, "deviationline", UI_Graph_ADD, 4, UI_Color_Pink, 48, 87, 7, 962, 542, 383, 386);
    // 发送
    UIGraphRefresh(&referee_data_for_ui->referee_id, 7, shoot_line[0], shoot_line[1], shoot_line[2], shoot_line[3], shoot_line[4], shoot_line[5], shoot_line[6]);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 7, state_circle[0], state_circle[1], state_circle[2], state_circle[3], Cap_voltage, Shoot_Local_Heat,total_voltage);
    UIGraphRefresh(&referee_data_for_ui->referee_id,7,Cap_voltage_arc,Allow_heat_arc,benchmark[0], benchmark[1],benchmark[2],benchmark[3],benchmark[4]);
    UIGraphRefresh(&referee_data_for_ui->referee_id,1,Deviation_arc);  

}


void UI_Init()
{
    ui_pub = PubRegister("ui_feed", sizeof(UI_Upload_Data_s));
    ui_sub = SubRegister("ui_cmd", sizeof(UI_Cmd_s));

    //UI_StaticInit();
}
float capV,totalheat,heattemp;
void UIDynamicRefresh()
{
    SubGetMessage(ui_sub, (void *)&ui_cmd_recv);
    if (ui_cmd_recv.ui_send_flag == 0) {
        UI_StaticInit();
    }
    // 小陀螺
    if (ui_cmd_recv.chassis_mode == CHASSIS_ROTATE) {
        UICircleDraw(&state_circle[0], "oc0", UI_Graph_Change, 9, UI_Color_Main, 10, 620, 160, 10);
    } else {
        UICircleDraw(&state_circle[0], "oc0", UI_Graph_Change, 9, UI_Color_White, 10, 620, 160, 10);
    }
    
    // 摩擦轮
    if (ui_cmd_recv.friction_mode == FRICTION_ON) {
        UICircleDraw(&state_circle[2], "oc2", UI_Graph_Change, 9, UI_Color_Main, 10, 960, 160, 10);
    } else {
        UICircleDraw(&state_circle[2], "oc2", UI_Graph_Change, 9, UI_Color_White, 10, 960, 160, 10);
    }
    // 电容&电容电压
    if (ui_cmd_recv.cap_online_flag == 0)
    {
        UICircleDraw(&state_circle[3], "oc3", UI_Graph_Change, 9, UI_Color_Pink, 10, 1300, 160, 10);
    }
    else
    {
        UICircleDraw(&state_circle[3], "oc3", UI_Graph_Change, 9, UI_Color_Green, 10, 1300, 160, 10);
    }
    

    if(ui_cmd_recv.supercap_voltage >= SUPERCAP_HIGHER_THRESHOLD_VOLTAGE){
        UIArcDraw(&Cap_voltage_arc, "powerline", UI_Graph_Change, 9, UI_Color_Green, 271, 272 + 60 * (ui_cmd_recv.supercap_voltage - SUPERCAP_MIN_VOLTAGE) / (SUPERCAP_MAX_VOLTAGE - SUPERCAP_MIN_VOLTAGE), 7, 956, 542, 383, 386);
    }
    else if (ui_cmd_recv.supercap_voltage >= SUPERCAP_LOWER_THRESHOLD_VOLTAGE){
        UIArcDraw(&Cap_voltage_arc, "powerline", UI_Graph_Change, 9, UI_Color_Orange, 271, 272 + 60 * (ui_cmd_recv.supercap_voltage - SUPERCAP_MIN_VOLTAGE) / (SUPERCAP_MAX_VOLTAGE - SUPERCAP_MIN_VOLTAGE), 7, 956, 542, 383, 386);
    }
    else if (ui_cmd_recv.supercap_voltage > SUPERCAP_LOWER_THRESHOLD_VOLTAGE)
    {
        UIArcDraw(&Cap_voltage_arc, "powerline", UI_Graph_Change, 9, UI_Color_Pink, 271, 272 + 60 * (ui_cmd_recv.supercap_voltage - SUPERCAP_MIN_VOLTAGE) / (SUPERCAP_MAX_VOLTAGE - SUPERCAP_MIN_VOLTAGE), 7, 956, 542, 383, 386);
    }
    else
    {
        UIArcDraw(&Cap_voltage_arc, "powerline", UI_Graph_Change, 9, UI_Color_Pink, 271, 272, 7, 956, 542, 383, 386);
    }
    // 本地热量
    totalheat=getTotal_heat(ui_cmd_recv.robot_level);
    heattemp=allow_shoot_heat(getTotal_heat(ui_cmd_recv.robot_level),ui_cmd_recv.Shooter_heat);
    // UIFloatDraw(&Cap_voltage, "of1", UI_Graph_ADD, 9, UI_Color_Pink, 12, 1, 4, 860, 640, (ui_cmd_recv.SuperCap_voltage)*1000);
    // if (ui_cmd_recv.Heat_Limit - ui_cmd_recv.Shooter_heat > 20) {
    
    if(heattemp>99){
        UIFloatDraw(&Shoot_Local_Heat, "of3", UI_Graph_Change, 7, UI_Color_Green, 20, 3, 3, 1100, 600, 1000.0f*allow_shoot_heat(getTotal_heat(ui_cmd_recv.robot_level),ui_cmd_recv.Shooter_heat));
        // UIArcDraw(&Allow_heat_arc, "heatline", UI_Graph_Change, 9, UI_Color_Green,266-3.8*0.1f*allow_shoot_heat(getTotal_heat(ui_cmd_recv.robot_level),ui_cmd_recv.Shooter_heat)/5, 266, 7, 956, 542, 383, 392);
        UIArcDraw(&Allow_heat_arc, "heatline", UI_Graph_Change, 9, UI_Color_Green,266-3.8*0.1f*heattemp/5, 266, 7, 956, 542, 383, 392);
        // UIFloatDraw(&Shoot_Local_Heat, "of1", UI_Graph_ADD, 9, UI_Color_Green, 20, 3, 3, 400, 780, (ui_cmd_recv.Shooter_heat) * 1000);
    } 
    else {
        UIFloatDraw(&Shoot_Local_Heat, "of3", UI_Graph_Change, 7, UI_Color_Purplish_red, 20, 3, 3, 1100, 600, 1000.0f*allow_shoot_heat(getTotal_heat(ui_cmd_recv.robot_level),ui_cmd_recv.Shooter_heat));
        // UIArcDraw(&Allow_heat_arc, "heatline", UI_Graph_Change, 9, UI_Color_Pink, 266-3.8*0.1f*allow_shoot_heat(getTotal_heat(ui_cmd_recv.robot_level),ui_cmd_recv.Shooter_heat)/5, 266, 7, 956, 542, 383, 392);
        UIArcDraw(&Allow_heat_arc, "heatline", UI_Graph_Change, 9, UI_Color_Pink, 266-3.8*0.1f*heattemp/5, 266, 7, 956, 542, 383, 392);
        // UIFloatDraw(&Shoot_Local_Heat, "of1", UI_Graph_ADD, 9, UI_Color_Purplish_red, 20, 3, 3, 400, 780, (ui_cmd_recv.Shooter_heat) * 1000);
    }
    //偏差                            
    UIArcDraw(&Deviation_arc, "deviationline", UI_Graph_Change, 4, UI_Color_Pink, 48 , 87, 7, 962, 542, 383, 386);
    
    // 动态UI发送
    UIGraphRefresh(&referee_data_for_ui->referee_id, 5, state_circle[0], state_circle[1], state_circle[2], state_circle[3], Cap_voltage);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 2, Shoot_Local_Heat,shoot_line[4]);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 2, Cap_voltage_arc,Allow_heat_arc);

    PubPushMessage(ui_pub, (void *)&ui_feedback_data);
}