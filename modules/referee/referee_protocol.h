/**
 * @file referee_protocol.h
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */

#ifndef referee_protocol_H
#define referee_protocol_H

#include "stdint.h"

/****************************宏定义部分****************************/

#define REFEREE_SOF          0xA5 // 起始字节,协议固定为0xA5
#define Robot_Red            0
#define Robot_Blue           1
#define Communicate_Data_LEN 5 // 自定义交互数据长度，该长度决定了我方发送和他方接收，自定义交互数据协议更改时只需要更改此宏定义即可

#pragma pack(1)

/****************************通信协议格式****************************/

/* 通信协议格式偏移，枚举类型,代替#define声明 */
typedef enum {
    FRAME_HEADER_Offset = 0,
    CMD_ID_Offset       = 5,
    DATA_Offset         = 7,
} JudgeFrameOffset_e;

/* 通信协议长度 */
typedef enum {
    LEN_HEADER = 5, // 帧头长
    LEN_CMDID  = 2, // 命令码长度
    LEN_TAIL   = 2, // 帧尾CRC16

    LEN_CRC8 = 4, // 帧头CRC8校验长度=帧头+数据长+包序号
} JudgeFrameLength_e;

/****************************帧头****************************/
/****************************帧头****************************/

/* 帧头偏移 */
typedef enum {
    SOF         = 0, // 起始位
    DATA_LENGTH = 1, // 帧内数据长度,根据这个来获取数据长度
    SEQ         = 3, // 包序号
    CRC8        = 4  // CRC8
} FrameHeaderOffset_e;

/* 帧头定义 */
typedef struct
{
    uint8_t SOF;
    uint16_t DataLength;
    uint8_t Seq;
    uint8_t CRC8;
} xFrameHeader;

/****************************cmd_id命令码说明****************************/
/****************************cmd_id命令码说明****************************/

/* 命令码ID,用来判断接收的是什么数据 */
typedef enum {
    ID_game_state                = 0x0001, // 比赛状态数据
    ID_game_result               = 0x0002, // 比赛结果数据
    ID_game_robot_survivors      = 0x0003, // 比赛机器人血量数据
    ID_event_data                = 0x0101, // 场地事件数据
    ID_supply_projectile_action  = 0x0102, // 场地补给站动作标识数据
    ID_supply_projectile_booking = 0x0103, // 场地补给站预约子弹数据
    ID_referee_warning           = 0x0104, // 裁判警告信息
    ID_dart_info                 = 0x0105, // 飞镖发射相关数据
    ID_game_robot_state          = 0x0201, // 机器人状态数据
    ID_power_heat_data           = 0x0202, // 实时功率热量数据
    ID_game_robot_pos            = 0x0203, // 机器人位置数据
    ID_buff_musk                 = 0x0204, // 机器人增益数据
    ID_aerial_robot_energy       = 0x0205, // 空中机器人能量状态数据
    ID_robot_hurt                = 0x0206, // 伤害状态数据
    ID_shoot_data                = 0x0207, // 实时射击数据
    ID_projectile_allowance      = 0x0208, // 允许发弹量数据
    ID_RFID_state                = 0x0209, // RFID状态数据
    ID_dart_client_cmd           = 0x020A, // 飞镖选手端指令数据
    ID_ground_robot_position     = 0x020B, // 地面机器人位置数据
    ID_radar_mark_data           = 0x020C, // 雷达标记进度数据
    ID_sentry_info               = 0x020D, // 哨兵自主决策信息
    ID_radar_info                = 0x020E, // 雷达自主决策信息
    ID_student_interactive       = 0x0301, // 机器人间交互数据
} CmdID_e;

/* 命令码数据段长,根据官方协议来定义长度，还有自定义数据长度 */
typedef enum {
    LEN_game_state               = 11,                        // 0x0001
    LEN_game_result              = 1,                        // 0x0002
    LEN_game_robot_HP            = 16,                       // 0x0003
    LEN_event_data               = 4,                        // 0x0101
    //LEN_supply_projectile_action = 4,                        // 0x0102
    LEN_REF_warning              = 3,                        // 0x0104
    LEN_dart_info                = 3,                        // 0x0105

    LEN_game_robot_state         = 13,                       // 0x0201
    LEN_power_heat_data          = 14,                       // 0x0202
    LEN_game_robot_pos           = 12,                       // 0x0203, PDF详细表为3个float
    LEN_buff_musk                = 8,                        // 0x0204
    LEN_aerial_robot_energy      = 2,                        // 0x0205, 2026常规表已不发送
    LEN_robot_hurt               = 1,                        // 0x0206
    LEN_shoot_data               = 7,                        // 0x0207
    LEN_projectile_allowance     = 8,                        // 0x0208
    LEN_RFID_state               = 5,                        // 0x0209
    LEN_dart_client_cmd          = 6,                        // 0x020A
    LEN_ground_robot_position    = 40,                       // 0x020B
    LEN_radar_mark_data          = 2,                        // 0x020C
    LEN_sentry_info              = 6,                        // 0x020D
    LEN_radar_info               = 1,                        // 0x020E
    LEN_receive_data             = 6 + Communicate_Data_LEN, // 0x0301

} JudgeDataLength_e;

/****************************接收数据的详细说明****************************/
/****************************接收数据的详细说明****************************/

/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef struct
{
    uint8_t game_type : 4;      // 比赛类型
    uint8_t game_progress : 4;  // 当前比赛阶段
    uint16_t stage_remain_time; // 当前阶段剩余时间
    uint64_t SyncTimeStamp;     // UNIX时间，当机器人正确连接到裁判系统的NTP服务器后生效
} ext_game_state_t;

/* ID: 0x0002  Byte:  1    比赛结果数据 */
/*
    0：平局
    1：红方胜利
    2：蓝方胜利
*/
typedef struct
{
    uint8_t winner;
} ext_game_result_t;

/* ID: 0x0003  Byte:  16    己方机器人血量数据 */
typedef struct
{
    uint16_t ally_1_robot_hp;   // 己方1号英雄机器人血量
    uint16_t ally_2_robot_hp;   // 己方2号工程机器人血量
    uint16_t ally_3_robot_hp;   // 己方3号步兵机器人血量
    uint16_t ally_4_robot_hp;   // 己方4号步兵机器人血量
    uint16_t reserved;          // 保留位
    uint16_t ally_7_robot_hp;   // 己方7号哨兵机器人血量
    uint16_t ally_outpost_hp;   // 己方前哨站血量
    uint16_t ally_base_hp;      // 己方基地血量
} ext_game_robot_HP_t;

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef struct
{
    uint32_t event_type;
} ext_event_data_t;

/* ID: 0x0102  Byte:  3    场地补给站动作标识数据 */
typedef struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;
/* ID: 0x0104  Byte:  3    裁判警告数据 */
typedef  struct 
{ 
  uint8_t level; 
  uint8_t offending_robot_id; 
  uint8_t count; 
}ext_referee_warning_t; 

/* ID: 0x0105  Byte:  3    飞镖发射相关数据 */
typedef struct
{
    uint8_t dart_remaining_time; // 己方飞镖发射剩余时间，单位：s
    uint16_t dart_info;          // 飞镖命中目标、命中次数、选定目标等bit域
} dart_info_t;

/* ID: 0X0201  Byte: 13    机器人状态数据 */
typedef struct
{
    uint8_t robot_id;    // 本机器人ID
    uint8_t robot_level; // 机器人等级
    union {
        uint16_t current_hp; // 当前血量
        uint16_t remain_HP;  // 旧字段名兼容
    };
    union {
        uint16_t max_hp; // 血量上限
        uint16_t max_HP; // 旧字段名兼容
    };
    union {
        uint16_t shooter_heat_cooling_per_s;     // 射击热量每秒冷却值
        uint16_t shooter_id1_42mm_cooling_rate;  // 旧字段名兼容
    };
    union {
        uint16_t shooter_heat_limit;              // 射击热量上限
        uint16_t shooter_id1_42mm_cooling_limit;  // 旧字段名兼容
    };
    uint16_t chassis_power_limit; // 底盘功率上限
    union {
        struct {
            uint8_t power_gimbal_output : 1;  // gimbal口24V输出状态
            uint8_t power_chassis_output : 1; // chassis口24V输出状态
            uint8_t power_shooter_output : 1; // shooter口24V输出状态
            uint8_t power_reserved : 5;
        };
        struct {
            uint8_t mains_power_gimbal_output : 1;  // 旧字段名兼容
            uint8_t mains_power_chassis_output : 1; // 旧字段名兼容
            uint8_t mains_power_shooter_output : 1; // 旧字段名兼容
            uint8_t mains_power_reserved : 5;
        };
        uint8_t power_output_bits; // 电源输出状态原始bit
    };
} ext_game_robot_state_t;

/* ID: 0X0202  Byte: 14    实时底盘缓冲能量和射击热量数据 */
typedef struct
{
    union {
        uint16_t reserved0;    // 保留位
        uint16_t chassis_volt; // 旧字段名兼容，2026协议中该位置为保留位
    };
    union {
        uint16_t reserved1;       // 保留位
        uint16_t chassis_current; // 旧字段名兼容，2026协议中该位置为保留位
    };
    union {
        float reserved_power; // 保留位
        float chassis_power;  // 旧字段名兼容，2026协议中该位置为保留位
    };
    union {
        uint16_t buffer_energy_j;      // 缓冲能量，单位：J
        uint16_t chassis_power_buffer; // 旧字段名兼容
    };
    union {
        uint16_t shooter_17mm_heat;  // 17mm发射机构射击热量
        uint16_t shooter_17mm_heat0; // 旧字段名兼容
    };
    union {
        uint16_t shooter_42mm_heat;  // 42mm发射机构射击热量
        uint16_t shooter_17mm_heat1; // 旧字段名兼容，2026协议该位置为42mm热量
    };
} ext_power_heat_data_t;

/* ID: 0x0203  Byte: 12    机器人位置数据 */
typedef struct
{
    float x;     // 本机器人位置x坐标，单位：m
    float y;     // 本机器人位置y坐标，单位：m
    float angle; // 本机器人测速模块的朝向，单位：度。正北为0度
} ext_game_robot_pos_t;

/* ID: 0x0204  Byte:  8    机器人增益和底盘能量数据 */
typedef struct
{
    union {
        uint8_t hp_recovery_buff_percent; // 回血增益百分比
        uint8_t recovery_buff;            // 旧字段名兼容
    };
    union {
        uint16_t heat_cooling_buff_per_s; // 射击热量冷却增益，单位：热量/s
        uint16_t cooling_buff;            // 旧字段名兼容
    };
    union {
        uint8_t defense_buff_percent; // 防御增益百分比
        uint8_t defence_buff;         // 旧字段名兼容
    };
    union {
        uint8_t negative_defense_buff_percent; // 负防御增益百分比
        uint8_t vulnerability_buff;            // 旧字段名兼容
    };
    union {
        uint16_t attack_buff_percent; // 攻击增益百分比
        uint16_t attack_buff;         // 旧字段名兼容
    };
    union {
        uint8_t remaining_energy_bits; // 剩余能量比例bit反馈
        uint8_t remaining_energy;      // 旧字段名兼容
    };
} ext_buff_musk_t;

/* ID: 0x0205  Byte:  2    空中机器人能量状态数据，2026常规表已不发送 */
typedef struct
{
    uint8_t airforce_status; // 空中机器人状态（0为正在冷却，1为冷却完毕，2为正在空中支援）
    uint8_t time_remain;     // 此状态的剩余时间
} aerial_robot_energy_t;

/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4; // 血量变化类型
} ext_robot_hurt_t;

/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef struct
{
    uint8_t bullet_type;
    union {
        uint8_t shooter_id;     // 发射机构ID
        uint8_t shooter_number; // 协议示例字段名
    };
    union {
        uint8_t bullet_freq;          // 弹丸射频，单位：Hz
        uint8_t launching_frequency;  // 协议示例字段名
    };
    union {
        float initial_speed_mps; // 弹丸初速度，单位：m/s
        float bullet_speed;      // 旧字段名兼容
    };
} ext_shoot_data_t;

/* ID: 0x0208 Byte : 8   允许发弹量 */
typedef struct
{
    uint16_t projectile_allowance_17mm;         // 机器人自身拥有的17mm允许发弹量
    uint16_t projectile_allowance_42mm;         // 42mm允许发弹量
    uint16_t remaining_gold_coin;               // 剩余金币数量
    uint16_t projectile_allowance_fortress_17mm; // 堡垒储备17mm允许发弹量
} projectile_allowance_t;

/* ID: 0x0209 Byte : 5   RFID状态 */
typedef struct
{
    uint32_t rfid_status;   // RFID状态bit0-31
    uint8_t rfid_status_2;  // RFID状态扩展bit0-5
} RFID_state_t;

/* ID: 0x020A Byte : 6   飞镖选手端指令数据 */
typedef struct
{
    uint8_t dart_launch_opening_status; // 飞镖发射站状态
    uint8_t reserved;                   // 保留位
    uint16_t target_change_time;        // 切换击打目标时的比赛剩余时间，单位：s
    uint16_t latest_launch_cmd_time;    // 最近一次确认发射指令时的比赛剩余时间，单位：s
} dart_client_cmd_t;

/* ID: 0x020B Byte : 40   地面机器人位置数据 */
typedef struct
{
    float hero_x;       // 己方英雄机器人x坐标，单位：m
    float hero_y;       // 己方英雄机器人y坐标，单位：m
    float engineer_x;   // 己方工程机器人x坐标，单位：m
    float engineer_y;   // 己方工程机器人y坐标，单位：m
    float standard_3_x; // 己方3号步兵机器人x坐标，单位：m
    float standard_3_y; // 己方3号步兵机器人y坐标，单位：m
    float standard_4_x; // 己方4号步兵机器人x坐标，单位：m
    float standard_4_y; // 己方4号步兵机器人y坐标，单位：m
    float reserved0;    // 保留位
    float reserved1;    // 保留位
} ground_robot_position_t;

/* ID: 0x020C Byte : 2   雷达标记进度数据 */
typedef struct
{
    uint16_t mark_progress; // 雷达标记/特殊标识状态bit域
} radar_mark_data_t;

/* ID: 0x020D Byte : 6   哨兵自主决策信息 */
typedef struct
{
    uint32_t sentry_info;   // 哨兵兑换/复活/金币状态bit域
    uint16_t sentry_info_2; // 哨兵脱战/姿态/可兑换发弹量bit域
} sentry_info_t;

/* ID: 0x020E Byte : 1   雷达自主决策信息 */
typedef struct
{
    uint8_t radar_info; // 雷达双倍易伤机会、加密等级等bit域
} radar_info_t;

_Static_assert(sizeof(xFrameHeader) == LEN_HEADER, "xFrameHeader size mismatch");
_Static_assert(sizeof(ext_game_state_t) == LEN_game_state, "ext_game_state_t size mismatch");
_Static_assert(sizeof(ext_game_robot_HP_t) == LEN_game_robot_HP, "ext_game_robot_HP_t size mismatch");
_Static_assert(sizeof(dart_info_t) == LEN_dart_info, "dart_info_t size mismatch");
_Static_assert(sizeof(ext_game_robot_state_t) == LEN_game_robot_state, "ext_game_robot_state_t size mismatch");
_Static_assert(sizeof(ext_power_heat_data_t) == LEN_power_heat_data, "ext_power_heat_data_t size mismatch");
_Static_assert(sizeof(ext_game_robot_pos_t) == LEN_game_robot_pos, "ext_game_robot_pos_t size mismatch");
_Static_assert(sizeof(ext_buff_musk_t) == LEN_buff_musk, "ext_buff_musk_t size mismatch");
_Static_assert(sizeof(aerial_robot_energy_t) == LEN_aerial_robot_energy, "aerial_robot_energy_t size mismatch");
_Static_assert(sizeof(ext_robot_hurt_t) == LEN_robot_hurt, "ext_robot_hurt_t size mismatch");
_Static_assert(sizeof(ext_shoot_data_t) == LEN_shoot_data, "ext_shoot_data_t size mismatch");
_Static_assert(sizeof(projectile_allowance_t) == LEN_projectile_allowance, "projectile_allowance_t size mismatch");
_Static_assert(sizeof(RFID_state_t) == LEN_RFID_state, "RFID_state_t size mismatch");
_Static_assert(sizeof(dart_client_cmd_t) == LEN_dart_client_cmd, "dart_client_cmd_t size mismatch");
_Static_assert(sizeof(ground_robot_position_t) == LEN_ground_robot_position, "ground_robot_position_t size mismatch");
_Static_assert(sizeof(radar_mark_data_t) == LEN_radar_mark_data, "radar_mark_data_t size mismatch");
_Static_assert(sizeof(sentry_info_t) == LEN_sentry_info, "sentry_info_t size mismatch");
_Static_assert(sizeof(radar_info_t) == LEN_radar_info, "radar_info_t size mismatch");
/****************************机器人交互数据****************************/
/****************************机器人交互数据****************************/
/* 发送的内容数据段最大为 113 检测是否超出大小限制?实际上图形段不会超，数据段最多30个，也不会超*/
/* 交互数据头结构 */
typedef struct
{
    uint16_t data_cmd_id; // 由于存在多个内容 ID，但整个cmd_id 上行频率最大为 10Hz，请合理安排带宽。注意交互部分的上行频率
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/* 机器人id */
typedef enum {
    // 红方机器人ID
    RobotID_RHero      = 1,
    RobotID_REngineer  = 2,
    RobotID_RStandard1 = 3,
    RobotID_RStandard2 = 4,
    RobotID_RStandard3 = 5,
    RobotID_RAerial    = 6,
    RobotID_RSentry    = 7,
    RobotID_RRadar     = 9,
    // 蓝方机器人ID
    RobotID_BHero      = 101,
    RobotID_BEngineer  = 102,
    RobotID_BStandard1 = 103,
    RobotID_BStandard2 = 104,
    RobotID_BStandard3 = 105,
    RobotID_BAerial    = 106,
    RobotID_BSentry    = 107,
    RobotID_BRadar     = 109,
} Robot_ID_e;

/* 交互数据ID */
typedef enum {
    UI_Data_ID_Del      = 0x100,
    UI_Data_ID_Draw1    = 0x101,
    UI_Data_ID_Draw2    = 0x102,
    UI_Data_ID_Draw5    = 0x103,
    UI_Data_ID_Draw7    = 0x104,
    UI_Data_ID_DrawChar = 0x110,

    /* 自定义交互数据部分 */
    Communicate_Data_ID = 0x0200,

} Interactive_Data_ID_e;
/* 交互数据长度 */
typedef enum {
    Interactive_Data_LEN_Head = 6,
    UI_Operate_LEN_Del        = 2,
    UI_Operate_LEN_PerDraw    = 15,
    UI_Operate_LEN_DrawChar   = 15 + 30,

    /* 自定义交互数据部分 */
    // Communicate_Data_LEN = 5,

} Interactive_Data_Length_e;

/****************************自定义交互数据****************************/
/*
    学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
    自定义交互数据 机器人间通信：0x0301。
    发送频率：上限 10Hz
*/
// 自定义交互数据协议，可更改，更改后需要修改最上方宏定义数据长度的值
typedef struct
{
    uint8_t data[Communicate_Data_LEN]; // 数据段,n需要小于113
} robot_interactive_data_t;

// 机器人交互信息_发送
typedef struct
{
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    robot_interactive_data_t Data; // 数据段
    uint16_t frametail;
} Communicate_SendData_t;
// 机器人交互信息_接收
typedef struct
{
    ext_student_interactive_header_data_t datahead;
    robot_interactive_data_t Data; // 数据段
} Communicate_ReceiveData_t;

/****************************UI交互数据****************************/

/*
 * 选手端绘图子内容 ID 0x0101 的 15 字节图形数据段。
 * 0x0102/0x0103/0x0104 分别是在数据段内连续放 2/5/7 个同样的图形结构。
 */
typedef struct
{
    uint8_t graphic_name[3];       // 图形名，删除/修改时作为索引，协议固定3字节
    uint32_t operate_tpye : 3;     // 图形操作：0空、1增加、2修改、3删除
    uint32_t graphic_tpye : 3;     // 图形类型：线/矩形/圆/椭圆/圆弧/浮点/整型/字符
    uint32_t layer : 4;            // 图层号：0~9
    uint32_t color : 4;            // 颜色编号，见 UI_Graph_Color_e
    uint32_t start_angle : 9;      // 圆弧起始角；数值/字符类型中复用为字号
    uint32_t end_angle : 9;        // 圆弧终止角；字符类型中复用为字符长度
    uint32_t width : 10;           // 线宽
    uint32_t start_x : 11;         // 起点/圆心 x 坐标
    uint32_t start_y : 11;         // 起点/圆心 y 坐标
    uint32_t radius : 10;          // 圆半径；浮点/整型低10位
    uint32_t end_x : 11;           // 终点/半轴；浮点/整型中间11位
    uint32_t end_y : 11;           // 终点/半轴；浮点/整型高11位
} Graph_Data_t;

typedef struct
{
    Graph_Data_t Graph_Control;
    char show_Data[30];
} String_Data_t; // 打印字符串数据

_Static_assert(sizeof(Graph_Data_t) == UI_Operate_LEN_PerDraw, "Graph_Data_t size mismatch");
_Static_assert(sizeof(String_Data_t) == UI_Operate_LEN_DrawChar, "String_Data_t size mismatch");

/* 删除操作 */
typedef enum {
    UI_Data_Del_NoOperate = 0,
    UI_Data_Del_Layer     = 1,
    UI_Data_Del_ALL       = 2, // 删除全部图层，后面的参数已经不重要了。
} UI_Delete_Operate_e;

/* 图形配置参数__图形操作 */
typedef enum {
    UI_Graph_ADD    = 1,
    UI_Graph_Change = 2,
    UI_Graph_Del    = 3,
} UI_Graph_Operate_e;

/* 图形配置参数__图形类型 */
typedef enum {
    UI_Graph_Line      = 0, // 直线
    UI_Graph_Rectangle = 1, // 矩形
    UI_Graph_Circle    = 2, // 整圆
    UI_Graph_Ellipse   = 3, // 椭圆
    UI_Graph_Arc       = 4, // 圆弧
    UI_Graph_Float     = 5, // 浮点型
    UI_Graph_Int       = 6, // 整形
    UI_Graph_Char      = 7, // 字符型

} UI_Graph_Type_e;

/* 图形配置参数__图形颜色 */
typedef enum {
    UI_Color_Main         = 0, // 红蓝主色
    UI_Color_Yellow       = 1,
    UI_Color_Green        = 2,
    UI_Color_Orange       = 3,
    UI_Color_Purplish_red = 4, // 紫红色
    UI_Color_Pink         = 5,
    UI_Color_Cyan         = 6, // 青色
    UI_Color_Black        = 7,
    UI_Color_White        = 8,

} UI_Graph_Color_e;

#pragma pack()

#endif
