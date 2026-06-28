// app
#include "robot_board.h"
#include "robot_params.h"
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
#include "general_def.h"
#include "user_lib.h"

#define Super_Cap_Charge_Voltage_MAX  23.0f // 电容最大电压
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)

/* 透视轨迹线参数：把车体前方两条地面参考线投影到选手端画面。 */
#define CHASSIS_TRACK_LINE_LAYER      2U
#define CHASSIS_TRACK_LINE_WIDTH      3U
#define CHASSIS_TRACK_BOTTOM_Y        240
#define CHASSIS_TRACK_BOTTOM_HALF_W   (TRACK_WIDTH * 0.9f)
#define CHASSIS_TRACK_NEAR_DIST       (0.5f * (TRACK_WIDTH + WHEEL_BASE) * 0.001f)
#define CHASSIS_TRACK_FAR_DIST        (CHASSIS_TRACK_NEAR_DIST + 8.0f * WHEEL_BASE * 0.001f)
#define CHASSIS_TRACK_CAMERA_HEIGHT   0.21f
#define CHASSIS_TRACK_PITCH_OFFSET    8.0f
#define CHASSIS_TRACK_PITCH_MIN      -15.0f
#define CHASSIS_TRACK_PITCH_MAX       35.0f
#define CHASSIS_TRACK_FOCAL_LENGTH  1020.0f

typedef struct
{
    uint8_t enable;
    uint8_t layer;
    uint8_t color;
    uint8_t width;
    uint32_t start_x;
    uint32_t start_y;
    uint32_t end_x;
    uint32_t end_y;
} UI_DebugLine_s;

typedef struct
{
    uint8_t enable;
    uint8_t layer;
    uint8_t color;
    uint8_t width;
    uint32_t center_x;
    uint32_t center_y;
    uint32_t radius;
} UI_DebugCircle_s;

volatile UI_DebugLine_s g_ui_debug_line = {
    .enable = 0U,
    .layer = 8U,
    .color = UI_Color_Purplish_red,
    .width = 5U,
    .start_x = 740U,
    .start_y = 420U,
    .end_x = 1700U,
    .end_y = 0U,
};

volatile UI_DebugCircle_s g_ui_debug_circle = {
    .enable = 1U,
    .layer = 8U,
    .color = UI_Color_Green,
    .width = 5U,
    .center_x = 740u,
    .center_y = 470U,
    .radius = 20U,
};

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

typedef enum {
    UI_STATUS_ROTATE = 0,
    UI_STATUS_FRICTION,
    UI_STATUS_HOLE,
    UI_STATUS_VISION,
    UI_STATUS_SMALL_RUNE,
    UI_STATUS_BIG_RUNE,
    UI_STATUS_CAP,
    UI_STATUS_COUNT,
} UI_Status_Index_e;

static Graph_Data_t benchmark[5];   // 基准线
static Graph_Data_t state_circle[UI_STATUS_COUNT]; // 业务状态灯
static String_Data_t Char_State[UI_STATUS_COUNT];  // 业务状态文字
static Graph_Data_t Cap_voltage_arc;
static Graph_Data_t Deviation_arc;
static Graph_Data_t Allow_heat_arc;
static Graph_Data_t chassis_track_line[2]; // 随pitch变化的底盘轨迹线
static Graph_Data_t debug_slash_line;      // 调试器可调斜线
static Graph_Data_t debug_circle;          // 调试器可调圆圈
static Graph_Data_t Cap_voltage;      // 电容电压
static Graph_Data_t total_voltage;      // 电容电压
static Graph_Data_t Shoot_Local_Heat; // 射击本地热量
int linex,liney,line,linex1,liney1,width,size;

static char ui_status_circle_name[UI_STATUS_COUNT][4] = {
    "rt0",
    "fr0",
    "ho0",
    "vi0",
    "sm0",
    "bg0",
    "cp0",
};

static char ui_status_text_name[UI_STATUS_COUNT][4] = {
    "tr0",
    "tf0",
    "th0",
    "tv0",
    "ts0",
    "tb0",
    "tc0",
};

static const char *ui_status_label[UI_STATUS_COUNT] = {
    "Rotate",
    "Friction",
    "Hole",
    "Vision",
    "Small",
    "Big",
    "Cap",
};

static const uint32_t ui_status_label_x[UI_STATUS_COUNT] = {
    500, 690, 910, 1090, 760, 940, 1270,
};

static const uint32_t ui_status_label_y[UI_STATUS_COUNT] = {
    125, 125, 125, 125, 75, 75, 125,
};

static const uint32_t ui_status_circle_x[UI_STATUS_COUNT] = {
    570, 790, 980, 1180, 840, 990, 1320,
};

static const uint32_t ui_status_circle_y[UI_STATUS_COUNT] = {
    160, 160, 160, 160, 110, 110, 160,
};

static uint8_t ui_status_last_valid = 0U;
static uint8_t ui_status_color_last[UI_STATUS_COUNT];
static uint8_t ui_static_inited = 0U;
static uint8_t ui_last_send_flag = 1U;
static uint8_t ui_live_refresh_divider = 0U;

static float getTotal_heat(uint8_t level);
static float allow_shoot_heat(float total,uint16_t temp);

static float UIClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int32_t UIClampInt32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float UIWrap360Float(float value)
{
    while (value < 0.0f) {
        value += 360.0f;
    }
    while (value >= 360.0f) {
        value -= 360.0f;
    }
    return value;
}

static uint8_t UIGetCapStatusColor(void)
{
    if (ui_cmd_recv.cap_online_flag == 0U) {
        return UI_Color_Pink;
    }
    if (ui_cmd_recv.cap_active_flag != 0U) {
        return UI_Color_Green;
    }
    if (ui_cmd_recv.cap_ready_flag != 0U) {
        return UI_Color_White;
    }
    if (ui_cmd_recv.SuperCap_mode == SUPERCAP_USE) {
        return UI_Color_Orange;
    }
    return UI_Color_Pink;
}

static uint8_t UIGetCapArcColor(void)
{
    if (ui_cmd_recv.cap_online_flag == 0U) {
        return UI_Color_Pink;
    }
    if (ui_cmd_recv.cap_ready_flag == 0U) {
        return UI_Color_Orange;
    }
    if (ui_cmd_recv.supercap_voltage >= SUPERCAP_HIGHER_THRESHOLD_VOLTAGE) {
        return UI_Color_Green;
    }
    if (ui_cmd_recv.supercap_voltage >= SUPERCAP_LOWER_THRESHOLD_VOLTAGE) {
        return UI_Color_Orange;
    }
    return UI_Color_Pink;
}

static void UIGetStatusColors(uint8_t color[UI_STATUS_COUNT])
{
    color[UI_STATUS_ROTATE] = (ui_cmd_recv.chassis_mode == CHASSIS_ROTATE) ? UI_Color_Main : UI_Color_White;
    color[UI_STATUS_FRICTION] = (ui_cmd_recv.friction_mode == FRICTION_ON) ? UI_Color_Main : UI_Color_White;
    color[UI_STATUS_HOLE] = (ui_cmd_recv.hole_mode != 0U) ? UI_Color_Green : UI_Color_White;
    color[UI_STATUS_VISION] = (ui_cmd_recv.nuc_flag == version_control) ? UI_Color_Green : UI_Color_White;
    color[UI_STATUS_SMALL_RUNE] = (ui_cmd_recv.vision_tx_mode == VISION_TX_MODE_KEY_B) ? UI_Color_Green : UI_Color_White;
    color[UI_STATUS_BIG_RUNE] = (ui_cmd_recv.vision_tx_mode == VISION_TX_MODE_KEY_F) ? UI_Color_Green : UI_Color_White;
    color[UI_STATUS_CAP] = UIGetCapStatusColor();
}

static void UIDrawStatusCircles(uint32_t operate, const uint8_t color[UI_STATUS_COUNT])
{
    for (uint8_t i = 0U; i < UI_STATUS_COUNT; ++i) {
        UICircleDraw(&state_circle[i],
                     ui_status_circle_name[i],
                     operate,
                     9,
                     color[i],
                     10,
                     ui_status_circle_x[i],
                     ui_status_circle_y[i],
                     10);
    }
}

static void UIRefreshStatusIfNeeded(uint8_t force)
{
    uint8_t color[UI_STATUS_COUNT];
    UIGetStatusColors(color);

    if (force == 0U &&
        ui_status_last_valid != 0U &&
        memcmp(color, ui_status_color_last, sizeof(color)) == 0) {
        return;
    }

    UIDrawStatusCircles(force ? UI_Graph_ADD : UI_Graph_Change, color);
    UIGraphRefresh(&referee_data_for_ui->referee_id,
                   7,
                   state_circle[0],
                   state_circle[1],
                   state_circle[2],
                   state_circle[3],
                   state_circle[4],
                   state_circle[5],
                   state_circle[6]);

    memcpy(ui_status_color_last, color, sizeof(color));
    ui_status_last_valid = 1U;
}

static void UIUpdateChassisTrackLines(uint32_t operate)
{
    const float pitch_deg = UIClampFloat(ui_cmd_recv.pitch_actual_angle_deg,
                                         CHASSIS_TRACK_PITCH_MIN,
                                         CHASSIS_TRACK_PITCH_MAX);
    const float pitch_rad = (pitch_deg + CHASSIS_TRACK_PITCH_OFFSET) * DEGREE_2_RAD;
    const float bottom_y = (float)CHASSIS_TRACK_BOTTOM_Y;
    const float screen_center_x = 0.5f * SCREEN_LENGTH;
    const float bottom_half_width = (float)CHASSIS_TRACK_BOTTOM_HALF_W;
    const float cos_pitch = arm_cos_f32(pitch_rad);
    const float sin_pitch = arm_sin_f32(pitch_rad);

    /*
     * 将地面上两条与车体前进方向平行的参考线投影到屏幕。
     * near/far 只影响视觉收束效果，不参与底盘控制。
     */
    float near_y_cam = cos_pitch * (-CHASSIS_TRACK_CAMERA_HEIGHT) - sin_pitch * CHASSIS_TRACK_NEAR_DIST;
    float near_z_cam = sin_pitch * (-CHASSIS_TRACK_CAMERA_HEIGHT) + cos_pitch * CHASSIS_TRACK_NEAR_DIST;
    float far_y_cam = cos_pitch * (-CHASSIS_TRACK_CAMERA_HEIGHT) - sin_pitch * CHASSIS_TRACK_FAR_DIST;
    float far_z_cam = sin_pitch * (-CHASSIS_TRACK_CAMERA_HEIGHT) + cos_pitch * CHASSIS_TRACK_FAR_DIST;

    if (near_z_cam < 0.15f) {
        near_z_cam = 0.15f;
    }
    if (far_z_cam < near_z_cam + 0.1f) {
        far_z_cam = near_z_cam + 0.1f;
    }

    const float far_half_width = bottom_half_width * near_z_cam / far_z_cam;
    const float horizon_y = bottom_y + CHASSIS_TRACK_FOCAL_LENGTH * near_y_cam / near_z_cam;
    const float far_y = bottom_y + CHASSIS_TRACK_FOCAL_LENGTH * far_y_cam / far_z_cam;

    const int32_t x_left_bottom = UIClampInt32((int32_t)(screen_center_x - bottom_half_width), 0, SCREEN_LENGTH);
    const int32_t x_right_bottom = UIClampInt32((int32_t)(screen_center_x + bottom_half_width), 0, SCREEN_LENGTH);
    const int32_t x_left_top = UIClampInt32((int32_t)(screen_center_x - far_half_width), 0, SCREEN_LENGTH);
    const int32_t x_right_top = UIClampInt32((int32_t)(screen_center_x + far_half_width), 0, SCREEN_LENGTH);
    const int32_t y_bottom = UIClampInt32(CHASSIS_TRACK_BOTTOM_Y, 0, SCREEN_WIDTH);
    const int32_t y_top = UIClampInt32((int32_t)(0.8f * far_y + 0.2f * horizon_y),
                                       40,
                                       CHASSIS_TRACK_BOTTOM_Y - 80);

    UILineDraw(&chassis_track_line[0], "tl0", operate, CHASSIS_TRACK_LINE_LAYER, UI_Color_Cyan,
               CHASSIS_TRACK_LINE_WIDTH, (uint32_t)x_left_bottom, (uint32_t)y_bottom,
               (uint32_t)x_left_top, (uint32_t)y_top);
    UILineDraw(&chassis_track_line[1], "tl1", operate, CHASSIS_TRACK_LINE_LAYER, UI_Color_Cyan,
               CHASSIS_TRACK_LINE_WIDTH, (uint32_t)x_right_bottom, (uint32_t)y_bottom,
               (uint32_t)x_right_top, (uint32_t)y_top);
}

static uint32_t UIClampDebugCoordinate(uint32_t value, uint32_t max_value)
{
    return (value > max_value) ? max_value : value;
}

static uint8_t UIClampDebugLayer(uint8_t layer)
{
    return (layer > 9U) ? 9U : layer;
}

static uint8_t UIClampDebugWidth(uint8_t width)
{
    return (width == 0U) ? 1U : width;
}

static uint32_t UIClampDebugRadius(uint32_t radius)
{
    return (radius == 0U) ? 1U : radius;
}

static void UIDrawDebugLine(uint32_t operate)
{
    if (g_ui_debug_line.enable == 0U) {
        return;
    }

    UILineDraw(&debug_slash_line,
               "dl0",
               operate,
               UIClampDebugLayer(g_ui_debug_line.layer),
               g_ui_debug_line.color,
               UIClampDebugWidth(g_ui_debug_line.width),
               UIClampDebugCoordinate(g_ui_debug_line.start_x, SCREEN_LENGTH),
               UIClampDebugCoordinate(g_ui_debug_line.start_y, SCREEN_WIDTH),
               UIClampDebugCoordinate(g_ui_debug_line.end_x, SCREEN_LENGTH),
               UIClampDebugCoordinate(g_ui_debug_line.end_y, SCREEN_WIDTH));
}

static void UIDrawDebugCircle(uint32_t operate)
{
    if (g_ui_debug_circle.enable == 0U) {
        return;
    }

    UICircleDraw(&debug_circle,
                 "dc0",
                 operate,
                 UIClampDebugLayer(g_ui_debug_circle.layer),
                 g_ui_debug_circle.color,
                 UIClampDebugWidth(g_ui_debug_circle.width),
                 UIClampDebugCoordinate(g_ui_debug_circle.center_x, SCREEN_LENGTH),
                 UIClampDebugCoordinate(g_ui_debug_circle.center_y, SCREEN_WIDTH),
                 UIClampDebugRadius(g_ui_debug_circle.radius));
}

static void UIDrawDeviationArc(uint32_t operate)
{
    const float offset_angle = UIWrap360Float((float)ui_cmd_recv.chassis_attitude_angle - YAW_ALIGN_ANGLE);
    const float offset_angle_left = UIWrap360Float(offset_angle - 10.0f);
    const float offset_angle_right = UIWrap360Float(offset_angle + 10.0f);

    /* 小圆弧表示当前云台相对底盘正前方的偏航方向，角度定义沿用裁判系统圆弧协议。 */
    UIArcDraw(&Deviation_arc, "dv0", operate, 4, UI_Color_Purplish_red,
              (uint32_t)offset_angle_left, (uint32_t)offset_angle_right,
              5, 956, 542, 50, 50);
}

static void UIDrawLiveGraphs(uint32_t operate)
{
    const float voltage = UIClampFloat(ui_cmd_recv.supercap_voltage,
                                       SUPERCAP_MIN_VOLTAGE,
                                       SUPERCAP_MAX_VOLTAGE);
    const uint32_t cap_end_angle =
        (ui_cmd_recv.cap_online_flag != 0U) ?
            (uint32_t)(272.0f + 60.0f * (voltage - SUPERCAP_MIN_VOLTAGE) /
                                   (SUPERCAP_MAX_VOLTAGE - SUPERCAP_MIN_VOLTAGE)) :
            272U;
    const uint8_t cap_color = UIGetCapArcColor();
    const float total_heat = getTotal_heat(ui_cmd_recv.robot_level);
    const float heat_left = allow_shoot_heat(total_heat, ui_cmd_recv.Shooter_heat);
    const float heat_display = UIClampFloat(heat_left, 0.0f, total_heat);
    const float heat_for_arc = UIClampFloat(heat_left, 0.0f, 500.0f);
    const uint8_t heat_color = (heat_left > 99.0f) ? UI_Color_Green : UI_Color_Purplish_red;
    const uint32_t heat_start_angle = (uint32_t)(266.0f - 0.076f * heat_for_arc);

    UIArcDraw(&Cap_voltage_arc, "pa0", operate, 9, cap_color, 271, cap_end_angle, 7, 956, 542, 383, 386);
    UIFloatDraw(&Cap_voltage, "cv0", operate, 9, cap_color, 15, 3, 3, 555, 550, (int32_t)(ui_cmd_recv.supercap_voltage * 1000.0f));
    UIArcDraw(&Allow_heat_arc, "ha0", operate, 9, heat_color, heat_start_angle, 266, 7, 956, 542, 383, 392);
    UIFloatDraw(&Shoot_Local_Heat, "sh0", operate, 7, heat_color, 20, 3, 3, 1100, 600, (int32_t)(heat_display * 1000.0f));
    UIDrawDeviationArc(operate);
}

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
    return 100.0f;
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
    UIDelete(&referee_data_for_ui->referee_id, UI_Data_Del_ALL, 0);

    UILineDraw(&benchmark[0], "b00", UI_Graph_ADD, 8, UI_Color_White, 10, 575, 530, 575, 550);
    UILineDraw(&benchmark[1], "b01", UI_Graph_ADD, 9, UI_Color_White, 10, 1345, 530, 1345, 550);
    UIArcDraw(&benchmark[2], "b02", UI_Graph_ADD, 7, UI_Color_White, 313, 316, 10, 956, 542, 383, 386);
    UIArcDraw(&benchmark[3], "b03", UI_Graph_ADD, 6, UI_Color_White, 223, 226, 10, 955, 541, 383, 386);
    UIArcDraw(&benchmark[4], "b04", UI_Graph_ADD, 5, UI_Color_White, 44, 47, 10, 961, 541, 383, 386);

    for (uint8_t i = 0U; i < UI_STATUS_COUNT; ++i) {
        snprintf(Char_State[i].show_Data, sizeof(Char_State[i].show_Data), "%s", ui_status_label[i]);
        UICharDraw(&Char_State[i],
                   ui_status_text_name[i],
                   UI_Graph_ADD,
                   7,
                   UI_Color_Orange,
                   22,
                   4,
                   ui_status_label_x[i],
                   ui_status_label_y[i],
                   ui_status_label[i]);
        UICharRefresh(&referee_data_for_ui->referee_id, Char_State[i]);
    }

    UIDrawLiveGraphs(UI_Graph_ADD);
    UIUpdateChassisTrackLines(UI_Graph_ADD);
    UIDrawDebugLine(UI_Graph_ADD);
    UIDrawDebugCircle(UI_Graph_ADD);
    UIFloatDraw(&total_voltage, "pw0", UI_Graph_ADD, 9, UI_Color_Green, 10, 0, 4, 860, 660, 24000);

    UIGraphRefresh(&referee_data_for_ui->referee_id, 5, benchmark[0], benchmark[1], benchmark[2], benchmark[3], benchmark[4]);
    UIRefreshStatusIfNeeded(1U);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 5, Cap_voltage_arc, Cap_voltage, Allow_heat_arc, Shoot_Local_Heat, total_voltage);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 2, chassis_track_line[0], chassis_track_line[1]);
    if (g_ui_debug_line.enable != 0U) {
        UIGraphRefresh(&referee_data_for_ui->referee_id, 1, debug_slash_line);
    }
    if (g_ui_debug_circle.enable != 0U) {
        UIGraphRefresh(&referee_data_for_ui->referee_id, 1, debug_circle);
    }
    UIGraphRefresh(&referee_data_for_ui->referee_id, 1, Deviation_arc);

    ui_static_inited = 1U;
    ui_live_refresh_divider = 0U;
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

    if (ui_static_inited == 0U ||
        (ui_cmd_recv.ui_send_flag == 0U && ui_last_send_flag != 0U)) {
        UI_StaticInit();
        ui_last_send_flag = ui_cmd_recv.ui_send_flag;
        PubPushMessage(ui_pub, (void *)&ui_feedback_data);
        return;
    }

    ui_last_send_flag = ui_cmd_recv.ui_send_flag;
    UIRefreshStatusIfNeeded(0U);

    ui_live_refresh_divider++;
    if (ui_live_refresh_divider >= 5U) {
        ui_live_refresh_divider = 0U;
        UIDrawLiveGraphs(UI_Graph_Change);
        UIUpdateChassisTrackLines(UI_Graph_Change);
        UIGraphRefresh(&referee_data_for_ui->referee_id, 5, Cap_voltage_arc, Cap_voltage, Allow_heat_arc, Shoot_Local_Heat, Deviation_arc);
        UIGraphRefresh(&referee_data_for_ui->referee_id, 2, chassis_track_line[0], chassis_track_line[1]);
        UIDrawDebugLine(UI_Graph_Change);
        if (g_ui_debug_line.enable != 0U) {
            UIGraphRefresh(&referee_data_for_ui->referee_id, 1, debug_slash_line);
        }
        UIDrawDebugCircle(UI_Graph_Change);
        if (g_ui_debug_circle.enable != 0U) {
            UIGraphRefresh(&referee_data_for_ui->referee_id, 1, debug_circle);
        }
    }

    PubPushMessage(ui_pub, (void *)&ui_feedback_data);
}
