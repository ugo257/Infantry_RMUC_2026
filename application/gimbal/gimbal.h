#ifndef GIMBAL_H
#define GIMBAL_H

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask_YAW(); //底盘板
void GimbalTask_PITCH();//云台板

extern volatile float test_yaw_ref;
extern volatile float yaw_fit_time_s;
extern volatile float yaw_angle_rad;
extern volatile float yaw_gyro_rad_s;
extern volatile float motor_torque_nm;
extern volatile float yaw_motor_pos_rad;
extern volatile float yaw_motor_total_rad;

#endif // GIMBAL_H
