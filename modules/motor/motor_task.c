#include "motor_task.h"
#include "LKmotor.h"
#include "HT04.h"
#include "dji_motor.h"
#include "dji_motor.h"
#include "step_motor.h"
#include "servo_motor.h"
#include "robot_board.h"
#include "DMmotor.h"

void MotorControlTask()
{
	//dji电机控制
    DJIMotorControl();
	//达秒电机控制
    DMMotorControl();
}
