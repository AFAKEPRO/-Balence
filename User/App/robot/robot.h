#ifndef __ROBOT_H
#define __ROBOT_H


/**
 * @brief 机器人初始化,请在开启rtos之前调用.这也是唯一需要放入main函数的函数
 * 
 */
void RobotInit(void);
void RobotTask(void);


#endif
