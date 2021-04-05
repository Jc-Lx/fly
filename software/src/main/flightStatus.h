#ifndef __FLIGHTSTATUS_H__
#define __FLIGHTSTATUS_H__

#include "mathTool.h"

typedef struct
{
    uint8_t  init;           //初始化状态
    uint8_t  failsafe;       //失控保护状态
    uint8_t  armed ;          //电机锁定状态
    uint8_t  flight;         //飞行状态
    uint8_t  placement;      //放置状态
    uint8_t  altControl;     //高度控制状态
    uint8_t  posControl;     //位置控制状态
    uint8_t  mode;
    uint32_t initFinishTime; //初始化完成时间
} FLIGHT_STATUS_t;


//放置状态
enum
{
    STATIC,		            //静止
    MOTIONAL			    //运动
};

//飞行状态
enum
{
    STANDBY,		        //待机
    TAKE_OFF,			    //起飞
    IN_AIR,				    //在空中
    LANDING,			    //降落
    FINISH_LANDING	        //降落完成
};

//电机锁定状态
enum
{
    DISARMED,	            //上锁
    ARMED				    //解锁
};

//水平方向控制状态
enum
{
    POS_HOLD,			    //悬停
    POS_CHANGED,			//飞行
    POS_BRAKE,				//刹车
    POS_BRAKE_FINISH	    //刹车完成
};

//垂直方向控制状态
enum
{
    ALT_HOLD,			    //悬停
    ALT_CHANGED,            //高度改变
    ALT_CHANGED_FINISH      //高度改变完成
};

//飞行模式
enum
{
    MANUAL = 0,			    //手动		(不带定高不带定点)
    SEMIAUTO,				//半自动 	(带定高不带定点)
    AUTO,					//自动		(带定高带定点)
    SPORT,                  //运动模式
    COMMAND,                //命令模式  (用于第三方控制)
    AUTOTAKEOFF,		    //自动起飞
    AUTOLAND,				//自动降落
    RETURNTOHOME,		    //自动返航
    AUTOCIRCLE,			    //自动绕圈
    AUTOPILOT,			    //自动航线
    FOLLOWME				//自动跟随
};

bool SetArmedStatus(uint8_t status);
uint8_t GetArmedStatus(void);
void PlaceStausCheck(Vector3f_t gyro);
uint8_t GetPlaceStatus(void);
void SetFlightMode(uint8_t mode);
uint8_t GetFlightMode(void);

#endif
