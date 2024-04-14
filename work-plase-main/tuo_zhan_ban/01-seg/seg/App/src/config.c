#include "config.h"

/*****************************************************************
* @sysWork：项目的主要功能初始化函数
* @arg:
*		无
* @return:
*		无
*****************************************************************/
void sysInit(void)
{
	
}

/*****************************************************************
* @sysWork：项目的主要功能逻辑函数
* @arg:
*		无
* @return:
*		无
*****************************************************************/
void sysWork(void)
{
	static int i = 0;	
	segDisplay();
	
	segBuff[0] = i;
	segBuff[1] = i+1;
	segBuff[2] = i+2;
	i = (i+1)%24;
	HAL_Delay(1000);
}

