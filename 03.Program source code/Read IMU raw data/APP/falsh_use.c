#include "AllHeader.h"

u8 buff[]={"R and W success!"};

/**
* Function       // detcet_flash
* @author       // yahboom--
* @date         //     
* @brief         // 检测w25q64外部flash
* @param[in]     // 无
* @param[out]    // 0，1
* @retval        // 0：成功，1：错误
* @par History   // 
*/
u8 detcet_flash(void)
{
	u16 i = 0;
	while(SpiFlashReadID()!=W25Q64)							//读写W25Q64的ID进行检测判断是否是该芯片
	{
		//时间到，异常
		if(SpiFlashReadID()==W25Q64) break;
		else
		{
			i++;
		}
		if(i>10000)
			return 1;
	}
	
	return 0;
		
}

void flash_test(void)
{
	//先擦后写
	SpiFlashEraseSector(0);//擦除删去，只有擦除才能写进去数据
	SpiFlashWrite(buff,0,sizeof(buff)); //写入数据，0扇区，字节大小
	memset(buff,0,sizeof(buff));//清 0
	SpiFlashRead(buff,0,sizeof(buff)); //读 
	LCD_ShowString(10,30,(u8*)buff,BLACK,WHITE,16,0);

}


void show_flash(void)
{
	char flash_buf[]="This is test";
	u8 flash_flag = 0;
	//flash测试
	flash_flag = detcet_flash();
	if(flash_flag == 0)
	{
		//找到该flash
		sprintf(flash_buf,"W25Q64 normal!");
		LCD_ShowString(10,15,(u8*)flash_buf,BLACK,WHITE,16,0);
		
		//正常进行读写操作
		flash_test();
	}
	else
	{
		sprintf(flash_buf,"W25Q64 fail....!");
		LCD_ShowString(10,15,(u8*)flash_buf,BLACK,WHITE,16,0);
	}

}
