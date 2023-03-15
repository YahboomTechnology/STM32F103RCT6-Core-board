#include "AllHeader.h"

u8 buff[]={"R and W success!"};

/**
* Function       // detcet_flash
* @author       // yahboom--
* @date         //     
* @brief         // ���w25q64�ⲿflash
* @param[in]     // ��
* @param[out]    // 0��1
* @retval        // 0���ɹ���1������
* @par History   // 
*/
u8 detcet_flash(void)
{
	u16 i = 0;
	while(SpiFlashReadID()!=W25Q64)							//��дW25Q64��ID���м���ж��Ƿ��Ǹ�оƬ
	{
		//ʱ�䵽���쳣
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
	//�Ȳ���д
	SpiFlashEraseSector(0);//����ɾȥ��ֻ�в�������д��ȥ����
	SpiFlashWrite(buff,0,sizeof(buff)); //д�����ݣ�0�������ֽڴ�С
	memset(buff,0,sizeof(buff));//�� 0
	SpiFlashRead(buff,0,sizeof(buff)); //�� 
	LCD_ShowString(10,30,(u8*)buff,BLACK,WHITE,16,0);

}


void show_flash(void)
{
	char flash_buf[]="This is test";
	u8 flash_flag = 0;
	//flash����
	flash_flag = detcet_flash();
	if(flash_flag == 0)
	{
		//�ҵ���flash
		sprintf(flash_buf,"W25Q64 normal!");
		LCD_ShowString(10,15,(u8*)flash_buf,BLACK,WHITE,16,0);
		
		//�������ж�д����
		flash_test();
	}
	else
	{
		sprintf(flash_buf,"W25Q64 fail....!");
		LCD_ShowString(10,15,(u8*)flash_buf,BLACK,WHITE,16,0);
	}

}
