#include "modbusMaster.h"
#include "usart.h"


uint16_t modbus04Temp01[128];	//从机1设备的04功能缓冲区
uint16_t modbus03Temp01[128];	//从机1设备的03功能缓冲区
uint16_t modbus03Temp02[128];	//从机2设备的03功能缓冲区
uint16_t modbus03Temp03[128];	//从机3设备的03功能缓冲区


uint16_t modbusTemp03[128];		//03功能码状态变量的缓冲区
uint16_t modbusTemp04[128];		//04功能码状态变量的缓冲区


static uint16_t GetCRC16(uint8_t *arr_buff, uint16_t len) {  //CRC校验程序
	uint16_t crc = 0xFFFF;
	uint16_t i, j;
	for (j = 0; j < len; j++) {
		crc = crc ^ *arr_buff++;
		for (i = 0; i < 8; i++) {
			if ((crc & 0x0001) > 0) {
				crc = crc >> 1;
				crc = crc ^ 0xa001;
			}
			else
				crc = crc >> 1;
		}
	}
	return (crc);
}


static void ModbusDecode(unsigned char *MDbuf, unsigned char len) {

	uint16_t  crc;
	uint8_t crch, crcl;
	uint16_t temp;

	if ((MDbuf[0]!=1)||(MDbuf[0] != 2)||(MDbuf[0] != 3))
	{
		return;
	}

	crc = GetCRC16(MDbuf, len - 2);								//计算CRC校验值
	crch = crc >> 8;
	crcl = crc & 0xFF;
	if ((MDbuf[len - 1] != crch) || (MDbuf[len - 2] != crcl)) return;	//如CRC校验不符时直接退出

	switch (MDbuf[0])
	{
	case 1:
		if (MDbuf[1] == 4)
		{
			for (uint8_t i = 0; i < MDbuf[2] / 2; i++)
			{
				modbus04Temp01[i] = (uint16_t)(MDbuf[3 + 2 * i] << 8) + MDbuf[4 + 2 * i];
			}
		}
		break;
	case 2:
		if (MDbuf[1] == 3)
		{
			for (uint8_t i = 0; i < MDbuf[2] / 2; i++)
			{
				modbus03Temp02[i] = (uint16_t)(MDbuf[3 + 2 * i] << 8) + MDbuf[4 + 2 * i];
			}
		}
		break;
	case 3:
		if (MDbuf[1] == 3)
		{
			for (uint8_t i = 0; i < MDbuf[2] / 2; i++)
			{
				modbus03Temp03[i] = (uint16_t)(MDbuf[3 + 2 * i] << 8) + MDbuf[4 + 2 * i];
			}
		}
		break;
	default:
		break;
	}
}

void sendDataMaster03(uint8_t slaveAdd, uint16_t start, uint16_t num) {
	uint8_t txBuf[10];
	uint16_t temp;
	txBuf[0] = slaveAdd;
	txBuf[1] = 0x03;
	txBuf[2] = (start&0xff00)>>8;
	txBuf[3] = start&0xff;
	txBuf[4] = (num&0xff00)>>8;
	txBuf[5] = num&0xff;
	temp = GetCRC16(txBuf, 6);
	txBuf[6] = (uint8_t)(temp & 0xff);
	txBuf[7] = (uint8_t)(temp >> 8);

	HAL_UART_Transmit(&huart1, txBuf, 8, 0xffff);
}

void sendDataMaster04(uint8_t slaveAdd, uint16_t start, uint16_t num) {
	uint8_t txBuf[10];
	uint16_t temp;
	txBuf[0] = slaveAdd;
	txBuf[1] = 0x04;
	txBuf[2] = (start & 0xff00) >> 8;
	txBuf[3] = start & 0xff;
	txBuf[4] = (num & 0xff00) >> 8;
	txBuf[5] = num & 0xff;
	temp = GetCRC16(txBuf, 6);
	txBuf[6] = (uint8_t)(temp & 0xff);
	txBuf[7] = (uint8_t)(temp >> 8);

	HAL_UART_Transmit(&huart2, txBuf, 8, 0xffff);
}

/**
* @brief  发送16功能码
* @param  huart: 发送用的串口
* @param  slaveAdd: 要发送给的从机
* @param  scrArray[]: 要发送的数组
* @param  start: 发送数据的开始位置
* @param  num: 发送数据的数量
* @retval void
*/
void sendDataMaster16(UART_HandleTypeDef *huart,uint8_t slaveAdd,uint8_t scrArray[],uint16_t start,uint16_t num) {

	uint8_t txBuf[128];
	uint16_t temp;
	uint8_t i;
	uint16_t txBufCount;

	txBuf[0] = slaveAdd;
	txBuf[1] = 0x10;
	txBuf[2] = (start&0xff00)>>8;         //数据的起始地址；
	txBuf[3] = start&0xff;
	txBuf[4] = (num&0xff00)>>8;         //数据的个数；
	txBuf[5] = num&0xff;
	txBuf[6] = num*2;         //数据的字节数；
	for (i = 0; i<num; i++) {
		txBuf[7 + 2 * i] = (uint8_t)(scrArray[i + txBuf[3]] >> 8);
		txBuf[8 + 2 * i] = (uint8_t)(scrArray[i + txBuf[3]] & 0xff);
	}
	temp = GetCRC16(txBuf, 2 * txBuf[5] + 7);
	txBuf[7 + 2 * txBuf[5]] = (uint8_t)(temp & 0xff);
	txBuf[8 + 2 * txBuf[5]] = (uint8_t)((temp >> 8) & 0xff);
	txBufCount = 9 + 2 * txBuf[5];
	HAL_UART_Transmit(&huart, txBuf, txBufCount, 0xffff);
}

void modbusMasterScan() {

	if (uart1_recv_end_flag)
	{
		ModbusDecode(Usart1ReceiveBuffer.BufferArray, Usart1ReceiveBuffer.BufferLen);
		Usart1ReceiveBuffer.BufferLen = 0;
		uart1_recv_end_flag = 0;
	}

	if (uart2_recv_end_flag)
	{
		ModbusDecode(Usart2ReceiveBuffer.BufferArray, Usart2ReceiveBuffer.BufferLen);
		Usart2ReceiveBuffer.BufferLen = 0;
		uart2_recv_end_flag = 0;
	}
}

