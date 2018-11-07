#include "modbusMaster.h"
#include "usart.h"

static uint16_t GetCRC16(uint8_t *arr_buff, uint16_t len) {  //CRCУ�����
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

	unsigned char i;
	unsigned char cnt;
	unsigned int  crc;
	unsigned char crch, crcl;

	if ((MDbuf[0] != gasRxAdd) && (MDbuf[0] != 0)) return;								//��ַ���ʱ���ٶԱ�֡���ݽ���У��
	crc = GetCRC16(MDbuf, len - 2);								//����CRCУ��ֵ
	crch = crc >> 8;
	crcl = crc & 0xFF;
	if ((MDbuf[len - 1] != crch) || (MDbuf[len - 2] != crcl)) return;	//��CRCУ�鲻��ʱֱ���˳�
	switch (MDbuf[1]) {											//��ַ��У���־�����󣬽��������룬ִ����ز���

	case 0x03:											//��ȡһ���������ļĴ���
		if ((MDbuf[2] == 0x00) && (MDbuf[3] <= 0x20)) {			//ֻ֧��0x0000��0x0020
			i = MDbuf[3];									//��ȡ�Ĵ�����ַ
			cnt = MDbuf[5];									//��ȡ����ȡ�ļĴ�������
			MDbuf[2] = cnt * 2;								//��ȡ���ݵ��ֽ�����Ϊ�Ĵ�����*2
			len = 3;										//֡ǰ�����е�ַ�������롢�ֽ�����3���ֽ�
			while (cnt--) {
				unsigned int LocalStatusArrayTemp = LocalStatusArray[i++];	//��ȡ����16λ���飬ת��Ϊ2��8λ���ݴ��뷢������
				MDbuf[len++] = LocalStatusArrayTemp >> 8;
				MDbuf[len++] = LocalStatusArrayTemp & 0xff;
			}
		}
		else {					//�Ĵ�����ַ����֧��ʱ�����ش�����
			MDbuf[1] = 0x83;	//���������λ��1
			MDbuf[2] = 0x02;	//�����쳣��Ϊ02-��Ч��ַ
			len = 3;
		}
		break;

	case 0x06:											//д�뵥���Ĵ���
		if ((MDbuf[2] == 0x00) && (MDbuf[3] <= 0x20)) {	//�Ĵ�����ַ֧��0x0000��0x0020
			i = MDbuf[3];								//��ȡ�Ĵ�����ַ
			LocalStatusArray[i] = MDbuf[5];				//����Ĵ�������
			len -= 2;									//����-2�����¼���CRC������ԭ֡
		}
		else {					//�Ĵ�����ַ����֧��ʱ�����ش�����{
			MDbuf[1] = 0x86;	//���������λ��1
			MDbuf[2] = 0x02;	//�����쳣��Ϊ02-��Ч��ַ
			len = 3;
		}
		break;

	case 0x10:
		if ((MDbuf[2] == 0x00) && (MDbuf[3] <= 0x20)) {		//�Ĵ�����ַ֧��0x0000��0x0020
			i = MDbuf[3];									//��ȡ�Ĵ�����ַ
			cnt = MDbuf[5];									//��ȡ��д��ļĴ�������
			unsigned char startNum = 7;						//���ö�ȡ��Ҫд������ݵĵ�ַ
			unsigned int writeTemp = 0;						//д����8λת16λ�Ļ���
			while (cnt--) {									//дcnt��
				writeTemp = MDbuf[startNum];					//��һλ����д�뻺��
				gasRxTxTemp[i] = (writeTemp << 8) + MDbuf[startNum + 1];	//����Ĵ�������
				i++;
				startNum = startNum + 2;
			}

			len = 6;			//����6֡���¼���CRC������ԭ֡
			for (uint8_t i = 3; i < 12; i++)
			{
				localData[i] = gasRxTxTemp[i];
			}
		}
		else {					//�Ĵ�����ַ����֧��ʱ�����ش�����{
			MDbuf[1] = 0x86;	//���������λ��1
			MDbuf[2] = 0x02;	//�����쳣��Ϊ02-��Ч��ַ
			len = 3;
		}
		break;

	default:					//������֧�ֵĹ�����
		MDbuf[1] |= 0x80;		//���������λ��1
		MDbuf[2] = 0x01;		//�����쳣��Ϊ01-��Ч����
		len = 3;
		break;
	}
	crc = GetCRC16(MDbuf, len);		//���㷵��֡��CRCУ��ֵ
	MDbuf[len++] = crc & 0xFF;		//CRC���ֽ�
	MDbuf[len++] = crc >> 8;		//CRC���ֽ�
	if (MDbuf[0] != 0)				//�㲥ģʽ����Ҫ���ͷ���֡
	{
		HAL_UART_Transmit(&huart2, MDbuf, len, 1000);	//���ͷ���֡
	}

	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
}

static void sendDataMaster03(uint8_t start, uint8_t num) {
	uint8_t txBuf[10];
	uint16_t temp;
	txBuf[0] = slaveAdd;
	txBuf[1] = 0x03;
	txBuf[2] = 0x00;
	txBuf[3] = start;
	txBuf[4] = 0x00;
	txBuf[5] = num;//��195λ
	temp = GetCRC16(txBuf, 6);
	txBuf[6] = (uint8_t)(temp & 0xff);
	txBuf[7] = (uint8_t)(temp >> 8);

	HAL_UART_Transmit(&huart1, txBuf, 8, 0xffff);
}


static void gasTxCommand16(uint8_t slaveAdd) {

	uint16_t temp;
	uint8_t i;

	gasTxBuf[0] = slaveAdd;
	gasTxBuf[1] = 0x10;
	gasTxBuf[2] = 0x00;         //���ݵ���ʼ��ַ��
	gasTxBuf[3] = 0x03;
	gasTxBuf[4] = 0x00;         //���ݵĸ�����
	gasTxBuf[5] = 0x0a;
	gasTxBuf[6] = 0x14;         //���ݵ��ֽ�����
	for (i = 0; i<gasTxBuf[5]; i++) {
		gasTxBuf[7 + 2 * i] = (uint8_t)(localData[i + gasTxBuf[3]] >> 8);
		gasTxBuf[8 + 2 * i] = (uint8_t)(localData[i + gasTxBuf[3]] & 0xff);
	}
	temp = GetCRC16(gasTxBuf, 2 * gasTxBuf[5] + 7);
	gasTxBuf[7 + 2 * gasTxBuf[5]] = (uint8_t)(temp & 0xff);
	gasTxBuf[8 + 2 * gasTxBuf[5]] = (uint8_t)((temp >> 8) & 0xff);
	gasTxCount = 9 + 2 * gasTxBuf[5];
	HAL_UART_Transmit(&huart2, gasTxBuf, gasTxCount, 0xffff);
}

static void gasAlermRx() {

	if (uart2_recv_end_flag)
	{
		ModbusDecode(Usart2ReceiveBuffer.BufferArray, Usart2ReceiveBuffer.BufferLen);
		Usart2ReceiveBuffer.BufferLen = 0;
		uart2_recv_end_flag = 0;
	}
}

