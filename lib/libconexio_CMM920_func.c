/***

	libconexio_CMM920_func.c - conexio_CMM920_functions library
	Copyright (C) 2015 Tomoyuki Niimi, Syunsuke Okamoto.<okamoto@contec.jp>

	This Library is proprietary Library. 
	Because, the Specification of conexio CMM920 is confidential.

***/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <sys/time.h>
#include "libconexio_CMM920.h"
#include "serialfunc.h"

#if 0
#define DbgPrint(fmt...)	printf(fmt)
#else
#define DbgPrint(fmt...)	do { } while (0)
#endif

int iWait;

static int iPort;

/***
	@function _conexio_cmm920_check_read_or_write
***/
static int _conexio_cmm920_check_read_or_write( int iFlag )
{
	switch( iFlag ){
	case CONEXIO_CMM920_SET_READING_READ:
	case CONEXIO_CMM920_SET_READING_WRITE:
		return 0;
	default:
		return 1;
	}
}

static int _conexio_cmm920_alloc_command( BYTE* command,  int size )
{
		command = (BYTE*)malloc( sizeof(BYTE) * size );

		if( command == (BYTE*) NULL ){
			return 1;
		}
		return 0;
}

BYTE _calc_Hex2Bcd( BYTE hex ){

	return ( ( hex / 16 ) * 10 + ( hex % 16 ) );

}

BYTE _calc_Bcd2Hex( BYTE bcd ){

	return ( ( bcd / 10 ) * 16 + ( bcd % 10 ) );

}

BYTE _conexio_cmm920_dBm2Hex( int dBm ){
	dBm = dBm * (-1);
	if( dBm >= 100 ){
		dBm -= 100;
	}
	return _calc_Bcd2Hex( (BYTE)dBm );
}

int _conexio_cmm920_Hex2dBm( BYTE hex ){
	int dBm;
	dBm = (int) _calc_Hex2Bcd( hex );
	
	if( dBm <= 10 ){
		dBm += 100;
	}
	dBm = dBm * (-1);
	return dBm;
}

int _conexio_cmm920_send_recv( BYTE Data[], int size ,int mode, int command ){
	int iRet, send_size = size;

	if( Data[0] == CONEXIO_CMM920_SET_READING_READ ){
		send_size = 1;
	}

	iRet = SendCommand(	Data, send_size,	mode, command );

	if( iRet < 0 )	return (-16 * 1) + iRet;

	usleep(iWait);

	iRet = RecvCommandAck(Data, &send_size,	mode, command );

	if( iRet < 0 )	return (-16 * 2) + iRet;
	
	return iRet;
}

// CONEXIO 920MHz Initialize
int conexio_cmm920_init(char *PortName){

	int iBaudrate = 115200;
	int iLength = 8;
	int iStop = 1;
	int iParity = 0;

	iPort = Serial_PortOpen_Full(
		PortName,
		iBaudrate,
		iLength,
		iStop,
		iParity,
		0 ,
		0
	);

	if(iPort < 0){
		return 1;
	}

	iWait = 10000;

	return 0;
}

//Exit処理
int conexio_cmm920_exit()
{
	if(iPort == 0) return 0;

	Serial_PortClose(iPort);

	return 0;
}

// 繝ｪ繧ｻ繝�繝郁ｦ∵ｱ�
int conexio_cmm920_reset()
{
	int iRet;
	DbgPrint("conexio_cmm920_reset A\n");

	BYTE Data[] = {0x01};
	int Size = 1;

	iRet = _conexio_cmm920_send_recv(
		Data,
		Size,
		CONEXIO_CMM920_MODE_COMMON,
		CONEXIO_CMM920_RESET
	);

	if( iRet ){
		DbgPrint("<conexio_cmm920_reset>:Sending Error\n");
	}

	return iRet;

}

// 險ｭ螳壹Δ繝ｼ繝芽ｦ∵ｱ�
int conexio_cmm920_mode(BYTE isWrite , int *code)
{
	int iRet;
	BYTE Data[2];
	int Size = 2;

	DbgPrint("conexio_cmm920_mode code:%x \n", *code );

	if( _conexio_cmm920_check_read_or_write( isWrite ) ){
		DbgPrint("<conexio_cmm920_mode>:Parameter Error\n");
		return -1;
	}

	Data[0] = isWrite;

	if( isWrite == CONEXIO_CMM920_SET_READING_WRITE ){
		switch( *code ){
		case CONEXIO_CMM920_SET_MODE_STOP:
		case CONEXIO_CMM920_SET_MODE_SETTING:
		case CONEXIO_CMM920_SET_MODE_RUN:
		case CONEXIO_CMM920_SET_MODE_TEST:
			break;
		default:
			DbgPrint("<conexio_cmm920_mode>:Parameter Error\n");
			return -1;		
		}
		Data[1] = *code;
	}

	iRet = _conexio_cmm920_send_recv(
		Data,
		Size,
		CONEXIO_CMM920_MODE_COMMON,
		CONEXIO_CMM920_SET_MODE
	);

	if( iRet < 0 ){
		DbgPrint("<conexio_cmm920_mode>:Communication Error %x\n", iRet);
	}
	else if( iRet > 0 )
	{
		DbgPrint("<conexio_cmm920_mode>:Result Error : %x\n", iRet );
	}
	else
	{
		if( Data[0] == CONEXIO_CMM920_SET_READING_READ ){
			*code = Data[1];
		}
	}

	return iRet;

}

// 繧｢繝峨Ξ繧ｹ險ｭ螳夊ｦ∵ｱ�
int conexio_cmm920_address(BYTE isWrite, unsigned short *panId, BYTE Addr[], unsigned short *shortAddr )
{
	int iRet;
	BYTE Data[13];
	int Size = 13;
	int i;

	DbgPrint("conexio_cmm920_address_setting PAN ID %x Short Addr : %x\n", * panId, *shortAddr);


	if( _conexio_cmm920_check_read_or_write( isWrite ) ){
		DbgPrint("<conexio_cmm920_address>:Parameter Error\n");
		return -1;
	}

	Data[0] = isWrite;

	if( isWrite == CONEXIO_CMM920_SET_READING_WRITE ){
		Data[1] = ( *panId & 0xFF00 )>> 8;
		Data[2] = ( *panId & 0xFF );
		for( i = 0 ; i < 8; i ++ ){
			Data[3 + i] = Addr[i];
		}
		Data[11] = ( *panId & 0xFF00 )>> 8;
		Data[12] = ( *panId & 0xFF );
	}

	iRet = _conexio_cmm920_send_recv(
		Data,
		Size,
		CONEXIO_CMM920_MODE_COMMON,
		CONEXIO_CMM920_SET_ADDRESS
	);

	if( iRet < 0 ){
		DbgPrint("<conexio_cmm920_address>:Communication Error %x\n", iRet);
	}
	else if( iRet > 0 )
	{
		DbgPrint("<conexio_cmm920_address>:Result Error : %x\n", iRet );
	}
	else
	{
		if( Data[0] == CONEXIO_CMM920_SET_READING_READ ){
			*panId = (Data[1] << 8 ) | Data[2];
			for( i = 0; i < 8; i ++){
				Addr[i] = Data[ 3 + i ];
			}
			*shortAddr = (Data[11] << 8 ) | Data[12];
		}
	}

	return iRet;

}

// 辟｡邱夊ｨｭ螳夊ｦ∵ｱゑｼ�**mW縲�**:ch縲�***kbps縲�***ﾎｼs)
int conexio_cmm920_wireless(BYTE isWrite, BYTE *bitrate, BYTE *channel, BYTE *power, char *sendSenceLvl,
	 char *recvSenceLvl, unsigned short *sendSenceTim, BYTE *sendSenceNum, BYTE *ackRetryNum, unsigned short *ackWaitTim)
{
	int iRet;
	BYTE Data[12];
	int Size = 12;
	int i;

	DbgPrint("conexio_cmm920_wireless bitrate %x channel:%x Power : %x \n", *bitrate, *channel, *power);


	if( _conexio_cmm920_check_read_or_write( isWrite ) ){
		DbgPrint("<conexio_cmm920_address>:Parameter Error\n");
		return -1;
	}

	Data[0] = isWrite;

	if( isWrite == CONEXIO_CMM920_SET_READING_WRITE ){
		Data[1] = *bitrate;
		Data[2] = *channel;
		Data[3] = *power;
		if( *sendSenceLvl > (-82) || *sendSenceLvl < (-110) ){
			Data[4] = 0xFF;
		}else{
			Data[4] = _conexio_cmm920_dBm2Hex( *sendSenceLvl );
		}
		if( *recvSenceLvl > (-10) || *recvSenceLvl < (-110) ){
			Data[5] = 0xFF;
		}else{
			Data[5] = _conexio_cmm920_dBm2Hex( *recvSenceLvl );
		}
		Data[6] = (*sendSenceTim & 0xFF00 ) >> 8;
		Data[7] = (*sendSenceTim & 0xFF );
		Data[8] = *sendSenceNum;
		Data[9] = *ackRetryNum;
		Data[10] = (*ackWaitTim & 0xFF00 ) >> 8;
		Data[11] = (*ackWaitTim & 0xFF );
	}

	iRet = _conexio_cmm920_send_recv(
		Data,
		Size,
		CONEXIO_CMM920_MODE_COMMON,
		CONEXIO_CMM920_SET_WIRELESS
	);

	if( iRet < 0 ){
		DbgPrint("<conexio_cmm920_address>:Communication Error %x\n", iRet);
	}
	else if( iRet > 0 )
	{
		DbgPrint("<conexio_cmm920_address>:Result Error : %x\n", iRet );
	}
	else
	{
		if( Data[0] == CONEXIO_CMM920_SET_READING_READ ){
			*bitrate = Data[1];
			*channel = Data[2];
			*power = Data[3];
			if( Data[4] != 0xFF ){
				*sendSenceLvl = _conexio_cmm920_Hex2dBm(Data[4]);
			}else{
				*sendSenceLvl = Data[4];
			}
			if( Data[5] != 0xFF ){
				*recvSenceLvl = _conexio_cmm920_Hex2dBm(Data[5]);
			}else{
				*recvSenceLvl = Data[5];
			}
			*sendSenceTim = ( Data[6] << 8 ) | Data[7];
			*sendSenceNum = Data[8];
			*ackRetryNum = Data[9];
			*ackWaitTim = ( Data[10] << 8 ) | Data[11];
		}
	}

	return iRet;
}

// 蠢懃ｭ泌ｾ�縺｡繧ｿ繧､繝櫁ｨｭ螳夊ｦ∵ｱ�
int conexio_cmm920_timer( BYTE isWrite, unsigned short *tim )
{

	int iRet;
	BYTE Data[3];
	int Size = 3;
	int i;

	DbgPrint("conexio_cmm920_timer Timer : %d\n", *tim);


	if( _conexio_cmm920_check_read_or_write( isWrite ) ){
		DbgPrint("<conexio_cmm920_timer>:Parameter Error\n");
		return -1;
	}

	Data[0] = isWrite;

	if( isWrite == CONEXIO_CMM920_SET_READING_WRITE ){
		
		Data[1] = (*tim & 0xFF00 ) >> 8;
		Data[2] = (*tim & 0xFF );
	}

	iRet = _conexio_cmm920_send_recv(
		Data,
		Size,
		CONEXIO_CMM920_MODE_COMMON,
		CONEXIO_CMM920_SET_TIMER
	);

	if( iRet < 0 ){
		DbgPrint("<conexio_cmm920_timer>:Communication Error %x\n", iRet);
	}
	else if( iRet > 0 )
	{
		DbgPrint("<conexio_cmm920_timer>:Result Error : %x\n", iRet );
	}
	else
	{
		if( Data[0] == CONEXIO_CMM920_SET_READING_READ ){
			*tim = ( Data[1] << 8 ) | Data[2];
		}
	}

	return iRet;
}

// 閾ｪ蜍柊CK蠢懃ｭ斐ヵ繝ｬ繝ｼ繝�險ｭ螳夊ｦ∵ｱ�
int conexio_cmm920_auto_ack_frame( BYTE isWrite, unsigned short *phr, unsigned char *fc_upper )
{

	int iRet;
	BYTE Data[4];
	int Size = 4;
	int i;

	DbgPrint("conexio_cmm920_auto_ack_frame PHR : %x FC[15:8] : %x\n", *phr, *fc_upper);


	if( _conexio_cmm920_check_read_or_write( isWrite ) ){
		DbgPrint("<conexio_cmm920_auto_ack_frame>:Parameter Error\n");
		return -1;
	}

	Data[0] = isWrite;

	if( isWrite == CONEXIO_CMM920_SET_READING_WRITE ){
		Data[1] = (*phr & 0xFF00 ) >> 8;
		Data[2] = (*phr & 0xFF );
		Data[3] = *fc_upper;
	}

	iRet = _conexio_cmm920_send_recv(
		Data,
		Size,
		CONEXIO_CMM920_MODE_COMMON,
		CONEXIO_CMM920_SET_ACK
	);

	if( iRet < 0 ){
		DbgPrint("<conexio_cmm920_auto_ack_frame>:Ack Error\n");
	}
	else if( iRet > 0 )
	{
		DbgPrint("<conexio_cmm920_auto_ack_frame>:Result Error : %x\n", iRet );
	}
	else
	{
		if( Data[0] == CONEXIO_CMM920_SET_READING_READ ){
			*phr = ( Data[1] << 8 ) | Data[2];
			*fc_upper = Data[3];
		}
	}

	return iRet;

}

// 繧｢繝ｳ繝�繝願ｨｭ螳夊ｦ∵ｱ�_繧｢繝ｳ繝�繝願ｨｭ螳�
int conexio_cmm920_antenna( BYTE isWrite, BYTE *antennaMode )
{
	int iRet;
	BYTE Data[2];
	int Size = 2;
	int i;

	DbgPrint("conexio_cmm920_antenna antenna mode : %x\n", *antennaMode );

	if( _conexio_cmm920_check_read_or_write( isWrite ) ){
		DbgPrint("<conexio_cmm920_antenna>:Parameter Error\n");
		return -1;
	}

	Data[0] = isWrite;

	if( isWrite == CONEXIO_CMM920_SET_READING_WRITE ){
		switch( *antennaMode ){

		case CONEXIO_CMM920_SET_ANTENNA_EXTERNAL:
			DbgPrint("<EXTERNAL>\n");
			break;
		case CONEXIO_CMM920_SET_ANTENNA_INTERNAL:
			DbgPrint("<INTERNAL>\n");
			break;
		default:
			DbgPrint("<conexio_cmm920_antenna>:Parameter Error\n");
			return -1;
		}
		Data[1] = *antennaMode;
	}

	iRet = _conexio_cmm920_send_recv(
		Data,
		Size,
		CONEXIO_CMM920_MODE_COMMON,
		CONEXIO_CMM920_SET_ANTENNA
	);

	if( iRet < 0 ){
		DbgPrint("<conexio_cmm920_antenna>:Ack Error\n");
	}
	else if( iRet > 0 )
	{
		DbgPrint("<conexio_cmm920_antenna>:Result Error : %x\n", iRet );
	}
	else
	{
		if( Data[0] == CONEXIO_CMM920_SET_READING_READ ){
			*antennaMode = Data[1];
		}
	}

	return iRet;
}

// Version Read
int conexio_cmm920_version(int *ver, int *rev)
{
	int iRet;
	BYTE Data[3];
	int Size = 3;
	int i;

	DbgPrint("conexio_cmm920_version ver : %d rev: %d \n", *ver, *rev );


	Data[0] = CONEXIO_CMM920_SET_READING_READ;

	iRet = _conexio_cmm920_send_recv(
		Data,
		Size,
		CONEXIO_CMM920_MODE_COMMON,
		CONEXIO_CMM920_SET_VERSION
	);

	if( iRet < 0 ){
		DbgPrint("<conexio_cmm920_version >:Ack Error\n");
	}
	else if( iRet > 0 )
	{
		DbgPrint("<conexio_cmm920_antenna>:Result Error : %x\n", iRet );
	}
	else
	{
		if( ver != NULL )
			*ver = _calc_Hex2Bcd(Data[1]);
		if( rev != NULL )
			*rev = _calc_Hex2Bcd(Data[2]);
	}

	return iRet;
}

// LSI險ｭ螳夊ｦ∵ｱ�
int conexio_cmm920_lsi(unsigned long lsi_addr, int isWrite, unsigned short *value)
{
	int iRet;
	BYTE Data[6];
	int Size = 6;
	int i;

	DbgPrint("conexio_cmm920_lsi addr : %lx value : %x \n", lsi_addr, *value );

	if( _conexio_cmm920_check_read_or_write( isWrite ) ){
		DbgPrint("<conexio_cmm920_lsi>:Parameter Error\n");
		return -1;
	}

	Data[0] = (lsi_addr & 0x00FF0000 ) >> 16;
	Data[1] = (lsi_addr & 0x0000FF00 ) >> 8;
	Data[2] = (lsi_addr & 0x000000FF );
	Data[3] = isWrite;

	if( isWrite == CONEXIO_CMM920_SET_READING_WRITE ){
		Data[4] = (*value & 0xFF00 ) >> 8;
		Data[5] = (*value & 0xFF );
	}

	iRet = _conexio_cmm920_send_recv(
		Data,
		Size,
		CONEXIO_CMM920_MODE_COMMON,
		CONEXIO_CMM920_SET_LSI
	);

	if( iRet < 0 ){
		DbgPrint("<conexio_cmm920_lsi>:Ack Error\n");
	}
	else if( iRet > 0 )
	{
		DbgPrint("<conexio_cmm920_lsi>:Result Error : %x\n", iRet );
	}
	else
	{
		if( Data[3] == CONEXIO_CMM920_SET_READING_READ ){
			*value = (Data[4] << 8) | Data[5];
		}
	}

	return iRet;

}

int conexio_cmm920_lsi_data_preamble_bit_len(int isWrite, BYTE length)
{
	unsigned long lsi_addr;
	unsigned short value;
	int iRet;

	if( length < 8 ){
		DbgPrint("<conexio_cmm920_lsi_data_preamble_bit_len>:Parameter Error : %x\n", iRet );
		return -1; 
	}
	lsi_addr = CONEXIO_CMM920_LSIADDRESS_PRELEN;
	value = (unsigned short) length;

	iRet = conexio_cmm920_lsi( lsi_addr, isWrite, &value );

	if( !iRet ){
		if( isWrite == CONEXIO_CMM920_SET_READING_READ ){
			length = (BYTE)value;
		}
	}else{
		DbgPrint("<conexio_cmm920_lsi_data_preamble_bit_len>:Error : %x\n", iRet );
	}
	
	return iRet;
}

int conexio_cmm920_lsi_data_whitening( int isWrite, BYTE isEnable )
{
	unsigned long lsi_addr;
	unsigned short value;
	int iRet;

	if( isEnable > 1 ){
		DbgPrint("<conexio_cmm920_lsi_data_whitening>:Parameter Error : %x\n", iRet );
		return -1; 
	}

	lsi_addr = CONEXIO_CMM920_LSIADDRESS_WHITENING;
	value = (unsigned short) isEnable;

	iRet = conexio_cmm920_lsi( lsi_addr, isWrite, &value );

	if( !iRet ){
		if( isWrite == CONEXIO_CMM920_SET_READING_READ ){
			isEnable = (BYTE)value;
		}
	}else{
		DbgPrint("<conexio_cmm920_lsi_data_whitening>:Error : %x\n", iRet );
	}
	
	return iRet;
}

int conexio_cmm920_lsi_diversity_enable( int isWrite, BYTE isEnable )
{
	unsigned long lsi_addr;
	unsigned short value;
	int iRet;

	if( isEnable > 1 ){
		DbgPrint("<conexio_cmm920_lsi_diversity_enable>:Parameter Error : %x\n", iRet );
		return -1; 
	}

	lsi_addr = CONEXIO_CMM920_LSIADDRESS_DIVER_ENABLE;
	value = (unsigned short) isEnable;

	iRet = conexio_cmm920_lsi( lsi_addr, isWrite, &value );

	if( !iRet ){
		if( isWrite == CONEXIO_CMM920_SET_READING_READ ){
			isEnable = (BYTE)value;
		}
	}else{
		DbgPrint("<conexio_cmm920_lsi_diversity_enable>:Error : %x\n", iRet );
	}
	
	return iRet;
}

int conexio_cmm920_data_send(BYTE buf[], int size, int hop, int send_mode, BYTE r_buf[])
{
	int iRet;
	iRet = SendTelegram(buf, size, hop, send_mode );

	if( send_mode != CONEXIO_CMM920_SENDDATA_MODE_NOACK_NORESP &&
		r_buf != NULL )
	{
		iRet = RecvTelegram(r_buf, &size, hop, NULL, NULL);
	}
	return iRet;
}

int conexio_cmm920_data_recv(BYTE buf[], int *size, int hop, int *r_channel, int *rssi ){

	return RecvTelegram(buf, size, hop , r_channel, rssi );
}

/// 920MHz蛻ｶ蠕｡繧ｳ繝槭Φ繝峨�ｮ騾∝女菫｡髢｢謨ｰ

int SendTelegram(BYTE buf[], int size, int hop, int send_mode){

	return SendTelegramSingleHop(buf, size, send_mode);

}

int SendTelegramSingleHop(BYTE buf[], int size, int send_mode){

	BYTE*	pktBuf;
	int pktSize;
	int i, ret;

	pktBuf = (BYTE*)malloc( (size + 3) * sizeof(BYTE) );
	pktSize = size + 3;

	switch( send_mode ){
	case CONEXIO_CMM920_SENDDATA_MODE_NOACK_NORESP:
	case CONEXIO_CMM920_SENDDATA_MODE_ACK_NORESP:
	case CONEXIO_CMM920_SENDDATA_MODE_NOACK_RESP:
	case CONEXIO_CMM920_SENDDATA_MODE_ACK_RESP:
		break;
	default:
		return 0;
	}

	pktBuf[0] = send_mode;
	pktBuf[1] = ( size + 4 ) >> 8;
	pktBuf[2] = ( size + 4 ) % 256;

	for( i = 0; i < size ; i ++ ){
		pktBuf[3 + i] = buf[i];
	}

	ret = (int)SendCommand(pktBuf, pktSize, CONEXIO_CMM920_MODE_RUN, CONEXIO_CMM920_SENDDATA);

	return ret;
}

int RecvTelegram(BYTE buf[], int *size , int hop, int *r_channel, int *rssi )
{
	return RecvTelegramSingleHop(buf, size, r_channel, rssi);
}

int RecvTelegramSingleHop(BYTE buf[], int *size , int *r_channel, int *rssi )
{

	BYTE*	pktBuf;
	int pktSize;
	int i, ret, d_size, crc_size;
	int iRet;
	int head_size = 5; //recv enable/disable + rx_channel + rx_power + phr(2byte)
	int foot_size;

	crc_size = 32;// crc ( bit length )
	foot_size = 2 + (crc_size / 8); // crc + rssi + ant
	if( size != NULL && *size != 0 ){
		pktSize = *size + head_size + foot_size;
		pktBuf = (BYTE*)malloc( pktSize * sizeof(BYTE) );
	}else{
		pktSize = 0;
		pktBuf = (BYTE*)malloc( ( 512 + head_size + foot_size ) * sizeof(BYTE) );
	}
	// Data Received
	iRet = RecvCommandAck(pktBuf, &pktSize, CONEXIO_CMM920_MODE_RUN, CONEXIO_CMM920_SENDDATA);

	if( iRet ){
		return iRet;
	}

	if( pktBuf[0] == 0x01 ){
		if( r_channel != NULL )
			*r_channel = _conexio_cmm920_Hex2dBm(pktBuf[1]);
		if( rssi != NULL )
			*rssi = _calc_Hex2Bcd(pktBuf[2]);

		d_size = pktBuf[3] * 256 + pktBuf[4];


		for( i = 0; i < d_size ; i ++ ){
			buf[i] = pktBuf[head_size + i];
		}
		//crc, rx_rssi, rx_ant
	}
	return 0;
}

// 險ｭ螳夐�∽ｿ｡繧ｳ繝槭Φ繝�
int SendCommand(BYTE buf[], int size, BYTE mode, BYTE command )
{
	BYTE *array;
	int length;
	int i;
	PCONEXIO920PACKET pac;

	pac = allocConexioCMM920_packet(pac, mode, command, 1);

	if(pac == NULL) return -1;
	pac->data = (BYTE*)malloc(sizeof(BYTE) * size);
	if(pac->data == NULL)
	{
		printf("Memory allocation error\n");
		free(pac);
		return -2;
	}else{
		for (i = 0; i < size; i++)
		{
			pac->data[i] = buf[i];
		}
	}
	array = pktGetBYTEArray( pac, size , &length);
	//length = pktGetLength(size);

	DbgPrint("Port %x, length :%d \n ",iPort, length);
	Serial_PutString(iPort, array, ( length * sizeof(BYTE) ) );

	DbgPrint("Send Data = ");

	for (i = 0; i < length; i++)
	{
		DbgPrint("%02X", array[i]);
	}

	DbgPrint("\n");

	// malloc縺ｧ遒ｺ菫昴＠縺溘Γ繝｢繝ｪ縺ｯ蠢�縺夊ｧ｣謾ｾ縺吶ｋ
	free(array);
	return 0;
}

int pktChkBYTEArray(PCONEXIO920PACKET pac, BYTE *array, int size )
{

	long cnt;
	int HeadSize, FootSize, pktSizeOffset;

	HeadSize = 8;	//繝倥ャ繝�繧ｵ繧､繧ｺ 8byte
	FootSize = 3; //繝輔ャ繧ｿ繧ｵ繧､繧ｺ縺ｯ 3byte(蝗ｺ螳�)
	pktSizeOffset = 5;

	if(array == NULL)
	{
		DbgPrint("<pktChkBYTEArray> Memory allocation error\n");
		return 1;
	}

	/* header Check */
	if( array[0] != pac->dle || 
		array[1] != pac->stx ||
		array[2] != (BYTE)((size + pktSizeOffset) >> 8) ||
		array[3] != (BYTE)((size + pktSizeOffset) % 256) ||
		array[4] != ( pac->command[0] | CONEXIO_CMM920_RECVCOMMAND ) ||
		array[5] != pac->command[1]
	){
		DbgPrint("<pktChkBYTEArray> Header error\n");
		return 2;
	}

	/* check sum Check */
	for(pac->sum = 0, cnt = 2; cnt < size + HeadSize; cnt++)
	{
		pac->sum += array[cnt];
	}
	if( array[size + HeadSize] != (BYTE)((pac->sum ^ 0xFF) + 1) ){
		DbgPrint("<pktChkBYTEArray> CRC Error\n");
		return 3;
	}

	/* footer Check */
	if( array[size + HeadSize + 1] != pac->dle ||
		array[size + HeadSize + 2] != pac->etx
	){
		DbgPrint("<pktChkBYTEArray> Footer Error\n");
		return 4;
	}

	for (cnt = 0; cnt < size; cnt ++)
	{
		pac->data[cnt] = array[cnt + HeadSize];
	}

	return 0;
}

int RecvCommandAck( BYTE *buf, int *size , BYTE mode, BYTE command )
{

	BYTE *array;
	BYTE head_data[4];
	int length;
	int i;
	int iRet;
	int d_size;
	PCONEXIO920PACKET pac;

	pac = allocConexioCMM920_packet(pac, 0, 0, 0);
	if(pac == NULL){
		DbgPrint("<RecvCommandAck> Memory Null Allocate.\n");
		return -1;
	}
	//size get
	Serial_GetString(iPort, &head_data[0], 4 * sizeof(BYTE));

	d_size = (head_data[2] * 256 + head_data[3]);
	if( d_size < 5 ){
		DbgPrint("<RecvCommandAck> Non Data Length \n");
		return 0;
	}
	d_size -= 5; // ヘッダサイズを引いて実データサイズを求める
	pac->data = (BYTE*)malloc(sizeof(BYTE) * (d_size) );
	if(pac->data == NULL)
	{
		DbgPrint("<RecvCommandAck> Memory allocation error\n");
		free(pac);
		return -2;
	}
	array = pktGetBYTEArray(pac, d_size, &length);
	//length = pktGetLength(size);

	for( i = 0; i < 4 ; i ++)
		array[i] = head_data[i];

	// Get Data (without header)
	Serial_GetString(iPort, &array[4], (length - 4) * sizeof(BYTE));

	DbgPrint("Recv Data = ");

	for (i = 0; i < length; i++)
	{
		DbgPrint("%02X", array[i]);
	}

	DbgPrint("\n");

	pac = allocConexioCMM920_packet(pac, mode, command, 1);
	if(pac == NULL) {
		DbgPrint("<RecvCommandAck> Memory Null Allocate.\n");
		return -3;
	}
	pac->data = (BYTE*)malloc(sizeof(BYTE) * (d_size) );
	if(pac->data == NULL)
	{
		DbgPrint("<RecvCommandAck> Memory allocation error\n");
		free(pac);
		return -4;
	}

	if( size != NULL && *size != 0 ){
		iRet = pktChkBYTEArray( pac, array , *size);
	}else{
		iRet = pktChkBYTEArray( pac, array , d_size);
	}
	if( iRet ){
		DbgPrint("<RecvCommandAck> pkt Chk Error\n");
		return -5;
		
	}
	free(array);

	for( i = 0; i < d_size; i ++ ){
		buf[i] = pac->data[i];
	}

	if(size != NULL)	*size = d_size;

	return 0;	

}

// 髮ｻ譁�蜿嶺ｿ｡髢｢謨ｰ
/*int RecvTelegram(int size)
{
	int GetLength;
	int RecvLength;
	int Wait = 1;
	int i;
	BYTE Stmp[size];
	BYTE Rtmp[size];
	BYTE *cRecv;
	PCONEXIO920PACKET pac;
	int ret = 0;
	int cnt = 0;
	BYTE ch;
	BYTE RSSI;

	cRecv = pktGetBYTEcRecvSend(size);

	// 髮ｻ譁�繧貞女菫｡
	Serial_GetString(iPort, cRecv, RecvLength * sizeof(BYTE));

	// 蜿嶺ｿ｡繝√Ε繝阪Ν蜿門ｾ�
	ch = cRecv[9];
	RecvChannelDisplay(ch);

	// 蜿嶺ｿ｡諢溷ｺｦ蜿門ｾ�
	RSSI = cRecv[10];
	RecvSensitivityDisplay(RSSI);

	// 蜿嶺ｿ｡邨先棡陦ｨ遉ｺ
	printf("Recv Telegram = ");

	for(i = 13; i < (size + 13); i++)
	{
		Rtmp[i -13] = cRecv[i];
		printf("%02X", Rtmp[i -13]);
	}

	printf("\n");

	// malloc縺ｧ遒ｺ菫昴＠縺溘Γ繝｢繝ｪ縺ｯ蠢�縺夊ｧ｣謾ｾ縺吶ｋ
	free(cRecv);
	return ret;
}
*/
/*
// 髮ｻ譁�騾∝女菫｡髢｢謨ｰ
int SendRecvTelegram(BYTE buf[], int size, BYTE MODE, BYTE COMMAND)
{
	BYTE *Array;
	int GetLength;
	int RecvLength;
	int Wait = 1;
	int i;
	BYTE Stmp[size];
	BYTE Rtmp[size];
	BYTE *cRecv;
	PCONEXIO920PACKET pac;
	int ret = 0;
	int cnt = 0;
	BYTE ch;
	BYTE RSSI;

	do{


		pac = allocConexioCMM920_packet(pac, MODE, COMMAND, 1);
		if(pac == NULL) return FALSE;
		pac->data = (BYTE*)malloc(sizeof(BYTE) * size);
		if(pac->data == NULL)
		{
			printf("Memory allocation error\n");
			free(pac);
			return FALSE;
		}else{
			for (i = 0; i < size; i++)
			{
				pac->data[i] = buf[i];
			}
		}

		Array = pktGetBYTEArraySend(pac, size);
		GetLength = pktGetLengthSend(size);
		RecvLength = pktGetLengthSend(size);
	    RecvLength = RecvLength + 8;
		cRecv = pktGetBYTEcRecvSend(size);

		Serial_PutString(iPort, Array, GetLength * sizeof(BYTE));

		sleep(Wait);

		// 騾∽ｿ｡譎ゅ�ｮ繧ｳ繝槭Φ繝峨ョ繝ｼ繧ｿ縺ｯ髱櫁｡ｨ遉ｺ
		(void)Serial_GetString(iPort, cRecv, GetLength * sizeof(BYTE));

		sleep(Wait);
		free(cRecv);
		cRecv = pktGetBYTEcRecvSend(size);

		// 謚倥ｊ霑斐＠髮ｻ譁�繧貞女菫｡
		Serial_GetString(iPort, cRecv, RecvLength * sizeof(BYTE));

		// 豈碑ｼ�縺励※蜿嶺ｿ｡螟ｱ謨玲凾縺ｯ謖�螳壼屓謨ｰ縺ｾ縺ｧ繝ｪ繝医Λ繧､
		for(i = 11; i < (size + 11); i++)
		{
			Stmp[i -11] = Array[i];
		}

		for(i = 13; i < (size + 13); i++)
		{
			Rtmp[i -13] = cRecv[i];
		}

		if(strncmp(Stmp, Rtmp, size) != 0){
			cnt++;
			printf("Retry %d Count", cnt);
			printf("\n");
		}

//		if(cnt == Retrycnt){
//			break;
//		}

	}while(strncmp(Stmp, Rtmp, size) != 0);

	// 蜿嶺ｿ｡繝√Ε繝阪Ν蜿門ｾ�
	ch = cRecv[9];
	RecvChannelDisplay(ch);

	// 蜿嶺ｿ｡諢溷ｺｦ蜿門ｾ�
	RSSI = cRecv[10];
	RecvSensitivityDisplay(RSSI);

	// 騾∝女菫｡邨先棡陦ｨ遉ｺ
	printf("Send Telegram = ");

	for(i = 11; i < (size + 11); i++)
	{
		Stmp[i -11] = Array[i];
		printf("%02X", Stmp[i -11]);
	}

	printf("\n");

	printf("Recv Telegram = ");

	for(i = 13; i < (size + 13); i++)
	{
		Rtmp[i -13] = cRecv[i];
		printf("%02X", Rtmp[i -13]);
	}

	printf("\n");

	// 豈碑ｼ�蜃ｦ逅�
	if(strncmp(Stmp, Rtmp, size) == 0){
		ret = 0;
	}else{
		ret = -1;
	}

	// malloc縺ｧ遒ｺ菫昴＠縺溘Γ繝｢繝ｪ縺ｯ蠢�縺夊ｧ｣謾ｾ縺吶ｋ
	free(Array);
	free(cRecv);
	return ret;
}
*/
/// 繧ｷ繝ｪ繧｢繝ｫ騾壻ｿ｡髢｢菫ゅ�ｮ螳夂ｾｩ

// 920MHz蛻ｶ蠕｡繧ｳ繝槭Φ繝峨↓蠢�隕√↑繝｡繝｢繝ｪ繧堤｢ｺ菫昴☆繧矩未謨ｰ
PCONEXIO920PACKET allocConexioCMM920_packet(PCONEXIO920PACKET pac, BYTE mode, BYTE com, BYTE isSend )
{
	pac = (PCONEXIO920PACKET)malloc(sizeof(CONEXIO920PACKET));

	if(pac == (PCONEXIO920PACKET)NULL) return pac;

	// 騾∽ｿ｡譎ゅ�ｯ 繝代こ繝�繝医�倥ャ繝�縺ｫ蛻晄悄蛟､繧剃ｻ｣蜈･縺吶ｋ( 蜿嶺ｿ｡繝代こ繝�繝医�ｮ蝣ｴ蜷医�ｯ荳崎ｦ�)
	if( isSend ){
		pac->dle = 0x10;
		pac->stx = 0x02;
		pac->size[0] = 0x00;
		pac->size[1] = 0x00;
		pac->command[0] = mode;
		pac->command[1] = com;
		pac->result = 0x00;
		pac->resultCode = 0x00;
		pac->etx = 0x03;
	}

	return pac;
}

// 920MHz蛻ｶ蠕｡繧ｳ繝槭Φ繝峨〒菴ｿ逕ｨ縺励◆繝｡繝｢繝ｪ繧定ｧ｣謾ｾ縺吶ｋ髢｢謨ｰ
void freeConexioCMM920_packet(PCONEXIO920PACKET pac)
{
	if(pac->data != (BYTE *)NULL) free(pac->data);

	if(pac != (PCONEXIO920PACKET)NULL ) free(pac);
}

// 920MHz蛻ｶ蠕｡繧ｳ繝槭Φ繝峨ｒ邨�縺ｿ遶九※繧矩未謨ｰ
BYTE* pktGetBYTEArray( PCONEXIO920PACKET pac, int size , int *retSize)
{
	BYTE* retArray;
	long cnt;
	int HeadSize, FootSize, pktSizeOffset;

	HeadSize = 8;	//繝倥ャ繝�繧ｵ繧､繧ｺ 8byte
	pktSizeOffset	= 5;
	FootSize = 3; //繝輔ャ繧ｿ繧ｵ繧､繧ｺ縺ｯ 3byte(蝗ｺ螳�)

	// malloc縺ｧ蠢�隕√↑繝｡繝｢繝ｪ鬆伜沺繧堤｢ｺ菫昴☆繧�
	retArray = (BYTE *)malloc(sizeof(BYTE) * (HeadSize + FootSize + size ));
	if(retArray == (BYTE*)NULL)
	{
		DbgPrint("<pktGetBYTEArray> Memory allocation error\n");
		return (BYTE*)NULL;
	}

	*retSize = HeadSize + FootSize + size;

	/* header */
	retArray[0] = pac->dle;
	retArray[1] = pac->stx;
	retArray[2] = (BYTE)((size + pktSizeOffset) >> 8);
	retArray[3] = (BYTE)((size + pktSizeOffset) % 256);
	retArray[4] = pac->command[0];
	retArray[5] = pac->command[1];
	retArray[6] = pac->result;
	retArray[7] = pac->resultCode;

	/* Data */
	for(cnt = 0; cnt < size; cnt++)
	{
		retArray[HeadSize + cnt] = pac->data[cnt];
	}

	/* check sum */
	for(pac->sum = 0, cnt = 2; cnt < size + HeadSize; cnt++)
	{
		pac->sum += retArray[cnt];
	}
	retArray[size + HeadSize] = (BYTE)((pac->sum ^ 0xFF) + 1);

	/* footer */
	retArray[size + HeadSize + 1] = pac->dle;
	retArray[size + HeadSize + 2] = pac->etx;


	// malloc縺ｧ遒ｺ菫昴＠縺溘Γ繝｢繝ｪ縺ｯ蠢� 縺夊ｧ｣謾ｾ縺吶ｋ
	freeConexioCMM920_packet(pac);

	return retArray;
}
