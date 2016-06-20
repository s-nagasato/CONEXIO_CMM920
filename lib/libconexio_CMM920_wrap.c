/***

	libconexio_CMM920_wrap.c - conexio_CMM920_Wrapper functions library
	Copyright (C) Syunsuke Okamoto.<okamoto@contec.jp>

	This Library is proprietary Library. 
	Because, the Specification of conexio CMM920 is confidential.

***/

#include <stdio.h>
#include "libconexio_CMM920.h"

// wrapper
int conexio_cmm920_set_mode(int code)
{
	return conexio_cmm920_mode(
			CONEXIO_CMM920_SET_READING_WRITE,
			&code
	);
}

int conexio_cmm920_set_address(unsigned short panId, BYTE Addr[], unsigned short shortAddr )
{
	return conexio_cmm920_address(
			CONEXIO_CMM920_SET_READING_WRITE,
			&panId,
			Addr,
			&shortAddr
	);
}

int conexio_cmm920_set_wireless(int iBitrate, BYTE channel, BYTE power, char sendLv, char recvLv, unsigned short sendTim, BYTE sendNum, BYTE ackRetryNum, unsigned short ackWaitTim){
	BYTE bBitrate;
	BYTE bcdChannel;
	BYTE bcdSendLv;
	BYTE bcdRecvLv;

	switch(iBitrate){
	case 50000:
		bBitrate = CONEXIO_CMM920_SET_WIRELESS_BITRATE_50KBPS;
		break;
	case 100000:
		bBitrate = CONEXIO_CMM920_SET_WIRELESS_BITRATE_50KBPS;
		break;
	default:
		return 1;
	}
	bcdChannel = _calc_Bcd2Hex(channel);

	bcdSendLv = _conexio_cmm920_dBm2Hex((int)sendLv);
	bcdRecvLv = _conexio_cmm920_dBm2Hex((int)recvLv);

	return conexio_cmm920_wireless(
		CONEXIO_CMM920_SET_READING_WRITE,
		&bBitrate,
		&bcdChannel,
		&power,
		&bcdSendLv,
		&bcdRecvLv,
		&sendTim,
		&sendNum,
		&ackRetryNum,
		&ackWaitTim
	);


}
int conexio_cmm920_set_timer( unsigned short tim ){

	return conexio_cmm920_timer(
		CONEXIO_CMM920_SET_READING_WRITE,
		&tim
		);

}
int conexio_cmm920_set_auto_ack_frame( unsigned short phr, unsigned char fc_upper ){
	return conexio_cmm920_auto_ack_frame(
		CONEXIO_CMM920_SET_READING_WRITE,
		&phr,
		&fc_upper
	);
}
int conexio_cmm920_set_antenna( BYTE antennaMode ){
	return conexio_cmm920_antenna(
		CONEXIO_CMM920_SET_READING_WRITE,
		&antennaMode
	);
}

int conexio_cmm920_set_wireless_default(void)
{
	return conexio_cmm920_set_wireless(
			100000,	//100kbps
			60,	//60ch
			CONEXIO_CMM920_SET_WIRELESS_POWER_01MW,
			-85,	//-85 dbm
			-95,		// -95 dbm
			130,			// 130(msec)
			6,
			4,					// ACK 4回
			100					// 100(msec)
	);
}

int conexio_cmm920_set_auto_ack_frame_default(void)
{
	return conexio_cmm920_set_auto_ack_frame(
		0x07,
		0x00
	);

}

int conexio_cmm920_set_timer_default(void)
{
	return conexio_cmm920_set_timer(10000);

}

