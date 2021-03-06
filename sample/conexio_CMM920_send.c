/******************************************************************************/
// libconexio_CMM920_main.c - libconexio_CMM920_test file
/******************************************************************************/

/*
-- Copyright (c) 2015 Tomoyuki Niimi, Syunsuke Okamoto

-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in
-- all copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
-- THE SOFTWARE.
--
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <time.h>
#include <sys/time.h>
#include "libconexio_CMM920.h"
#include "conexio_cmm920.h"

#define APP_VERSION "1.0.0"

int main(int argc, char **argv)
{
	int iRet;

	char DevName[26]="/dev/ttyO3";
	char cMsg[512]="1234567890abcdefghijklmnopqrstuvwxyz";
	char cRecv[512]={0};
	int i=2;
	int ret = 0;
	int sig = 0;
	int cnt = 1;
	int cntOK = 0, cntNG = 0;
	int Antenna = 0;
	long iCnt = 1;
	long Retrycnt = 20;
	int iWait = 700;
	unsigned short myId = 0x1234;
	unsigned short myAddr = 0x5678;
	int leng;
	int rxp;
	int iHop = CONEXIO_CMM920_HOP_SINGLE;

	BYTE multi64bitAddr[8]={0};

	if( argc >= 1 ){

		strcpy( DevName, argv[1] ); 

		if( argc >= 2 ){

			// 引数解析
			while(i < argc){
				printf("[%d] %s\n", i, argv[i]);

				//help
				if(strncmp(argv[i], "--help", strlen("--help")) == 0 ){
					printf("Version : %s \n", APP_VERSION );
					printf("conexio_CMM920_send [device] [options]\n");
					printf("options:\n");
					printf(" -a[AntennaType]\n");
					printf(" -c[Count]\n");
					printf(" -r[Retry Count]\n");
					printf(" -d[Data]\n");
					printf(" -w[Wait(ms)]\n");
					printf(" -h[hop mode]\n");
					printf("Usage:\n");
					printf("Antenna type internal [0] external [1]\n");
					printf("infinite loop [-1]\n");
					return 0;
				}
				// Antenna
				if(strncmp(argv[i], "-a", strlen("-a")) == 0){
					if(sscanf(argv[i], "-a%d", &Antenna) != 1){
						ret = -1;
					}
				}

				// Cnt
				if(strncmp(argv[i], "-c", strlen("-c")) == 0){
					if(sscanf(argv[i], "-c%ld", &iCnt) != 1){
						ret = -1;
					}
				}

				// Retrycnt
				if(strncmp(argv[i], "-r", strlen("-r")) == 0){
					if(sscanf(argv[i], "-r%ld", &Retrycnt) != 1){
						ret = -1;
					}
				}

				// Wait
				if(strncmp(argv[i], "-w", strlen("-w")) == 0){
					if(sscanf(argv[i], "-w%d", &iWait) != 1){
						ret = -1;
					}
				}

				// hop
				if(strncmp(argv[i], "-h", strlen("-h")) == 0){
					if(sscanf(argv[i], "-h%d", &iHop) != 1){
						ret = -1;
					}
				}

				// Data
				if(strncmp(argv[i], "-d", strlen("-d")) == 0){
					if(strlen(argv[i]) > 2){
						strcpy(cMsg, &argv[i][2]);
					}
				}

				i++;
			}
			if(ret){
				printf("param error\n");
				return ret;
			}
		}
	}
	
	printf("Wait = %dms Cnt = %ld\n", iWait, iCnt);


	iRet = conexio_cmm920_init(DevName);

	if( iRet ){
		printf("open error\n");
		return 1;
	}

	// 空読み
//	(void)Serial_GetString(iPort, cRecv, sizeof(cRecv));

	// リセット要求
	iRet = ResetRequest();

	if( iRet ){
		printf("Reset Error\n");
	}
	// 設定モード要求
	iRet = SettingRequest();

	if( iRet ){
		printf("Setting Error\n");
	}else{
		printf("Setting OK\n");
	}

	// アンテナ設定要求
	if(Antenna == 0){
		// 内部アンテナ
		(void)InternalAntennaRequest();
	}
	else{
		// 外部アンテナ
		(void)ExternalAntennaRequest();
	}

	// 無線設定要求
//	(void)WirelessSettingRequest();
	conexio_cmm920_set_wireless(
				100000,	//100kbps
				55,		//55ch
				CONEXIO_CMM920_SET_WIRELESS_POWER_20MW,
				-85,	//-85 dbm
				-95,	// -95 dbm
				130,	// 130(usec)
				6,
				4,		// ACK 4回
				100		// 100(usec)
		);


	// 応答待ちタイマ設定要求
//	(void)TimerRequest();
	conexio_cmm920_set_timer(100);

	if( iHop == CONEXIO_CMM920_HOP_MULTI )
	{
		conexio_cmm920_set_hop_mode(iHop);
		conexio_cmm920_set_address(myId, multi64bitAddr, myAddr);
	}

	// 動作モード要求
	(void)RunRequest();

	// 初回の受信側の準備待ち
//	usleep(iWait * 2 * 1000);

	while(1){

		// 電文送受信
//		ret = (int)TelegramSend();

		if(iHop == CONEXIO_CMM920_HOP_SINGLE){
			iRet = conexio_cmm920_data_send_single(
					cMsg,
					36,
					CONEXIO_CMM920_SENDDATA_MODE_NOACK_RESP,
					NULL
			);
		}else{
			iRet = conexio_cmm920_data_send_multi(
				&cMsg[0],
				36,
				CONEXIO_CMM920_SENDDATA_MODE_ACK_NORESP,
				0x1234,
				myId,
				0x9ABC,
				myAddr,
				NULL
			);
		}
		usleep(iWait * 1000);
		for( i = 0; i < Retrycnt ; i ++ ){
			memset(&cRecv[0], 0x00, 512 * sizeof(char) );
			leng = 0;
			rxp = 0;
			if( iHop == CONEXIO_CMM920_HOP_SINGLE){
			iRet = conexio_cmm920_data_recv(
				&cRecv[0],
				&leng,
				CONEXIO_CMM920_HOP_SINGLE,
				NULL,
				&rxp,
				NULL
			);
			}else{
				iRet = conexio_cmm920_data_recv_multi(
					&cRecv[0],
					&leng,
					NULL,
					NULL,
					NULL,
					NULL,
					NULL,
					NULL,
					NULL				
				);
			}
//			if( iRet == 0 && leng == 0 ){
//			if( leng == 0 ){
//				i--;
//				continue;
//			}
			if( leng > 0 ){
				printf("rxpower = %d \n" , rxp );
			}
			if( strncmp(&cRecv[0], &cMsg[0],36 ) == 0 ){
				cntOK++;
				printf("OK %d/%d Count\n", cntOK, cnt);

				break;
			}else{
				if( i == Retrycnt - 1 ){
					cntNG++;
					printf("NG %d/%d Count\n", cntNG,cnt);
				}
			}
			usleep( iWait * 1000 );
		}
//		usleep(Looptimer);



		if(cnt == iCnt){
			break;
		}else{
			cnt++;
		}
	}

	// 停止モード要求
	(void)StopRequest();

	iRet = conexio_cmm920_exit();

	return ret;
}
