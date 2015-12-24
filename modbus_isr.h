//modbus realization
//
//values
enum modbus_exceptions{
	illegal_function=1,
	illegal_data_address=2,
	illegal_data_value=3,
	slave_device_failure=4,
	acknowledge=5,
	slave_device_busy=6
};
static 	int8	MB_Buffer[256];		//incoming buffer
static	int8	MB_Length=0;			//message length
static	int16	ValidReq;			//valid req count
static	int16	CRCReq;				//CRC error count
static	int16	ExcReq;				//ecxcept error count
//
//CRC table
/* Table of CRC values for high–order byte */
const int8 modbus_auchCRCHi[] = {
   0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
   0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
   0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
   0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
   0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
   0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
   0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
   0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
   0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
   0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
   0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
   0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
   0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
   0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
   0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
   0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
   0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
   0x40
};

/* Table of CRC values for low–order byte */
const int8 modbus_auchCRCLo[] = {
   0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
   0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
   0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
   0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
   0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
   0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
   0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
   0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
   0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
   0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
   0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
   0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
   0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
   0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
   0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
   0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
   0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
   0x40
};
//procedures
//
//
//UART preset
#USE RS232(baud=9600, xmit=TX, rcv=RX, enable=DIR, parity=N)
void	RsInit(void){
	MB_Length=0;	//set clear buffer
	//select rate
	switch(Speed){
		case	1200:	setup_uart(1200);
						break;
		case	1800:	setup_uart(1800);
						break;
		case	2400:	setup_uart(2400);
						break;
		case	4800:	setup_uart(4800);
						break;
		case	7200:	setup_uart(7200);
						break;
		case	9600:	setup_uart(9600);
						break;
		case	14400:	setup_uart(14400);
						break;
		case	19200:	setup_uart(19200);
						break;
		default:		setup_uart(9600);
	};	
//	dmy=getc();
}
//

//usart settings

//usart interrupt vector - packet interception
#INT_RDA
void	Incoming(void){
	MB_Buffer[MB_Length]=getc();	//read stream
	MB_Length++;					//set new point
	output_low(STAT);
	set_timer1((int16)(0xffff-35000000/Speed));	//set wait time
	clear_interrupt(INT_TIMER1);		//clear timer 1 int
	enable_interrupts(INT_TIMER1);	//enable timer 1 interrupt
//	clear_interrupt(int_rda);		//clear interrupt flag
}
//
//calculate CRC16 : 1st - buffer, 2d - length of data
int16	ModbusCRC(int8 * buf, int8 len){
	int8	i,index;
	int8	CRC_Low=0xff,CRC_High=0xff;
	//
	for(i=0; i<len; i++){
		index=CRC_High^buf[i];
		CRC_High=CRC_Low^modbus_auchCRCHi[index];
		CRC_Low=modbus_auchCRCLo[index];
	}
	//
	return	make16(CRC_High,CRC_Low);
}
//read registers
static int16	CRCr,CRCc;
static int8	j;
//

//add register data
void	AddRegData(int8 regist){
	int32	flcopy;
	restart_wdt();
			switch(regist){
				case 0x00:	//gamma 0					
					MB_Buffer[j]=make8(Gamma0,1);	//high part
					j++;
					MB_Buffer[j]=make8(Gamma0,0);	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x01:	//gamma1
					MB_Buffer[j]=make8(Gamma1,1);	//high part
					j++;
					MB_Buffer[j]=make8(Gamma1,0);	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x02:	//pulse width 1/2 amp				
					MB_Buffer[j]=make8(PulseWdt0,1);	//high part
					j++;
					MB_Buffer[j]=make8(PulseWdt0,0);	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x03:	//pulse width 1/2 amp
					MB_Buffer[j]=0;	//high part
					j++;
					MB_Buffer[j]=0;	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x04:	//pulse amplitude			
					MB_Buffer[j]=make8(Amplitude0,1);	//high part
					j++;
					MB_Buffer[j]=make8(Amplitude0,0);	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x05:	//pulse amplitude				
					MB_Buffer[j]=0;	//high part
					j++;
					MB_Buffer[j]=0;	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x06:	//temp of gen				
					MB_Buffer[j]=make8(T_GEN,1);	//high part
					j++;
					MB_Buffer[j]=make8(T_GEN,0);	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x07:	//temp 0					
					MB_Buffer[j]=make8(T_SR0,1);	//high part
					j++;
					MB_Buffer[j]=make8(T_SR0,0);	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x08:	//temp 1
					MB_Buffer[j]=make8(T_SR1,1);	//high part
					j++;
					MB_Buffer[j]=make8(T_SR1,0);	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x09:	//dummy command					
					MB_Buffer[j]=0x00;	//high part
					j++;
					MB_Buffer[j]=0x00;	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x0A:	//dummy command					
					MB_Buffer[j]=0x00;	//high part
					j++;
					MB_Buffer[j]=0x00;	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x0B:	//saw level
					MB_Buffer[j]=make8(SAW_A0,1);	//high part
					j++;
					MB_Buffer[j]=make8(SAW_A0,0);	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x0C:	//saw amplitude			
					MB_Buffer[j]=make8(SAW_A1,1);	//high part
					j++;
					MB_Buffer[j]=make8(SAW_A1,0);	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x0D:	//input coef 0					
					MB_Buffer[j]=make8(AmpCoef0,1);	//high part
					j++;
					MB_Buffer[j]=make8(AmpCoef0,0);	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x0E:	//input coef 1
					MB_Buffer[j]=make8(AmpCoef1,1);	//high part
					j++;
					MB_Buffer[j]=make8(AmpCoef1,0);	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x0F:	//measure points
					MB_Buffer[j]=make8(TAQ,1);	//high part
					j++;
					MB_Buffer[j]=make8(TAQ,0);	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x10:	//filter width				
					MB_Buffer[j]=make8(FWDT,1);	//high part
					j++;
					MB_Buffer[j]=make8(FWDT,0);	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x11:	//measures count
					MB_Buffer[j]=make8(PCOUNT,1);	//high part
					j++;
					MB_Buffer[j]=make8(PCOUNT,0);	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x12:	//speed register
					MB_Buffer[j]=make8(Speed,1);	//high part
					j++;
					MB_Buffer[j]=make8(Speed,0);	//low part
					j++;
					MB_Length+=2;
					break;
				case 0x13:	//host address register
					MB_Buffer[j]=make8(Addr,1);	//high part
					j++;
					MB_Buffer[j]=make8(Addr,0);	//low part
					j++;								
					MB_Length+=2;
					break;
				case 0x14:	//Valid request register
					MB_Buffer[j]=make8(ValidReq,1);	//high part
					j++;
					MB_Buffer[j]=make8(ValidReq,0);	//low part
					j++;								
					MB_Length+=2;
					break;
				case 0x15:	//CRC error register
					MB_Buffer[j]=make8(CRCReq,1);	//high part
					j++;
					MB_Buffer[j]=make8(CRCReq,0);	//low part
					j++;								
					MB_Length+=2;
					break;
				case 0x16:	//Exception register
					MB_Buffer[j]=make8(ExcReq,1);	//high part
					j++;
					MB_Buffer[j]=make8(ExcReq,0);	//low part
					j++;								
					MB_Length+=2;
					break;
				case 0x17:	//KGT				
					//copy
					memcpy(&flcopy,&KGT,4);
					//store to buf
					MB_Buffer[j]=make8(flcopy,0);	//high part
					j++;
					MB_Buffer[j]=make8(flcopy,1);	//high part
					j++;
					MB_Buffer[j]=make8(flcopy,2);	//high part
					j++;
					MB_Buffer[j]=make8(flcopy,3);	//high part
					j++;
					MB_Length+=4;
					break;
				case 0x18:	//KGU				
					//copy
					memcpy(&flcopy,&KGU,4);
					//store to buf
					MB_Buffer[j]=make8(flcopy,0);	//high part
					j++;
					MB_Buffer[j]=make8(flcopy,1);	//high part
					j++;
					MB_Buffer[j]=make8(flcopy,2);	//high part
					j++;
					MB_Buffer[j]=make8(flcopy,3);	//high part
					j++;
					MB_Length+=4;
					break;
				case 0x19:	//KST0
					//copy
					memcpy(&flcopy,&KST0,4);
					//store to buf
					MB_Buffer[j]=make8(flcopy,0);	//high part
					j++;
					MB_Buffer[j]=make8(flcopy,1);	//high part
					j++;
					MB_Buffer[j]=make8(flcopy,2);	//high part
					j++;
					MB_Buffer[j]=make8(flcopy,3);	//high part
					j++;
					MB_Length+=4;
					break;
				case 0x1A:	//KST1				
					MB_Buffer[j]=0x00;	//high part
					j++;
					MB_Buffer[j]=0x00;	//low part
					j++;
					MB_Buffer[j]=0x00;	//low part
					j++;
					MB_Buffer[j]=0x00;	//low part
					j++;
					MB_Length+=4;
					break;
				case 0x1B:	//T0_CAL
					MB_Buffer[j]=make8(T0_CAL,1);	//high part
					j++;
					MB_Buffer[j]=make8(T0_CAL,0);	//low part
					j++;								
					MB_Length+=2;
					break;
				case 0x1C:	//T1_CAL
					MB_Buffer[j]=make8(T1_CAL,1);	//high part
					j++;
					MB_Buffer[j]=make8(T1_CAL,0);	//low part
					j++;								
					MB_Length+=2;
					break;
				case 0x1D:	//TG_CAL
					MB_Buffer[j]=make8(TG_CAL,1);	//high part
					j++;
					MB_Buffer[j]=make8(TG_CAL,0);	//low part
					j++;								
					MB_Length+=2;
					break;
				default:	//no valid registers							
					MB_Buffer[1]|=0x80;
					MB_Buffer[2]=illegal_data_value;
					MB_Length=5;
					ExcReq++;
					ValidReq--; 
			};	
}
//
static	int8	bytecount;
//write registers
void	SetRegData(int8 reg){
	int32	flcopy;
	restart_wdt();
			switch(reg){
				case 0x00:	//dummy command					
					j+=2;	//set next data
					break;
				case 0x01:	//dummy command					
					j+=2;	//set next data
					break;
				case 0x02:	//dummy command					
					j+=2;	//set next data
					break;
				case 0x03:	//dummy command					
					j+=2;	//set next data
					break;
				case 0x04:	//dummy command					
					j+=2;	//set next data
					break;
				case 0x05:	//dummy command					
					j+=2;	//set next data
					break;
				case 0x06:	//dummy command					
					j+=2;	//set next data
					break;
				case 0x07:	//dummy command					
					j+=2;	//set next data
					break;
				case 0x08:	//dummy command					
					j+=2;	//set next data
					break;
				case 0x09:	//dummy command					
					j+=2;	//set next data
					break;
				case 0x0A:	//dummy command					
					j+=2;	//set next data
					break;
				case 0x0B:	//SAW_A0
					//read data from buffer
					SAW_A0=make16(MB_Buffer[j],MB_Buffer[j+1]);
					j+=2;	//set next data
					break;
				case 0x0C:	//SAW_A1
					//read data from buffer
					SAW_A1=make16(MB_Buffer[j],MB_Buffer[j+1]);
					j+=2;	//set next data
					break;
				case 0x0D:	//KU0
					//read data from buffer
					AmpCoef0=make16(MB_Buffer[j],MB_Buffer[j+1]);
					j+=2;	//set next data
					break;
				case 0x0E:	//KU1
					//read data from buffer
					AmpCoef1=make16(MB_Buffer[j],MB_Buffer[j+1]);
					j+=2;	//set next data
					break;
				case 0x0F:	//TAQ
					//read data from buffer
					TAQ=make16(MB_Buffer[j],MB_Buffer[j+1]);
					j+=2;	//set next data
					break;
				case 0x10:	//FWDT
					//read data from buffer
					FWDT=make16(MB_Buffer[j],MB_Buffer[j+1]);
					j+=2;	//set next data
					break;
				case 0x11:	//PCOUNT
					//read data from buffer
					PCOUNT=make16(MB_Buffer[j],MB_Buffer[j+1]);
					j+=2;	//set next data
					break;
				case 0x12:	//
					j+=2;	//set next data
					break;
				case 0x13:	//
					Addr=make16(MB_Buffer[j],MB_Buffer[j+1]);
					j+=2;	//set next data
					break;
				case 0x14:	//Valid request register	- no acton
					j+=2;	//set next data
					break;
				case 0x15:	//CRC error register - no action
					j+=2;	//set next data
					break;
				case 0x16:	//Exception register - no action
					j+=2;	//set next data
					break;
				case 0x17:	//KGT
					flcopy=make32(MB_Buffer[j+3],MB_Buffer[j+2],MB_Buffer[j+1],MB_Buffer[j]);
					memcpy(&KGT,&flcopy,4);
					j+=4;	//set next data
					break;
				case 0x18:	//KGU
					flcopy=make32(MB_Buffer[j+3],MB_Buffer[j+2],MB_Buffer[j+1],MB_Buffer[j]);
					memcpy(&KGU,&flcopy,4);
					j+=4;	//set next data
					break;
				case 0x19:	//KST0
					flcopy=make32(MB_Buffer[j+3],MB_Buffer[j+2],MB_Buffer[j+1],MB_Buffer[j]);
					memcpy(&KST0,&flcopy,4);
					j+=4;	//set next data
					break;
				case 0x1A:	//dummy command			
					j+=4;	//set next data
					break;
				case 0x1B:	//T0_CAL
					//read data from buffer
					T0_CAL=make16(MB_Buffer[j],MB_Buffer[j+1]);
					j+=2;	//set next data
					break;
				case 0x1C:	//T1_CAL
					//read data from buffer
					T1_CAL=make16(MB_Buffer[j],MB_Buffer[j+1]);
					j+=2;	//set next data
					break;
				case 0x1D:	//TG_CAL
					//read data from buffer
					TG_CAL=make16(MB_Buffer[j],MB_Buffer[j+1]);
					j+=2;	//set next data
					break;
				default:	//no valid registers							
					MB_Buffer[1]|=0x80;
					MB_Buffer[2]=illegal_data_value;
					MB_Length=5;
					ExcReq++;
					ValidReq--;
			};	
}
//
//timer 2 interrupt vector - packet recognition
#INT_TIMER1
void	Reaction(void){
	int8	StAddr,EnAddr;	
	int8	bytecount;
	int16	k,l;
	//end of tramsmittion process
	disable_interrupts(INT_RDA);
	output_high(STAT);	//on led
	//
	if (MB_Length<4)	goto exit;	//exit if to small
	//detect adress
	if((MB_Buffer[0]!=make8(Addr,0))&&(MB_Buffer[0]!=0))	goto exit;	//if not broadcast & not qwn address - then exit
	//calculate CRC
	CRCc=ModbusCRC(MB_Buffer,MB_Length-2);
	CRCr=make16(MB_Buffer[MB_Length-2],MB_Buffer[MB_Length-1]);
	//
	if(CRCc!=CRCr){				//if error CRC then exit
		CRCReq++;	
		goto exit;
	};
	//proceed command
	//analizing function code
	switch(MB_Buffer[1]){
		//CMODE
		case	0x42:
				if(MB_Length==5){
					switch(MB_Buffer[2]){
						case	0x00:	//normal cycle
							OneCycle=0;	//clear one cycle flag
							DaqState=DaqStart;	//start new measuring
							MB_Length=4;
							ValidReq++;
							break;
						case	0x01:
							OneCycle=1;	//set one cycle flag
							DaqState=DaqStart;	//set new cycle
							for(k=0; k<PCOUNT; k++){
								gam_buf0[k]=0;	//clear measuring buffer
								};
							MB_Length=4;
							ValidReq++;
							break;
						case	0x02:	//reset mcu
							reset_cpu();
							break;
						default:
						MB_Buffer[1]|=0x80;
						MB_Buffer[2]=illegal_data_value;
						ExcReq++;
						MB_Length=5;
						};
					}else{
					MB_Buffer[1]|=0x80;
					MB_Buffer[2]=illegal_data_address;
					MB_Length=5;
					ExcReq++;
					};
				break;
		//read buffer
		case	0x43:
				if(MB_Length==7){
					switch(MB_Buffer[3]){
						case	0x00:	//channel 0
							//pack
							l=2;
							for(k=0; k<0x100; k+=2){
								MB_Buffer[l]=0;
								l++;
								};
							MB_Length=132;
							ValidReq++;
							break;
						case	0x01:	//channel 1
							//pack
							l=2;
							for(k=0; k<0x100; k+=2){
								MB_Buffer[l]=0;
								l++;
								};
							MB_Length=132;
							ValidReq++;
							break;
						default:						
						MB_Buffer[1]|=0x80;
						MB_Buffer[2]=illegal_data_value;
						ExcReq++;
						MB_Length=5;
						};
					}else{
					MB_Buffer[1]|=0x80;
					MB_Buffer[2]=illegal_data_address;
					MB_Length=5;
					ExcReq++;
					};
				break;
		//get registers
		case	0x44:
				if(MB_Length==7){
					//valig req propose
					MB_Length=4;
					ValidReq++;
					//action
					//forming answer
					j=2;	//set start answer pointer
					StAddr=MB_Buffer[3];
					EnAddr=(MB_Buffer[3]+MB_Buffer[4]);
					for(bytecount=StAddr; bytecount<EnAddr; bytecount++){
						AddRegData(bytecount);		//get registers						
					};
					}else{
					MB_Buffer[1]|=0x80;
					MB_Buffer[2]=illegal_data_address;
					MB_Length=5;
					ExcReq++;
					};		
				break;
		//set registers
		case	0x45:
				if((MB_Length>=8)&&(!(MB_Length&0x01))){
					//action
					j=4;	//set start answer pointer
					StAddr=MB_Buffer[3];
					EnAddr=MB_Length-2;
					//valig req propose
					MB_Length=4;
					ValidReq++;
					for(bytecount=StAddr; j<EnAddr; bytecount++){
						SetRegData(bytecount);		//get registers
					};
					//forming answer
					MB_Length=4;
					ValidReq++;
					}else{
					MB_Buffer[1]|=0x80;
					MB_Buffer[2]=illegal_data_address;
					MB_Length=5;
					ExcReq++;
					};		
				break;
		//write to EEPROM command
		case	0x65:
				if(MB_Length==4){
					BUWrite();
					MB_Length=4;
					ValidReq++;
					}else{
					MB_Buffer[1]|=0x80;
					MB_Buffer[2]=illegal_data_address;
					MB_Length=5;
					ExcReq++;
					};		
				break;
		//set speed
		case	0x6A:
				if(MB_Length==5){
					MB_Length=4;
					ValidReq++;
					switch(MB_Buffer[2]){
						case	0x00:	//1200
							Speed=1200;
							setup_uart(1200);
							break;
						case	0x01:	//1800
							Speed=1800;
							setup_uart(1800);
							break;
						case	0x02:	//2400
							Speed=2400;
							setup_uart(2400);
							break;
						case	0x03:	//4800
							Speed=4800;
							setup_uart(4800);
							break;
						case	0x04:	//7200
							Speed=7200;
							setup_uart(7200);
							break;
						case	0x05:	//9600
							Speed=9600;
							setup_uart(9600);
							break;
						case	0x06:	//14400
							Speed=14400;
							setup_uart(14400);
							break;
						case	0x07:	//19200
							Speed=19200;
							setup_uart(19200);
							break;
						default:
							MB_Buffer[1]|=0x80;
							MB_Buffer[2]=illegal_data_value;
							MB_Length=5;
							ExcReq++;
					};
					output_toggle(STAT);
					//pause for master
					delay_ms(100);
					output_toggle(STAT);					
					}else{
					MB_Buffer[1]|=0x80;
					MB_Buffer[2]=illegal_data_address;
					MB_Length=5;
					ExcReq++;
					};		
				break;
		//send id
		case	0x6B:
				if(MB_Length==4){
					MB_Buffer[2]="M";
					MB_Buffer[3]="W";
					MB_Buffer[4]="T";
					MB_Buffer[5]="-";
					MB_Buffer[6]="S";
					MB_Buffer[7]="P";
					MB_Buffer[8]="M";
					MB_Buffer[9]="3";
					MB_Buffer[10]="v";
					MB_Buffer[11]="1";
					MB_Buffer[12]=".";
					MB_Buffer[13]="0";
					MB_Buffer[14]="0";
					MB_Length=16;
					ValidReq++;
					}else{
					MB_Buffer[1]|=0x80;
					MB_Buffer[2]=illegal_data_address;
					MB_Length=5;
					ExcReq++;
					};		
				break;
		//not recognized
		default:		//not recognized command
		MB_Buffer[1]|=0x80;	//set error code
		MB_Buffer[2]=illegal_function;
		MB_Length=5;
		ExcReq++;
	};	
	//send answer
	if(MB_Buffer[0]!=0){	//if unicast req - then send answer
	CRCc=ModbusCRC(MB_Buffer,MB_Length-2);	//calc CRC
	MB_Buffer[MB_Length-2]=make8(CRCc,1);	//high CRC
	MB_Buffer[MB_Length-1]=make8(CRCc,0);	//low CRC
	//send
	for(bytecount=0; bytecount<MB_Length; bytecount++){
		restart_wdt();
		putc(MB_Buffer[bytecount]);
		output_low(STAT);	//toggle led
		};
		goto exit;
	};	
	//
	
exit:	//exit
	output_high(STAT);	//on led
	MB_Length=0;
	clear_interrupt(INT_RDA);	
	clear_interrupt(INT_TIMER1);	
	disable_interrupts(INT_TIMER1);	
	enable_interrupts(INT_RDA);
}
//