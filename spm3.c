#include "spm3.h"
#include "ee_var.h"

//directives
//#USE FAST_IO(A)
//#USE FAST_IO(C)
//#USE FAST_IO(D)
//const
float	const	f2int32=209715.2*0.995;
//variables
//buffers
//channel 0
//static	int8	data_buff0[256];	//reduced data buffer for channel 0
//static	int8	data_hist0[256];	//redused histogramm for ch0
static	int16	data_filt0[64];		//filtering buffer for ch0
static	int16	data_mean0;			//mean value
//static	int16	redu_mean0;			//reduces mean value
static	int16	maxRise0;			//max value at rising saw
static	int16	iMaxRise0;			//index of maximum at rising saw
static	int16	maxFall0;			//max value at falling saw
static	int16	iMaxFall0;			//index of maximum at falling saw
static	int16	alp_buf0[128];		//alpha meaning buffer
//static	int16	alp_sort[64];		//alpha sorted buffer
static	int16	bet_buf0[128];		//beta meaning buffer
//static	int16	bet_sort[64];		//beta sorted buffer
static	int16	gam_buf0[128];		//gamma meaning buffer
//static	int16	gam_sort[64];		//gamma sorted buffer
static	int16	Amplitude0;			//amplitude in B*1000
static	int16	PulseWdt0;			//pulse width in uS/10
static	int16	iGamma0;			//gamma in Daq intervals
static	int16	Gamma0;				//gamma in uS/10
static	int16	AmpCoef0;			//input amplifier coeff
static	float	KST0;				//
static	int16	T0_CAL;				//
static	signed int16	T_SR0;				//temperature of cn0
//channel 1
//static	int8	data_buff1[128];	//reduced data buffer for channel 1
//static	int8	data_hist1[128];	//redused histogramm for ch1
//static	int16	data_filt1[64];		//filtering buffer for ch1
//static	int16	data_mean1;			//mean value
//static	int16	redu_mean1;			//reduces mean value
//static	int16	maxRise1;			//max value at rising saw
//static	int16	iMaxRise1;			//index of maximum at rising saw
//static	int16	maxFall1;			//max value at falling saw
//static	int16	iMaxFall1;			//index of maximum at falling saw
//static	int16	gam_buf1[32];		//gamma meaning buffer
//static	int16	Amplitude1;			//amplitude in B*1000
//static	int16	PulseWdt1;			//pulse width in uS/10
//static	int16	iGamma1;			//gamma in Daq intervals
static	int16	Gamma1;				//gamma in uS/10
static	int16	AmpCoef1;			//input amplifier coeff
static	float	KST1;				//
static	int16	T1_CAL;				//
static	signed int16	T_SR1;				//temperature of cn1
//common
static 	int16	TAQ;				//number of data samples at measuring period
static	int16	FWDT;				//filering window width
static	int16	PCOUNT;				//measuring periods for meaning
static	int1	OneCycle;			//one cycle flag
static	int8	iGam=0;				//gamma meaning index
//saw
static	signed int16	T_GEN;				//generator temperature
static	int16	SAW_A0;				//level of saw
static	int32	SawLevel;			//current saw level
static	int16	SAW_A1;				//saw amplitude
static	int32	SawStep;			//saw step
static	float	KGT;				//
static	float	KGU;				//
static	int16	TG_CAL;				//
//communication
static	int16	Speed;				//USART speed
static	int16	Addr;				//own USART host adress
//filtering
static	int8	iWin;				//meaning window index
static	int32	AlpSum;
static	int32	BetSum;
static	int32	GamSum;
//temperature regulator
static	int16	Hst=50;				//Histeresis


//states
enum	daq_states{
	DaqIdle,		//idle state - state after daq stop
	DaqStart,		//cyclic mesuring start for one cycle
	DaqMeasure,		//measuring
	DaqScale,		//scaling measured values
	DaqEnd			//ending measuring cycle
};
int8	DaqState;	//Daq procedure state
//
//math variables
float	kgv;
float	Va;
float	delta_gamma, delta_gamma2;
float	T_0, T_G, T_G_k;
float	Gam0, Gam0_k;
float	k1, tsn;
float	W;
float	Haw0, Haw0_k;
float	AA, BB, a0, a1, a2, a3, a4, a5;
float	Am0, Am0_k;
float	T_S0, T_S0_k;
//procedures
//mathematics
char Calc() // Humidity calculation
	{	  
	float difftime0;
 	//coeff validation
 	if (kgv==0) kgv=1;
 	if (Va==0) Va=1;
 	//
 	delta_gamma =(kgt/kgv)*(T_0/Va)*(T_G-T_G_k);	//generator temperature correction
	delta_gamma2=k1*(T_S0-tsn);	//object temperature correction
 	difftime0=(Gam0-Gam0_k);	//addition parameter estimation
 	difftime0=difftime0+delta_gamma+delta_gamma2;	//adding corecction
 	//main regression
	W=AA+BB*(a0+a1*(Am0-Am0_k)*10+a2*(Haw0-Haw0_k)/1000+a3*difftime0/1000+a4*(T_G-T_G_k)/100+a5*(T_S0-T_S0_k)/100);
 
    return 0;
}
//Peripherals setup
void	InitMcu(void){		//initialization of MCU
	
   //oscillator configuration
   setup_oscillator(OSC_32MHZ|OSC_NORMAL|OSC_31250);
   //disable PSP
   setup_psp(PSP_DISABLED);
   //WDT config
   setup_wdt(WDT_OFF);
   //disable comparators
   setup_comparator(NC_NC_NC_NC);
   setup_vref(FALSE);
   //config ADC module
   setup_adc_ports(AN0_TO_AN5|VSS_VREF);
   setup_adc(ADC_CLOCK_DIV_16|ADC_TAD_MUL_2);
   
   //port configuration
   set_tris_a(0x3f);
   set_tris_c(0xf8);
   set_tris_d(0x00);
   //set high all selectors
   output_high(LDAC);
   output_high(MEM0);
   output_high(MEM1);
   output_high(DAC0);
   output_high(DAC1);
   output_high(KU0);
   output_high(KU1);
   output_high(STAT);

   
   //timers configuration
   setup_timer_0(RTCC_INTERNAL);
   
   setup_timer_1(T1_INTERNAL | T1_DIV_BY_8);
   
   setup_timer_2(T2_DISABLED,0,1);
   
   setup_timer_3(T3_INTERNAL | T3_DIV_BY_1);
   //setup communications
   //setup usart
   output_low(DIR);	//enable receiving
   //SPI configuration
   setup_spi(SPI_MASTER|SPI_CLK_DIV_4|SPI_H_TO_L);
   
   //interrupt configuration
   enable_interrupts(INT_RDA);
   
   enable_interrupts(GLOBAL);
   
   //States reset
   DaqState=DaqIdle;	//initial state
}
//
//read registers from EE
void	BURead(void){	//back up read
	int32	temp;
	//reading
	SAW_A0=make16(read_eeprom(EE_SAW_A0+1),read_eeprom(EE_SAW_A0)); //rise saw level
	SAW_A1=make16(read_eeprom(EE_SAW_A1+1),read_eeprom(EE_SAW_A1)); //rise saw amplitude
	AmpCoef0=make16(read_eeprom(EE_KU0+1),read_eeprom(EE_KU0)); //
	AmpCoef1=make16(read_eeprom(EE_KU1+1),read_eeprom(EE_KU1)); //
	TAQ=make16(read_eeprom(EE_TAQ+1),read_eeprom(EE_TAQ)); //
	PCOUNT=make16(read_eeprom(EE_PCOUNT+1),read_eeprom(EE_PCOUNT)); //
	FWDT=make16(read_eeprom(EE_FWDT+1),read_eeprom(EE_FWDT)); //
	Speed=make16(read_eeprom(EE_Speed+1),read_eeprom(EE_Speed)); //
	Addr=make16(read_eeprom(EE_Host+1),read_eeprom(EE_Host)); //
	T0_CAL=make32(read_eeprom(EE_T0_CAL+1),read_eeprom(EE_T0_CAL)); //
	T1_CAL=make16(read_eeprom(EE_T1_CAL+1),read_eeprom(EE_T1_CAL)); //
	TG_CAL=make16(read_eeprom(EE_TG_CAL+1),read_eeprom(EE_TG_CAL)); //
	temp=make32(read_eeprom(EE_KGT+2),read_eeprom(EE_KGT+3),read_eeprom(EE_KGT),read_eeprom(EE_KGT+1)); //
	memcpy(&KGT,&temp,4);
	temp=make32(read_eeprom(EE_KGU+2),read_eeprom(EE_KGU+3),read_eeprom(EE_KGU),read_eeprom(EE_KGU+1)); //
	memcpy(&KGU,&temp,4);
	temp=make32(read_eeprom(EE_KST0+2),read_eeprom(EE_KST0+3),read_eeprom(EE_KST0),read_eeprom(EE_KST0+1)); //
	memcpy(&KST0,&temp,4);
	temp=make32(read_eeprom(EE_KST1+2),read_eeprom(EE_KST1+3),read_eeprom(EE_KST1),read_eeprom(EE_KST1+1)); //
	memcpy(&KST1,&temp,4);	
}
//
//write registers to EE
void	BUWrite(void){	//back up write
	int32	temp;
	setup_wdt(WDT_OFF);
	//writing
	//int16
	write_eeprom(EE_SAW_A0,make8(SAW_A0,0));	write_eeprom(EE_SAW_A0+1,make8(SAW_A0,1));
	write_eeprom(EE_SAW_A1,make8(SAW_A1,0));	write_eeprom(EE_SAW_A1+1,make8(SAW_A1,1));
	write_eeprom(EE_KU0,make8(AmpCoef0,0));		write_eeprom(EE_KU0+1,make8(AmpCoef0,1));
	write_eeprom(EE_KU1,make8(AmpCoef1,0));		write_eeprom(EE_KU1+1,make8(AmpCoef1,1));
	write_eeprom(EE_TAQ,make8(TAQ,0));			write_eeprom(EE_TAQ+1,make8(TAQ,1));
	write_eeprom(EE_PCOUNT,make8(PCOUNT,0));	write_eeprom(EE_PCOUNT+1,make8(PCOUNT,1));
	write_eeprom(EE_FWDT,make8(FWDT,0));		write_eeprom(EE_FWDT+1,make8(FWDT,1));
	write_eeprom(EE_Speed,make8(Speed,0));		write_eeprom(EE_Speed+1,make8(Speed,1));
	write_eeprom(EE_Host,make8(Addr,0));		write_eeprom(EE_Host+1,make8(Addr,1));
	write_eeprom(EE_T0_CAL,make8(T0_CAL,0));	write_eeprom(EE_T0_CAL+1,make8(T0_CAL,1));
	write_eeprom(EE_T1_CAL,make8(T1_CAL,0));	write_eeprom(EE_T1_CAL+1,make8(T1_CAL,1));
	write_eeprom(EE_TG_CAL,make8(TG_CAL,0));	write_eeprom(EE_TG_CAL+1,make8(TG_CAL,1));
	//float
	memcpy(&temp,&KGT,4);
	write_eeprom(EE_KGT+2,make8(temp,3));		write_eeprom(EE_KGT+3,make8(temp,2));
	write_eeprom(EE_KGT,make8(temp,1));			write_eeprom(EE_KGT+1,make8(temp,0));
	memcpy(&temp,&KGU,4);
	write_eeprom(EE_KGU+2,make8(temp,3));		write_eeprom(EE_KGU+3,make8(temp,2));
	write_eeprom(EE_KGU,make8(temp,1));			write_eeprom(EE_KGU+1,make8(temp,0));
	memcpy(&temp,&KGT,4);
	write_eeprom(EE_KST0+2,make8(temp,3));		write_eeprom(EE_KST0+3,make8(temp,2));
	write_eeprom(EE_KST0,make8(temp,1));		write_eeprom(EE_KST0+1,make8(temp,0));
	memcpy(&temp,&KGT,4);
	write_eeprom(EE_KST1+2,make8(temp,3));		write_eeprom(EE_KST1+3,make8(temp,2));
	write_eeprom(EE_KST1,make8(temp,1));			write_eeprom(EE_KST1+1,make8(temp,0));
	
	setup_wdt(WDT_ON);
}
//
//temperature measuring
void	Termometer(void){
	signed int16	temp;
	//generator
	set_adc_channel(2);	//select generator channel
	delay_us(10);		//small wait
	temp=read_adc();	//read value
	T_GEN=(signed int16)((temp-205)*24.414);
	//t0
	set_adc_channel(5);	//select generator channel
	delay_us(10);		//small wait
	temp=read_adc();	//read value
	T_SR0=(signed int16)((temp-205)*24.414);
	//t1
	set_adc_channel(4);	//select generator channel
	delay_us(10);		//small wait
	temp=read_adc();	//read value
	T_SR1=(signed int16)((temp-205)*24.414);
}
//
//DAC setting
void	SetSawDac(int32	level){
	int16	course;
	//fine part
	output_low(DAC0);						//select fine DAC
	spi_write((make8(level,1)&0x03)|0x10);	//send high part
	spi_write(make8(level,0));				//send low part
	output_high(DAC0);						//deselect
	//course part
	course=(make16(make8(level,2),make8(level,1)))>>2;
	output_low(DAC1);						//select course DAC
	spi_write((make8(course,1)&0x0f)|0x10);	//send high part
	spi_write(make8(course,0));				//send low part
	output_high(DAC1);						//deselect
	output_low(LDAC);						//send DAC
	output_high(LDAC);						//strobe
}
//KU setting
void	SetKU0(int16 coeff){
	int16	dacval;
	setup_wdt(WDT_OFF);
	if((coeff<1)&(coeff>6))	coeff=1;
	dacval=(0x1fff>>coeff)|0x7000;
	output_low(KU0);				//select KU0 DAC
	spi_write(make8(dacval,1));		//send high part
	spi_write(make8(dacval,0));		//send low part
	output_high(KU0);				//deselect
	setup_wdt(WDT_ON);
}
void	SetKU1(int16 coeff){
	int16	dacval;
	setup_wdt(WDT_OFF);
	if((coeff<1)&(coeff>6))	coeff=1;
	dacval=(0x1fff>>coeff)|0x7000;
	output_low(KU1);				//select KU0 DAC
	spi_write(make8(dacval,1));		//send high part
	spi_write(make8(dacval,0));		//send low part
	output_high(KU1);				//deselect
	setup_wdt(WDT_ON);
}
//

//ISR
#include	"modbus_isr.h"
//
//
//
void main(void){  
	//addition variables
	int16	CycleCounter;
	int16	MaxCycles;
	int16	cnt;
	int16	course;
	int16	ADC0;	//current ADC value
	int16	Wrk0=0;	//sum mean value
	int16	FiltFact;	//filtering factor
	int16	StorAddr;	//storage address
	int8	OutBufAddr;	//out buffer address
	int32	Sum;
	int16	Mx=0x01ff;	//mean value
	int16	BetTrsh=0x01ff; //beta treshold
	int16	bet_cur;//current beta value
	
	
	int1	changed;
	
	
	InitMcu();		//mcu init   
	BURead();		//read registers from EE
	RsInit();

   //interrupt configuration
   enable_interrupts(INT_RDA);
   disable_interrupts(INT_TIMER1);
   enable_interrupts(GLOBAL);
   	
   	output_high(TERMO);
   	//for debug !!!
   	DaqState=DaqStart;
	//main cycle
	while(TRUE){
		restart_wdt();
		//state procesor
		switch(DaqState){
			case	DaqIdle:
				Termometer();
				//wait for state changing
				break;
			case	DaqStart:
				//prepare for measuring
				bet_cur=0;//reset beta value
				//set input amplifiers
				SetKU0(AmpCoef0);
				SetKU1(AmpCoef1);
				//zero buffers & variables
				for(cnt=0;cnt<FWDT;cnt++){
					data_filt0[cnt]=0;	//clear buffer
					};
//				for(cnt=0;cnt<0x100;cnt++){
//					data_hist0[cnt]=0;	//clear buffer
//					};
				StorAddr=0;
				OutBufAddr=0;
				iWin=0;
				iMaxFall0=0;
				iMaxRise0=0;
				maxFall0=0;
				maxRise0=0;
				data_mean0=0;
				//DAC step estim
				SawLevel=(int32)(((float)SAW_A0/1000)*f2int32);
				SawStep=(int32)(((float)SAW_A1/1000)*f2int32/(TAQ/2));
				//next state
				DaqState=DaqMeasure;	//set measuring state
				break;
			case	DaqMeasure:
				//preset initial saw level
				//fine part
//				output_low(DAC0);						//select fine DAC
//				spi_write((make8(SawLevel,1)&0x03)|0x10);	//send high part
//				spi_write(make8(SawLevel,0));				//send low part
//				output_high(DAC0);						//deselect
				//course part
				course=(make16(make8(SawLevel,2),make8(SawLevel,1)))>>2;
				output_low(DAC1);						//select course DAC
				spi_write((make8(course,1)&0x0f)|0x10);	//send high part
				spi_write(make8(course,0));				//send low part
				output_high(DAC1);						//deselect
				output_low(LDAC);						//send DAC
				output_high(LDAC);						//strobe				
				//select ADC channel
				set_adc_channel(0);	//select ch0
				//cycle variables
				MaxCycles=TAQ>>1;// /2
				FiltFact=(FWDT>>1)+1;// /2
				Sum=0;
//				redFact=(int8)(TAQ>>8)-1;	//estim reduse factor /128
//				redCount=0;
//============================================================================================
				//rising saw part
				//initial filtering
				for(CycleCounter=0; CycleCounter<FiltFact; CycleCounter++){					
					restart_wdt();
					//start ADC conv
					read_adc(ADC_START_ONLY);	//start conversion
					//estim next level
					SawLevel+=SawStep;
					//transfer new DAC value
					//fine part
//					output_low(DAC0);						//select fine DAC
//					spi_write((make8(SawLevel,1)&0x03)|0x10);	//send high part
//					spi_write(make8(SawLevel,0));				//send low part
//					output_high(DAC0);						//deselect
					//course part
					course=(make16(make8(SawLevel,2),make8(SawLevel,1)))>>2;
					output_low(DAC1);						//select course DAC
					spi_write((make8(course,1)&0x0f)|0x10);	//send high part
					spi_write(make8(course,0));				//send low part
					output_high(DAC1);						//deselect
					//wait for sync
					//not imlem yet!!!
					//read ADC
					ADC0=read_adc(ADC_READ_ONLY);
					//trigg LDAC
					output_low(LDAC);						//send DAC
					output_high(LDAC);						//strobe
					//meaning
					Wrk0-=data_filt0[iWin];	//calc new sum mean val
					Wrk0+=ADC0;
					data_filt0[iWin]=ADC0;
					//next filter value
					if(iWin==0){
						iWin=FWDT;
						iWin--;
						}else	iWin--;
					//calc mean value
					data_mean0=Wrk0>>6;
					};
				//============================================================================	
				//part after intial filtering
				for(CycleCounter=FiltFact; CycleCounter<MaxCycles; CycleCounter++){
					restart_wdt();
					//start ADC conv
					read_adc(ADC_START_ONLY);	//start conversion
					//estim next level
					SawLevel+=SawStep;
					//transfer new DAC value
					//fine part
//					output_low(DAC0);						//select fine DAC
//					spi_write((make8(SawLevel,1)&0x03)|0x10);	//send high part
//					spi_write(make8(SawLevel,0));				//send low part
//					output_high(DAC0);						//deselect
					//course part
					course=(make16(make8(SawLevel,2),make8(SawLevel,1)))>>2;
					output_low(DAC1);						//select course DAC
					spi_write((make8(course,1)&0x0f)|0x10);	//send high part
					spi_write(make8(course,0));				//send low part
					output_high(DAC1);						//deselect
					//wait for sync
					//not imlem yet!!!
					//read ADC
					ADC0=read_adc(ADC_READ_ONLY);
					//trigg LDAC
					output_low(LDAC);						//send DAC
					output_high(LDAC);						//strobe
					//meaning
					Wrk0-=data_filt0[iWin];	//calc new sum mean val
					Wrk0+=ADC0;
					data_filt0[iWin]=ADC0;
					//next filter value
					if(iWin==0){
						iWin=FWDT;
						iWin--;
						}else	iWin--;
					//calc mean value
					data_mean0=Wrk0>>6;
					Sum+=data_mean0;
					//extremum
					//estim max val
					if(data_mean0>maxRise0){
						iMaxRise0=StorAddr;
						maxRise0=data_mean0;
						};
					StorAddr++;
					//beta measuring
					if (data_mean0>BetTrsh)	bet_cur++;
					};	
					
//=======================================================================================================														
				//falling saw part
				output_low(STAT);
				MaxCycles=TAQ>>1;
				for(CycleCounter=0; CycleCounter<MaxCycles; CycleCounter++){
					restart_wdt();
					//start ADC conv
					read_adc(ADC_START_ONLY);	//start conversion
					//estim next level
					SawLevel-=SawStep;
					//transfer new DAC value
					//fine part
//					output_low(DAC0);						//select fine DAC
//					spi_write((make8(SawLevel,1)&0x03)|0x10);	//send high part
//					spi_write(make8(SawLevel,0));				//send low part
//					output_high(DAC0);						//deselect
					//course part
					course=(make16(make8(SawLevel,2),make8(SawLevel,1)))>>2;
					output_low(DAC1);						//select course DAC
					spi_write((make8(course,1)&0x0f)|0x10);	//send high part
					spi_write(make8(course,0));				//send low part
					output_high(DAC1);						//deselect
					//wait for sync
					//not imlem yet!!!
					//read ADC
					ADC0=read_adc(ADC_READ_ONLY);
					//trigg LDAC
					output_low(LDAC);						//send DAC
					output_high(LDAC);						//strobe
					//meaning
					Wrk0-=data_filt0[iWin];	//calc new sum mean val
					Wrk0+=ADC0;
					data_filt0[iWin]=ADC0;
					//next filter value
					if(iWin==0){
						iWin=FWDT;
						iWin--;
 						}else	iWin--;
					//calc mean value
					data_mean0=Wrk0>>6;
					Sum+=data_mean0;
					//extremum
					//estim max val
					if(data_mean0>maxFall0){
						iMaxFall0=StorAddr;
						maxFall0=data_mean0;
						};
					StorAddr++;
					//beta measuring
					if (data_mean0>BetTrsh)	bet_cur++;
					};
//==========================================
					//final part					
				for(CycleCounter=0; CycleCounter<FiltFact; CycleCounter++){
					restart_wdt();
					ADC0=0;
					//meaning
					Wrk0-=data_filt0[iWin];	//calc new sum mean val
					Wrk0+=ADC0;
					data_filt0[iWin]=ADC0;
					//next filter value
					if(iWin==0){
						iWin=FWDT;
						iWin--;
						}else	iWin--;
					//calc mean value
					data_mean0=Wrk0>>6;
					Sum+=data_mean0;
					//extremum
					//estim max val
					if(data_mean0>maxFall0){
						iMaxFall0=StorAddr;
						maxFall0=data_mean0;
						};
					StorAddr++;
					//beta measuring
					if (data_mean0>BetTrsh)	bet_cur++;
					};
				DaqState=DaqScale;
				break;
//========================================================================================
			case	DaqScale:
				//scaling & result storing
				//temperature
				Termometer();
				restart_wdt();
				//regulator
				if(T_GEN>(TG_CAL+Hst)){	//if temperature above rated & histeresis
					output_low(TERMO);				//off heater
				};
				if(T_GEN<(TG_CAL-Hst)){	//if temperature below rated & histeresis
					output_high(TERMO);				//on heater
				};
				
				//Mx estimation
				Mx=(int16)(Sum>>11);
				//amplitude - alpha
				alp_buf0[iGam]=(MaxFall0+MaxRise0)>>1;
				//treshold estimation
				BetTrsh=(int16)((Mx+alp_buf0[iGam])>>1);
				//pulse width
				bet_buf0[iGam]=bet_cur;
				//gamma
				iGamma0=iMaxFall0-iMaxRise0;
				//gamma meaning
				gam_buf0[iGam]=iGamma0; //meaning buffer
				//next index
				if(iGam==0){
					iGam=PCOUNT-1;
					}else iGam--;
				//median
				//copy arrays				
//				memcpy(alp_sort,alp_buf0,sizeof(alp_sort));
//				memcpy(bet_sort,bet_buf0,sizeof(bet_sort));
//				memcpy(gam_sort,gam_buf0,sizeof(gam_sort));
				
				AlpSum=0;
				BetSum=0;
				GamSum=0;

				for (cnt=0; cnt<PCOUNT;cnt++){
					AlpSum+=alp_buf0[cnt];
					BetSum+=bet_buf0[cnt];
					GamSum+=gam_buf0[cnt];
				};
				//sorting
				//do{
				//	changed=0;	//reset flag
				//	for(cnt=1; cnt<PCOUNT; cnt++){
						//alpha
				//		if(alp_sort[cnt-1]>alp_sort[cnt]){
				//			restart_wdt();
				//			changed=1;	//change exist
				//			Wrk0=alp_sort[cnt];
				//			alp_sort[cnt]=alp_sort[cnt-1];
				//			alp_sort[cnt-1]=Wrk0;
				//			};
						//beta
				//		if(bet_sort[cnt-1]>bet_sort[cnt]){
				//			restart_wdt();
				//			changed=1;	//change exist
				//			Wrk0=bet_sort[cnt];
				//			bet_sort[cnt]=bet_sort[cnt-1];
				//			bet_sort[cnt-1]=Wrk0;
				//			};
						//gamma
				//		if(gam_sort[cnt-1]>gam_sort[cnt]){
				//			restart_wdt();
				//			changed=1;	//change exist
				//			Wrk0=gam_sort[cnt];
				//			gam_sort[cnt]=gam_sort[cnt-1];
				//			gam_sort[cnt-1]=Wrk0;
				//			};
				//		};
				//	}while(changed);
				//median estimation
				//Wrk0=PCOUNT>>1;	//div 2
				//alpha
				//Amplitude0=((int16)(alp_sort[Wrk0]*2.4414))>>(AmpCoef0);
				Wrk0=(int16)(AlpSum>>7); //divide by 128
				Amplitude0=((int16)((Wrk0*2.4414)-0))>>(AmpCoef0);
				//beta
				//PulseWdt0=bet_sort[Wrk0]+bet_sort[Wrk0]>>2;
				Wrk0=(int16)(BetSum>>7); //divide by 128
				PulseWdt0=(Wrk0<<0)+(Wrk0>>2); //mult by 1.25
				//gamma
				//Gamma0=gam_sort[Wrk0];
				Gamma0=(int16)(GamSum>>7); //divide by 128
				Gamma0=(Gamma0<<1)+(Gamma0>>1);//multiplying by 2.5
				//repeat testing
				if(OneCycle){
					DaqState=DaqIdle;	//if one cycle  - next-idle
					}else DaqState=DaqStart;//else normal measuring
				output_high(STAT);
//				Calc();// addition calculations
				break;			
	};
	//		
	};
}