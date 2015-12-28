#include "spm3.h"
#include "ee_var.h"

//directives
//#USE FAST_IO(A)
//#USE FAST_IO(C)
//#USE FAST_IO(D)

//const
float const volts2dac = 4096.0 / 20.0; // use for value from volts to DAC conversion

//variables
//buffers
//channel 0
static int16 data_filt0[64];		//filtering buffer for ch0
static int16 data_mean0;			//mean value
static int16 maxRise0;			//max value at rising saw
static int16 iMaxRise0;			//index of maximum at rising saw
static int16 maxFall0;			//max value at falling saw
static int16 iMaxFall0;			//index of maximum at falling saw
static int16 alp_buf0[128];		//alpha meaning buffer
static int16 bet_buf0[128];		//beta meaning buffer
static int16 gam_buf0[128];		//gamma meaning buffer
static int16 Amplitude0;			//amplitude in B*1000
static int16 PulseWdt0;			//pulse width in uS/10
static int16 Gamma0;				//gamma in uS/10
static int16 AmpCoef0;			//input amplifier coeff
static float KST0;				//
static int16 T0_CAL;				//
static signed int16 T_SR0;				//temperature of cn0
static int16 Gamma1;				//gamma in uS/10
static int16 AmpCoef1;			//input amplifier coeff
static float KST1;				//
static int16 T1_CAL;				//
static signed int16 T_SR1;				//temperature of cn1

//common
static int16 TAQ;		//number of data samples at measuring period
static int16 FWDT;				//filering window width
static int16 PCOUNT;				//measuring periods for meaning
static int1 OneCycle;			//one cycle flag
static int8 iGam = 0;				//gamma meaning index

//saw
static signed int16 T_GEN;//generator temperature
static int16 SAW_A0;				//level of saw
static int16 sawStartLevel;			//current saw level
static int16 SAW_A1;				//saw amplitude
static int16 sawEndLevel;			//saw step
static float KGT;				//
static float KGU;				//
static int16 TG_CAL;				//

//communication
static int16 Speed;				//USART speed
static int16 Addr;				//own USART host adress

//filtering
static int8 iWin;				//meaning window index
static int32 AlpSum;
static int32 BetSum;
static int32 GamSum;

//temperature regulator
static int16 Hst = 50;				//Histeresis

//states
enum daq_states
{
	DaqIdle,		//idle state - state after daq stop
	DaqStart,		//cyclic mesuring start for one cycle
	DaqMeasure,		//measuring
	DaqScale,		//scaling measured values
	DaqEnd			//ending measuring cycle
};
int8 DaqState;	//Daq procedure state
//
//math variables
/*float kgv;
float Va;
float delta_gamma, delta_gamma2;
float T_0, T_G, T_G_k;
float Gam0, Gam0_k;
float k1, tsn;
float W;
float Haw0, Haw0_k;
float AA, BB, a0, a1, a2, a3, a4, a5;
float Am0, Am0_k;
float T_S0, T_S0_k;
*/

//procedures
//mathematics
/*char Calc() // Humidity calculation
 {
 float difftime0;
 //coeff validation
 if (kgv == 0)
 kgv = 1;
 if (Va == 0)
 Va = 1;
 //
 delta_gamma = (kgt / kgv) * (T_0 / Va) * (T_G - T_G_k);	//generator temperature correction
 delta_gamma2 = k1 * (T_S0 - tsn);	//object temperature correction
 difftime0 = (Gam0 - Gam0_k);	//addition parameter estimation
 difftime0 = difftime0 + delta_gamma + delta_gamma2;	//adding corecction
 //main regression
 W = AA
 + BB
 * (a0 + a1 * (Am0 - Am0_k) * 10
 + a2 * (Haw0 - Haw0_k) / 1000
 + a3 * difftime0 / 1000 + a4 * (T_G - T_G_k) / 100
 + a5 * (T_S0 - T_S0_k) / 100);

 return 0;
 }*/

//Peripherals setup
void InitMcu(void)
{		//initialization of MCU

	//oscillator configuration
	setup_oscillator(OSC_32MHZ | OSC_NORMAL | OSC_31250);
	//disable PSP
	setup_psp(PSP_DISABLED);
	//WDT config
	setup_wdt(WDT_OFF);
	//disable comparators
	setup_comparator(NC_NC_NC_NC);
	setup_vref(FALSE);

	//config ADC module
	setup_adc_ports(AN0_TO_AN5 | VSS_VREF);
	setup_adc(ADC_CLOCK_DIV_16 | ADC_TAD_MUL_2);

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

	setup_timer_2(T2_DISABLED, 0, 1);

	setup_timer_3(T3_INTERNAL | T3_DIV_BY_1);

	//setup communications
	//setup usart
	output_low(DIR);	//enable receiving

	//SPI configuration
	setup_spi(SPI_MASTER | SPI_CLK_DIV_4 | SPI_H_TO_L);

	//interrupt configuration
	enable_interrupts(INT_RDA);

	enable_interrupts(GLOBAL);

	//States reset
	DaqState = DaqIdle;	//initial state
}
//
//read registers from EE
void BURead(void)
{	//back up read
	int32 temp;

	//reading
	SAW_A0 = make16(read_eeprom(EE_SAW_A0 + 1), read_eeprom(EE_SAW_A0)); //rise saw level
	SAW_A1 = make16(read_eeprom(EE_SAW_A1 + 1), read_eeprom(EE_SAW_A1)); //rise saw amplitude
	AmpCoef0 = make16(read_eeprom(EE_KU0 + 1), read_eeprom(EE_KU0)); //
	AmpCoef1 = make16(read_eeprom(EE_KU1 + 1), read_eeprom(EE_KU1)); //
	TAQ = make16(read_eeprom(EE_TAQ + 1), read_eeprom(EE_TAQ)); //
	PCOUNT = make16(read_eeprom(EE_PCOUNT + 1), read_eeprom(EE_PCOUNT)); //
	FWDT = make16(read_eeprom(EE_FWDT + 1), read_eeprom(EE_FWDT)); //
	Speed = make16(read_eeprom(EE_Speed + 1), read_eeprom(EE_Speed)); //
	Addr = make16(read_eeprom(EE_Host + 1), read_eeprom(EE_Host)); //
	T0_CAL = make32(read_eeprom(EE_T0_CAL + 1), read_eeprom(EE_T0_CAL)); //
	T1_CAL = make16(read_eeprom(EE_T1_CAL + 1), read_eeprom(EE_T1_CAL)); //
	TG_CAL = make16(read_eeprom(EE_TG_CAL + 1), read_eeprom(EE_TG_CAL)); //
	temp = make32(read_eeprom(EE_KGT + 2), read_eeprom(EE_KGT + 3),
		read_eeprom(EE_KGT), read_eeprom(EE_KGT + 1)); //
	memcpy(&KGT, &temp, 4);
	temp = make32(read_eeprom(EE_KGU + 2), read_eeprom(EE_KGU + 3),
		read_eeprom(EE_KGU), read_eeprom(EE_KGU + 1)); //
	memcpy(&KGU, &temp, 4);
	temp = make32(read_eeprom(EE_KST0 + 2), read_eeprom(EE_KST0 + 3),
		read_eeprom(EE_KST0), read_eeprom(EE_KST0 + 1)); //
	memcpy(&KST0, &temp, 4);
	temp = make32(read_eeprom(EE_KST1 + 2), read_eeprom(EE_KST1 + 3),
		read_eeprom(EE_KST1), read_eeprom(EE_KST1 + 1)); //
	memcpy(&KST1, &temp, 4);
}
//
//write registers to EE
void BUWrite(void)
{	//back up write
	int32 temp;

	setup_wdt(WDT_OFF);

	//writing
	//int16
	write_eeprom(EE_SAW_A0, make8(SAW_A0, 0));
	write_eeprom(EE_SAW_A0 + 1, make8(SAW_A0, 1));
	write_eeprom(EE_SAW_A1, make8(SAW_A1, 0));
	write_eeprom(EE_SAW_A1 + 1, make8(SAW_A1, 1));
	write_eeprom(EE_KU0, make8(AmpCoef0, 0));
	write_eeprom(EE_KU0 + 1, make8(AmpCoef0, 1));
	write_eeprom(EE_KU1, make8(AmpCoef1, 0));
	write_eeprom(EE_KU1 + 1, make8(AmpCoef1, 1));
	write_eeprom(EE_TAQ, make8(TAQ, 0));
	write_eeprom(EE_TAQ + 1, make8(TAQ, 1));
	write_eeprom(EE_PCOUNT, make8(PCOUNT, 0));
	write_eeprom(EE_PCOUNT + 1, make8(PCOUNT, 1));
	write_eeprom(EE_FWDT, make8(FWDT, 0));
	write_eeprom(EE_FWDT + 1, make8(FWDT, 1));
	write_eeprom(EE_Speed, make8(Speed, 0));
	write_eeprom(EE_Speed + 1, make8(Speed, 1));
	write_eeprom(EE_Host, make8(Addr, 0));
	write_eeprom(EE_Host + 1, make8(Addr, 1));
	write_eeprom(EE_T0_CAL, make8(T0_CAL, 0));
	write_eeprom(EE_T0_CAL + 1, make8(T0_CAL, 1));
	write_eeprom(EE_T1_CAL, make8(T1_CAL, 0));
	write_eeprom(EE_T1_CAL + 1, make8(T1_CAL, 1));
	write_eeprom(EE_TG_CAL, make8(TG_CAL, 0));
	write_eeprom(EE_TG_CAL + 1, make8(TG_CAL, 1));

	//float
	memcpy(&temp, &KGT, 4);
	write_eeprom(EE_KGT + 2, make8(temp, 3));
	write_eeprom(EE_KGT + 3, make8(temp, 2));
	write_eeprom(EE_KGT, make8(temp, 1));
	write_eeprom(EE_KGT + 1, make8(temp, 0));

	memcpy(&temp, &KGU, 4);
	write_eeprom(EE_KGU + 2, make8(temp, 3));
	write_eeprom(EE_KGU + 3, make8(temp, 2));
	write_eeprom(EE_KGU, make8(temp, 1));
	write_eeprom(EE_KGU + 1, make8(temp, 0));

	memcpy(&temp, &KGT, 4);
	write_eeprom(EE_KST0 + 2, make8(temp, 3));
	write_eeprom(EE_KST0 + 3, make8(temp, 2));
	write_eeprom(EE_KST0, make8(temp, 1));
	write_eeprom(EE_KST0 + 1, make8(temp, 0));

	memcpy(&temp, &KGT, 4);
	write_eeprom(EE_KST1 + 2, make8(temp, 3));
	write_eeprom(EE_KST1 + 3, make8(temp, 2));
	write_eeprom(EE_KST1, make8(temp, 1));
	write_eeprom(EE_KST1 + 1, make8(temp, 0));

	setup_wdt(WDT_ON);
}

//temperature measuring
void Termometer(void)
{
	signed int16
		temp;

	//generator
	set_adc_channel(2);	//select generator channel
	delay_us(10);		//small wait
	temp = read_adc();	//read value
	T_GEN = (signed int16)((temp - 205)*24.414);

	//t0
	set_adc_channel(5);	//select generator channel
	delay_us(10);		//small wait
	temp = read_adc();	//read value
	T_SR0 = (signed int16)((temp - 205)*24.414);

	//t1
	set_adc_channel(4);	//select generator channel
	delay_us(10);		//small wait
	temp = read_adc();	//read value
	T_SR1 = (signed int16)((temp - 205)*24.414);
}

//DAC setting
void SetSawDac(int16 level) {

	output_low(DAC1);			//select course DAC
	spi_write((make8(level, 1) & 0x0f) | 0x10);	//send high part
	spi_write(make8(level, 0));		//send low part
	output_high(DAC1);			//deselect
	output_low(LDAC);			//send DAC
	output_high(LDAC);			//strobe
}

//KU setting
void SetKU0(int16 coeff)
{
	int16 dacval;
	setup_wdt(WDT_OFF);
	if ((coeff < 1) & (coeff > 6))
		coeff = 1;
	dacval = (0x1fff >> coeff) | 0x7000;
	output_low(KU0);				//select KU0 DAC
	spi_write(make8(dacval, 1));		//send high part
	spi_write(make8(dacval, 0));		//send low part
	output_high(KU0);				//deselect
	setup_wdt(WDT_ON);
}

void SetKU1(int16 coeff)
{
	int16 dacval;
	setup_wdt(WDT_OFF);
	if ((coeff < 1) & (coeff > 6))
		coeff = 1;
	dacval = (0x1fff >> coeff) | 0x7000;
	output_low(KU1);				//select KU0 DAC
	spi_write(make8(dacval, 1));		//send high part
	spi_write(make8(dacval, 0));		//send low part
	output_high(KU1);				//deselect
	setup_wdt(WDT_ON);
}

//ISR
#include	"modbus_isr.h"

void main(void)
{
	//addition variables
	int16 maxCycles = 0;
	int16 cnt;
	int16 ADC0;	//current ADC value
	int32 Wrk0 = 0;	//sum mean value
	int16 StorAddr;	//storage address
	int8 OutBufAddr;	//out buffer address
	int32 Sum;
	int16 BetTrsh = 0x01ff; //beta treshold
	int16 bet_cur; //current beta value
	int16 filterWidth = 1;
	int16 sawLevel;

	InitMcu();		//mcu init
	BURead();		//read registers from EE
	RsInit();

	//interrupt configuration
	enable_interrupts(INT_RDA);
	disable_interrupts(INT_TIMER1);
	enable_interrupts(GLOBAL);

	output_high(TERMO);

	DaqState = DaqStart;
	//main cycle
	while (TRUE)
	{
		restart_wdt();
		//state procesor
		switch (DaqState)
		{
		case DaqIdle:
			Termometer();
			//wait for state changing
			break;

		case DaqStart:
			//prepare for measuring
			bet_cur = 0;		//reset beta value

			//set input amplifiers
			SetKU0(AmpCoef0);
			SetKU1(AmpCoef1);

			//filtering window setup
			if (FWDT > 0 && FWDT < 7)
			{
				filterWidth = (int16)(0x0001 << FWDT); //power 2 for FWDT factor
			}
			else
				filterWidth = 1;

			//zero buffers & variables
			for (cnt = 0; cnt < filterWidth; cnt++)
			{
				data_filt0[cnt] = 0;	//clear buffer
			}

			StorAddr = 0;
			OutBufAddr = 0;
			iWin = 0;
			iMaxFall0 = 0;
			iMaxRise0 = 0;
			maxFall0 = 0;
			maxRise0 = 0;
			data_mean0 = 0;

			//DAC parameters initialization
			sawStartLevel = (int16)(((float)SAW_A0 / 1000) * volts2dac); //convert to ADC discretes

			sawEndLevel = sawStartLevel
				+ (int16)(((float)SAW_A1 / 1000) * volts2dac);

			if (sawEndLevel >= 4096){
				sawEndLevel = 4095;
			}
				

			//next state
			DaqState = DaqMeasure;	//set measuring state
			break;

		case DaqMeasure:
			//indicate
			output_toggle(STAT);

			//set initial DAC level
			SetSawDac(sawStartLevel);

			//read data from ADC
			//select ADC channel
			set_adc_channel(0);	//select ch0

			Sum = 0;
			Wrk0 = 0;

			// Rising saw part
			for (sawLevel = sawStartLevel; sawLevel <= sawEndLevel; sawLevel++)
			{

				restart_wdt();

				ADC0 = read_adc(); //read measured value

				SetSawDac(sawLevel);	//set next saw level

				//filtering
				Wrk0 -= data_filt0[iWin];	//calc new sum mean val
				
				Wrk0 += ADC0;

				data_filt0[iWin] = ADC0;

				//next filter value
				if (iWin == 0)
				{
					iWin = filterWidth;
					iWin--;
				}
				else
					iWin--;
					
				//cycle meaning
				/*Wrk0=0;
				int8 i;
				for (i = 0; i<filterWidth; i++)
				{
					Wrk0 +=data_filt0[i];
				}*/
					

				//calc mean value
				data_mean0 = (int16)(Wrk0 >> FWDT); //divide by filterWidth = 2^FWDT
				Sum += data_mean0;

				//extremum
				//estim max val
				if (data_mean0 > maxRise0)
				{
					iMaxRise0 = StorAddr;
					maxRise0 = data_mean0;
				};

				StorAddr++;

				//beta measuring
				if (data_mean0 > BetTrsh)
					bet_cur++;

				maxCycles++; //add cycle count

			}

			DaqState = DaqScale;
			break;

		case DaqScale:
			//scaling & result storing

			//temperature
			Termometer();
			restart_wdt();

			//regulator
			if (T_GEN > (TG_CAL + Hst))
			{ //if temperature above rated & histeresis
				output_low(TERMO);				//off heater
			}

			if (T_GEN < (TG_CAL - Hst))
			{			//if temperature below rated & histeresis
				output_high(TERMO);				//on heater
			}

			//amplitude - alpha
			alp_buf0[iGam] = maxRise0;

			//treshold estimation
			BetTrsh = (int16)(alp_buf0[iGam] >> 1);

			//pulse width
			bet_buf0[iGam] = bet_cur;

			//gamma meaning
			gam_buf0[iGam] = iMaxRise0; //meaning buffer

			//next index
			if (iGam == 0)
			{
				iGam = PCOUNT - 1;
			}
			else
				iGam--;

			AlpSum = 0;
			BetSum = 0;
			GamSum = 0;

			for (cnt = 0; cnt < PCOUNT; cnt++)
			{
				restart_wdt();
				AlpSum += alp_buf0[cnt];
				BetSum += bet_buf0[cnt];
				GamSum += gam_buf0[cnt];
			}

			//alpha
			Amplitude0 = ((int16)(((float)(AlpSum >> 7) * 2.4414) - 0)) >> (AmpCoef0); //divide sum by 128 and conver to volts with amplify coeff

			//beta
			PulseWdt0 = (int16)((float)(BetSum >> 7) * 10000.0 / maxCycles); //divide sum by 128 and norming

			//gamma
			Gamma0 = (int16)((float)(GamSum >> 7) * 10000.0 / maxCycles); //divide sum by 128 and norming

			//repeat testing
			if (OneCycle)
			{
				DaqState = DaqIdle;	//if one cycle  - next-idle
			}
			else
				DaqState = DaqStart;	//else normal measuring
	
			break;
		}
	}
}
