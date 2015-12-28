//SPM-3 EEPROM storage structure
//define adresses of EEPROM locations
/*		LocName		Addr	Discr			*/
/*==========================================*/
#define	EE_SAW_A0	0x00	//define saw level
#define	EE_SAW_A1	0x02	//define saw amplitude
#define	EE_KU0		0x04	//amp coeff for in0
#define	EE_KU1		0x06	//amp coeff for in1
#define	EE_TAQ		0x08	//number of DAQ points
#define	EE_PCOUNT	0x0A	//num of measures for meaning
#define	EE_FWDT		0x0C
#define	EE_Speed	0x0E	//serial device speed
#define	EE_Host		0x10	//host address & serial number	<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<!!! Device ID!!!
#define	EE_T0_CAL	0x12	//cal temperature of cn0 
#define	EE_T1_CAL	0x14	//cal temperature of cn1 
#define	EE_TG_CAL	0x16	//cal temperature of generator 
#define	EE_KGT		0x18	//termo coeff of generator
#define	EE_KGU		0x1C	//voltage coeff of generator
#define	EE_KST0		0x20	//termo coeff of cn0
#define	EE_KST1		0x24	//termo coeff of cn1

//EEPROM data definition
#ROM	0xf00000={0000,	//EE_SAW_A0
				  20000,	//EE_SAW_A1
				  0x0001,	//EE_KU0				  
				  0x0001,	//EE_KU1
				  0x0800,	//EE_TAQ
				  0x0080,	//EE_PCOUNT
				  0x0003,	//EE_FWDT
				  0x4B00,	//EE_Speed
				  0x0001,	//EE_Host				  
				  0x09C4,	//EE_TO_CAL
				  0x09C4,	//EE_T1_CAL
				  0x0BB8,	//EE_TG_CAL
				  0x0000,	//EE_KGT
				  0x0000,
				  0x7F00,	//EE_KGU
				  0x0000,
				  0x0000,	//EE_KST0
				  0x0000,
				  0x0000,	//EE_KST1
				  0x0000
}
//