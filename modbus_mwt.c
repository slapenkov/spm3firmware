/////////////////////////////////////////////////////////////////////////
////                            modbus.c                             ////
////                                                                 ////
////        MODBUS protocol driver for serial communications.        ////
////                                                                 ////
////        Refer to documentation at http://www.modbus.org for      ////
////        more information on MODBUS.                              ////
////                                                                 ////
/////////////////////////////////////////////////////////////////////////

//#define SERIAL 0
//#define TCP 1
#define MASTER 0
#define SLAVE 1

#ifndef MODBUS_TYPE
#define MODBUS_TYPE MASTER
#endif

   #ifndef MODBUS_SERIAL_RX_PIN
   #define MODBUS_SERIAL_RX_PIN       PIN_C6   // Data receive pin
   #endif

   #ifndef MODBUS_SERIAL_TX_PIN
   #define MODBUS_SERIAL_TX_PIN       PIN_C7   // Data transmit pin
   #endif

   #ifndef MODBUS_SERIAL_ENABLE_PIN
   #define MODBUS_SERIAL_ENABLE_PIN   PIN_B4   // Controls DE pin.  RX low, TX high.
   #endif

   #ifndef MODBUS_SERIAL_RX_ENABLE
   #define MODBUS_SERIAL_RX_ENABLE    PIN_B5   // Controls RE pin.  Should keep low.
   #endif

   #use rs232(baud=MODBUS_SERIAL_BAUD, xmit=MODBUS_SERIAL_TX_PIN, rcv=MODBUS_SERIAL_RX_PIN, parity=N, force_sw, stream=MODBUS_SERIAL)
   
   #define RCV_OFF() {setup_uart(FALSE);}

#ifndef MODBUS_SERIAL_RX_BUFFER_SIZE
#define MODBUS_SERIAL_RX_BUFFER_SIZE  48      //size of send/rcv buffer
#endif

int1 modbus_serial_new=0;
const int8 modbus_serial_wait=50;

/********************************************************************
These exceptions are defined in the MODBUS protocol.  These can be
used by the slave to communicate problems with the transmission back
to the master who can use these to better check exceptions.
********************************************************************/
typedef enum _exception{ILLEGAL_FUNCTION=1, ILLEGAL_DATA_ADDRESS, ILLEGAL_DATA_VALUE,
      SLAVE_DEVICE_FAILURE, ACKNOWLEDGE, SLAVE_DEVICE_BUSY, MEMORY_PARITY_ERROR,
      GATEWAY_PATH_UNAVAILABLE, GATEWAY_TARGET_NO_RESPONSE} exception;

/*
Stages of MODBUS reception.  Used to keep our ISR fast enough.
*/
enum {MODBUS_GETADDY=0, MODBUS_GETFUNC, MODBUS_GETDATA} modbus_serial_state = 0;

/*
Global value holding our current CRC value.
*/
union
{
   int8 b[2];
   int16 d;
} modbus_serial_crc;

/********************************************************************
Our receive struct.  This is used when receiving data as a master or
slave.  Once a message is sent to you with your address, you should
begin processing that message.  Refer to ex_modbus_master.c and
ex_modbus_slave.c to see how to properly use this structure.
********************************************************************/
struct
{
   int8 address;
   int8 len;   //number of bytes in the message received
   int8 func;  //the function of the message received
   exception error;  //error recieved, if any
   int8 data[MODBUS_SERIAL_RX_BUFFER_SIZE]; //data of the message received
} modbus_rx;

/* Table of CRC values for high–order byte */
const char modbus_auchCRCHi[] = {
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
const char modbus_auchCRCLo[] = {
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

// Purpose:    Enable data reception
// Inputs:     None
// Outputs:    None
void RCV_ON(void)
{
      while(kbhit(MODBUS_SERIAL)) {getc();} // Clear RX buffer. Clear RDA interrupt flag. Clear overrun error flag.
      #if getenv("AUART")
//==         setup_uart(UART_ADDRESS);
//==         setup_uart(TRUE);
      #else
         setup_uart(TRUE);
      #endif
}

// Purpose:    Initialize RS485 communication. Call this before
//             using any other RS485 functions.
// Inputs:     None
// Outputs:    None
void modbus_init()
{
   int8 i;

   output_low(MODBUS_SERIAL_ENABLE_PIN);

   RCV_ON();

   setup_timer_2(T2_DIV_BY_16,249,5);   //~4ms interrupts

   enable_interrupts(GLOBAL);
}

// Purpose:    Start our timeout timer
// Inputs:     Enable, used to turn timer on/off
// Outputs:    None
void MODBUS_ENABLE_TIMEOUT(int1 enable)
{
   disable_interrupts(INT_TIMER2);
   if (enable) {
      set_timer2(0);
      clear_interrupt(INT_TIMER2);
      enable_interrupts(INT_TIMER2);
   }
}

// Purpose:    Check if we have timed out waiting for a response
// Inputs:     None
// Outputs:    None
#int_timer2
void modbus_timeout_now(void)
{
   if((modbus_serial_state == MODBUS_GETDATA) && (modbus_serial_crc.d == 0x0000) && (!modbus_serial_new))
   {
      modbus_rx.len-=2;
      modbus_serial_new=TRUE;
   }
   else
   {
      modbus_serial_new=FALSE;
      modbus_serial_crc.d=0xFFFF;
      modbus_serial_state=MODBUS_GETADDY;
      MODBUS_ENABLE_TIMEOUT(FALSE);
   }
}

// Purpose:    Calculate crc of data and updates global crc
// Inputs:     Character
// Outputs:    None
void modbus_calc_crc(char data)
{
  int8 uIndex ; // will index into CRC lookup table

  uIndex = modbus_serial_crc.b[1] ^ data; // calculate the CRC
  modbus_serial_crc.b[1] = modbus_serial_crc.b[0] ^ modbus_auchCRCHi[uIndex];
  modbus_serial_crc.b[0] = modbus_auchCRCLo[uIndex];
}

// Purpose:    Puts a character onto the serial line
// Inputs:     Character
// Outputs:    None
void modbus_serial_putc(int8 c)
{
   fputc(c, MODBUS_SERIAL);
   modbus_calc_crc(c);
   delay_us(1000000/MODBUS_SERIAL_BAUD); //one stop bit.  not exact
}

// Purpose:   Interrupt service routine for handling incoming RS485 data
// Inputs:    None
// Outputs:   None
#int_rda
void incomming_modbus_serial() {
   char c;

   c=fgetc(MODBUS_SERIAL);

   if (!modbus_serial_new)
   {
      if(modbus_serial_state == MODBUS_GETADDY)
      {
         modbus_serial_crc.d = 0xFFFF;
         modbus_rx.address = c;
         modbus_serial_state++;
         modbus_rx.len = 0;
         modbus_rx.error=0;
      }
      else if(modbus_serial_state == MODBUS_GETFUNC)
      {
         modbus_rx.func = c;
         modbus_serial_state++;
      }
      else if(modbus_serial_state == MODBUS_GETDATA)
      {
         if (modbus_rx.len>=MODBUS_SERIAL_RX_BUFFER_SIZE) {modbus_rx.len=MODBUS_SERIAL_RX_BUFFER_SIZE-1;}
         modbus_rx.data[modbus_rx.len]=c;
         modbus_rx.len++;
      }

      modbus_calc_crc(c);
      MODBUS_ENABLE_TIMEOUT(TRUE);
   }
}

// Purpose:    Send a message over the RS485 bus
// Inputs:     1) The destination address
//             2) The number of bytes of data to send
//             3) A pointer to the data to send
//             4) The length of the data
// Outputs:    TRUE if successful
//             FALSE if failed
// Note:       Format:  source | destination | data-length | data | checksum
void modbus_serial_send(int8 to, int8 func, int8* data, int8 len)
{
   int8 i;
   int8 crc_low, crc_high;

   modbus_serial_crc.d=0xFFFF;
   modbus_serial_new=FALSE;

   //==RCV_OFF();
   output_high(MODBUS_SERIAL_ENABLE_PIN);
   //delay_us(3000);
   delay_us(3500000/MODBUS_SERIAL_BAUD); //one stop bit.  not exact

   modbus_serial_putc(to);
   modbus_serial_putc(func);

   for (i=0;i<len;++i)
   {
      modbus_serial_putc(*data);
      data++;
   }

   crc_high=modbus_serial_crc.b[1];
   crc_low=modbus_serial_crc.b[0];

   modbus_serial_putc(crc_high);
   modbus_serial_putc(crc_low);

//   delay_us(3000);
   delay_us(3500000/MODBUS_SERIAL_BAUD); //one stop bit.  not exact

   RCV_ON();
   output_low(MODBUS_SERIAL_ENABLE_PIN);
}

// Purpose:    Wait for wait time for the RS485 bus to become idle
// Inputs:     TRUE - restart the watch dog timer to prevent reset
//             FALSE - watch dog timer not restarted
// Outputs:    None
void modbus_serial_wait_for_bus(int1 clrwdt)
{
   int16 i;

   //==RCV_OFF();
   for(i=0; i <= (modbus_serial_wait*20); ++i)
   {
      if(!input(MODBUS_SERIAL_RX_PIN))
         i = 0;
      else
         delay_us(50);

      if(clrwdt)
         restart_wdt();
   }
}

// Purpose:    Get a message from the RS485 bus and store it in a buffer
// Inputs:     1) TRUE  - wait for a message
//                FALSE - only check if a message is available
// Outputs:    TRUE if a message was received
//             FALSE if wait is FALSE and no message is available
// Note:       Data will be filled in at the modbus_rx struct:
int1 modbus_serial_receive(int1 wait)
{
   int8 i;

   while(wait && (!modbus_serial_new));

   if(!modbus_serial_new)
      return FALSE;
   else if(modbus_rx.func & 0x80)
   {
      modbus_rx.error = modbus_rx.data[0];
      modbus_rx.len = 1;
   }
   return TRUE;
}



#if (MODBUS_TYPE==MASTER)
/*MODBUS Master Functions*/

#ifdef MODBUS_SERIAL_RX_BUFFER_SIZE
#define MODBUS_RX_BUFFER_SIZE MODBUS_SERIAL_RX_BUFFER_SIZE
#else
#define MODBUS_RX_BUFFER_SIZE MODBUS_TCP_RX_BUFFER_SIZE
#endif

/********************************************************************
A data buffer.  Used internally to send and receive data.  Unless
you have a reason to, you should not access this variable.
********************************************************************/
int8 data_buf[MODBUS_RX_BUFFER_SIZE];

/********************************************************************
The following structs are used for read/write_sub_request.  These
functions take in one of these structs.
Please refer to the MODBUS protocol specification if you do not
understand the members of the structure.
********************************************************************/
typedef struct _read_sub_request
{
   int8 reference_type;
   int16 file_number;
   int16 record_number;
   int16 record_length;
} read_sub_request;

typedef struct _write_sub_request
{
   int8 reference_type;
   int16 file_number;
   int16 record_number;
   int16 record_length;
   int16 data[MODBUS_RX_BUFFER_SIZE-8];
} write_sub_request;


/********************************************************************
The following functions are defined in the MODBUS protocol.  Please
refer to http://www.modbus.org for the purpose of each of these.
All functions take the slaves address as their first parameter.
Each function returns the exception code received from the response.
The function will return 0 if there were no errors in transmission.
********************************************************************/

/*
read_coils
Input:     int8       address            Slave Address
           int16      start_address      Address to start reading from
           int16      quantity           Amount of addresses to read
Output:    exception                     0 if no error, else the exception
*/
exception read_coils(int8 address, int16 start_address, int16 quantity)
{
   data_buf[0] = make8(start_address,1);
   data_buf[1] = make8(start_address,0);

   data_buf[2] = make8(quantity,1);
   data_buf[3] = make8(quantity,0);

   modbus_serial_send(address, 0x01, &(data_buf[0]), 4);

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
read_discrete_input
Input:     int8       address            Slave Address
           int16      start_address      Address to start reading from
           int16      quantity           Amount of addresses to read
Output:    exception                     0 if no error, else the exception
*/
exception read_discrete_input(int8 address, int16 start_address, int16 quantity)
{
   data_buf[0] = make8(start_address,1);
   data_buf[1] = make8(start_address,0);

   data_buf[2] = make8(quantity,1);
   data_buf[3] = make8(quantity,0);

   modbus_serial_send(address, 0x02, &(data_buf[0]), 4);

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
read_holding_registers
Input:     int8       address            Slave Address
           int16      start_address      Address to start reading from
           int16      quantity           Amount of addresses to read
Output:    exception                     0 if no error, else the exception
*/
exception read_holding_registers(int8 address, int16 start_address, int16 quantity)
{
   data_buf[0] = make8(start_address,1);
   data_buf[1] = make8(start_address,0);

   data_buf[2] = make8(quantity,1);
   data_buf[3] = make8(quantity,0);

   modbus_serial_send(address, 0x03, &(data_buf[0]), 4);

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
read_input_registers
Input:     int8       address            Slave Address
           int16      start_address      Address to start reading from
           int16      quantity           Amount of addresses to read
Output:    exception                     0 if no error, else the exception
*/
exception read_input_registers(int8 address, int16 start_address, int16 quantity)
{
   data_buf[0] = make8(start_address,1);
   data_buf[1] = make8(start_address,0);

   data_buf[2] = make8(quantity,1);
   data_buf[3] = make8(quantity,0);

   modbus_serial_send(address, 0x04, &(data_buf[0]), 4);

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
write_single_coil
Input:     int8       address            Slave Address
           int16      output_address     Address to write into
           int16      output_value       Value to write
Output:    exception                     0 if no error, else the exception
*/
exception write_single_coil(int8 address, int16 output_address, int16 output_value)
{
   data_buf[0] = make8(output_address,1);
   data_buf[1] = make8(output_address,0);

   data_buf[2] = make8(output_value,1);
   data_buf[3] = make8(output_value,0);

   modbus_serial_send(address, 0x05, &(data_buf[0]), 4);

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
write_single_register
Input:     int8       address            Slave Address
           int16      reg_address        Address to write into
           int16      reg_value          Value to write
Output:    exception                     0 if no error, else the exception
*/
exception write_single_register(int8 address, int16 reg_address, int16 reg_value)
{
   data_buf[0] = make8(reg_address,1);
   data_buf[1] = make8(reg_address,0);

   data_buf[2] = make8(reg_value,1);
   data_buf[3] = make8(reg_value,0);

   modbus_serial_send(address, 0x06, &(data_buf[0]), 4);

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
read_exception_status
Input:     int8       address            Slave Address
Output:    exception                     0 if no error, else the exception
*/
exception read_exception_status(int8 address)
{
   modbus_serial_send(address, 0x07, 0, 0);

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
diagnostics
Input:     int8       address            Slave Address
           int16      sub_func           Subfunction to send
           int16      data               Data to send, changes based on subfunction
Output:    exception                     0 if no error, else the exception
*/
exception diagnostics(int8 address, int16 sub_func, int16 data)
{
   data_buf[0] = make8(sub_func,1);
   data_buf[1] = make8(sub_func,0);

   data_buf[2] = make8(data,1);
   data_buf[3] = make8(data,0);

   modbus_serial_send(address, 0x08, &(data_buf[0]), 4);

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
get_comm_event_couter
Input:     int8       address            Slave Address
Output:    exception                     0 if no error, else the exception
*/
exception get_comm_event_counter(int8 address)
{
   modbus_serial_send(address, 0x0B, 0, 0);

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
get_comm_event_log
Input:     int8       address            Slave Address
Output:    exception                     0 if no error, else the exception
*/
exception get_comm_event_log(int8 address)
{
   modbus_serial_send(address, 0x0C, &(data_buf[0]), 4);

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
write_multiple_coils
Input:     int8       address            Slave Address
           int16      start_address      Address to start at
           int16      quantity           Amount of coils to write to
           int8*      values             A pointer to an array holding the data to write
Output:    exception                     0 if no error, else the exception
*/
exception write_multiple_coils(int8 address, int16 start_address, int16 quantity,
                           int8 *values)
{
   int8 i;
   int8 count;

   if(quantity != 0)
      count = (int8)((quantity/8)+1);

   data_buf[0] = make8(start_address,1);
   data_buf[1] = make8(start_address,0);

   data_buf[2] = make8(quantity,1);
   data_buf[3] = make8(quantity,0);

   data_buf[4] = count;

   for(i=0; i < count; ++i)
   {
      data_buf[i+5] = values[i];
   }

   modbus_serial_send(address, 0x0F, &(data_buf[0]), (5+count));

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
write_multiple_registers
Input:     int8       address            Slave Address
           int16      start_address      Address to start at
           int16      quantity           Amount of coils to write to
           int16*     values             A pointer to an array holding the data to write
Output:    exception                     0 if no error, else the exception
*/
exception write_multiple_registers(int8 address, int16 start_address, int16 quantity,
                           int16 *values)
{
   int8 i;

   data_buf[0] = make8(start_address,1);
   data_buf[1] = make8(start_address,0);

   data_buf[2] = make8(quantity,1);
   data_buf[3] = make8(quantity,0);

   data_buf[4] = (int8)(2*quantity);

   for(i=0; i < quantity; ++i)
   {
      data_buf[i+5] = make8(values[i],1);
      data_buf[i+6] = make8(values[++i],0);
   }

   modbus_serial_send(address, 0x10, &(data_buf[0]), 5+(2*quantity));

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
report_slave_id
Input:     int8       address            Slave Address
Output:    exception                     0 if no error, else the exception
*/
exception report_slave_id(int8 address)
{
   modbus_serial_send(address, 0x11, 0, 0);

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
read_file_record
Input:     int8                address            Slave Address
           int8                byte_count         Number of bytes to read
           read_sub_request*   request            Structure holding record information
Output:    exception                              0 if no error, else the exception
*/
exception read_file_record(int8 address, int8 byte_count, read_sub_request *request)
{
   int8 i;

   data_buf[0] = byte_count;

   for(i=0; i < (byte_count/7); i+=7)
   {
      data_buf[i+1] = request->reference_type;
      data_buf[i+2] = make8(request->file_number, 1);
      data_buf[i+3] = make8(request->file_number, 0);
      data_buf[i+4] = make8(request->record_number, 1);
      data_buf[i+5] = make8(request->record_number, 0);
      data_buf[i+6] = make8(request->record_length, 1);
      data_buf[i+7] = make8(request->record_length, 0);
   }

   modbus_serial_send(address, 0x14, &(data_buf[0]), byte_count+1);

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
write_file_record
Input:     int8                address            Slave Address
           int8                byte_count         Number of bytes to read
           read_sub_request*   request            Structure holding record/data information
Output:    exception                              0 if no error, else the exception
*/
exception write_file_record(int8 address, int8 byte_count, write_sub_request *request)
{
   int8 i, j=0;

   data_buf[0] = byte_count;

   for(i=0; i < byte_count; i+=(7+(j*2)))
   {
      data_buf[i+1] = request->reference_type;
      data_buf[i+2] = make8(request->file_number, 1);
      data_buf[i+3] = make8(request->file_number, 0);
      data_buf[i+4] = make8(request->record_number, 1);
      data_buf[i+5] = make8(request->record_number, 0);
      data_buf[i+6] = make8(request->record_length, 1);
      data_buf[i+7] = make8(request->record_length, 0);

      for(j=0; (j < request->record_length) && (j < MODBUS_RX_BUFFER_SIZE-8); j+=2)
      {
         data_buf[j+(i+8)] = make8(request->data[j], 1);
         data_buf[j+(i+9)] = make8(request->data[j], 0);
      }
   }

   modbus_serial_send(address, 0x15, &(data_buf[0]), byte_count+1);

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
mask_write_register
Input:     int8       address            Slave Address
           int16      reference_address  Address to mask
           int16      AND_mask           A mask to AND with the data at reference_address
           int16      OR_mask            A mask to OR with the data at reference_address
Output:    exception                              0 if no error, else the exception
*/
exception mask_write_register(int8 address, int16 reference_address,
                           int16 AND_mask, int16 OR_mask)
{
   data_buf[0] = make8(reference_address,1);
   data_buf[1] = make8(reference_address,0);

   data_buf[2] = make8(AND_mask,1);
   data_buf[3] = make8(AND_mask,0);

   data_buf[4] = make8(OR_mask, 1);
   data_buf[5] = make8(OR_mask, 0);

   modbus_serial_send(address, 0x16, &(data_buf[0]), 6);

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
read_write_multiple_registers
Input:     int8       address                Slave Address
           int16      read_start             Address to start reading
           int16      read_quantity          Amount of registers to read
           int16      write_start            Address to start writing
           int16      write_quantity         Amount of registers to write
           int16*     write_registers_value  Pointer to an aray us to write
Output:    exception                         0 if no error, else the exception
*/
exception read_write_multiple_registers(int8 address, int16 read_start,
                                    int16 read_quantity, int16 write_start,
                                    int16 write_quantity,
                                    int16 *write_registers_value)
{
   int8 i;

   data_buf[0] = make8(read_start,1);
   data_buf[1] = make8(read_start,0);

   data_buf[2] = make8(read_quantity,1);
   data_buf[3] = make8(read_quantity,0);

   data_buf[4] = make8(write_start, 1);
   data_buf[5] = make8(write_start, 0);

   data_buf[6] = make8(write_quantity, 1);
   data_buf[7] = make8(write_quantity, 0);

   data_buf[8] = (int8)(2*write_quantity);

   for(i=0; i < write_quantity ; i+=2)
   {
      data_buf[i+1] = make8(write_registers_value[i], 1);
      data_buf[i+2] = make8(write_registers_value[i+1], 0);
   }

   modbus_serial_send(address, 0x17, &(data_buf[0]), 9+(2*write_quantity));

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

/*
read_FIFO_queue
Input:     int8       address           Slave Address
           int16      FIFO_address      FIFO address
Output:    exception                    0 if no error, else the exception
*/
exception read_FIFO_queue(int8 address, int16 FIFO_address)
{
   data_buf[0] = make8(FIFO_address, 1);
   data_buf[1] = make8(FIFO_address, 0);

   modbus_serial_send(address, 0x18, &(data_buf[0]), 2);

   if(address)
      while(!modbus_serial_receive(FALSE));

   return modbus_rx.error;
}

#else
/*MODBUS Slave Functions*/

#ifdef MODBUS_SERIAL_RX_BUFFER_SIZE
#define MODBUS_RX_BUFFER_SIZE MODBUS_SERIAL_RX_BUFFER_SIZE
#else
#define MODBUS_RX_BUFFER_SIZE MODBUS_TCP_RX_BUFFER_SIZE
#endif

/********************************************************************
A data buffer.  Used internally to send and receive data.  Unless
you have a reason to, you should not access this variable.
********************************************************************/
int8 data_buf[MODBUS_RX_BUFFER_SIZE];

/********************************************************************
The following structs are used for read/write_sub_request_rsp.  These
functions take in one of these structs.  Please refer to the MODBUS 
protocol specification if you do not understand the members of the 
structure.
********************************************************************/
typedef struct _read_sub_request_rsp
{
   int8 record_length;
   int8 reference_type;
   int16 data[MODBUS_SERIAL_RX_BUFFER_SIZE-3];
} read_sub_request_rsp;

typedef struct _write_sub_request_rsp
{
   int8 reference_type;
   int16 file_number;
   int16 record_number;
   int16 record_length;
   int16 data[MODBUS_SERIAL_RX_BUFFER_SIZE-8];
} write_sub_request_rsp;


/********************************************************************
The following slave functions are defined in the MODBUS protocol.  
Please refer to http://www.modbus.org for the purpose of each of 
these.  All functions take the slaves address as their first 
parameter.
********************************************************************/

/*
read_coils_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      coil_data          Pointer to an array of data to send
Output:    void
*/
void read_coils_rsp(int8 address, int8 byte_count, int8* coil_data)
{
   int8 i;

   data_buf[0] = byte_count;

   for(i=0; i < byte_count; ++i)
   {
      data_buf[i+1] = *coil_data;
      coil_data++;
   }

   modbus_serial_send(address, 0x01, &(data_buf[0]), 2);
}

/*
read_discrete_input_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      input_data         Pointer to an array of data to send
Output:    void
*/
void read_discrete_input_rsp(int8 address, int8 byte_count, int8 *input_data)
{
   int8 i;

   data_buf[0] = byte_count;

   for(i=0; i < byte_count; ++i)
   {
      data_buf[i+1] = *input_data;
      input_data++;
   }

   modbus_serial_send(address, 0x02, &(data_buf[0]), 4);
}

/*
read_holding_registers_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      reg_data           Pointer to an array of data to send
Output:    void
*/
void read_holding_registers_rsp(int8 address, int8 byte_count, int8 *reg_data)
{
   int8 i;

   data_buf[0] = byte_count;

   for(i=0; i < byte_count; ++i)
   {
      data_buf[i+1] = *reg_data;
      reg_data++;
   }

   modbus_serial_send(address, 0x03, &(data_buf[0]), 4);
}

/*
read_input_registers_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      input_data         Pointer to an array of data to send
Output:    void
*/
void read_input_registers_rsp(int8 address, int8 byte_count, int8 *input_data)
{
   int8 i;
   data_buf[0] = byte_count;

   for(i=0; i < byte_count; ++i)
   {
      data_buf[i+1] = *input_data;
      input_data++;
   }

   modbus_serial_send(address, 0x04, &(data_buf[0]), 4);
}

/*
write_single_coil_rsp
Input:     int8       address            Slave Address
           int16      output_address     Echo of output address received
           int16      output_value       Echo of output value received
Output:    void
*/
void write_single_coil_rsp(int8 address, int16 output_address, int16 output_value)
{
   data_buf[0] = make8(output_address,1);
   data_buf[1] = make8(output_address,0);

   data_buf[2] = make8(output_value,1);
   data_buf[3] = make8(output_value,0);

   modbus_serial_send(address, 0x05, &(data_buf[0]), 4);
}

/*
write_single_register_rsp
Input:     int8       address            Slave Address
           int16      reg_address        Echo of register address received
           int16      reg_value          Echo of register value received
Output:    void
*/
void write_single_register_rsp(int8 address, int16 reg_address, int16 reg_value)
{
   data_buf[0] = make8(reg_address,1);
   data_buf[1] = make8(reg_address,0);

   data_buf[2] = make8(reg_value,1);
   data_buf[3] = make8(reg_value,0);

   modbus_serial_send(address, 0x06, &(data_buf[0]), 4);
}

/*
read_void_status_rsp
Input:     int8       address            Slave Address
Output:    void
*/
void read_void_status_rsp(int8 address, int8 data)
{
   modbus_serial_send(address, 0x07, &data, 1);
}

/*
diagnostics_rsp
Input:     int8       address            Slave Address
           int16      sub_func           Echo of sub function received
           int16      data               Echo of data received
Output:    void
*/
void diagnostics_rsp(int8 address, int16 sub_func, int16 data)
{
   data_buf[0] = make8(sub_func,1);
   data_buf[1] = make8(sub_func,0);

   data_buf[2] = make8(data,1);
   data_buf[3] = make8(data,0);

   modbus_serial_send(address, 0x08, &(data_buf[0]), 4);
}

/*
get_comm_event_counter_rsp
Input:     int8       address            Slave Address
           int16      status             Status, refer to MODBUS documentation
           int16      event_count        Count of events
Output:    void
*/
void get_comm_event_counter_rsp(int8 address, int16 status, int16 event_count)
{
   data_buf[0] = make8(status, 1);
   data_buf[1] = make8(status, 0);

   data_buf[2] = make8(event_count, 1);
   data_buf[3] = make8(event_count, 0);

   modbus_serial_send(address, 0x0B, &(data_buf[0]), 4);
}

/*
get_comm_event_counter_rsp
Input:     int8       address            Slave Address
           int16      status             Status, refer to MODBUS documentation
           int16      event_count        Count of events
           int16      message_count      Count of messages
           int8*      events             Pointer to event data
           int8       events_len         Length of event data in bytes
Output:    void
*/
void get_comm_event_log_rsp(int8 address, int16 status, int16 event_count,
                                 int16 message_count, int8 *events, int8 events_len)
{
   int8 i;

   data_buf[0] = (events_len+6);

   data_buf[1] = make8(status, 1);
   data_buf[2] = make8(status, 0);

   data_buf[3] = make8(event_count, 1);
   data_buf[4] = make8(event_count, 0);

   data_buf[5] = make8(message_count, 1);
   data_buf[6] = make8(message_count, 0);

   for(i=0; i < events_len; ++i)
   {
      data_buf[i+7] = *events;
      events++;
   }

   modbus_serial_send(address, 0x0C, &(data_buf[0]), data_buf[0]);
}

/*
write_multiple_coils_rsp
Input:     int8       address            Slave Address
           int16      start_address      Echo of address to start at
           int16      quantity           Echo of amount of coils written to      
Output:    void
*/
void write_multiple_coils_rsp(int8 address, int16 start_address, int16 quantity)
{
   data_buf[0] = make8(start_address,1);
   data_buf[1] = make8(start_address,0);

   data_buf[2] = make8(quantity,1);
   data_buf[3] = make8(quantity,0);

   modbus_serial_send(address, 0x0F, &(data_buf[0]), 4);
}

/*
write_multiple_registers_rsp
Input:     int8       address            Slave Address
           int16      start_address      Echo of address to start at
           int16      quantity           Echo of amount of registers written to
Output:    void
*/
void write_multiple_registers_rsp(int8 address, int16 start_address, int16 quantity)
{
   data_buf[0] = make8(start_address,1);
   data_buf[1] = make8(start_address,0);

   data_buf[2] = make8(quantity,1);
   data_buf[3] = make8(quantity,0);

   modbus_serial_send(address, 0x10, &(data_buf[0]), 4);
}

/*
report_slave_id_rsp
Input:     int8       address            Slave Address
           int8       slave_id           Slave Address
           int8       run_status         Are we running?
           int8*      data               Pointer to an array holding the data
           int8       data_len           Length of data in bytes
Output:    void
*/
void report_slave_id_rsp(int8 address, int8 slave_id, int8 run_status,
                              int8 *data, int8 data_len)
{
   int8 i;

   data_buf[0] = data_len+2;
   data_buf[1] = slave_id;
   data_buf[2] = run_status;

   for(i=0; i < data_len; ++i)
   {
      data_buf[i+3] = *data;
      data++;
   }

   modbus_serial_send(address, 0x11, &(data[0]), data_buf[0]);
}

/*
read_file_record_rsp
Input:     int8                     address            Slave Address
           int8                     byte_count         Number of bytes to send
           read_sub_request_rsp*    request            Structure holding record/data information
Output:    void
*/
void read_file_record_rsp(int8 address, int8 byte_count, read_sub_request_rsp *request)
{
   int8 i=0,j;

   data_buf[0] = byte_count;

   while(i < byte_count);
   {
      data_buf[i+1] = request->record_length;
      data_buf[i+2] = request->reference_type;

      for(j=0; (j < data_buf[i+1]) && (j < MODBUS_SERIAL_RX_BUFFER_SIZE-3); j+=2)
      {
         data_buf[j+(i+3)] = make8(request->data[j], 1);
         data_buf[j+(i+4)] = make8(request->data[j], 0);
      }

      i += (request->record_length)+1;
      request++;
   }

   modbus_serial_send(address, 0x14, &(data_buf[0]), byte_count+1);
}

/*
write_file_record_rsp
Input:     int8                     address            Slave Address
           int8                     byte_count         Echo of number of bytes sent
           write_sub_request_rsp*   request            Echo of Structure holding record information
Output:    void
*/
void write_file_record_rsp(int8 address, int8 byte_count, write_sub_request_rsp *request)
{
   int8 i, j=0;

   data_buf[0] = byte_count;

   for(i=0; i < byte_count; i+=(7+(j*2)))
   {
      data_buf[i+1] = request->reference_type;
      data_buf[i+2] = make8(request->file_number, 1);
      data_buf[i+3] = make8(request->file_number, 0);
      data_buf[i+4] = make8(request->record_number, 1);
      data_buf[i+5] = make8(request->record_number, 0);
      data_buf[i+6] = make8(request->record_length, 1);
      data_buf[i+7] = make8(request->record_length, 0);

      for(j=0; (j < request->record_length) && (j < MODBUS_SERIAL_RX_BUFFER_SIZE-8); j+=2)
      {
         data_buf[j+(i+8)] = make8(request->data[j], 1);
         data_buf[j+(i+9)] = make8(request->data[j], 0);
      }
   }

   modbus_serial_send(address, 0x15, &(data_buf[0]), byte_count+1);
}

/*
mask_write_register_rsp
Input:     int8        address            Slave Address
           int16       reference_address  Echo of reference address
           int16       AND_mask           Echo of AND mask
           int16       OR_mask            Echo or OR mask
Output:    void
*/
void mask_write_register_rsp(int8 address, int16 reference_address,
                           int16 AND_mask, int16 OR_mask)
{
   data_buf[0] = make8(reference_address,1);
   data_buf[1] = make8(reference_address,0);

   data_buf[2] = make8(AND_mask,1);
   data_buf[3] = make8(AND_mask,0);

   data_buf[4] = make8(OR_mask, 1);
   data_buf[5] = make8(OR_mask, 0);

   modbus_serial_send(address, 0x16, &(data_buf[0]), 6);
}

/*
read_write_multiple_registers_rsp
Input:     int8        address            Slave Address
           int16*      data               Pointer to an array of data
           int8        data_len           Length of data in bytes
Output:    void
*/
void read_write_multiple_registers_rsp(int8 address, int16 *data, int8 data_len)
{
   int8 i;

   data_buf[0] = data_len*2;

   for(i=0; i < data_buf[0]; i+=2)
   {
      data_buf[i+1] = make8(data[i], 1);
      data_buf[i+2] = make8(data[i], 0);
   }

   modbus_serial_send(address, 0x17, &(data_buf[0]), data_buf[0]+1);
}

/*
read_FIFO_queue_rsp
Input:     int8        address            Slave Address
           int16       FIFO_len           Length of FIFO in bytes
           int16*      data               Pointer to an array of data
Output:    void
*/
void read_FIFO_queue_rsp(int8 address, int16 FIFO_len, int16 *data)
{
   int8 i;
   int16 byte_count;

   byte_count = ((FIFO_len*2)+2);

   data_buf[0] = make8(byte_count, 1);
   data_buf[1] = make8(byte_count, 0);

   data_buf[2] = make8(FIFO_len, 1);
   data_buf[3] = make8(FIFO_len, 0);

   for(i=0; i < FIFO_len; i+=2)
   {
      data_buf[i+4] = make8(data[i], 1);
      data_buf[i+5] = make8(data[i], 0);
   }

   modbus_serial_send(address, 0x18, &(data_buf[0]), 2);
}

#endif
