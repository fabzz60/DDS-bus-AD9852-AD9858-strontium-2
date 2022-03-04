//*****************************************************************************
//*****************************************************************************
//                   Laboratoire de Physique des Lasers
//                            Wiotte Fabrice
//                        Ingénieur d'étude CNRS
//                    Service Electronique D002 Rdc
//                CNRS – URM7538 – Université Paris 13
//                       99 Avenue J.-B. Clément
//                         93430 Villetaneuse
//                    http://www-lpl.univ-paris13.fr/
//
// This is part of revision 2.1.0.12573 of the EK-TM4C1294XL Firmware Package.
//                      I/O control via a web server.
//*****************************************************************************
//*****************************************************************************
//             Table 1: Character LCD pins with 1 Controller
//           **********************************************//
//                    1 VSS Power supply (GND)
//                    2 VCC Power supply (+5V)
//                    3 VEE Contrast adjust
//                    4 RS  0 = Instruction input
//                    1 = Data input
//                    5 R/W 0 = Write to LCD module
//                    1 = Read from LCD module
//                    6 EN  Enable signal
//                    7 D0  Data bus line 0 (LSB)
//                    8 D1  Data bus line 1
//                    9 D2  Data bus line 2
//                    10  D3    Data bus line 3
//                    11  D4    Data bus line 4
//                    12  D5    Data bus line 5
//                    13  D6    Data bus line 6
//                    14  D7    Data bus line 7 (MSB)
//**************************************************************************//
//**********************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "inc/hw_ssi.h"
#include "driverlib/ssi.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_uart.h"
#include "inc/hw_types.h"
#include "driverlib/flash.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/rom_map.h"
#include "utils/locator.h"
#include "utils/lwiplib.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "httpserver_raw/httpd.h"
#include "drivers/pinout.h"
#include "io.h"
#include "cgifuncs.h"
#include <stdio.h>
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include <string.h>
#include <stdlib.h>     /* atoi */

//#include "tm4c1294ncpdt.h"
//char temp[7];
//char temp[4];
int t = 0;
char s[20];
char bufdata[8];
unsigned int data;
unsigned int uart0_data;
unsigned long data_frequency = 0;
//unsigned int data_codeur1;
unsigned long data_codeur1;
unsigned int data_codeur2;
unsigned int select_DDS =0;
// SSIO instructions//
void ssi0PutData(int instruction,long long data,int num_byte);
//void ssi0PutData(unsigned long data,int num_byte);
void init_SPI0_mode_single_bit(void);

// UART0
#define UART0_DR_R              (*((volatile unsigned long *)0x4000C000))
#define UART0_FR_R              (*((volatile unsigned long *)0x4000C018))
#define UART0_IBRD_R            (*((volatile unsigned long *)0x4000C024))
#define UART0_FBRD_R            (*((volatile unsigned long *)0x4000C028))
#define UART0_LCRH_R            (*((volatile unsigned long *)0x4000C02C))
#define UART0_CTL_R             (*((volatile unsigned long *)0x4000C030))

#define UART_FR_RXFF            0x00000040  // UART Receive FIFO Full
#define UART_FR_TXFF            0x00000020  // UART Transmit FIFO Full
#define UART_FR_RXFE            0x00000010  // UART Receive FIFO Empty
#define UART_LCRH_WLEN_8        0x00000060  // 8 bit word length
#define UART_LCRH_FEN           0x00000010  // UART Enable FIFOs
#define UART_CTL_UARTEN         0x00000001  // UART Enable

//*****************************************************************************
//
// SSI registers (SSI0)
//
//*****************************************************************************
#define SSI0_CR0_R              (*((volatile uint32_t *)0x40008000))
#define SSI0_CR1_R              (*((volatile uint32_t *)0x40008004))
#define SSI0_DR_R               (*((volatile uint32_t *)0x40008008))
#define SSI0_SR_R               (*((volatile uint32_t *)0x4000800C))
#define SSI0_CPSR_R             (*((volatile uint32_t *)0x40008010))
#define SSI0_IM_R               (*((volatile uint32_t *)0x40008014))
#define SSI0_RIS_R              (*((volatile uint32_t *)0x40008018))
#define SSI0_MIS_R              (*((volatile uint32_t *)0x4000801C))
#define SSI0_ICR_R              (*((volatile uint32_t *)0x40008020))
#define SSI0_DMACTL_R           (*((volatile uint32_t *)0x40008024))
#define SSI0_PP_R               (*((volatile uint32_t *)0x40008FC0))
#define SSI0_CC_R               (*((volatile uint32_t *)0x40008FC8))

//*****************************************************************************
//
// GPIO registers (PORTA)
//
//*****************************************************************************
#define GPIO_PORTA_DATA_R       (*((volatile uint32_t *)0x400583FC))
#define GPIO_PORTA_DIR_R        (*((volatile uint32_t *)0x40058400))
#define GPIO_PORTA_AFSEL_R      (*((volatile uint32_t *)0x40058420))
#define GPIO_PORTA_PUR_R        (*((volatile uint32_t *)0x40058510))
#define GPIO_PORTA_DEN_R        (*((volatile uint32_t *)0x4005851C))
#define GPIO_PORTA_AMSEL_R      (*((volatile uint32_t *)0x40058528))
#define GPIO_PORTA_PCTL_R       (*((volatile uint32_t *)0x4005852C))
#define SYSCTL_PRGPIO_R0        0x00000001  // GPIO Port A Peripheral Ready

// ************************************************ *****************************
//
// Registres GPIO (PORTB)
//
// ************************************************ *****************************
//#define GPIO_PORTB_DATA_BITS_R ((uint32_t * volatile) 0x40059000)
#define GPIO_PORTB_DATA_R (* ((uint32_t * volatile) 0x400593FC))
#define GPIO_PORTB_DIR_R (* ((uint32_t * volatile) 0x40059400))
#define GPIO_PORTB_AFSEL_R (* ((volatile uint32_t *) 0x40059420))
#define GPIO_PORTB_PUR_R (* ((volatile uint32_t *) 0x40059510))
#define GPIO_PORTB_DEN_R (* ((volatile uint32_t *) 0x4005951C))
//#define GPIO_PORTB_LOCK_R (* ((uint32_t * volatile) 0x40059520))
//#define GPIO_PORTB_CR_R (* ((volatile uint32_t *) 0x40059524))
#define GPIO_PORTB_AMSEL_R (* ((volatile uint32_t *) 0x40059528))
#define GPIO_PORTB_PCTL_R (* ((uint32_t * volatile) 0x4005952C))
#define SYSCTL_PRGPIO_R1        0x00000002  // GPIO Port B Peripheral Ready

//*****************************************************************************
//
// GPIO registers (PORTH)
//
//*****************************************************************************
#define GPIO_PORTH_DATA_R       (*((volatile uint32_t *)0x4005F3FC))
#define GPIO_PORTH_DIR_R        (*((volatile uint32_t *)0x4005F400))
#define GPIO_PORTH_AFSEL_R      (*((volatile uint32_t *)0x4005F420))
#define GPIO_PORTH_PUR_R        (*((volatile uint32_t *)0x4005F510))
#define GPIO_PORTH_DEN_R        (*((volatile uint32_t *)0x4005F51C))
#define GPIO_PORTH_AMSEL_R      (*((volatile uint32_t *)0x4005F528))
#define GPIO_PORTH_PCTL_R       (*((volatile uint32_t *)0x4005F52C))
#define SYSCTL_PRGPIO_R         (*((volatile uint32_t *)0x400FEA08))
#define SYSCTL_RCGCGPIO_R7      0x00000080  // GPIO Port H Run Mode Clock
#define SYSCTL_PRGPIO_R7        0x00000080  // GPIO Port H Peripheral Ready

//*****************************************************************************
//
// GPIO registers (PORTN)
//
//*****************************************************************************
#define GPIO_PORTN_DATA_BITS_R  ((volatile uint32_t *)0x40064000)
#define GPIO_PORTN_DATA_R       (*((volatile uint32_t *)0x400643FC))
#define GPIO_PORTN_DIR_R        (*((volatile uint32_t *)0x40064400))
#define GPIO_PORTN_AFSEL_R      (*((volatile uint32_t *)0x40064420))
#define GPIO_PORTN_PUR_R        (*((volatile uint32_t *)0x40064510))
#define GPIO_PORTN_SLR_R        (*((volatile uint32_t *)0x40064518))
#define GPIO_PORTN_DEN_R        (*((volatile uint32_t *)0x4006451C))
#define GPIO_PORTN_LOCK_R       (*((volatile uint32_t *)0x40064520))
#define GPIO_PORTN_CR_R         (*((volatile uint32_t *)0x40064524))
#define GPIO_PORTN_AMSEL_R      (*((volatile uint32_t *)0x40064528))
#define GPIO_PORTN_PCTL_R       (*((volatile uint32_t *)0x4006452C))

//*****************************************************************************
//
// GPIO registers (PORTM)
//
//*****************************************************************************
#define GPIO_PORTM_DATA_BITS_R  ((volatile uint32_t *)0x40063000)
#define GPIO_PORTM_DATA_R       (*((volatile uint32_t *)0x400633FC))
#define GPIO_PORTM_DIR_R        (*((volatile uint32_t *)0x40063400))
#define GPIO_PORTM_IS_R         (*((volatile uint32_t *)0x40063404))
#define GPIO_PORTM_IBE_R        (*((volatile uint32_t *)0x40063408))
#define GPIO_PORTM_IEV_R        (*((volatile uint32_t *)0x4006340C))
#define GPIO_PORTM_IM_R         (*((volatile uint32_t *)0x40063410))
#define GPIO_PORTM_RIS_R        (*((volatile uint32_t *)0x40063414))
#define GPIO_PORTM_MIS_R        (*((volatile uint32_t *)0x40063418))
#define GPIO_PORTM_ICR_R        (*((volatile uint32_t *)0x4006341C))
#define GPIO_PORTM_AFSEL_R      (*((volatile uint32_t *)0x40063420))
#define GPIO_PORTM_DR2R_R       (*((volatile uint32_t *)0x40063500))
#define GPIO_PORTM_DR4R_R       (*((volatile uint32_t *)0x40063504))
#define GPIO_PORTM_DR8R_R       (*((volatile uint32_t *)0x40063508))
#define GPIO_PORTM_ODR_R        (*((volatile uint32_t *)0x4006350C))
#define GPIO_PORTM_PUR_R        (*((volatile uint32_t *)0x40063510))
#define GPIO_PORTM_PDR_R        (*((volatile uint32_t *)0x40063514))
#define GPIO_PORTM_SLR_R        (*((volatile uint32_t *)0x40063518))
#define GPIO_PORTM_DEN_R        (*((volatile uint32_t *)0x4006351C))
#define GPIO_PORTM_LOCK_R       (*((volatile uint32_t *)0x40063520))
#define GPIO_PORTM_CR_R         (*((volatile uint32_t *)0x40063524))
#define GPIO_PORTM_AMSEL_R      (*((volatile uint32_t *)0x40063528))
#define GPIO_PORTM_PCTL_R       (*((volatile uint32_t *)0x4006352C))
#define GPIO_PORTM_ADCCTL_R     (*((volatile uint32_t *)0x40063530))
#define GPIO_PORTM_DMACTL_R     (*((volatile uint32_t *)0x40063534))
#define GPIO_PORTM_SI_R         (*((volatile uint32_t *)0x40063538))
#define GPIO_PORTM_DR12R_R      (*((volatile uint32_t *)0x4006353C))
#define GPIO_PORTM_WAKEPEN_R    (*((volatile uint32_t *)0x40063540))
#define GPIO_PORTM_WAKELVL_R    (*((volatile uint32_t *)0x40063544))
#define GPIO_PORTM_WAKESTAT_R   (*((volatile uint32_t *)0x40063548))
#define GPIO_PORTM_PP_R         (*((volatile uint32_t *)0x40063FC0))
#define GPIO_PORTM_PC_R         (*((volatile uint32_t *)0x40063FC4))

//*****************************************************************************
//
// GPIO registers (PORTL)
//
//*****************************************************************************
#define GPIO_PORTL_DATA_BITS_R  ((volatile uint32_t *)0x40062000)
#define GPIO_PORTL_DATA_R       (*((volatile uint32_t *)0x400623FC))
#define GPIO_PORTL_DIR_R        (*((volatile uint32_t *)0x40062400))
#define GPIO_PORTL_IS_R         (*((volatile uint32_t *)0x40062404))
#define GPIO_PORTL_IBE_R        (*((volatile uint32_t *)0x40062408))
#define GPIO_PORTL_IEV_R        (*((volatile uint32_t *)0x4006240C))
#define GPIO_PORTL_IM_R         (*((volatile uint32_t *)0x40062410))
#define GPIO_PORTL_RIS_R        (*((volatile uint32_t *)0x40062414))
#define GPIO_PORTL_MIS_R        (*((volatile uint32_t *)0x40062418))
#define GPIO_PORTL_ICR_R        (*((volatile uint32_t *)0x4006241C))
#define GPIO_PORTL_AFSEL_R      (*((volatile uint32_t *)0x40062420))
#define GPIO_PORTL_DR2R_R       (*((volatile uint32_t *)0x40062500))
#define GPIO_PORTL_DR4R_R       (*((volatile uint32_t *)0x40062504))
#define GPIO_PORTL_DR8R_R       (*((volatile uint32_t *)0x40062508))
#define GPIO_PORTL_ODR_R        (*((volatile uint32_t *)0x4006250C))
#define GPIO_PORTL_PUR_R        (*((volatile uint32_t *)0x40062510))
#define GPIO_PORTL_PDR_R        (*((volatile uint32_t *)0x40062514))
#define GPIO_PORTL_SLR_R        (*((volatile uint32_t *)0x40062518))
#define GPIO_PORTL_DEN_R        (*((volatile uint32_t *)0x4006251C))
#define GPIO_PORTL_LOCK_R       (*((volatile uint32_t *)0x40062520))
#define GPIO_PORTL_CR_R         (*((volatile uint32_t *)0x40062524))
#define GPIO_PORTL_AMSEL_R      (*((volatile uint32_t *)0x40062528))
#define GPIO_PORTL_PCTL_R       (*((volatile uint32_t *)0x4006252C))
#define GPIO_PORTL_ADCCTL_R     (*((volatile uint32_t *)0x40062530))
#define GPIO_PORTL_DMACTL_R     (*((volatile uint32_t *)0x40062534))
#define GPIO_PORTL_SI_R         (*((volatile uint32_t *)0x40062538))
#define GPIO_PORTL_DR12R_R      (*((volatile uint32_t *)0x4006253C))
#define GPIO_PORTL_WAKEPEN_R    (*((volatile uint32_t *)0x40062540))
#define GPIO_PORTL_WAKELVL_R    (*((volatile uint32_t *)0x40062544))
#define GPIO_PORTL_WAKESTAT_R   (*((volatile uint32_t *)0x40062548))
#define GPIO_PORTL_PP_R         (*((volatile uint32_t *)0x40062FC0))
#define GPIO_PORTL_PC_R         (*((volatile uint32_t *)0x40062FC4))

                                            // Gating Control
#define SYSCTL_PRGPIO_R         (*((volatile uint32_t *)0x400FEA08))
//#define SYSCTL_RCGCGPIO_R12     0x00001000  // GPIO Port N Run Mode Clock
#define SYSCTL_PRGPIO_R12       0x00001000  // GPIO Port N Peripheral Ready
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608))


#define PB0 (*((volatile unsigned long *)0x40059004))
#define PB1 (*((volatile unsigned long *)0x40059008))
#define PB2 (*((volatile unsigned long *)0x40059010))
#define PB3 (*((volatile unsigned long *)0x40059020))

#define PN0 (*((volatile unsigned long *)0x40064004))
#define PN1 (*((volatile unsigned long *)0x40064008))
#define PN2 (*((volatile unsigned long *)0x40064010))
#define PN3 (*((volatile unsigned long *)0x40064020))
#define PN4 (*((volatile unsigned long *)0x40064040))

#define PM0 (*((volatile unsigned long *)0x40063004))
#define PM1 (*((volatile unsigned long *)0x40063008))
#define PM2 (*((volatile unsigned long *)0x40063010))
#define PM3 (*((volatile unsigned long *)0x40063020))
#define PM4 (*((volatile unsigned long *)0x40063040))
#define PM5 (*((volatile unsigned long *)0x40063080))
#define PM6 (*((volatile unsigned long *)0x40063100))
#define PM7 (*((volatile unsigned long *)0x40063200))


#define PH0 (*((volatile unsigned long *)0x4005F004))
#define PH1 (*((volatile unsigned long *)0x4005F008))
#define PH2 (*((volatile unsigned long *)0x4005F010))
#define PH3 (*((volatile unsigned long *)0x4005F020))

#define PF0 (*((volatile unsigned long *)0x00050005))

#define RS (*((volatile unsigned long *)0x40062040))     // RS LCD ---PL4
#define E  (*((volatile unsigned long *)0x40062080))     // ENABLE LCD  --PL5
#define PL6 (*((volatile unsigned long *)0x40062100))    //CS DDS3

//AD9852//
//long long  FTW0 = 0x88888888888;   // 10MHz
//long long  FTW0 = 0xA3D70A3D70A;   // 12MHz
int  FTW_ADRESS = 0x02;   //F0
int  FTW_NUM_BYTE = 0x06;

//float  FTW0 = 0x88888888888;   // 10MHz
float  FTW0;
float  FTW1;
float  FTW2;
float  FTW3;
float  FTW4;
float  FTW5;


unsigned long input_data_amplitude_dds1 = 0;
unsigned long input_data_amplitude_dds2 = 0;
unsigned long input_data_amplitude_dds3 = 0;
unsigned long input_data_amplitude_dds4 = 0;
unsigned long input_data_amplitude_dds5 = 0;

//static char state;
unsigned char temp[8];
int input_data_cs;
unsigned char uart_reset_DDS = 0;
unsigned long mot_32bits;

unsigned short int OSK_ADRESS = 0x08;
int OSK = 2048;
int OSK_NUM_BYTE = 0x02;

unsigned short int PLL_ADRESS = 0x07;
//long PLL = 0x10542560;      //PLL x20 bypass inv sync FSK mode Ramp + Triangle bit auto
//long PLL = 0x10540160;    //PLL x20 bypass inv sync FSK mode Ramp
//long PLL = 0x104A0100;  //PLL X10
//long PLL = 0x104F0100;  //PLL x15
//long PLL = 0x104F0160;    //PLL x15 bypass inv sync OSK enable
long PLL = 0x10510160;      // PLL x17 bypass inv sync OSK enable
int PLL_NUM_BYTE = 0x04;
//long PLL = 0x10200160;    // PLL OFF  bypass inv sync OSK enable


// Flash ROM addresses must be 1k byte aligned, e.g., 0x8000, 0x8400, 0x8800...
// frequency in memory CH1
#define FLASH0                  0x8000  // location in flash to write; make sure no program code is in this block
#define FLASH5                  0x8400
// frequency in memory CH2
#define FLASH1                  0x20400
#define FLASH6                  0x20000
// frequency in memory CH3
#define FLASH2                  0x10000
#define FLASH7                  0x10800
// frequency in memory CH4
#define FLASH3                  0x30000
#define FLASH8                  0x30400

#define GPIO_STRENGTH_12MA      0x00000077  // 12mA drive strength


unsigned int Amplitude_channel_0_DDS1;
unsigned int Amplitude_channel_0_DDS2;
unsigned int Amplitude_channel_0_DDS3;
unsigned int Amplitude_channel_0_DDS4;

uint32_t FTW0_memory[2];
long long read_byte;
long long read_flash;

uint32_t FTW1_memory[2];
long long  read_byte1;
long long  read_flash1;

uint32_t FTW2_memory[2];
long long  read_byte2;
long long  read_flash2;

uint32_t FTW3_memory[2];
long long  read_byte3;
long long  read_flash3;

uint32_t FTW4_memory[2];
long long  read_byte4;
long long  read_flash4;

uint32_t ACR1_memory[1];
int read_byte5;
int read_flash5;

uint32_t ACR2_memory[1];
long read_byte6;
long read_flash6;

uint32_t ACR3_memory[1];
long read_byte7;
long read_flash7;

uint32_t ACR4_memory[1];
long read_byte8;
long read_flash8;

int read_byte9;
int read_flash9;
/*unsigned char Encoder_A(void)
{
  return PN0;      // 0x01 if pressed, 0x00 if not pressed
  //  PN0 = 0x00;
    //return 0;
}
unsigned char Encoder_B(void)
{
  return PN1;      // 0x02 if pressed, 0x00 if not pressed
}

unsigned char Encoder_C(void)
{
  return PB0;      // 0x01 if pressed, 0x00 if not pressed
}
unsigned char Encoder_D(void)
{
  return PB2;      // 0x04 if pressed, 0x00 if not pressed
}
*/

//fonctions//
void PortBIntHandler(void);
void PortNIntHandler(void);
void ssi0PutData(int instruction,long long data,int num_byte);
int Lcd_Cmd(int portM);
int lcd_display(char *disp);
int Lcd_Clear();
void Lcd_Init(void);
void write_immediate(void);
//*****************************************************************************
//! Source files for the internal file system image can be found in the ``fs''
//! directory.  If any of these files are changed, the file system image
//! (io_fsdata.h) should be rebuilt by running the following command from the
//! enet_io directory:
//!
//! ../../../../tools/bin/makefsfile -i fs -o io_fsdata.h -r -h -q
//!
//! For additional details on lwIP, refer to the lwIP web page at:
//! http://savannah.nongnu.org/projects/lwip/
//
//*****************************************************************************

//*****************************************************************************
//
// Defines for setting up the system clock.
//
//*****************************************************************************
#define SYSTICKHZ                10
#define SYSTICKMS               (10000 / SYSTICKHZ)

//*****************************************************************************
//
// Interrupt priority definitions.  The top 3 bits of these values are
// significant with lower values indicating higher priority interrupts.
//
//*****************************************************************************
#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xE0
#define GPION_INT_PRIORITY      0xC0
#define GPIOB_INT_PRIORITY      0xF0
//#define GPION_INT_PRIORITY      0x80
//#define GPIOB_INT_PRIORITY      0xC0
//#define ETHERNET_INT_PRIORITY   0xE0
//*****************************************************************************
//
// A set of flags.  The flag bits are defined as follows:
//
//     0 -> An indicator that the animation timer interrupt has occurred.
//
//*****************************************************************************
#define FLAG_TICK            0
static volatile unsigned long g_ulFlags;

//*****************************************************************************
//
// External Application references.
//
//*****************************************************************************
extern void httpd_init(void);

//*****************************************************************************
//
// SSI tag indices for each entry in the g_pcSSITags array.
//
//*****************************************************************************
#define SSI_INDEX_LEDSTATE  0
#define SSI_INDEX_FORMVARS  1
#define SSI_INDEX_SPEED     2

//*****************************************************************************
//
// This array holds all the strings that are to be recognized as SSI tag
// names by the HTTPD server.  The server will call SSIHandler to request a
// replacement string whenever the pattern <!--#tagname--> (where tagname
// appears in the following array) is found in ".ssi", ".shtml" or ".shtm"
// files that it serves.
//
//*****************************************************************************
static const char *g_pcConfigSSITags[] =
{
    "LEDtxt",        // SSI_INDEX_LEDSTATE
    "FormVars",      // SSI_INDEX_FORMVARS
    "speed"          // SSI_INDEX_SPEED
};

//*****************************************************************************
//
// The number of individual SSI tags that the HTTPD server can expect to
// find in our configuration pages.
//
//*****************************************************************************
#define NUM_CONFIG_SSI_TAGS     (sizeof(g_pcConfigSSITags) / sizeof (char *))

//*****************************************************************************
//
// Prototypes for the various CGI handler functions.
//
//*****************************************************************************
static char *ControlCGIHandler(int32_t iIndex, int32_t i32NumParams,
                               char *pcParam[], char *pcValue[]);
static char *SetTextCGIHandler(int32_t iIndex, int32_t i32NumParams,
                               char *pcParam[], char *pcValue[]);
//*****************************************************************************
//
// Prototype for the main handler used to process server-side-includes for the
// application's web-based configuration screens.
//
//*****************************************************************************
static int32_t SSIHandler(int32_t iIndex, char *pcInsert, int32_t iInsertLen);

//*****************************************************************************
//
// CGI URI indices for each entry in the g_psConfigCGIURIs array.
//
//*****************************************************************************
#define CGI_INDEX_CONTROL       0
#define CGI_INDEX_TEXT          1

//*****************************************************************************
//
// This array is passed to the HTTPD server to inform it of special URIs
// that are treated as common gateway interface (CGI) scripts.  Each URI name
// is defined along with a pointer to the function which is to be called to
// process it.
//
//*****************************************************************************
static const tCGI g_psConfigCGIURIs[] =
{
    { "/iocontrol.cgi", (tCGIHandler)ControlCGIHandler },
    { "/settxt.cgi", (tCGIHandler)SetTextCGIHandler }// CGI_INDEX_CONTROL
};

//*****************************************************************************
//
// The number of individual CGI URIs that are configured for this system.
//
//*****************************************************************************
#define NUM_CONFIG_CGI_URIS     (sizeof(g_psConfigCGIURIs) / sizeof(tCGI))

//*****************************************************************************
//
// The file sent back to the browser by default following completion of any
// of our CGI handlers.  Each individual handler returns the URI of the page
// to load in response to it being called.
//
//*****************************************************************************
#define DEFAULT_CGI_RESPONSE    "/io_cgi.ssi"

//*****************************************************************************
//
// The file sent back to the browser in cases where a parameter error is
// detected by one of the CGI handlers.  This should only happen if someone
// tries to access the CGI directly via the broswer command line and doesn't
// enter all the required parameters alongside the URI.
//
//*****************************************************************************
#define PARAM_ERROR_RESPONSE    "/perror.htm"

#define JAVASCRIPT_HEADER                                                     \
    "<script type='text/javascript' language='JavaScript'><!--\n"
#define JAVASCRIPT_FOOTER                                                     \
    "//--></script>\n"

//*****************************************************************************
//
// Timeout for DHCP address request (in seconds).
//
//*****************************************************************************
#ifndef DHCP_EXPIRE_TIMER_SECS
#define DHCP_EXPIRE_TIMER_SECS  100
#endif

//*****************************************************************************
//
// The current IP address.
//
//*****************************************************************************
uint32_t g_ui32IPAddress;

//*****************************************************************************
//
// The system clock frequency.  Used by the SD card driver.
//
//*****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// This CGI handler is called whenever the web browser requests iocontrol.cgi.
//
//*****************************************************************************
static char *
ControlCGIHandler(int32_t iIndex, int32_t i32NumParams, char *pcParam[],
                  char *pcValue[])
{
    int32_t i32LEDState, i32Speed;
    bool bParamError;

    //
    // We have not encountered any parameter errors yet.
    //
    bParamError = false;

    //
    // Get each of the expected parameters.
    //
    i32LEDState = FindCGIParameter("LEDOn", pcParam, i32NumParams);
    i32Speed = GetCGIParam("speed_percent", pcParam, pcValue, i32NumParams,
            &bParamError);

    //
    // Was there any error reported by the parameter parser?
    //
    if(bParamError || (i32Speed < 0) || (i32Speed > 100))
    {
        return(PARAM_ERROR_RESPONSE);
    }

    //
    // We got all the parameters and the values were within the expected ranges
    // so go ahead and make the changes.
    //
    io_set_led((i32LEDState == -1) ? false : true);
    io_set_animation_speed(i32Speed);

    //
    // Send back the default response page.
    //
    return(DEFAULT_CGI_RESPONSE);
}

//*****************************************************************************
//
// This CGI handler is called whenever the web browser requests settxt.cgi.
//
//*****************************************************************************
static char *
SetTextCGIHandler(int32_t i32Index, int32_t i32NumParams, char *pcParam[],
                  char *pcValue[])
{
    long lStringParam;
    long lStringParam2;
    long lStringParam3;
    char pcDecodedString[10]={'0','0','0','0','0','0','0','0','0'};
    char pcDecodedString2[8]={'0','0','0','0','0','0','0'};
    char pcDecodedString3[2]={'0','0'};
    //char pcDecodedString[5];
    //
    // Find the parameter that has the string we need to display.
    //
    lStringParam = FindCGIParameter("DispText", pcParam, i32NumParams);
    lStringParam2 = FindCGIParameter("DispText2", pcParam, i32NumParams);
    lStringParam3 = FindCGIParameter("DispText3", pcParam, i32NumParams);

    DecodeFormString(pcValue[lStringParam], pcDecodedString, 10);
    mot_32bits = atoi(pcDecodedString);

    if(mot_32bits<=130000000)
    {
    UARTprintf("\n");
    UARTprintf("frequency(Hz) send from Ethernet");
    UARTprintf("\n");
    UARTprintf(pcDecodedString);


    //UARTprintf("datas send in memory\n");
    pcDecodedString[9]='0';
    pcDecodedString[8]='0';
    pcDecodedString[7]='0';
    pcDecodedString[6]='0';
    pcDecodedString[5]='0';
    pcDecodedString[4]='0';
    pcDecodedString[3]='0';
    pcDecodedString[2]='0';
    pcDecodedString[1]='0';
    pcDecodedString[0]='0';
    //DecodeFormString(pcValue[lStringParam2], pcDecodedString2, 8);
    }
    DecodeFormString(pcValue[lStringParam2], pcDecodedString2, 8);
    data_codeur2 = atoi(pcDecodedString2);
    if(data_codeur2<=4095)
    {
    UARTprintf("\n");
    UARTprintf("amplitude(0-4095) send from Ethernet");
    UARTprintf("\n");
    UARTprintf(pcDecodedString2);

    //UARTprintf("datas send in memory\n");
    pcDecodedString2[7]='0';
    pcDecodedString2[6]='0';
    pcDecodedString2[5]='0';
    pcDecodedString2[4]='0';
    pcDecodedString2[3]='0';
    pcDecodedString2[2]='0';
    pcDecodedString2[1]='0';
    pcDecodedString2[0]='0';
    //DecodeFormString(pcValue[lStringParam3], pcDecodedString3, 2);
    }
    DecodeFormString(pcValue[lStringParam3], pcDecodedString3, 2);
    input_data_cs = atoi(pcDecodedString3);
    UARTprintf("\n");
    UARTprintf("select DDS send from Ethernet");
    UARTprintf("\n");
    UARTprintf(pcDecodedString3);

    pcDecodedString3[1]='0';
    pcDecodedString3[0]='0';

    if (input_data_cs == 1)
     {
       FTW0 = (mot_32bits * 281474976710656.0 / 340000000); //Convert to command for DDS  //ad9852
       input_data_amplitude_dds1 = data_codeur2;
     }

    if (input_data_cs == 2)
    {
       FTW1 = (mot_32bits * 281474976710656.0 / 340000000); //Convert to command for DDS  //ad9852
       input_data_amplitude_dds2 = data_codeur2;
    }

    if (input_data_cs == 3)
    {
       FTW3 = (mot_32bits * 281474976710656.0 / 340000000); //Convert to command for DDS  //ad9852
       input_data_amplitude_dds3 = data_codeur2;
    }

    if (input_data_cs == 4)
    {
       FTW4 = (mot_32bits * 281474976710656.0 / 340000000); //Convert to command for DDS  //ad9852
       input_data_amplitude_dds4 = data_codeur2;
    }

    if(input_data_cs == 1)
    {
        //place in flash memory frequency //
        FTW0_memory[0] = (long long)FTW0;
        FTW0_memory[1] = (long long)FTW0>>32;
        ACR1_memory[0] = (int)input_data_amplitude_dds1;
        FlashErase(FLASH0),FlashErase(FLASH5);
        FlashProgram(FTW0_memory, FLASH0, sizeof(FTW0_memory));
        FlashProgram(ACR1_memory, FLASH5, sizeof(ACR1_memory));
        // CS DDS 1
        PB1 =0x00; // CS DDS 1
        PB0 =0x01; // CS DDS 2
        PL6 =0x40; // CS DDS 3
        PB3 =0x08; // CS DDS 4
        ssi0PutData(PLL_ADRESS,PLL,PLL_NUM_BYTE);
        ssi0PutData(OSK_ADRESS,input_data_amplitude_dds1,OSK_NUM_BYTE);
        ssi0PutData(FTW_ADRESS,(long long)FTW0,FTW_NUM_BYTE);
        //PB1 =0x02; // CS DDS 1
      }
    if(input_data_cs == 2)
        {

        FTW1_memory[0] = (long long)FTW1;
        FTW1_memory[1] = (long long)FTW1>>32;
        ACR2_memory[0] = (int)input_data_amplitude_dds2;
        FlashErase(FLASH1),FlashErase(FLASH6);
        FlashProgram(FTW1_memory, FLASH1, sizeof(FTW1_memory));
        FlashProgram(ACR2_memory, FLASH6, sizeof(ACR2_memory));
        // CS DDS 2
        PB1 =0x02; // CS DDS 1
        PB0 =0x00; // CS DDS 2
        PL6 =0x40; // CS DDS 3
        PB3 =0x08; // CS DDS 4
        ssi0PutData(PLL_ADRESS,PLL,PLL_NUM_BYTE);
        ssi0PutData(OSK_ADRESS,input_data_amplitude_dds2,OSK_NUM_BYTE);
        ssi0PutData(FTW_ADRESS,(long long)FTW1,FTW_NUM_BYTE);
        //PB0 =0x01; // CS DDS 2
        }
    if(input_data_cs == 3)
        {
        FTW3_memory[0] = (long long)FTW3;
        FTW3_memory[1] = (long long)FTW3>>32;
        ACR3_memory[0] = (int)input_data_amplitude_dds3;
        FlashErase(FLASH2),FlashErase(FLASH7);
        FlashProgram(FTW3_memory, FLASH2, sizeof(FTW3_memory));
        FlashProgram(ACR3_memory, FLASH7, sizeof(ACR3_memory));
        // CS DDS 3
        PB1 =0x02; // CS DDS 1
        PB0 =0x01; // CS DDS 2
        PL6 =0x00; // CS DDS 3
        PB3 =0x08; // CS DDS 4
        ssi0PutData(PLL_ADRESS,PLL,PLL_NUM_BYTE);
        ssi0PutData(OSK_ADRESS,input_data_amplitude_dds3,OSK_NUM_BYTE);
        ssi0PutData(FTW_ADRESS,(long long)FTW3,FTW_NUM_BYTE);
        //PB0 =0x01; // CS DDS 2
        }
    if(input_data_cs == 4)
        {
        FTW4_memory[0] = (long long)FTW4;
        FTW4_memory[1] = (long long)FTW4>>32;
        ACR4_memory[0] = (int)input_data_amplitude_dds4;
        FlashErase(FLASH3),FlashErase(FLASH8);
        FlashProgram(FTW4_memory, FLASH3, sizeof(FTW4_memory));
        FlashProgram(ACR4_memory, FLASH8, sizeof(ACR4_memory));
        // CS DDS 4
        PB1 =0x02; // CS DDS 1
        PB0 =0x01; // CS DDS 2
        PL6 =0x40; // CS DDS 3
        PB3 =0x00; // CS DDS 4
        ssi0PutData(PLL_ADRESS,PLL,PLL_NUM_BYTE);
        ssi0PutData(OSK_ADRESS,input_data_amplitude_dds4,OSK_NUM_BYTE);
        ssi0PutData(FTW_ADRESS,(long long)FTW4,FTW_NUM_BYTE);
        //PB0 =0x01; // CS DDS 2
        }

    //
    // Tell the HTTPD server which file to send back to the client.
    //
    return(DEFAULT_CGI_RESPONSE);
}

//*****************************************************************************
//
// This function is called by the HTTP server whenever it encounters an SSI
// tag in a web page.  The iIndex parameter provides the index of the tag in
// the g_pcConfigSSITags array. This function writes the substitution text
// into the pcInsert array, writing no more than iInsertLen characters.
//
//*****************************************************************************
static int32_t
SSIHandler(int32_t iIndex, char *pcInsert, int32_t iInsertLen)
{
    //
    // Which SSI tag have we been passed?
    //
    switch(iIndex)
    {
        case SSI_INDEX_LEDSTATE:
            io_get_ledstate(pcInsert, iInsertLen);
            break;

        case SSI_INDEX_FORMVARS:
            usnprintf(pcInsert, iInsertLen,
                    "%sls=%d;\nsp=%d;\n%s",
                    JAVASCRIPT_HEADER,
                    io_is_led_on(),
                    io_get_animation_speed(),
                    JAVASCRIPT_FOOTER);
            break;

        case SSI_INDEX_SPEED:
            io_get_animation_speed_string(pcInsert, iInsertLen);
            break;

        default:
            usnprintf(pcInsert, iInsertLen, "??");
            break;
    }

    //
    // Tell the server how many characters our insert string contains.
    //
    return(strlen(pcInsert));
}
//*****************************************************************************
//
// The interrupt handler for the SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Call the lwIP timer handler.
    //
    lwIPTimer(SYSTICKMS);
}

//*****************************************************************************
//
// The interrupt handler for the timer used to pace the animation.
//
//*****************************************************************************
void
AnimTimerIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Indicate that a timer interrupt has occurred.
    //
    HWREGBITW(&g_ulFlags, FLAG_TICK) = 1;
}

//*****************************************************************************
//
// Display an lwIP type IP Address.
//
//*****************************************************************************
void
DisplayIPAddress(uint32_t ui32Addr)
{
    char pcBuf[16];

    //
    // Convert the IP Address into a string.
    //
    usprintf(pcBuf, "%d.%d.%d.%d", ui32Addr & 0xff, (ui32Addr >> 8) & 0xff,
            (ui32Addr >> 16) & 0xff, (ui32Addr >> 24) & 0xff);

    //
    // Display the string.
    //
    UARTprintf(pcBuf);
    Lcd_Clear();
    Lcd_Cmd(0x82);
    lcd_display("     IP adress");
    Lcd_Cmd(0xc2);
    sprintf(pcBuf,"%d.%d.%d.%d", ui32Addr & 0xff, (ui32Addr >> 8) & 0xff,(ui32Addr >> 16) & 0xff, (ui32Addr >> 24) & 0xff);
    lcd_display(pcBuf);
    Lcd_Cmd(0x91);
    lcd_display("DDS bus AD9852");
    Lcd_Cmd(0xd2);
    lcd_display("Electronique");
}
void Displaydata(uint32_t ui32temp)
{
    char pcBuf1[16];

    //
    // Convert the data_codeur into a string.
    //
    usprintf(pcBuf1, "%d.%d.%d.%d", ui32temp & 0xff, (ui32temp >> 8) & 0xff,
                (ui32temp >> 16) & 0xff, (ui32temp >> 24) & 0xff);

    //
    // Display the string.
    //
    UARTprintf(pcBuf1);
    //Lcd_Clear();
    //Lcd_Cmd(0x83);
    //lcd_display("    From Ethernet");
    //Lcd_Cmd(0xc0);
    //sprintf(pcBuf1,"%d.%d.%d.%d", ui32temp & 0xff, (ui32temp >> 8) & 0xff,(ui32temp >> 16) & 0xff, (ui32temp >> 24) & 0xff);
    //lcd_display(pcBuf1);
    //Lcd_Cmd(0x91);
    //lcd_display("DDS bus AD9852");
    //Lcd_Cmd(0xd2);
    //lcd_display("Electronique");

}
//*****************************************************************************
//
// Required by lwIP library to support any host-related timer functions.
//
//*****************************************************************************
void
lwIPHostTimerHandler(void)
{
    uint32_t ui32NewIPAddress;
    //uint32_t ui32Newvalue;

    //
    // Get the current IP address.
    //
    ui32NewIPAddress = lwIPLocalIPAddrGet();

    //
    // See if the IP address has changed.
    //
    if(ui32NewIPAddress != g_ui32IPAddress)
    {
        //
        // See if there is an IP address assigned.
        //
        if(ui32NewIPAddress == 0xffffffff)
        {
            //
            // Indicate that there is no link.
            //
            UARTprintf("Waiting for link.\n");
        }
        else if(ui32NewIPAddress == 0)
        {
            //
            // There is no IP address, so indicate that the DHCP process is
            // running.
            //
            UARTprintf("Waiting for IP address.\n");
        }
        else
        {
            //
            // Display the new IP address.
            //
            UARTprintf("IP Address: ");
            DisplayIPAddress(ui32NewIPAddress);
            UARTprintf("\n");
            UARTprintf("Open a browser and enter the IP address.\n");

        }

        //
        // Save the new IP address.
        //
        g_ui32IPAddress = ui32NewIPAddress;

    }

    //
    // If there is not an IP address.
    //
    if((ui32NewIPAddress == 0) || (ui32NewIPAddress == 0xffffffff))
    {
        //
        // Do nothing and keep waiting.
        //
    }
}
void PortNIntHandler(void)
{

/* if (GPIOIntStatus(GPIO_PORTN_BASE, false) & PN0) // PN0 was interrupt cause
    {

        GPIOIntRegister(GPIO_PORTN_BASE, PortNIntHandler);   // Register our handler function for port D
        GPIOIntTypeSet(GPIO_PORTN_BASE, PN0,GPIO_FALLING_EDGE);  // Configure PD3 for rising edge trigger
        GPIOIntClear(GPIO_PORTN_BASE, PN0);  // Clear interrupt flag

        //if(data_codeur >=1000000) {data_codeur = data_codeur; }
        //if(data_codeur <1084) {data_codeur = 1084; }

       //if(data_codeur >0 && data_codeur <=1000000)
       // {
        if (Encoder_A())
        {
             if(Encoder_B())
              {
                UARTprintf("Button FAST +\n");
                //data_codeur+=1020;  // pas 1mA sur 10 ohms = 10mV
                //data_codeur++;
                //ssi0PutData(0x01,data_codeur<<4,3);
                SysCtlDelay(100);
                //PH1 = 0x00;
                //SysCtlDelay(10);
                //PH1 = 0x02;

              }
            else
              {
                //if(data_codeur <= 553095)
                //{
                UARTprintf("Button FAST -\n");
                //data_codeur-=1020; // pas 1mA  sur 10 ohms = 10mV
                //data_codeur--;
                //ssi0PutData(0x01,data_codeur<<4,3);
                SysCtlDelay(100);
                //PH1 = 0x00;
                //SysCtlDelay(10);
                //PH1 = 0x02;
               // }

              }
        }
        //data_codeur_memory[0] = (long)data_codeur;
        //FlashErase(FLASH0);
        //FlashProgram(data_codeur_memory, FLASH0, sizeof(data_codeur_memory));
        //UARTprintf("datas send in memory\n");
  //  }

}
*/
}
void PortBIntHandler(void)
{
/* if (GPIOIntStatus(GPIO_PORTB_BASE, false) & PB0)
    {

        GPIOIntRegister(GPIO_PORTB_BASE, PortBIntHandler);   // Register our handler function for port D
        GPIOIntTypeSet(GPIO_PORTB_BASE, PB0,GPIO_FALLING_EDGE);  // Configure PD3 for rising edge trigger
        GPIOIntClear(GPIO_PORTB_BASE, PB0);  // Clear interrupt flag

        //if(data_codeur <1) {data_codeur = 1; }

        //if(data_codeur >0 && data_codeur <=1048575)
       // {

        if (Encoder_C())
             {
            if(Encoder_D())
              {
                UARTprintf("Button SLOW +\n");
                //data_codeur+=9; // pas 0.01mA  sur 10 ohms = 10uV

                //data_codeur++;
                //ssi0PutData(0x01,data_codeur<<4,3);
                SysCtlDelay(100);
                //PH1 = 0x00;
                //SysCtlDelay(10);
                //PH1 = 0x02;
               }
            else
              {
                UARTprintf("Button SLOW -\n");
                //data_codeur-=9;  // pas 0.01mA  sur 10 ohms = 10uV
                //data_codeur--;
                //ssi0PutData(0x01,data_codeur<<4,3);
                SysCtlDelay(100);
                //PH1 = 0x00;
                //SysCtlDelay(10);
                //PH1 = 0x02;
              }


             }
                //data_codeur_memory[0] = (long)data_codeur;
                //FlashErase(FLASH0);
                //FlashProgram(data_codeur_memory, FLASH0, sizeof(data_codeur_memory));
                //UARTprintf("datas send in memory\n");
        }


     //  }
*/
}
//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void UARTIntHandler(void)
{
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts

   while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
     {
        temp[t++] = UART0_DR_R; //Read from buffer
           if(t==7)
            {
             write_immediate();
             t = 0; //Reset read length
            }
        }
}
/************************************************************/
/* Prototype - write_immediate                              */
/*                                                          */
/*  Description                                             */
/*  Reads incoming frequency and send to DDS immediately    */
/************************************************************/
void write_immediate()  //AD9852
{
  input_data_cs = (unsigned long)(temp[6]);
  if(input_data_cs==1) {PB1 = 0x00;} else { PB1 = 0x02;}  //CS1  DDS1 ON
  if(input_data_cs==2) {PB0 = 0x00;} else { PB0 = 0x01;}  //CS2  DDS2 ON
  if(input_data_cs==3) {PL6 = 0x00;} else { PL6 = 0x40;}  //CS3  DDS3 ON
  if(input_data_cs==4) {PB3 = 0x00;} else { PB3 = 0x08;}  //CS4  DDS4 ON
  //if(input_data_cs==5) {PE2 = 0x00;} else { PE2 = 0x04;}  //CS4  DDS5 ON

  mot_32bits = ((unsigned long)(temp[5]) << 24) | ((unsigned long)(temp[4]) << 16) | ((unsigned long)(temp[3]) << 8) | ((unsigned long)(temp[2]));
  //uart_reset_DDS = temp[7];

  if(input_data_cs==1 && uart_reset_DDS == 0)
  {
    FTW0 = (mot_32bits * 281474976710656.0 / 340000000); //Convert to command for DDS  //ad9852
    input_data_amplitude_dds1 = ((unsigned long)(temp[1]) << 8) | ((unsigned long)(temp[0]));
  }

  if(input_data_cs==2 && uart_reset_DDS == 0)
   {
     FTW1 = (mot_32bits * 281474976710656.0 / 340000000); //Convert to command for DDS  //ad9852
     input_data_amplitude_dds2 = ((unsigned long)(temp[1]) << 8) | ((unsigned long)(temp[0]));
   }

  if(input_data_cs==3 && uart_reset_DDS == 0)
    {
      FTW3 = (mot_32bits * 281474976710656.0 / 340000000); //Convert to command for DDS  //ad9852
      input_data_amplitude_dds3 = ((unsigned long)(temp[1]) << 8) | ((unsigned long)(temp[01]));
    }

  if(input_data_cs==4 && uart_reset_DDS == 0)
    {
      FTW4 = (mot_32bits * 281474976710656.0 / 340000000); //Convert to command for DDS  //ad9852
      input_data_amplitude_dds4 = ((unsigned long)(temp[1]) << 8) | ((unsigned long)(temp[01]));
    }

  if(input_data_cs==5 && uart_reset_DDS == 0)
    {
      FTW5 = (mot_32bits * 281474976710656.0 / 340000000); //Convert to command for DDS  //ad9852
      input_data_amplitude_dds5 = ((unsigned long)(temp[1]) << 8) | ((unsigned long)(temp[0]));
    }

    if(input_data_cs==1)
    {
    //place in flash memory frequency //
    FTW0_memory[0] = (long long)FTW0;
    FTW0_memory[1] = (long long)FTW0>>32;
    ACR1_memory[0] = (int)input_data_amplitude_dds1;
    FlashErase(FLASH0),FlashErase(FLASH5);
    FlashProgram(FTW0_memory, FLASH0, sizeof(FTW0_memory));
    FlashProgram(ACR1_memory, FLASH5, sizeof(ACR1_memory));
    ssi0PutData(PLL_ADRESS,PLL,PLL_NUM_BYTE);
    ssi0PutData(OSK_ADRESS,input_data_amplitude_dds1,OSK_NUM_BYTE);
    ssi0PutData(FTW_ADRESS,(long long)FTW0,FTW_NUM_BYTE);
    Lcd_Clear();
    Lcd_Cmd(0x81);
    lcd_display("   data from USB");
    Lcd_Cmd(0xc0);
    lcd_display("   Channel 1");
    Lcd_Cmd(0x90);
    sprintf(s,"F(Hz) = %d",mot_32bits);
    lcd_display(s);
    Lcd_Cmd(0xd0);
    sprintf(s,"AMP(0-4095)=%d",input_data_amplitude_dds1);
    lcd_display(s);
    PB1 = 0x02;
    }


   if(input_data_cs==2)
   {
    FTW1_memory[0] = (long long)FTW1;
    FTW1_memory[1] = (long long)FTW1>>32;
    ACR2_memory[0] = (int)input_data_amplitude_dds2;
    FlashErase(FLASH1),FlashErase(FLASH6);
    FlashProgram(FTW1_memory, FLASH1, sizeof(FTW1_memory));
    FlashProgram(ACR2_memory, FLASH6, sizeof(ACR2_memory));
    ssi0PutData(PLL_ADRESS,PLL,PLL_NUM_BYTE);
    ssi0PutData(OSK_ADRESS,input_data_amplitude_dds2,OSK_NUM_BYTE);
    ssi0PutData(FTW_ADRESS,(long long)FTW1,FTW_NUM_BYTE);
    Lcd_Clear();
    Lcd_Cmd(0x81);
    lcd_display("   data from USB");
    Lcd_Cmd(0xc0);
    lcd_display("   Channel 2");
    Lcd_Cmd(0x90);
    sprintf(s,"F(Hz) = %d",mot_32bits);
    lcd_display(s);
    Lcd_Cmd(0xd0);
    sprintf(s,"AMP(0-4095)=%d",input_data_amplitude_dds2);
    lcd_display(s);
    PB0 = 0x01;
   }

  if(input_data_cs==3)
  {
    FTW3_memory[0] = (long long)FTW3;
    FTW3_memory[1] = (long long)FTW3>>32;
    ACR3_memory[0] = (int)input_data_amplitude_dds3;
    FlashErase(FLASH2),FlashErase(FLASH7);
    FlashProgram(FTW3_memory, FLASH2, sizeof(FTW3_memory));
    FlashProgram(ACR3_memory, FLASH7, sizeof(ACR3_memory));
    ssi0PutData(PLL_ADRESS,PLL,PLL_NUM_BYTE);
    ssi0PutData(OSK_ADRESS,input_data_amplitude_dds3,OSK_NUM_BYTE);
    ssi0PutData(FTW_ADRESS,FTW3,FTW_NUM_BYTE);
    Lcd_Clear();
    Lcd_Cmd(0x81);
    lcd_display("   data from USB");
    Lcd_Cmd(0xc0);
    lcd_display("   Channel 3");
    Lcd_Cmd(0x90);
    sprintf(s,"F(Hz) = %d",mot_32bits);
    lcd_display(s);
    Lcd_Cmd(0xd0);
    sprintf(s,"AMP(0-4095)=%d",input_data_amplitude_dds3);
    lcd_display(s);
    PL6 = 0x40;
   }
  if(input_data_cs==4)
  {
    FTW4_memory[0] = (long long)FTW4;
    FTW4_memory[1] = (long long)FTW4>>32;
    ACR4_memory[0] = (int)input_data_amplitude_dds4;
    FlashErase(FLASH3),FlashErase(FLASH8);
    FlashProgram(FTW4_memory, FLASH3, sizeof(FTW4_memory));
    FlashProgram(ACR4_memory, FLASH8, sizeof(ACR4_memory));
    ssi0PutData(PLL_ADRESS,PLL,PLL_NUM_BYTE);
    ssi0PutData(OSK_ADRESS,input_data_amplitude_dds4,OSK_NUM_BYTE);
    ssi0PutData(FTW_ADRESS,FTW4,FTW_NUM_BYTE);
    Lcd_Clear();
    Lcd_Cmd(0x81);
    lcd_display("   data from USB");
    Lcd_Cmd(0xc0);
    lcd_display("   Channel 4");
    Lcd_Cmd(0x90);
    sprintf(s,"F(Hz) = %d",mot_32bits);
    lcd_display(s);
    Lcd_Cmd(0xd0);
    sprintf(s,"AMP(0-4095)=%d",input_data_amplitude_dds4);
    lcd_display(s);
    PB3 = 0x08;
  }
  if(input_data_cs==5)
  {
    Lcd_Clear();
    Lcd_Cmd(0x81);
    lcd_display("   data from USB");
    Lcd_Cmd(0xc0);
    lcd_display("   Channel 5");
    Lcd_Cmd(0x90);
    lcd_display(" Frequency(Hz)");
    Lcd_Cmd(0xd1);
    sprintf(s,"F(Hz) = %d",mot_32bits);
    lcd_display(s);
    SysCtlDelay(50000000);  //37.5ns * 50000000 = 1,875 s
    Lcd_Clear();
    Lcd_Cmd(0x81);
    lcd_display("   data from USB");
    Lcd_Cmd(0xc0);
    lcd_display("   Channel 5");
    Lcd_Cmd(0x90);
    Lcd_Cmd(0x90);
    lcd_display("  Amp(0-2048)");
    Lcd_Cmd(0xd0);
    sprintf(s,"AMP(0-4095)=%d",input_data_amplitude_dds5);
    lcd_display(s);
  }

 /* if (uart_reset_DDS == 1 && input_data_cs==1)
     {
          PH0 =0x01; // reset DDS
          SysCtlDelay(1000);
          PH0 =0x00;
     }
  if (uart_reset_DDS == 1 && input_data_cs==2)
       {
          PH0 =0x01; // reset DDS
          SysCtlDelay(1000);
          PH0 =0x00;
       }

  if (uart_reset_DDS == 1 && input_data_cs==3)
         {
            PH0 =0x01; // reset DDS
            SysCtlDelay(1000);
            PH0 =0x00;
         }
  if (uart_reset_DDS == 1 && input_data_cs==4)
          {
             PH0 =0x01; // reset DDS
             SysCtlDelay(1000);
             PH0 =0x00;
          }

     */

}
void init_SPI0_mode_single_bit(void)
{
    // Enable Peripheral SSI0
    //if Single bit serial mode SPI
     //***********************//
    // SSI0 is used with the following GPIO Pin Mapping
    // SSI1CLK   : PA2
    // SSI1FSS   : PA3
    // SSI1XDAT0 : PA4
    //************************//

    //*************************************//
    //Polarity Phase       Mode
    //   0       0   SSI_FRF_MOTO_MODE_0
    //   0       1   SSI_FRF_MOTO_MODE_1
    //   1       0   SSI_FRF_MOTO_MODE_2
    //   1       1   SSI_FRF_MOTO_MODE_3
    //*************************************//

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);  //(SSI0Tx)
    GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);  //(SSI0Rx)

     // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.
    //GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
    GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_4);
    SSIConfigSetExpClk(SSI0_BASE, 120000000, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 10000000, 8);
    //
    // Enable the SSI0 module.
    //
    SSI0_CR1_R = 0x00000000;
    SSIEnable(SSI0_BASE); // Enable the SSI
}
// init data transfert SPI data to send SSI0
//*****************************************************************************
//
//  SSI0 SERIAL PORT
//
//*****************************************************************************
void ssi0PutData( int instruction,long long data,int num_byte)
{
   int i=0;
   SSI0_DR_R =  instruction;
    while( num_byte )
     {
          while(!(SSI0_SR_R & SSI_SR_TNF)) {} // wait until there is space to write in the buffer
          SSI0_DR_R =  data >>(num_byte-1-i)*8;
          num_byte--;
     }

    while( !( SSI0_SR_R & SSI_SR_TNF ) )
          {
            ;
          }
}
void init_UART0(void)
{
  // Enable Peripheral UART0
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
  (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}
void PortA_Init(void)
{
                                   // activate clock for Port A
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
                                   // allow time for clock to stabilize
  //while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R0) == 0){};
  GPIO_PORTA_DIR_R |= 0xC0;             // make PA6,7 out
  //GPIO_PORTA_AFSEL_R &= ~0xFF;     // disable alt funct on PA7-0
  GPIO_PORTA_DEN_R |= 0xFF;        // enable digital I/O on PA7-0
                                   // configure PA7-0  as GPIO
  //GPIO_PORTA_PCTL_R &= ~0xFFFFFFFF; // 4) configure PA7-0  as GPIO
  GPIO_PORTA_AMSEL_R &= ~0xFF;     // disable analog functionality on PA7-0
}
void PortB_Init(void)
{
    // activate clock for Port B
   // volatile  uint32_t ui32Loop;
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
      // allow time for clock to stabilize
  //while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R1) == 0){};
  //GPIO_PORTB_DIR_R |= 0x0F;        // make PB OUT
  GPIO_PORTB_DIR_R |= 0x0F;
  //GPIO_PORTB_AFSEL_R &= ~0x00;     // disable alt funct on PB5-0
  GPIO_PORTB_DEN_R |= 0x0F;        //  activer les E / S numériques sur PB2-0
                                   // configure PB3-0 as GPIO
  //GPIO_PORTB_PCTL_R &= ~0x0000FFFF; // 4) configure PB3-0 as GPIO
  GPIO_PORTB_AMSEL_R &= ~0x0F;     // disable analog functionality on PB2-0
  // Faites une lecture factice pour insérer quelques cycles après avoir activé le périphérique.
  //ui32Loop = SYSCTL_RCGCGPIO_R;
}
void PortN_Init(void)
{
    // activate clock for Port N
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
      // allow time for clock to stabilize
  //while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};
  GPIO_PORTN_DIR_R |= 0x00;        // make PN5-0 en entrée
  GPIO_PORTN_AFSEL_R &= ~0x00;     // disable alt funct on PN5-0
  GPIO_PORTN_DEN_R |= 0x07;        //  activer les E / S numériques sur PN2-0
                                   // configure PN3-0 as GPIO
  GPIO_PORTN_PCTL_R &= ~0x0000FFFF; // 4) configure PN7-0 as GPIO
  GPIO_PORTN_AMSEL_R &= ~0x07;     // disable analog functionality on PN2-0
}
void PortH_Init(void)  //// Initialize GPIO Port H
{
                                  // activate clock for Port H
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
                                   // allow time for clock to stabilize
  //while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};
  GPIO_PORTH_DIR_R |= 0x0F;            // make PH0   out
  //GPIO_PORTH_AFSEL_R &= ~0x00;     // désactiver la fonction alt sur PH3-0
  //GPIO_PORTH_PUR_R = 0x03;        // enable pull-up on
  GPIO_PORTH_DEN_R |= 0x0F;          // enable digital I/O on PH3-0
  GPIO_PORTH_PCTL_R &= ~0x0000FFFF; // 4) configure PH3-0 as GPIO
  GPIO_PORTH_AMSEL_R = ~0x0F;         // disable analog functionality on PH3-0
}
void PortM_Init(void)
{
                                   // activate clock for Port M
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;
                                   // allow time for clock to stabilize
  //while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};
  GPIO_PORTM_DIR_R |= 0xFF;        // make PM7-0 out
  //GPIO_PORTM_AFSEL_R &= ~0xFF;     // disable alt funct on PM7-0
  GPIO_PORTM_DEN_R |= 0xFF;        // enable digital I/O on PM7-0
                                   // configure PM7-0 as GPIO
  GPIO_PORTM_PCTL_R &= ~0xFFFFFFFF; // 4) configure PM7-0 as GPIO
  GPIO_PORTM_AMSEL_R &= ~0xFF;     // disable analog functionality on PM7-0
}
void PortL_Init(void)
{
                                   // activate clock for Port L
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;
                                   // allow time for clock to stabilize
  //while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};
  GPIO_PORTL_DIR_R |= 0xFF;        // make PM7-0 out
  //GPIO_PORTM_AFSEL_R &= ~0xFF;     // disable alt funct on PM7-0
  GPIO_PORTL_DEN_R |= 0xFF;        // enable digital I/O on PM7-0
                                   // configure PM7-0 as GPIO
  GPIO_PORTL_PCTL_R &= ~0xFFFFFFFF; // 4) configure PM7-0 as GPIO
  GPIO_PORTL_AMSEL_R &= ~0xFF;     // disable analog functionality on PM7-0
}

void PeripheralEnableInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
}
void Setup_GPIO(void)
{
    // Configure GPIOs
    // 1. Enable Clock to the GPIO Modules (SYSCTL->RCGCGPIO)
    //SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    //SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
    //SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
    //allow time for clock to stabilize (SYSCTL-PRGPIO)
    //while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R1) == 0){};
    //while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};
    // 2. Unlock GPIO only PD7, PF0 on TM4C123G; PD7, PE7 on TM4C1294 (GPIO->LOCK and GPIO->CR)
    // 3. Set Analog Mode Select bits for each Port (GPIO->AMSEL 0=digital, 1=analog)
    //GPIO_PORTB_AMSEL_R &= ~0x00;
    //GPIO_PORTN_AMSEL_R &= ~0x00;
    //GPIO_PORTB_DIR_R = 0X07;        // make PB5-0 en entire
    //GPIO_PORTN_DIR_R = 0X07;        // make PB5-0 en entire
    // 4. Set Port Control Register for each Port (GPIO->PCTL, check the PCTL table)
    // 5. Set Alternate Function Select bits for each Port (GPIO->AFSEL 0=regular I/O, 1=PCTL peripheral)
    // 6. Set Output pins for each Port (Direction of the Pins: GPIO->DIR 0=input, 1=output)
    //GPIO_PORTB_DEN_R = 0x07;        //  activer les E / S numériques
    //GPIO_PORTN_DEN_R = 0x07;        //  activer les E / S numériques

    // 7. Set PUR bits (internal Pull-Up Resistor), PDR (Pull-Down Resistor), ODR (Open Drain) for each Port (0: disable, 1=enable)

    // 8. Set Digital ENable register on all GPIO pins (GPIO->DEN 0=disable, 1=enable)

}
int Lcd_Port(int portM)
{
     GPIO_PORTM_DATA_R = portM;
     return 0;
}
//  Function for sending command to LCD
int Lcd_Cmd(int portM)
{
     RS = 0x00;             // => RS = 0
     Lcd_Port(portM);
     E  = 0x20;             // Enable = 1
     SysCtlDelay(15000);
     E  = 0x00;             // Enable = 0
     return 0;
}
//  Function for sending data to LCD
int Lcd_data(int portM)
{
    Lcd_Port(portM);
    RS   = 0x10;  // RS = 1
    E   = 0x20;    //Enable = 1
    SysCtlDelay(15000);
    E   = 0x00;    // Enable = 0
    return 0;
}
int Lcd_Clear()
{
    Lcd_Cmd(0);
    Lcd_Cmd(1);
    return 0;
}
void Lcd_display_off()
{
    Lcd_Cmd(0);
    Lcd_Cmd(0x0C);
}
void Lcd_Shift_Right()
{
    Lcd_Cmd(0x01);
    Lcd_Cmd(0x18);
}

void Lcd_Shift_Left()
{
    Lcd_Cmd(0x01);
    Lcd_Cmd(0x1C);
}
//  Function for initializing LCD
void Lcd_Init()
{
  SysCtlDelay(10000);
  Lcd_Cmd(0x38);     //Function set: 2 Line, 8-bit, 5x7 dots
  SysCtlDelay(10000);
  //Lcd_Cmd(0x0f);     // Display on Cursor blinking
  //SysCtlDelay(10000);
  Lcd_Cmd(0x0c);
  SysCtlDelay(10000);
  Lcd_Cmd(0x01);     //Clear LCD
  SysCtlDelay(10000);
  Lcd_Cmd(0x06);     //Entry mode, auto increment with no shift
  SysCtlDelay(10000);
  Lcd_Cmd(0x83);  // DDRAM addresses 0x80..0x8F + 0xC0..0xCF are used.
  SysCtlDelay(10000);
}
//  Function for sending string to LCD
int lcd_display(char *disp)
{
    int x=0;
    while(disp[x]!=0)
    {
        Lcd_data(disp[x]);
        x++;
    }
    return 0;
}
//*****************************************************************************
//
// This example demonstrates the use of the Ethernet Controller and lwIP
// TCP/IP stack to control various peripherals on the board via a web
// browser.
//
//*****************************************************************************
int
main(void)

{
    //
    // Run from the PLL at 120 MHz.
    //
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 120000000);
    PeripheralEnableInit();
    init_UART0();
    IntEnable(INT_UART0);  //enable the UART interrupt
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts
    PortA_Init();
    //Setup_GPIO();
    PortB_Init();
    PortN_Init();
    PortH_Init();
    PortM_Init();
    PortL_Init();
    Lcd_Init();
    SysCtlDelay(20000);
    Lcd_Clear();
    SysCtlDelay(400000);
    Lcd_Cmd(0x83);
    lcd_display("Welcome to");
    Lcd_Cmd(0xc0);
    lcd_display("CNRS-LPL-IG-UP13");
    //SysCtlDelay(10000000);  //37.5ns * 50000000 = 1,875 s
    Lcd_Cmd(0x94);
    lcd_display("service");
    Lcd_Cmd(0xd2);
    lcd_display("Electronique");
    //SysCtlDelay(5000000);  //37.5ns * 50000000 = 1,875 s
    SysCtlDelay(20000000);  //37.5ns * 50000000 = 1,875 s
    Lcd_Clear();
    SysCtlDelay(400000);
    Lcd_Cmd(0x80);
    lcd_display("Rack DDS AD9852");
    Lcd_Cmd(0xc0);
    lcd_display("   CH1,2,3,4");
    Lcd_Cmd(0x94);
    lcd_display(" PLLx17");
    Lcd_Cmd(0xd2);
    lcd_display(" Clk=340MHz");
    SysCtlDelay(20000000);  //37.5ns * 50000000 = 1,875 s
    //SysCtlDelay(10000000);  //37.5ns * 50000000 = 1,875 s
    GPIOPinConfigure(GPIO_PF0_EN0LED0);
    GPIOPinConfigure(GPIO_PF4_EN0LED1);
    GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    //PH2 = 0x04;  //CLR DAC
    //PH1 = 0x00;  //LDAC = 0 the LDAC pin is not required, and it must be connected to 0 V permanently.
    uint32_t ui32User0 = 0x00B61A00;
    uint32_t ui32User1 = 0x00740200;
    uint8_t pui8MACArray[8];
    SysCtlDelay(1000);
    IntEnable(INT_GPION);
    IntEnable(INT_GPIOB);
    //Enable Peripheral SSI0 single bit SPI
    init_SPI0_mode_single_bit();  // data send to SPI0 PORT @60Mbs

    // Pin N0 setup
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);        // Enable port N
    //GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, PN0);  // Init PN0 as input
    //GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, PN1);  // Init PN1 as input
    // Interrupt setup
    //GPIOIntDisable(GPIO_PORTN_BASE, PN0);        // Disable interrupt for PN0 (in case it was enabled)
    //GPIOIntClear(GPIO_PORTN_BASE, PN0);      // Clear pending interrupts for PN0
    //GPIOIntRegister(GPIO_PORTN_BASE, PortNIntHandler);     // Register our handler function for port D
    //GPIOIntTypeSet(GPIO_PORTN_BASE, PN0,GPIO_RISING_EDGE); // Configure PN0 for falling edge trigger
    //GPIOIntEnable(GPIO_PORTN_BASE, PN0);     // Enable interrupt for PN0


    // Pin B0 setup
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);        // Enable port B
    //GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, PB0);
    //GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, PB2);

    // Interrupt setup
    //GPIOIntDisable(GPIO_PORTB_BASE, PB0);        // Disable interrupt for PB0 (in case it was enabled)
    //GPIOIntClear(GPIO_PORTB_BASE, PB0);      // Clear pending interrupts for PB0
    //GPIOIntRegister(GPIO_PORTB_BASE, PortBIntHandler);     // Register our handler function for port D
    //GPIOIntTypeSet(GPIO_PORTB_BASE, PB0,GPIO_RISING_EDGE); // Configure PB0 for falling edge trigger
    //GPIOIntEnable(GPIO_PORTB_BASE, PB0);     // Enable interrupt for PB0

    //SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);
    //lire les valeurs stockées en mémoire du uC//
    //read_byte =  *(unsigned long*)FLASH0;
    //read_flash = read_byte;

    // Enable processor interrupts.
    IntMasterEnable();
    //SysCtlDelay(10000);
    //SPI Settings DAC11001A
    //Mode: Mode-1 (CPOL: 0, CPHA: 1)
    //Select VREF, TnH mode (Good THD), LDAC mode and power-up the DAC  0100: Reference span = 10 V ± 1.25 V
    //ssi0PutData(0x02,0x004D00,0x03);
    //ssi0PutData(0x02,0x000D00,3); // bit [9:6] = 0100: Reference span = 10 V ± 1.25 V
    //SysCtlDelay(20);
    //ssi0PutData(0x01,0x00000<<4,3);   // 0v out
    //SysCtlDelay(20000);
    //ssi0PutData(0x01,102600<<4,0x03);   // 102600
    //SysCtlDelay(20000);
    //ssi0PutData(0x01,102601<<4,0x03);   // 102601

    //Write zero code to the DAC
    //ssi0PutData(0x01,read_flash<<4,0x03);   //

    // Configure the device pins.
    //
    //PinoutSet(true, false);

    //
    // Configure debug port for internal use.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);

    //
    // Clear the terminal and print a banner.
    //
    UARTprintf("\033[2J\033[H");
    UARTprintf("Ethernet IO start\n\n");
    UARTprintf("IP Address: ");

    //
    // Configure SysTick for a periodic interrupt = 1Hz
    //
    //SysTickPeriodSet(g_ui32SysClock / SYSTICKHZ);
    SysTickPeriodSet(g_ui32SysClock / 1);
    SysTickEnable();
    SysTickIntEnable();
    //
    // Tell the user what we are doing just now.
    //
    UARTprintf("Waiting for IP.\n");

    //
    // Convert the 24/24 split MAC address from NV ram into a 32/16 split
    // MAC address needed to program the hardware registers, then program
    // the MAC address into the Ethernet Controller registers.
    //

    pui8MACArray[0] = ((ui32User0 >>  0) & 0xff);
    pui8MACArray[1] = ((ui32User0 >>  8) & 0xff);
    pui8MACArray[2] = ((ui32User0 >> 16) & 0xff);
    pui8MACArray[3] = ((ui32User1 >>  0) & 0xff);
    pui8MACArray[4] = ((ui32User1 >>  8) & 0xff);
    pui8MACArray[5] = ((ui32User1 >> 16) & 0xff);

    //
    // Initialze the lwIP library, using DHCP.
    //
    lwIPInit(g_ui32SysClock, pui8MACArray, 0, 0, 0, IPADDR_USE_DHCP);

    //
    // Setup the device locator service.
    //
    LocatorInit();
    LocatorMACAddrSet(pui8MACArray);
    LocatorAppTitleSet("EK-TM4C1294XL enet_io");

    //
    // Initialize a sample httpd server.
    //
    httpd_init();

    // Set the interrupt priorities.  We set the SysTick interrupt to a higher
    // priority than the Ethernet interrupt to ensure that the file system
    // tick is processed if SysTick occurs while the Ethernet handler is being
    // processed.  This is very likely since all the TCP/IP and HTTP work is
    // done in the context of the Ethernet interrupt.
    //
    IntPrioritySet(INT_EMAC0, ETHERNET_INT_PRIORITY);
    IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);
    IntPrioritySet(INT_GPION, GPION_INT_PRIORITY);
    IntPrioritySet(INT_GPIOB, GPION_INT_PRIORITY);

    //
    // Pass our tag information to the HTTP server.
    //
    http_set_ssi_handler((tSSIHandler)SSIHandler, g_pcConfigSSITags,NUM_CONFIG_SSI_TAGS);

    //
    // Pass our CGI handlers to the HTTP server.
    //
    http_set_cgi_handlers(g_psConfigCGIURIs, NUM_CONFIG_CGI_URIS);

    //
    // Initialize IO controls
    //
    io_init();

    UARTprintf("demarrage OK : program pass\n");
    UARTprintf("demarrage OK : start\n");

    PH0 =0x01; // reset DDS
    SysCtlDelay(1000);
    PH0 =0x00;

    // CS DDS 1
    PB1 =0x00,PB0 = 0x01,PL6 = 0x40,PB3 = 0x08;
    //lire les valeurs stockées en mémoire du uC//
    read_byte =  *(long long *)FLASH0;
    read_flash = read_byte;
    read_byte5 =  *(int *)FLASH5;
    read_flash5 = read_byte5;
    ssi0PutData(PLL_ADRESS,PLL,PLL_NUM_BYTE);
    ssi0PutData(OSK_ADRESS,read_flash5,OSK_NUM_BYTE);
    ssi0PutData(FTW_ADRESS,(long long)read_flash,FTW_NUM_BYTE);

    SysCtlDelay(1000);

    // CS DDS 2
    PB1 =0x02,PB0 = 0x00,PL6 = 0x40,PB3 = 0x08;
    //lire les valeurs stockées en mémoire du uC//
    read_byte1 =  *(long long *)FLASH1;
    read_flash1 = read_byte1;
    read_byte6 =  *(int *)FLASH6;
    read_flash6 = read_byte6;
    ssi0PutData(PLL_ADRESS,PLL,PLL_NUM_BYTE);
    ssi0PutData(OSK_ADRESS,read_flash6,OSK_NUM_BYTE);
    ssi0PutData(FTW_ADRESS,(long long)read_flash1,FTW_NUM_BYTE);

    SysCtlDelay(1000);

    // CS DDS 3
    PB1 =0x02,PB0 = 0x01,PL6 = 0x00,PB3 = 0x08;
    //lire les valeurs stockées en mémoire du uC//
    read_byte2 =  *(long long *)FLASH2;
    read_flash2 = read_byte2;
    read_byte7 =  *(int *)FLASH7;
    read_flash7 = read_byte7;
    ssi0PutData(PLL_ADRESS,PLL,PLL_NUM_BYTE);
    ssi0PutData(OSK_ADRESS,read_flash7,OSK_NUM_BYTE);
    ssi0PutData(FTW_ADRESS,(long long)read_flash2,FTW_NUM_BYTE);

    SysCtlDelay(1000);

    // CS DDS 4
    PB1 =0x02,PB0 = 0x01,PL6 = 0x40,PB3 = 0x00;
    //lire les valeurs stockées en mémoire du uC//
    read_byte4 =  *(long long *)FLASH3;
    read_flash4 = read_byte4;
    read_byte8 =  *(int *)FLASH8;
    read_flash8 = read_byte8;
    ssi0PutData(PLL_ADRESS,PLL,PLL_NUM_BYTE);
    ssi0PutData(OSK_ADRESS,read_flash8,OSK_NUM_BYTE);
    ssi0PutData(FTW_ADRESS,(long long)read_flash4,FTW_NUM_BYTE);

   while(1){

        //
        // Wait for a new tick to occur.
        //
        while(!g_ulFlags)
        {
            // Do nothing.
            //
        }
//

        //
        // Clear the flag now that we have seen it.
        //
        HWREGBITW(&g_ulFlags, FLAG_TICK) = 0;

        };// Loop forever echoing data through the UART.

}
