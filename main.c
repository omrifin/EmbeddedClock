/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:   PIC18 or PIC24 USB Microcontrollers
 Hardware:    The code is natively intended to be used on the following
		hardware platforms: PICDEM? FS USB Demo Board,
		PIC18F87J50 FS USB Plug-In Module, or
		Explorer 16 + PIC24 USB PIM.  The firmware may be
		modified for use on other USB platforms by editing the
		HardwareProfile.h file.
 Complier:    Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:   Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the ?Company?) for its PIC? Microcontroller is intended and
 supplied to you, the Company?s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN ?AS IS? CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
					 coding style
********************************************************************/


//	========================	INCLUDES	========================
#ifdef _VISUAL
#include "VisualSpecials.h"
#endif // VISUAL

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include "mtouch.h"
#include "BMA150.h"
#include "oled.h"
#include "soft_start.h"
#include "OledGraphics.h"


//	========================	CONFIGURATION	========================

#if defined(PIC18F46J50_PIM)
   //Watchdog Timer Enable bit:
#pragma config WDTEN = OFF          //WDT disabled (control is placed on SWDTEN bit)
//PLL Prescaler Selection bits:
#pragma config PLLDIV = 3           //Divide by 3 (12 MHz oscillator input)
//Stack Overflow/Underflow Reset Enable bit:
#pragma config STVREN = ON            //Reset on stack overflow/underflow enabled
//Extended Instruction Set Enable bit:
#pragma config XINST = OFF          //Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
//CPU System Clock Postscaler:
#pragma config CPUDIV = OSC1        //No CPU system clock divide
//Code Protection bit:
#pragma config CP0 = OFF            //Program memory is not code-protected
//Oscillator Selection bits:
#pragma config OSC = ECPLL          //HS oscillator, PLL enabled, HSPLL used by USB
//Secondary Clock Source T1OSCEN Enforcement:
#pragma config T1DIG = ON           //Secondary Oscillator clock source may be selected
//Low-Power Timer1 Oscillator Enable bit:
#pragma config LPT1OSC = OFF        //Timer1 oscillator configured for higher power operation
//Fail-Safe Clock Monitor Enable bit:
#pragma config FCMEN = OFF           //Fail-Safe Clock Monitor disabled
//Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit:
#pragma config IESO = OFF           //Two-Speed Start-up disabled
//Watchdog Timer Postscaler Select bits:
#pragma config WDTPS = 32768        //1:32768
//DSWDT Reference Clock Select bit:
#pragma config DSWDTOSC = INTOSCREF //DSWDT uses INTOSC/INTRC as reference clock
//RTCC Reference Clock Select bit:
#pragma config RTCOSC = T1OSCREF    //RTCC uses T1OSC/T1CKI as reference clock
//Deep Sleep BOR Enable bit:
#pragma config DSBOREN = OFF        //Zero-Power BOR disabled in Deep Sleep (does not affect operation in non-Deep Sleep modes)
//Deep Sleep Watchdog Timer Enable bit:
#pragma config DSWDTEN = OFF        //Disabled
//Deep Sleep Watchdog Timer Postscale Select bits:
#pragma config DSWDTPS = 8192       //1:8,192 (8.5 seconds)
//IOLOCK One-Way Set Enable bit:
#pragma config IOL1WAY = OFF        //The IOLOCK bit (PPSCON<0>) can be set and cleared as needed
//MSSP address mask:
#pragma config MSSP7B_EN = MSK7     //7 Bit address masking
//Write Protect Program Flash Pages:
#pragma config WPFP = PAGE_1        //Write Protect Program Flash Page 0
//Write Protection End Page (valid when WPDIS = 0):
#pragma config WPEND = PAGE_0       //Write/Erase protect Flash Memory pages starting at page 0 and ending with page WPFP[5:0]
//Write/Erase Protect Last Page In User Flash bit:
#pragma config WPCFG = OFF          //Write/Erase Protection of last page Disabled
//Write Protect Disable bit:
#pragma config WPDIS = OFF          //WPFP[5:0], WPEND, and WPCFG bits ignored

#else
#error No hardware board defined, see "HardwareProfile.h" and __FILE__
#endif



//	========================	Global VARIABLES	========================
#pragma udata
//You can define Global Data Elements here

 static int hours=0, minutes=0, seconds=0;


//	========================	PRIVATE PROTOTYPES	========================
static void InitializeSystem(void);
static void ProcessIO(void);
static void UserInit(void);
static void YourHighPriorityISRCode();
static void YourLowPriorityISRCode();

void mainMenu();
void clearScreen();
void showDigital();
BOOL CheckButtonPressed(void);
BOOL initSecondaryDigitalClock();
void initClockConfig();
void updateTime();

//	========================	VECTOR REMAPPING	========================
#if defined(__18CXX)
  //On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
  //the reset, high priority interrupt, and low priority interrupt
  //vectors.  However, the current Microchip USB bootloader 
  //examples are intended to occupy addresses 0x00-0x7FF or
  //0x00-0xFFF depending on which bootloader is used.  Therefore,
  //the bootloader code remaps these vectors to new locations
  //as indicated below.  This remapping is only necessary if you
  //wish to program the hex file generated from this project with
  //the USB bootloader.  If no bootloader is used, edit the
  //usb_config.h file and comment out the following defines:
  //#define PROGRAMMABLE_WITH_SD_BOOTLOADER

#if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
#define REMAPPED_RESET_VECTOR_ADDRESS     0xA000
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0xA008
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0xA018
#else 
#define REMAPPED_RESET_VECTOR_ADDRESS     0x00
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0x08
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0x18
#endif

#if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
extern void _startup(void);        // See c018i.c in your C18 compiler dir
#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
void _reset(void)
{
	_asm goto _startup _endasm
}
#endif
#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
void Remapped_High_ISR(void)
{
	_asm goto YourHighPriorityISRCode _endasm
}
#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
void Remapped_Low_ISR(void)
{
	_asm goto YourLowPriorityISRCode _endasm
}

#if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
//Note: If this project is built while one of the bootloaders has
//been defined, but then the output hex file is not programmed with
//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
//As a result, if an actual interrupt was enabled and occured, the PC would jump
//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
//would effective reset the application.

//To fix this situation, we should always deliberately place a 
//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
//hex file of this project is programmed with the bootloader, these sections do not
//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
//programmed using the bootloader, then the below goto instructions do get programmed,
//and the hex file still works like normal.  The below section is only required to fix this
//scenario.
#pragma code HIGH_INTERRUPT_VECTOR = 0x08
void High_ISR(void)
{
	_asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
}
#pragma code LOW_INTERRUPT_VECTOR = 0x18
void Low_ISR(void)
{
	_asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
}
#endif  //end of "#if defined(||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER))"

#pragma code

//	========================	Application Interrupt Service Routines	========================
  //These are your actual interrupt handling routines.
#pragma interrupt YourHighPriorityISRCode

void YourHighPriorityISRCode()
{
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.
	seconds++;
	if (INTCONbits.T0IF)
	{
		

	if (seconds == 60)
	{
		minutes += 1;
		seconds = 0;
	}
	if (minutes == 60)
	{
		hours += 1;
		minutes = 0;
	}
	if (hours == 24)
	{
		hours = 0;
		minutes = 0;
		seconds = 0;
	}
		
		TMR0H = 0x48 ;				//Write 0xCE51 to 16 bit Timer0
		TMR0L = 0xE3 ;
		INTCONbits.T0IF = 0 ;		//Re'arm Timer0 Interrupt
	}
}



#pragma interruptlow YourLowPriorityISRCode
void YourLowPriorityISRCode()
{
	//Check which interrupt flag caused the interrupt.
	//Service the interrupt
	//Clear the interrupt flag
	//Etc.

} //This return will be a "retfie", since this is in a #pragma interruptlow section 
#endif




//	========================	Board Initialization Code	========================
#pragma code
#define ROM_STRING rom unsigned char*

/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the application code
 *                  initialization that is required.
 *
 * Note:
 *
 *****************************************************************************/
void UserInit(void)
{

	// Soft Start the APP_VDD
	while (!AppPowerReady())
		;

	/* Initialize the mTouch library */
	mTouchInit();

	/* Call the mTouch callibration function */
	mTouchCalibrate();

	/* Initialize the accelerometer */
	InitBma150();

	/* Initialize the oLED Display */
	ResetDevice();
	FillDisplay(0x00);
	
}//end UserInit


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
#if (defined(__18CXX) & !defined(PIC18F87J50_PIM))
	ADCON1 |= 0x0F;                 // Default all pins to digital


#if defined(PIC18F87J50_PIM) || defined(PIC18F46J50_PIM)
//On the PIC18F87J50 Family of USB microcontrollers, the PLL will not power up and be enabled
//by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
//This allows the device to power up at a lower initial operating frequency, which can be
//advantageous when powered from a source which is not gauranteed to be adequate for 48MHz
//operation.  On these devices, user firmware needs to manually set the OSCTUNE<PLLEN> bit to
//power up the PLL.
	{
		unsigned int pll_startup_counter = 600;
		OSCTUNEbits.PLLEN = 1;  //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
		while (pll_startup_counter--);
	}
	//Device switches over automatically to PLL output after PLL is locked and ready.
#endif

#if defined(PIC18F46J50_PIM)
//Configure all I/O pins to use digital input buffers.  The PIC18F87J50 Family devices
//use the ANCONx registers to control this, which is different from other devices which
//use the ADCON1 register for this purpose.
	ANCON0 = 0xFF;                  // Default all pins to digital
	ANCON1 = 0xFF;                  // Default all pins to digital
	T0CON = 0B01010001 ;			//Clock init
#endif

//  The USB specifications require that USB peripheral devices must never source
//  current onto the Vbus pin.  Additionally, USB peripherals should not source
//  current on D+ or D- when the host/hub is not actively powering the Vbus line.
//  When designing a self powered (as opposed to bus powered) USB peripheral
//  device, the firmware should make sure not to turn on the USB module and D+
//  or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//  firmware needs some means to detect when Vbus is being powered by the host.
//  A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
//  can be used to detect when Vbus is high (host actively powering), or low
//  (host is shut down or otherwise not supplying power).  The USB firmware
//  can then periodically poll this I/O pin to know when it is okay to turn on
//  the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//  peripheral device, it is not possible to source current on D+ or D- when the
//  host is not actively providing power on Vbus. Therefore, implementing this
//  bus sense feature is optional.  This firmware can be made to use this bus
//  sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//  HardwareProfile.h file.    
#if defined(USE_USB_BUS_SENSE_IO)
	tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

//  If the host PC sends a GetStatus (device) request, the firmware must respond
//  and let the host know if the USB peripheral device is currently bus powered
//  or self powered.  See chapter 9 in the official USB specifications for details
//  regarding this request.  If the peripheral device is capable of being both
//  self and bus powered, it should not return a hard coded value for this request.
//  Instead, firmware should check if it is currently self or bus powered, and
//  respond accordingly.  If the hardware has been configured like demonstrated
//  on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//  currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//  is used for this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//  has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
//  to it in HardwareProfile.h.
#if defined(USE_SELF_POWER_SENSE_IO)
	tris_self_power = INPUT_PIN;  // See HardwareProfile.h
#endif

	UserInit();

}//end InitializeSystem
<<<<<<< HEAD



//	========================	Application Code	========================

void ProtectedoledPutROMString(rom unsigned char *ptr,unsigned char page, unsigned char col)
=======
 
 
 
//  ========================    Application Code    ========================
 
 
/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
void mainTraverse(int c);

BOOL CheckButtonPressed(void)
>>>>>>> 449baffe0e2dd5d01f1fc8ee45eb8b86e5762c7b
{
	INTCONbits.T0IE = 0 ;			//Timer0 Overflow Interrupt Disable
	oledPutROMString( ptr, page, col ) ;
	INTCONbits.T0IE = 1 ;			//Timer0 Overflow Interrupt Enabled
}

void initClockConfig()
{


// Initialize Timer 0
	T0CON = 0x07 ;
// Initialize Timer Interrupt
	RCONbits.IPEN = 1 ;			//Prio Enable
	INTCON2bits.TMR0IP = 1 ;	//Use Hi-Prio
	INTCON = 0xE0 ;				//Enable Timer Interrupt

	T0CON |= 0x80 ;				//Start the Timer



//========================================================
T0CONbits.T08BIT = 0 ;			//Timer0 16BIT COUNTER
	T0CONbits.T0CS = 0 ;			//Clock Source -- Internal
	T0CONbits.PSA = 0 ;				//Use Pre-Scaler
//	T0CONbits.T0PS = 1 ;			//Prescale 1:4
	T0CONbits.TMR0ON = 1 ;			//Set Timer to ON
	RCONbits.IPEN = 1 ;				//Use Priority Interrutps
	INTCON2bits.T0IP = 1 ;			//Timer0 High-Priority

	INTCONbits.GIE = 1 ;			//Enable Interrupts
	INTCONbits.PEIE = 1 ;
	INTCONbits.T0IE = 1 ;			//Timer0 Overflow Interrupt Enabled


}

void subMenu1()
{
	
    static char toprint[24];
	unsigned char RA1 = '1',RA2 = '2';
	int currChoice = 1, i;
	clearScreen();
	initSecondaryDigitalClock();
	while(1) {
		
		sprintf(toprint,"Sub menu 1");
		oledPutString(toprint, 0, 0, 1);  
		
		for(i = 1; i < 6; i++)
		{
			if(i == 1) 
				sprintf(toprint, "operation 1");
			if(i == 2) 
				sprintf(toprint, "Back");
			if(i != 2 && i != 1) 
				sprintf(toprint, "subMenu %d", i);
			if(i == currChoice)
				oledPutString(toprint, i, 2 * 6, 0);
			else 
				oledPutString(toprint, i, 2 * 6, 1);
		}
		
		if( CheckUDVolt(mTouchReadButton(RA1), mTouchReadButton(RA2)) == 1)    //Pressed up           
		{
			if(currChoice > 1) 
				currChoice--;
		}
		if(CheckUDVolt(mTouchReadButton(RA1), mTouchReadButton(RA2)) == 2) //Pressed down
		{
			if(currChoice < 5) 
				currChoice++;
		}
		if(CheckButtonPressed())
		{
		
			if(currChoice == 2)
			{
				clearScreen();
				showDigital();
			}
			if(currChoice == 1)
			{
				sprintf(toprint, "op x executed");
				oledPutString(toprint, 1, 2 * 6, 1);
				DelayMs(500);
				mainMenu();
			}
		}
		DelayMs(20);
	}
<<<<<<< HEAD
=======
    else if (y<x && x-y>75 && y < 600){ 
		return 2;}       //Down
   else return 0;   //Not pushed
}
 
 
int GetAccVal(char c){  //Check the accelerometer's readings
/*
x: msb 3 lsb 2
y: msb 5 lsb 4
z: msb 7 lsb 6
*/
BYTE msb,lsb;
BYTE mask= 0b10000000;
int signextend= 0xFC00;//2^15+2^14+2^13+2^12+2^11+2^10+2^9
int val=0,n1,n2;
    if(c=='x'){n1=3;n2=2;}
    if(c=='y'){n1=5;n2=4;}
    if(c=='z'){n1=7;n2=6;}
    msb=BMA150_ReadByte(n1);
    lsb=BMA150_ReadByte(n2);
    lsb=lsb>>6;
    val+=(int)msb;
    val=val<<2;
    val+=(int)lsb;
    mask= mask&msb;
    if(mask== 0b10000000){
        val|=signextend;
    }
    return val;
}
int truev(int x){
    if(x<0) return -x;
    else return x;
>>>>>>> 449baffe0e2dd5d01f1fc8ee45eb8b86e5762c7b
}

BOOL CheckButtonPressed(void)
{
	static char buttonPressed = FALSE;
	static unsigned long buttonPressCounter = 0;

	if (PORTBbits.RB0 == 0)
	{
		if (buttonPressCounter++ > 5)
		{
			buttonPressCounter = 0;
			buttonPressed = TRUE;
		}
	}
	else
	{
		if (buttonPressed == TRUE)
		{
			if (buttonPressCounter == 0)
			{
				buttonPressed = FALSE;
				return TRUE;
			}
			else
			{
				buttonPressCounter--;
			}
		}
	}

	return FALSE;
}




	//	========================	Our clock code	========================

// void mainTraverse(int c) {
// 	switch(c){
// 		case 1: 
// 			subMenu1();
// 			break;
// 		//case 2: subMenu2();break;
// 		//case 3: subMenu3();break;
// 		//case 4: subMenu4();break;
// 		default: 
// 			break;
// 	}
// }

void clearScreen() {
	int i;
<<<<<<< HEAD
	for(i = 1; i < 8; i++)
	{
		oledPutROMString("                      ", i, 0); 
	}
}

int CheckUDVolt(unsigned int x, unsigned int y){
    if(x < y && y - x > 75 && x < 800)
    {
		DelayMs(50);
        return 1;       //Up
	}
    else if (y < x && x - y > 75 && y < 800)
	{ 
		DelayMs(50);
		return 2;
	}       //Down
    else 
	   return 0;   //Not pushed
}

void mainMenu()
{  
    static char toprint[24];
	unsigned char RA1 = '1', RA2 = '2';
	int currChoice = 1, i;
	
	
	clearScreen();
	initSecondaryDigitalClock();
	while(1) {
	  
	   
	for(i = 1; i < 7; i++)
	{
		if(i == 1) 
			sprintf(toprint, "Display Mode");
		if(i == 2) 
			sprintf(toprint, "12H / 24H Interval");
		if(i == 3) 
			sprintf(toprint, "Set Time");
		if(i == 4) 
			sprintf(toprint, "Set Date");
		if(i == 5) 
			sprintf(toprint, "Set Alarm");
		if(i == 6) 
			sprintf(toprint, "Exit (Back)");
		if(i == currChoice)
			oledPutString(toprint, i, 2 * 6, 0);
		else if (i!=currChoice)
			oledPutString(toprint, i, 2 * 6, 1);
	}

	if(CheckUDVolt(mTouchReadButton(RA1), mTouchReadButton(RA2)) == 1)    //Pressed up           
	{
		if(currChoice > 1) {
			currChoice--;
		}
	}
	if(CheckUDVolt(mTouchReadButton(RA1), mTouchReadButton(RA2)) == 2) //Pressed down
	{
		if(currChoice < 7) 
			currChoice++;
	}
		
	if(CheckButtonPressed())
	{
		
		if(currChoice == 6)
		{
			clearScreen();
			showDigital();
		}

		if(currChoice==3)	//Set time
		{
			clearScreen();
		//	setTime();

		}	
=======
	for(i=1;i<8;i++){
		oledPutROMString("                      ", i, 0); 
	}
}
void subMenu1()
{
    static char toprint[24];
    int i;
	unsigned char RA1='1',RA2='2';
	int currChoice=1;
while(1){
	clearScreen();
    sprintf(toprint,"Sub menu 1");
    oledPutString(toprint, 0, 0,1);  
   
 	
    for(i=1;i<6;i++)
    {
		if(i==5) sprintf(toprint, "operation 1");
	    else sprintf(toprint, "submenu %d",i);
	    if(i == currChoice)oledPutString(toprint, i ,2*6,0);
	   	else oledPutString(toprint, i ,2*6,1);
		
    }

    if( CheckUDVolt(mTouchReadButton(RA1),mTouchReadButton(RA2))==1)    //Pressed up           
{
	if(currChoice > 1) currChoice--;
>>>>>>> 449baffe0e2dd5d01f1fc8ee45eb8b86e5762c7b

		if(currChoice == 1)
		{
			sprintf(toprint, "op x executed");
			oledPutString(toprint, 1, 2 * 6, 1);
			DelayMs(500);
			subMenu1();
		}
	}
	DelayMs(20);
	}
}	

BOOL initSecondaryDigitalClock()	//might get current computer's clock
{
<<<<<<< HEAD
    static char toprint[64];
    sprintf(toprint, "%02d:%02d:%02d", hours, minutes, seconds);
    oledPutString(toprint,0,80);
	return 1;

}



void showDigital()
{
	
	static char toprint[64];
	while(1)
	{
		DelayMs(80);
		//FillDisplay(0x00);
		sprintf(toprint, "%02d:%02d:%02d", hours, minutes, seconds);;
    	oledPutStringPrimary(toprint, 10, 0);
		sprintf(toprint, "18/02");
		oledPutString(toprint, 15, 90);		

		if(CheckButtonPressed())
		{
			clearScreen();
			mainMenu();
		}
		showDigital();
		//DelayMs(10);	
	}
}


void updateTime()
{
	
	
	


}


/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
void main(void)
{

	InitializeSystem();
	initClockConfig();
	showDigital();
	
	while (1)							//Main is Usualy an Endless Loop
	{
		//advanceClock()
		
		
	}
}//end main


=======
 	if(currChoice < 5) currChoice++;
}
	if(CheckButtonPressed())
{
	mainTraverse(currChoice);
}
 DelayMs(20);
}
}

void mainTraverse(int c){
	switch(c){
		case 1: subMenu1();break;
		//case 2: subMenu2();break;
		//case 3: subMenu3();break;
		//case 4: subMenu4();break;
		default: break;
	}
}
void mainMenu()
{  
    static char toprint[24];
    int i;
	unsigned char RA1='1',RA2='2';
	int currChoice=1;
while(1){
	clearScreen();
    sprintf(toprint,"Main menu");
    oledPutString(toprint, 0, 0,1);  
   
 	
    for(i=1;i<5;i++)
    {
	    sprintf(toprint, "submenu %d",i);
	    if(i == currChoice)oledPutString(toprint, i ,2*6,0);
	   	else oledPutString(toprint, i ,2*6,1);
    }

    if( CheckUDVolt(mTouchReadButton(RA1),mTouchReadButton(RA2))==1)    //Pressed up           
{
	if(currChoice > 1) currChoice--;

} 
    if( CheckUDVolt(mTouchReadButton(RA1),mTouchReadButton(RA2))==2) //Pressed down
{
 	if(currChoice < 4) currChoice++;
}
	if(CheckButtonPressed())
{
	mainTraverse(currChoice);
}
 DelayMs(10);
}
}
 

void main(void)
{
    InitializeSystem();
    mainMenu();
    while(1)                            //Main is Usualy an Endless Loop
    {
 
    }      
}//end main
 
 
/** EOF main.c *************************************************/
>>>>>>> 449baffe0e2dd5d01f1fc8ee45eb8b86e5762c7b
