//##################################################
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//       Controller c-file edited by Laskar
//            Original file from EE502
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//##################################################

#include "PeripheralHeaderIncludes.h"
#include "DSP2803x_EPWM_defines.h"
#include "IQmathLib.h"

#include "Ctrl_A.h"
#include "Ctrl_ASM.h"
#include "PWM_1ch_CNF.h"
#include "ADC_SOC_CNF.h"
#include "nRF24L01P.h"
#include "PWM_1ch_CNF.h"
//#include "SFO_V6.h" // comment for disable the SFO and HRPWM
#include "Settings.h"

__interrupt void ePWM_ISR(void);
__interrupt void nRF_RX(void);
//__interrupt void Timer_ISR(void); // PFC OK ; comment this interrupt

//====================================================================
// The following declarations are required in order to use the SFO
// library functions:
//

// ============================ Comment for disable the SFO and HRPWM-START ============================

//int MEP_ScaleFactor; // Global variable used by the SFO library
//                     // Result can be used for all HRPWM channels
//                     // This variable is also copied to HRMSTEP
//                     // register by SFO(0) function.
//// General System variables - useful for debug

// ============================= Comment for disable the SFO and HRPWM-END =============================

Uint16 status;
//====================================================================

Uint32 Data = 0;// get for nRF
Uint32 nRF_VCount = 0;    // ADCINA1
Uint32 nRF_ICount = 0;    // ADCINA0
Uint32 nRF_Count = 0;     // nRF Count
Uint32 Bug_Count = 0;     // Bug Count
//Uint32 ePWM_Count = 0;  //
Uint16 ID = 0;            // nRF data type

Uint16 Index = 2;

long Vbus = _IQ24(0.0);     // DC Bus Voltage
long VbusFilt = _IQ24(0.0); // Bus Voltage Filter

volatile long   nRF_IADC     = _IQ24(0.0);      // nRF Result
long            nRF_IADCFilt = _IQ24(0.0);      // nRF Result Filter

volatile long   nRF_VADC     = _IQ24(0.0);      // nRF Result
long            nRF_VADCFilt = _IQ24(0.0);      // nRF Result Filter

// Variable for CV-CC Feature
Uint16 CC_Mode = 0;
Uint16 CV_Mode = 1;

Uint16 Soft_Start = 0;  // Soft start variable
Uint16 DCDC_Start = 1;  // Soft start variable
Uint16 ePWM_ON = 1;     // Starting EPWM
Uint16 nRF_ERROR = 0;   // nRF Error Indicator
Uint16 PFC_OK = 0;      // PFC Indicator
Uint16 Load = 0;		// 0: Ele Load	1: Battery

Uint16 UpdatePWM = 1;   //
Uint16 nRF = 1;         // nRF transmit/receive status
Uint16 nRF_Start = 0;	// 0: CV Mode Start	1: CC Mode Start

#define PWM_CH 5
volatile struct EPWM_REGS *ePWM[PWM_CH] =
                          {&EPwm1Regs, &EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs};

// ------------------- Module Variables (mr) -------------------
// PWMDRV_4ch_HiResUpDwnCntFreq 1
// volatile long mr_Period;

// ADCDRV_1ch 0
// volatile long ADCDRV_A0;

// ============================= Function and variable for CNTL_2P2Z ==============================
// Declare the net nodes/variables being used by the DP Lib Macro here

#pragma DATA_SECTION(CNTL_2P2Z_CoefStruct1, "CNTL_2P2Z_Coef");
struct CNTL_2P2Z_CoefStruct CNTL_2P2Z_CoefStruct1;
long mr_CNTL_2P2Z_IRef; // Current Reference
long mr_CNTL_2P2Z_VRef; // Voltage Reference
long C2P2Z_Out;         // Controller Output

long IRef[25] = {	_IQ24(0.02999997139), _IQ24(0.05499994755),
					_IQ24(0.07499998808), _IQ24(0.09999996424),
					_IQ24(0.11999994520), _IQ24(0.13999998570),
					_IQ24(0.15999996660), _IQ24(0.18999999760),
					_IQ24(0.21999996900), _IQ24(0.25000000000),
					_IQ24(0.26999998090), _IQ24(0.29299998280),
					_IQ24(0.31999999280), _IQ24(0.33399999140),
					_IQ24(0.34299993520), _IQ24(0.35799998040),
					_IQ24(0.36799998040), _IQ24(0.36999999520),
					_IQ24(0.37999999520), _IQ24(0.40999996660),
					_IQ24(0.43999999760), _IQ24(0.47999995950),
					_IQ24(0.50999997140), _IQ24(0.51999997140),
					_IQ24(0.52999997140)						};

// 500W _IQ24(0.5099999714):FullSync,	_IQ24(0.5299999714):HalfSync,	_IQ24(0.5199999714):FullBridge

// ========================= Function and variable for CNTL_2P2Z END =============================

// ========================= Parameters Passed to ADC_SOC_Cnf Configuration =============================
int ADCCnf_ChSel[16]		= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};                    // ADC Channel
int ADCCnf_TrigSel[16]		= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};                    // ADC Trigger Event
int ADCCnf_ACQPS[16]		= {25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25};	// 0x19 = Sample window is 15 cycles long (14 + 1 clock cycles).
// ======================= Parameters Passed to ADC_SOC_Cnf Configuration END ===========================

void error(void)
{
    ESTOP0;         // Stop here and handle error

	EALLOW;         // Enable protected register access
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;		// 0=GPIO,  1=EPWM1A,  2=Resv,  3=Resv
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;		// 0=GPIO,  1=EPWM2A,  2=Resv,  3=Resv
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;		// 0=GPIO,  1=EPWM3A,  2=Resv,  3=Resv
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;		// 0=GPIO,  1=EPWM4A,  2=SYNCI, 3=SYNCO
	EDIS;	        // Disable register access
}

int PWM_Init(void)
{
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	EDIS;

	PWM_1chHiResUpDwnCnt_CNF(1,300,1,0,0);
	PWM_1chHiResUpDwnCnt_CNF(2,300,0,0,1);
	PWM_1chHiResUpDwnCnt_CNF(3,300,0,0,1);
	PWM_1chHiResUpDwnCnt_CNF(4,300,0,0,0);

	EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;          // Enable TBCLK within the EPWM
    EPwm1Regs.TBCTL.bit.SWFSYNC = 1;                // Synchronize high resolution phase to start HR period (Last step in HR initialization)
	EDIS;

	return 0;
}

// ============================ Comment for disable the SFO and HRPWM - START ============================

//void SFO_Init(void)
//{
//	// Calling SFO() updates the HRMSTEP register with calibrated MEP_ScaleFactor.
//	// HRMSTEP must be populated with a scale factor value prior to enabling
//	// high resolution period control.
//
//	status = SFO_INCOMPLETE;
//    while(status == SFO_INCOMPLETE)
//    {
//    	// Call until complete
//        status = SFO();
//        if (status == SFO_ERROR)
//        {
//            error();    // SFO function returns 2 if an error occurs & # of MEP steps/coarse step
//        }              // exceeds maximum of 255.
//    }
//}

// ============================= Comment for disable the SFO and HRPWM - END =============================

int ADC_Init(void)
{

	ADCCnf_ChSel[0]   = 0;	                                        // Channel A0
	ADCCnf_TrigSel[0] = ADCTRIG_EPWM1_SOCA;                         // Trigger Event = 5
	ADCCnf_ACQPS[0]   = 6;	                                        // Sample window is 7 cycles long (6 + 1 clock cycles).

	ADC_SOC_CNF(ADCCnf_ChSel, ADCCnf_TrigSel, ADCCnf_ACQPS, 0, 0);  // ADC Initialization Function

	// ---------------------------- Disable PWM for Master if no Voltage/Current Sense (ADC) Needed - START ----------------------------
	//EPwm1Regs.ETSEL.bit.SOCAEN = 1;				// Enable EPWMxSOCA pulse
	//EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;	// These bits determine when a EPWMxSOCA pulse will be generated ; TBCLK = CMPB
	//EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;			// Generate the EPWMxSOCA pulse on the first event
	//EPwm1Regs.CMPA.half.CMPA = 10;
	// ----------------------------- Disable PWM for Master if no Voltage/Current Sense (ADC) Needed - END -----------------------------
	return 0;
}


int SPI_Init(void)								// LSPCLK = 15 MHz
{
	// Initialize SPI FIFO registers
	SpiaRegs.SPICCR.bit.SPISWRESET = 0; 		// Reset SPI
	SpiaRegs.SPICCR.bit.SPICHAR = 15;			// 16-bit character
	SpiaRegs.SPICCR.bit.SPILBK = 0;				// Disable Loop-back mode
	SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;

	SpiaRegs.SPICTL.bit.SPIINTENA = 0;
	SpiaRegs.SPICTL.bit.TALK = 1;				// Master/Slave Transmit Enable
	SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;		// SPI configured as a master
	SpiaRegs.SPICTL.bit.CLK_PHASE = 1;

	SpiaRegs.SPISTS.all = 0x0000;				// Clear SPI status register
	SpiaRegs.SPIBRR = 5;						// Baud rate = 30 MHz / 5 = 6 MHz
	SpiaRegs.SPIFFCT.all = 0;					// FIFO transmit delay 0 SPI_CLK

	SpiaRegs.SPIFFTX.bit.TXFFIL = 4;			// TXFFIL4  transmit FIFO interrupt level bits
	SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;			// SPI FIFO enhancements enable
	SpiaRegs.SPIFFTX.bit.SPIRST = 1;			// SPI reset

	SpiaRegs.SPIFFRX.bit.RXFFIL = 4;			// Set RX FIFO level to 2
	SpiaRegs.SPIPRI.bit.FREE = 0;				// Free run, continue SPI operation regardless of suspend or when the suspend occurred
	SpiaRegs.SPIPRI.bit.TRIWIRE = 0;			// Normal 4-wire SPI mode

	SpiaRegs.SPICCR.bit.SPISWRESET = 1; 		// Enable SPI

	SpiaRegs.SPIFFTX.bit.TXFIFO = 1;
	SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;

	return 0;
}


int ISR_Init(void)
{
	DINT;
	EALLOW;

	PieVectTable.EPWM1_INT = &ePWM_ISR;
	PieVectTable.XINT3 = &nRF_RX;
//	PieVectTable.TINT0 = &Timer_ISR;

	GpioIntRegs.GPIOXINT3SEL.bit.GPIOSEL = 12;  // XINT3 is GPIO12

	EDIS;                                       // This is needed to disable write to EALLOW protected registers

	EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;	// Trigger on CTR ZERO ( Up ) ; These bits determine when a EPWMxSOCA pulse will be generated ; TBCLK = CMPB
	EPwm1Regs.ETSEL.bit.INTEN = 1;				// Enable ePWM Interrupt (EPWM2_INT) Generation										Literature SPRUGE9E, Table 63
	EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;			// Generate pulse on every event ; Generate an interrupt on the first event		    Literature SPRUGE9E, Table 64
	EPwm1Regs.ETCLR.bit.INT = 1;	            // Clear Interrupt Flag																Literature SPRUGE9E, Table 66

	XIntruptRegs.XINT3CR.bit.POLARITY = 2;      // Falling Edge interrupt

	return 0;
}


void Timers_Init(void)
{
	InitCpuTimers();
	ConfigCpuTimer(&CpuTimer0, 60, 80000);		// 60MHz CPU Frequency ; 1 second Period (in uSeconds)
	CpuTimer0Regs.TCR.all = 0x4000;				// Use write-only instruction to set TSS bit = 0
}


int ISR_Start(void)
{
	DINT;

	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
//	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;		    // Enable TINT0 in the PIE: Group 1 interrupt 7
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;		    // Enable ePWM1 Interrupt (INT3.1)
	PieCtrlRegs.PIEIER12.bit.INTx1 = 1;         // Enable XINT3 (INT12.1)

//	IER = IER | M_INT1;		    			    // Enable interrupt group 1 (TINT0)
	IER = IER | M_INT3; 				        // Enable interrupt group 3 (ePWM)
	IER = IER | M_INT12; 				        // Enable interrupt group 12

	XIntruptRegs.XINT3CR.bit.ENABLE = 1;        // Enable XINT3

//	PieCtrlRegs.PIEACK.bit.ACK1  = 1;
	PieCtrlRegs.PIEACK.bit.ACK3  = 1;
	PieCtrlRegs.PIEACK.bit.ACK12 = 1;

	EINT;
	ERTM;

	return 0;
}


// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Main Control Function - START %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Ctrl(void)
{
	PWM_Init();
//	SFO_Init();
	ADC_Init();
	Ctrl_Init();
	ISR_Init();
//	Timers_Init();
	SPI_Init();
	nRF_Init();
	EALLOW;

	// HRPWM MEP Steps Value ; Typically around 93 for 60 MHz Clock
	EPwm1Regs.HRMSTEP = 90;
	EPwm2Regs.HRMSTEP = 90;
	EPwm3Regs.HRMSTEP = 90;
	EPwm4Regs.HRMSTEP = 90;

	EDIS;


	// PWMDRV_1ch_UpDwnCntFreq 1
	PWMDRV_4ch_HiResUpDwnCntFreq_Period1 = &C2P2Z_Out;				//Q24 ; Control the PWM period from the Controller Output

	// ADCDRV_1ch 0
//	ADCDRV_1ch_Rlt0 = &ADCDRV_A0;									//Q24

// ------------------------------ Pointer for CNTL_2P2Z 1 -----------------------------------

	//change----------
//	CNTL_2P2Z_Ref1 = &mr_CNTL_2P2Z_IRef;				//Q24 ; For the Current Reference
	CNTL_2P2Z_Ref1 = &mr_CNTL_2P2Z_VRef;                //Q24 ; For the Voltage Reference
	//change----------

	//change----------
//	CNTL_2P2Z_Fdbk1 = &nRF_IADC;						//Q24 ; For the Current Feedback
	CNTL_2P2Z_Fdbk1 = &nRF_VADC;                        //Q24 ; For the Voltage Feedback
	//change----------

	CNTL_2P2Z_Out1 = &C2P2Z_Out;						// Pointer for the Controller Output
	CNTL_2P2Z_Coef1 = &CNTL_2P2Z_CoefStruct1.b2;        // Pointer for b2 Coefficient

// ================== Initialization Controller Module Variables (mr) START ==================
	// ADCDRV_1ch 1
//	ADCDRV_A0 = _IQ24(0.0);

	// CNTL_2P2Z 1 Parameter Setting

	CNTL_2P2Z_CoefStruct1.b2 = _IQ26( 0.0);

//	CNTL_2P2Z_CoefStruct1.b1 = _IQ26( 0.00001 );
//	CNTL_2P2Z_CoefStruct1.b0 = _IQ26( 0.00001 );

	CNTL_2P2Z_CoefStruct1.b1 = _IQ26( 0.00015 );
	CNTL_2P2Z_CoefStruct1.b0 = _IQ26( 0.0001 );

	CNTL_2P2Z_CoefStruct1.a2 = _IQ26( 0.0);
//	CNTL_2P2Z_CoefStruct1.a1 = _IQ26( 1.5 );
	CNTL_2P2Z_CoefStruct1.a1 = _IQ26( 1.0);

	// Maximum Period means Minimum Frequency --------------------------------------
//	CNTL_2P2Z_CoefStruct1.max = _IQ14( 370 );       // for minimum frequency : 81kHz
	CNTL_2P2Z_CoefStruct1.max = _IQ14( 360 );       // for minimum frequency : 83kHz

	// Minimum Period means Maximum Frequency --------------------------------------
	CNTL_2P2Z_CoefStruct1.i_min = _IQ14( 328 );
//	CNTL_2P2Z_CoefStruct1.i_min = _IQ24( -0.9 );    // from DSP forum
	CNTL_2P2Z_CoefStruct1.min = _IQ14( 328 );       // for maximum frequency : 91kHz

	// Set Controller Reference ( Current / Voltage )
	// Current Reference -----------------------------------------------------------
//	mr_CNTL_2P2Z_IRef = _IQ24(0.189904749);         // for I reference : 3.0A
	mr_CNTL_2P2Z_IRef = _IQ24(0.298023223);         // for I reference : 5.0A
//	mr_CNTL_2P2Z_IRef = _IQ24();                    // for I reference : 7.6A
	// Voltage Reference -----------------------------------------------------------
	mr_CNTL_2P2Z_VRef = _IQ24(0.619628906);	        // for V reference : 400V
//	mr_CNTL_2P2Z_VRef = _IQ24(0.1591796875);        // for V reference : 100V
//	mr_CNTL_2P2Z_VRef = _IQ24(0.097412109);         // for V reference : 60V

	// Controller Output -----------------------------------------------------------
	C2P2Z_Out = _IQ14(333);

	// Feedback Initialization Value -----------------------------------------------
	nRF_IADC = _IQ24(0.0);                          // Current Feedback
	nRF_VADC = _IQ24(0.0);                          // Voltage Feedback

// =================== Initialization Controller Module Variables (mr) END ===================

	ISR_Start();

	while(1)
// =================================== While(1) Loop - START ===================================
	{
		AdcRegs.ADCSOCFRC1.bit.SOC0 = 1;

// ---------------------------------- NRF Reading Data START ----------------------------------

		if(nRF)     // Start Read data from nRF Enable Status
		{
			GpioDataRegs.GPATOGGLE.bit.GPIO20 = 1;  // GPIO20 set

			ID = SPI_Rx_Read();                     // Read SPI from the nRF Module
			while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
			Data = SpiaRegs.SPIRXBUF;
			Data <<= 16;
			while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
			Data |= SpiaRegs.SPIRXBUF;
			Data >>= 8;
			// convert 16bit into 32bit

			if(ID == 0x00AA)    // 0xAA = nRF_IADC for the Current Sense
			{
				nRF_IADC = Data;
				nRF_VCount++;

				if(DCDC_Start)
				{
					UpdatePWM = 1;
				}

				nRF_IADCFilt = _IQ24mpy(_IQ24(0.98),nRF_IADCFilt) + _IQ24mpy(_IQ24(0.02),nRF_IADC);

//===================================== CC-CV START =======================================
// --------------------- Comment for only Constant Voltage Mode START ---------------------

				if( (nRF_IADC > CV_Target) && (CV_Mode == 1) )    // For Starting the Constant Current Mode
				{
					CV_Mode = 0;
					CC_Mode = 1;

				    // Set Current Controller Parameter
				    CNTL_2P2Z_Ref1            = &mr_CNTL_2P2Z_IRef;     // for Current Reference
				    CNTL_2P2Z_Fdbk1           = &nRF_IADC;              // for Current Feedback
//				    CNTL_2P2Z_CoefStruct1.min = _IQ14( 328 );           // for maximum frequency : 91kHz
				}

// ---------------------- Comment for only Constant Voltage Mode END ----------------------
//====================================== CC-CV END ========================================

			}

			else

			if(ID == 0x00EA)    // 0xEA = nRF_VADC for the Voltage Sense
			{
				nRF_VADC = Data;
				nRF_ICount++;

				if(DCDC_Start)
				{
					UpdatePWM = 1;
				}

				nRF_VADCFilt = _IQ24mpy(_IQ24(0.98),nRF_VADCFilt) + _IQ24mpy(_IQ24(0.02),nRF_VADC);

//===================================== CC-CV START =======================================
// --------------------- Comment for only Constant Voltage Mode START ---------------------

                if( (nRF_VADC > CC_Target) && (CV_Mode == 0) )
                {
                    CV_Mode = 1;
                    CC_Mode = 0;

                    // Set Voltage Controller Parameter
                    CNTL_2P2Z_Ref1            = &mr_CNTL_2P2Z_VRef;     // for Voltage Reference
                    CNTL_2P2Z_Fdbk1           = &nRF_VADC;              // for Voltage Feedback
//                  CNTL_2P2Z_CoefStruct1.min = _IQ14( 328 );           // for maximum frequency : 91kHz
                }

// ---------------------- Comment for only Constant Voltage Mode END ----------------------
//====================================== CC-CV END ========================================

			}

// ---------- start comment for only Current and Voltage info from nRF ----------

//			else if(ID == 0x00B5)
//			{
//				GpioDataRegs.GPATOGGLE.bit.GPIO24 = 1;
//				if(Index != 24)
//				{
//					Index++;
//					mr_CNTL_2P2Z_IRef = IRef[Index];
//					CNTL_2P2Z_Ref1 = &mr_CNTL_2P2Z_IRef;
//				}
//
//			}
//			else if(ID == 0x005B)
//			{
//				GpioDataRegs.GPATOGGLE.bit.GPIO24 = 1;
//				if(Index != 0)
//				{
//					Index--;
//					mr_CNTL_2P2Z_IRef = IRef[Index];
//					CNTL_2P2Z_Ref1 = &mr_CNTL_2P2Z_IRef;
//				}
//			}

// ---------- finish comment for only Current and Voltage info from nRF ----------

			else
			{
				Bug_Count++; // Count Bug from nRF
			}
			nRF_Count++;
			nRF = 0;
			SPI_Rx_DR();
		}
// ----------------------------------- NRF Reading Data END -----------------------------------


//		Vbus = AdcResult.ADCRESULT0;
//		Vbus <<= 12;
//		VbusFilt = _IQ24mpy(_IQ24(0.98),VbusFilt) + _IQ24mpy(_IQ24(0.02),Vbus);
//
//		{
//			if(DCDC_Start)
//			{
//				nRF_ERROR = 1;
//			}
//		}
//
//		if(nRF_IADCFilt > nRF_IADCOVP)
//		{
//			if(DCDC_Start)
//			{
//				nRF_ERROR = 1;
//			}
//		}
//
//		if(nRF_VADCFilt > nRF_VADCOCP)
//		{
//			if(DCDC_Start)
//			{
//				nRF_ERROR = 1;
//			}
//		}

// ============================ Comment for disable the SFO and HRPWM - START ============================

		// Call the scale factor optimizer lib function SFO()
		// periodically to track for any change due to temp/voltage.
		// This function generates MEP_ScaleFactor by running the
		// MEP calibration module in the HRPWM logic. This scale
		// factor can be used for all HRPWM channels. HRMSTEP
		// register is automatically updated by the SFO function.

//		status = SFO(); // in background, MEP calibration module continuously updates MEP_ScaleFactor
//		if (status == SFO_ERROR)
//		{
//			error();   // SFO function returns 2 if an error occurs & # of MEP steps/coarse step
//		}              // exceeds maximum of 255.
//		if (status == SFO_COMPLETE)
//		{
//
//		}

// ============================= Comment for disable the SFO and HRPWM - END =============================

	}

// ==================================== While(1) Loop - END ====================================

}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Main Control Function - END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




__interrupt void nRF_RX(void)
{
	nRF = 1;
	nRF_Start = 1;
	PieCtrlRegs.PIEACK.bit.ACK12 = 1;
}

__interrupt void ePWM_ISR(void)
{
	if(UpdatePWM == 1)
	{
		Ctrl_ISR();
		UpdatePWM = 0;
	}

// ============================ Comment for disable the SFO and HRPWM - START ============================

//	if(CV_Mode == 1 && CC_Mode == 1)										// CC_Mode to CV_Mode
//	{
//		CNTL_2P2Z_Ref1 = &mr_CNTL_2P2Z_IRef;
//
//	    //change----------
//  	CNTL_2P2Z_Fdbk1 = &nRF_IADC;
//	    CNTL_2P2Z_Fdbk1 = &nRF_VADC;
//	    //change----------
//
//		CC_Mode = 0;
//	}

// ============================= Comment for disable the SFO and HRPWM - END =============================

	if(ePWM_ON)
	{
		EALLOW;                                 // Enable protected register access
		GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;		// 0=GPIO,  1=EPWM1A,  2=Resv,  3=Resv
		GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;		// 0=GPIO,  1=EPWM2A,  2=Resv,  3=Resv
		GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;		// 0=GPIO,  1=EPWM3A,  2=Resv,  3=Resv
		GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;		// 0=GPIO,  1=EPWM4A,  2=SYNCI, 3=SYNCO
		EDIS;	                                // Disable register access
		ePWM_ON = 0;
	}

	if(nRF_ERROR)
	{
		EALLOW;                                 // Enable protected register access
		GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;		// 0=GPIO,  1=EPWM1A,  2=Resv,  3=Resv
		GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;		// 0=GPIO,  1=EPWM2A,  2=Resv,  3=Resv
		GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;		// 0=GPIO,  1=EPWM3A,  2=Resv,  3=Resv
		GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;		// 0=GPIO,  1=EPWM4A,  2=SYNCI, 3=SYNCO
		EDIS;	                                // Disable register access
	}

//	GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
	EPwm1Regs.ETCLR.bit.INT = 1;
	PieCtrlRegs.PIEACK.bit.ACK3 = 1;
}

// ========================== Interrupt for Integrating PFC and Soft Starting - START ==========================

//__interrupt void Timer_ISR(void)
//{
//	if(!Soft_Start && !DCDC_Start)
//	{
//		if(VbusFilt > Soft_Start_TH)
//		{
//			PFC_OK++;
//		}
//		if(PFC_OK > 50)
//		{
//			if(nRF_Start == 1)							// Battery Load
//			{
//				Load = 1;
//				//GpioDataRegs.GPADAT.bit.GPIO24 = 1;
//				CC_Mode = 1;
//				CV_Mode = 0;
//			}
//			else										// Ele Load
//			{
//				Load = 0;
//				//GpioDataRegs.GPADAT.bit.GPIO24 = 0;
//				CC_Mode = 1;
//				CV_Mode = 1;
//			}
//			Soft_Start = 1;
//			ePWM_ON = 1;
//		}
//	}
//
//	else
//
//	if(Soft_Start && !DCDC_Start)          // Active when Soft_Start = 1 and DCDC_Start = 0
//	{
//		if(C2P2Z_Out == _IQ14( 171.79))         // Set nRF Error when Controller Output reach some value
//		{
//			nRF_ERROR = 1;
//			PieCtrlRegs.PIEIER1.bit.INTx7 = 0;	// Enable TINT0 in the PIE: Group 1 interrupt 7
//		}
//		else
//		{
//			UpdatePWM = 1;
//		}
//
//		if(nRF_Count > 500)                     // nRF Count reach 500 for Soft Starting
//		{
//			DCDC_Start = 1;
//			Soft_Start = 0;
//		}
//	}
//
//	if(!Soft_Start && DCDC_Start)               // DCDC Start and no Soft Starting
//	{
//		PieCtrlRegs.PIEIER1.bit.INTx7 = 0;		// Enable TINT0 in the PIE: Group 1 interrupt ;7
//	}
//
//	PieCtrlRegs.PIEACK.bit.ACK1 = 1;
//}

// =========================== Interrupt for Integrating PFC and Soft Starting - END ===========================




