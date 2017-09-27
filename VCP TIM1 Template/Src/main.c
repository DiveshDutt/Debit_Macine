/* FILE        : main.c
 PROJECT       : PROG8125
 PROGRAMMER    : Divesh Dutt
 FIRST VERSION : 2017-07-18
 DESCRIPTION   : This Project is used to implement debit machine if the user enter the correct pin
 and correct password he/she gets the access of account and display the remaining money in his account.
 ********************************************************************************
 FILE          : main.c
 PROJECT       : PROG8125
 PROGRAMMER    : R. Hofer
 FIRST VERSION : 2016-02-0
 REVISED:		 : 2017-01-23 - Changed to run on the Nucleo-32 and in TrueSTUDIO
 DESCRIPTION   : Demonstrates a debit machine banking
 transaction that implements a state
 machine.
 State machine diagram is found on slide 14 of
 Week 9 Switch Statement and State Machine.pptx
 in k:\Rhofer\PROG8125

 Switches are assigned as follows
 PA0			PA1			PA4			PA3
 chequing		savings		ok			cancel

 Note: Don't use PA2 as it is connected to VCP TX and you'll
 lose printf output ability.
 */
/*******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "debounce.h"
#include "HD44780.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static const int16_t chequingPbPin = 0; //setting the pin assigned to each pb
static const int16_t savingsPbPin = 1;		//don't use pin 2 as it's connected
static const int16_t okPbPin = 4;		//to VCP TX
static const int16_t cancelPbPin = 3;
enum pushButton {
	None, chequing, savings, ok, cancel
};
//enumerated values for use with if
//(pbPressed == value) type conditional
//statements

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HD44780_Init();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// FUNCTION      : setTone
// DESCRIPTION   : Calculates the PWM Period needed to obtain the freq
//				 : passed and the duty cycle of the PAM to
//				 : 50% (1/2 of the period)
// PARAMETERS    : None
// RETURNS       : nothing
void setTone(int32_t freq) {
	int32_t pwmPeriod = 1000000000 / (freq * 250); //value can vary between 2 and 65535
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = pwmPeriod;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pwmPeriod / 2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	/* adding this as ST Tech Support said PWM should be stopped before
	 * calling HAL_TIM_PWM_ConfigChannel and I've been getting flakey start-up
	 * i.e.: sometime PWM starts up, other times the line remains stuck high.
	 **************************************/
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	/*************************************/
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	HAL_TIM_MspPostInit(&htim1);
}

// FUNCTION      : waitForPBRelease
// DESCRIPTION   : Loops until the PB that is currently
//				 : pressed and at a logic low
//				 : is released. Release is debounced
// PARAMETERS    : pin and port
// RETURNS       : nothing
void waitForPBRelease(const int16_t pin, const char port) {
	while (deBounceReadPin(pin, port, 10) == 0) {
		//do nothing wait for key press to be released
	}
}

// FUNCTION      : startUpLCDSplashScreen()
// DESCRIPTION   : displays Debit Demo for 2s
//                 on line 1 of the display and
//				 : Disappears
// PARAMETERS    : None
// RETURNS       : nothing
void startUpLCDSplashScreen(void) {
	char stringBuffer[16] = { 0 };
	HD44780_GotoXY(0, 0);
	snprintf(stringBuffer, 16, "   Debit Demo");
	HD44780_PutStr(stringBuffer);
	HAL_Delay(2000);
	HD44780_ClrScr();
}

// FUNCTION      : pulsePWM
// DESCRIPTION   : Turns on the PWM for the pulseTime in ms
//                 provided and then turns off PWM
// PARAMETERS    : address of Timer Handle var (e.g.: &htim1) and pulseTime in ms
// RETURNS       : nothing
void pulsePWM(TIM_HandleTypeDef* htim1, int32_t pulseTime) {
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_Delay(pulseTime);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
}

//  FUNCTION      : pushButtonInit
//   DESCRIPTION   : Calls deBounceInit to initialize ports that
//                   will have pushbutton on them to be inputs.
//			         intitializing PA0,PA1,PA4 and PA3
//                   Switches are assigned as follows
//                   PA0			PA1			PA4			PA3
//                   chequing		savings		ok			cancel
//
//                   Note: Don't use PA2 as it is connected to VCP TX and you'll
//                   lose printf output ability.
//   PARAMETERS    : None
//   RETURNS       : nothing
void pushButtonInit() {
	deBounceInit(chequingPbPin, 'A', 1); 		//1 = pullup resistor enabled
	deBounceInit(savingsPbPin, 'A', 1); 		//1 = pullup resistor enabled
	deBounceInit(okPbPin, 'A', 1); 			//1 = pullup resistor enabled
	deBounceInit(cancelPbPin, 'A', 1); 		//1 = pullup resistor enabled
}

// FUNCTION      : displayWelcome()
// DESCRIPTION   : clears the LCD display and displays
//                 Welcome on line 1 of the display
// PARAMETERS    : None
// RETURNS       : nothing
void displayWelcome() {
	char stringBuffer[16] = { 0 };
	HD44780_ClrScr();
	snprintf(stringBuffer, 16, "Welcome ! ");
	HD44780_PutStr(stringBuffer);

}

// FUNCTION      : displayAmount()
// DESCRIPTION   : clears the LCD display and displays
//                 the $amount received on line 1 of the display
// PARAMETERS    : float - amount to display
// RETURNS       : nothing
void displayAmount(float amount) {
	char stringBuffer[16] = { 0 };
	HD44780_ClrScr();
	snprintf(stringBuffer, 16, "$%.2f", amount);
	HD44780_PutStr(stringBuffer);
}

// FUNCTION      : checkIfAmountRecd()
// DESCRIPTION   : accept the Amount entered by user
// PARAMETERS    : none
// RETURNS       : float, the amount in $ to be debited
float checkIfAmountRecd() {
	float debitAmount = 0;
	printf("waiting for debit Amount to be received on serial port\r\n");
	printf("Enter the Amount");
	HD44780_GotoXY(0, 0);
	displayWelcome();
	HD44780_GotoXY(0, 1);
	HD44780_PutStr("Enter Amount");
	int16_t result = 0;
	result = scanf("%f", &debitAmount);
	if (result == 0)		//then somehow non-float chars were entered
			{						//and nothing was assigned to %f
		fpurge(stdin); 	//clear the last erroneous char(s) from the input stream
	}
	return debitAmount;
}

// FUNCTION      : checkOkOrCancel()
// DESCRIPTION   : Checks whether the OK or Cancel
//                 button has been pressed.
// PARAMETERS    : none
// RETURNS       : int8_t, 3 if cancel pressed, 4 if ok
//                 ok pressed. 0 returned if neither
//                 has pressed.
int8_t checkOkOrCancel() {
	if (deBounceReadPin(cancelPbPin, 'A', 10) == 0) {
		//then the cancel pushbutton has been pressed
		return cancel;
	} else if (deBounceReadPin(okPbPin, 'A', 10) == 0) {
		//then ok pressed
		return ok;

	} else {
		return None; //as ok or cancel was not pressed.
	}
}
int8_t checkorSaving() {
	if (deBounceReadPin(chequingPbPin, 'A', 10) == 0) {
		return chequing;
	}
	if (deBounceReadPin(savingsPbPin, 'A', 10) == 0) {
		return savings;
	}
	if (deBounceReadPin(cancelPbPin, 'A', 10) == 0) {
		return cancel;
	}
	return None;     // as chequing,savings and cancel none is pressed
}
// FUNCTION      : displayOkOrCancel()
// DESCRIPTION   : displays "OK or Cancel?" on line 2 of LCD
// PARAMETERS    : none
// RETURNS       : none.
void displayOkCancel() {
	char stringBuffer[16] = { 0 };
	HD44780_GotoXY(0, 1); //move to second line first position
	snprintf(stringBuffer, 16, "OK or Cancel?");
	HD44780_PutStr(stringBuffer);
}
// FUNCTION      : displayCheckOrSaving()
// DESCRIPTION   : display Check Or Saving on  LCD
// PARAMETERS    : none
// RETURNS       : none.
void displayCheckOrSaving() {
	char StringBuffer[16] = { 0 };
	char stringBuffer[20] = { 0 };
	HD44780_GotoXY(0, 0);
	snprintf(StringBuffer, 16, "Check or");
	HD44780_PutStr(StringBuffer);
	HD44780_GotoXY(0, 1);
	snprintf(StringBuffer, 16, "Saving");
	HD44780_PutStr(stringBuffer);
}
// FUNCTION      : displayTransactionCancelled()
// DESCRIPTION   : this function dispalys the Transaction Cancelled on LCD
// PARAMETERS    : none
// RETURNS       : none.
void displayTransactionCancelled() {
	printf("Transaction Cancelled \r\n");
	HD44780_ClrScr();
	HD44780_GotoXY(0, 0);
	HD44780_PutStr("Transaction");
	HD44780_GotoXY(0, 1);
	HD44780_PutStr("Cancelled");
	HAL_Delay(1000);
	HD44780_ClrScr();
}
// FUNCTION      :displayTransactionApproved()
// DESCRIPTION   : this function dispalys the Approve on LCD
// PARAMETERS    : none
// RETURNS       : none
void displayTransactionApproved() {
	printf("transaction Approved");
	HD44780_GotoXY(0, 0);
	HD44780_PutStr("Transcation");
	HD44780_GotoXY(0, 1);
	HD44780_PutStr("Approved");
	HAL_Delay(1000);
}

/* USER CODE END 0 */
int main(void) {
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	printf("Debit Card State Machine\r\n");
	HD44780_Init();
	/* setup Port A bits 0,1,2 and 3, i.e.: PA0-PA3 for input */
	pushButtonInit();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
		int i = 0;                        //use as an index in char Pin[20]
		char Pin[20] = { 0 }; //use to store the Pin number entered from user using getchar();
		char balance[30] = { 0 };          //used to print the remaining balance
		static float Balance = 15000;      // default balance of the user
		static float enteredCode = 0;       //debit pin code entered by user
		static float amount = 0;           //used to hold the transaction amount
		static int8_t transactionState = 1;  //initialise the state from state 1
		int8_t pbPressed = 0;  //will hold pushbutton defined above depending on
							   //the pushbutton pressed
		int Account;           //hold the account Number enter by user
		int32_t buffer[10] = { 0 };
		int32_t AccountNumber[10] = { 111, 2222, 3333, 4444, 5555, 6666, 7777,
				8888, 9999, 0000 }; //account number of the user should be from that array
		/*states:   1   display Welcome Screen, wait for $ amount input from
		 Serial port
		 2   @ amount Received, waiting for Ok or Cancel button
		 3   OK receivepinCoded, waiting for chequing or Savings button
		 4   C or S received, waiting for PIN to be entered from
		 Serial Port
		 5   Pin Correct, send transaction data to bank. Waiting
		 for OK back from Bank If OK from Bank received. Print
		 Reciept, Record transaction. Move back to State 1.
		 6   Cancel Pressed. Display "Transaction Cancelled" back to
		 state 1
		 */

		switch (transactionState) {
		case 1: //check if an entered Account number is in the database if yes ask for amount and display it
			putchar("Enter Account Number");
			scanf("%i", &Account);
			for (int i = 0; i < 10; i++) {
				if (Account = AccountNumber[i]) {
					amount = checkIfAmountRecd();
					if (amount != 0)  //returns a 0 if an transaction amount has
							{ 			//NOT been received on the serial port.
						displayAmount(amount); //but if we're we've received a debitAmount
						displayOkCancel(); //so display it and the prompt ok or cancel
						transactionState = 2; //and do that before we move on to state 2
					}
				}
			}

			break;
		case 2: 	//amount has been received waiting for Ok or cancel pressed
			pbPressed = checkOkOrCancel();

			if (pbPressed != 0) {
				if (pbPressed == cancel) {
					//then cancel was pressed.
					printf("Cancel Pressed\r\n");
					HD44780_ClrScr();
					HD44780_GotoXY(0, 0);

					transactionState = 6;
				}
				if (pbPressed == ok) {
					//then ok pressed
					printf("OK Pressed\r\n");

					transactionState = 3;
					HD44780_ClrScr();
				}

			}
			break;
		case 3:
			//ask for check or saving on lcd and wait for it
			displayCheckOrSaving();
			pbPressed = checkorSaving();
			if (pbPressed != 0) {
				if (pbPressed == savings) {
					//savings is pressed
					printf("savings pressed\r\n");
					transactionState = 4;
					HAL_Delay(1000);

				}
				if (pbPressed == chequing) {
					//chequing is pressed
					printf("chequing pressed\r\n ");
					transactionState = 4;
				}
				if (pbPressed == cancel) {
					//cancel is pressed
					transactionState = 6;
				}

			}
			break;
		case 4:                      //ask for user pin using getchar function
			HD44780_ClrScr();
			HD44780_GotoXY(0, 0);
			HD44780_PutStr("Enter Pin");

			pbPressed = checkOkOrCancel();
			printf("Enter your Pin\r\n");
			for (int j = 0; j < 5; j++) {
				while ((Pin[i] = getchar()) != '\n') { //get the char on serial port using getchar()
					i++;                    //get the data till enter is pressed
					getchar();
				}
				HD44780_ClrScr();
				transactionState = 5;
			}
			break;
		case 5: // match the code if it is incorrect display incorrect pin on lcd but if correct display
			if (enteredCode == 4554) {           // incorrect Pin

				Balance = Balance - amount;
				snprintf(balance, 30, "Balance is %.2d"); //calculates the remaining balance of user and display it on LCD
				HD44780_PutStr(balance);
				transactionState = 1;
			} else {
				HD44780_ClrScr(); //displays incorrect pin on LCD if pin is pressed other than 4554
				HD44780_GotoXY(0, 0);
				HD44780_PutStr("Incorect Pin");
				transactionState = 6;
			}
			break;
		case 6:
			displayTransactionCancelled(); //displays transaction Cancelled on LCD
			transactionState = 1;
			break;

		} 	//closing brace for switch
	} //closing brace for while(1)
//		/* USER CODE END 3 */
} //closing brace for main

/** System Clock Configuration **/
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_ADC;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}

	/**Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 9090;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 4045;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	/* adding this as ST Tech Support said PWM should be stopped before
	 * calling HAL_TIM_PWM_ConfigChannel and I've been getting flakey start-up
	 * i.e.: sometime PWM starts up, other times the line remains stuck high.
	 **************************************/
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	/*************************************/
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim1);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}

}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

	/*Configure GPIO pin : PB3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
