/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);

static void spi_tx(void);
static void ttn_init(const uint16_t rst_dur, const uint16_t pwm_max);
static void ttn_rowcol(const uint16_t row, const uint16_t col);
static void ttn_ramwrite(const uint16_t vs);
static void ttn_ramread();
static void ttn_pxvo(const uint16_t vs);
static void pxe_ffy(int data);
static void ttn_col(const uint16_t col);
static void ttn_row(const uint16_t row);
static void ttn_pxvoRef(const uint16_t vs);

uint8_t buffrx[1] = {0};
uint8_t bufftx[1] = {0};

uint16_t master_tx[1] = {0};
uint16_t master_rx[1] = {0};

int ttn_cmd = 0;
int n_frame = 0;
int vs = 0;
int read = 0;
int end = 0xFFFF;
int col_now = 0;
uint16_t col_max = 56;
int row_now = 0;
uint16_t row_max = 78;
double temp_avg = 0;
double temp_avg_init = 0;
unsigned int nframe = 0;
unsigned int n_pixel_temp = 0;
unsigned int step;
unsigned int vref;
unsigned int ctr_op = 0;
uint16_t tol = 20;
unsigned int ctr_hi = 0;
unsigned int ctr_low = 0;
unsigned int count_pxon = 0;
unsigned int tmp=0;
unsigned int count = 0;
unsigned int count2 = 0;
uint16_t tempfootprint[988];
signed int tempdac = 0;
signed long long temperr_prev = 0;
unsigned int pid_true = 0;
int tempmax = 600;
unsigned int temp0 = 395;
signed int tempref_dyn = 0;
signed int temperr = 0;
signed int tempP = 0;
int tempPkr = 70;//15//3;
signed int tempI = 0;
signed int tempD = 0;
int tempPir = 1;//5//5;
int tempPd = 150;//10//2;
signed long long temperr_prevD = 0;
int tempmin = 300;
unsigned char btrx = 0;
int tau63[2];
uint16_t vreg = 730;
unsigned int tempavg = 0;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();

  // Start TTN clock
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    // PWM Generation Error
    Error_Handler();
  }

	/*if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
  }

	if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}*/

  HAL_UART_Receive_IT(&huart4, buffrx, 1);

  while (1){
	  switch(ttn_cmd){
			// INITALIZE TTN
      case 1:
        // Turn ON LED
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

        // Reset TTN
        ttn_init(3,50);

        n_frame = 0;

				// Set Vref DAC to 0V
				//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 0);

				// Set Peltier DAC to 0V
				//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, 0);

        // Check TTN is connected and operational using temperature px
        ttn_rowcol(1,1);
				ttn_ramwrite(0);
				vs = 0;
				read = 0;
				// Sweep Vs from 0 to 1023 until vreg reached
				while(vs<1024 && read<vreg){
					// Read pixel at Vs value
					ttn_pxvo(vs);
					vs++;
				}
				vs--;

        // Compare Vs,tau with optimal range
				if(vs>255 && vs<426){
					// ACK and BT string end to Firefly
          bufftx[0] = 'a';
          HAL_UART_Transmit_IT(&huart4, bufftx, 1);
				}
				else{
					// BAD RESULT and BT string end to Firefly
          bufftx[0] = 'n';
          HAL_UART_Transmit_IT(&huart4, bufftx, 1);
				}

        //while(U1STAbits.TRMT==0); what's this for??
        pxe_ffy(end);
        // Operation complete, deassert flag
        ttn_cmd = 0;

        // Turn OFF LED
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
        break;

			// TAKE TEMPERATURE FOOTPRINT
      case 2:
				// Turn on LED
      	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

				// Traverse all temperature px
				for(col_now=1; col_now<col_max; col_now+=3){
						ttn_col(col_now);

					for(row_now=1; row_now<row_max; row_now+=3){
						// Change pixel
						//ttn_rowcol(row, col);
						ttn_row(row_now);
						ttn_ramwrite(0);
						vs = 0;
						read = 0;
						// Sweep Vs from 0 to 1023 until treg reached
						while((vs<1024) && (read < vreg)){
							// Read pixel at Vs value
							ttn_pxvo(vs);
							vs++;
						}
						// Store correct Vs in RAM
						vs--;
						ttn_ramwrite(vs);
						// Send to Firefly
						pxe_ffy(vs);

						// If temp pixel
						if((row_now>27) && (row_now<51) && (col_now>30)){
							temp_avg = temp_avg + vs;
							if (nframe==1){
								n_pixel_temp=vs;
							}
						}
					}
				}

				temp_avg = temp_avg/72;
				temp_avg_init = temp_avg;

				// Operation complete, deassert flag
				ttn_cmd = 0;
				// ACK and BT string end to Firefly
				bufftx[0] = 'b';
				HAL_UART_Transmit_IT(&huart4, bufftx, 1);
				pxe_ffy(end);

				// Turn OFF LED
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			break;

			// SEARCH OPTIMAL REFERENCE ELECTRODE VOLTAGE
			case 3:
			// Turn ON LED
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

			int vmax, vmin;
			vref = 512;
			step = 512; // While-loop divides step by 2 at the start

			// Initialize at midpoint Vref = 0 (i.e. DAC 512/1023)
			ctr_op = 0;
			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, vref);

			// Parse one frame to get initial ctr_op
			for(col_now=0; col_now<col_max; col_now++){
					ttn_col(col_now);

				for(row_now=0; row_now<row_max; row_now++){
					ttn_row(row_now);
					// Change pixel and clear RAM
					// Discharge behaviour of chemical px
					if((col_now%3!=1) || (row_now%3!=1)){
						ttn_ramwrite(0);
						// Read pixel Vout at Vs 0
						ttn_pxvoRef(0);
						vmin = read;
						// Read pixel Vout at Vs 700
						ttn_pxvoRef(800);
						vmax = read;

						// If Px is ON, increment counter
						if(vmin<100 && vmax>(vreg-tol)){
							ctr_op++;
						}
					}
				}
			}

			// Binary search algorithm to maximize # of ON px
			while(step > 4){

				step /= 2;

				ctr_hi = 0;
				//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, vref+step);

				// Parse one frame
				for(col_now=0; col_now<col_max; col_now++){
					ttn_col(col_now);

					for(row_now=0; row_now<row_max; row_now++){
					ttn_row(row_now);
						// Change pixel and clear RAM
						// Discharge behaviour of chemical px
						if((col_now%3!=1) || (row_now%3!=1)){
							ttn_ramwrite(0);
							// Read pixel Vout at Vs 0
							ttn_pxvoRef(0);
							vmin = read;
							// Read pixel Vout at Vs 700
							ttn_pxvoRef(800);
							vmax = read;

							// If Px is ON, increment counter
							if(vmin<100 && vmax>(vreg-tol)){
								ctr_hi++;
							}
						}
					}
				}

				ctr_low = 0;
				//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, vref-step);

				// Parse one frame
				for(col_now=0; col_now<col_max; col_now++){
					ttn_col(col_now);

					for(row_now=0; row_now<row_max; row_now++){
					ttn_row(row_now);
						// Change pixel and clear RAM
						// Discharge behaviour of chemical px
						if((col_now%3!=1) || (row_now%3!=1)){
							ttn_ramwrite(0);
							// Read pixel Vout at Vs 0
							ttn_pxvoRef(0);
							vmin = read;
							// Read pixel Vout at Vs 700
							ttn_pxvoRef(800);
							vmax = read;

							// If Px is ON, increment counter
							if(vmin<100 && vmax>(vreg-tol)){
								ctr_low++;
							}
						}
					}
				}

				// Adjust Vref and step size if current Vref is not optimal
				if(ctr_op<ctr_hi){
					vref += step;
					ctr_op = ctr_hi;
				}
				else if(ctr_op<ctr_low){
					vref -= step;
					ctr_op = ctr_low;
				}
			}

			count_pxon = ctr_op;

			// Set optimal Vref
			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, vref);

			// Store flags to indicate ON/OFF state of px and send to Firefly
			for(col_now=0; col_now<col_max; col_now++){
					ttn_col(col_now);
				for(row_now=0; row_now<row_max; row_now++){
					ttn_row(row_now);
					// Change pixel and clear RAM
					//ttn_rowcol(row, col);

					// Only search on chemical px
					if((col_now%3==1) && (row_now%3==1)){
						// Read RAM value of temperature px
						//ttn_ramread();
						vs = 511;
					}
					else{ // Discharge behaviour of chemical px
						ttn_ramwrite(0);
						// Read pixel Vout at Vs 0
						ttn_pxvoRef(0);
						// ttn_pxvo(0);
						vmin = read;
						//ttn_ramwrite(0);
						// Read pixel Vout at Vs 700
						ttn_pxvoRef(800);
						// ttn_pxvo(700);
						vmax = read;

						// Determine if px is ON/OFF

						// Px ON, write 511 to RAM
						if(vmin<100 && vmax>(vreg-tol)){
							vs = 511;
						}
						// Px discharge too fast, write 0 to RAM
						else if(vmin<100 && vmax<100){
							vs = 0;
						}
						// Px discharge too slow, write 1023 to RAM
						else if(vmin>(vreg-tol) && vmax>(vreg-tol)){
							vs = 1023;
						}
					}

					// Write flags to RAM
					ttn_ramwrite(vs);

					// Send to Firefly
					pxe_ffy(vs);
				}
			}

			// Operation complete, deassert flag
			ttn_cmd = 0;

			// ACK and BT string end to Firefly
			bufftx[0] = 'd';
			HAL_UART_Transmit_IT(&huart4, bufftx, 1);
			pxe_ffy(end);

			// Turn OFF LED
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		break;

		// CALIBRATE ARRAY Vs VALUES
		case 4:
		// Turn ON LED
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

			int vsFlag;

			for(col_now=0; col_now<col_max; col_now++){
					ttn_col(col_now);

				for(row_now=0; row_now<row_max; row_now++){
					ttn_row(row_now);
					// Change pixel and clear RAM
					ttn_ramread();
					vsFlag = read;

					// Only calibrate chemical px UNCOMMENT THIS TO CHECK ACTIVE PIXELS
					//if (vsFlag == 511){
						// Determine if px is ON/OFF
						// Ignore OFF px (RAM == 0 or 1023 from Vref step)
						vs = 0;
						int out1=0;

						// Sweep Vs from 0 to 1023 until vreg reached

						while((vs<1024) && (out1 < (vreg-tol))){

							// Read pixel at Vs value
							ttn_pxvo(vs);
							out1 = read;
							vs++;
						}
						// Correct Vs due to while-loop
						vsFlag = vs-1;
					//}

					// Write Vs to RAM
					ttn_ramwrite(vsFlag);

					// Send to Firefly
					pxe_ffy(vsFlag);
				}
			}

			// Operation complete, deassert flag
			ttn_cmd = 0;
			// ACK and BT string end to Firefly
			bufftx[0] = 'e';
			HAL_UART_Transmit_IT(&huart4, bufftx, 1);

			// Firefly end frame
			pxe_ffy(end);

			// Turn OFF LED
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		break;

		// LOAD TEMPERATURE FOOTPRINT
		case 5:
			// Turn ON LED
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

			HAL_Delay(500);

			count = 0;
			temp_avg = 0;

			// Traverse all temperature px
			for(col_now=1; col_now<col_max; col_now+=3){
					ttn_col(col_now);

				for(row_now=1; row_now<row_max; row_now+=3){
					ttn_row(row_now);
					// Change pixel
					// Store correct Vs in RAM
					vs = (tempfootprint[count] << 8) | tempfootprint[count+1];
					ttn_ramwrite(vs);
					count+=2;

					// If temp pixel
					if((row_now>27) && (row_now<51) && (col_now>30)){
						temp_avg = temp_avg + vs;
						if (nframe==1){
							n_pixel_temp=vs;
						}
					}

				}
			}

			temp_avg = temp_avg/72;
			temp_avg_init = temp_avg;

			// Operation complete, deassert flag
			ttn_cmd = 0;
			// ACK and BT string end to Firefly
			bufftx[0] = 'c';
			HAL_UART_Transmit_IT(&huart4, bufftx, 1);

			// Firefly end frame
			pxe_ffy(end);

			// Turn OFF LED
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		break;

		// REGULATE PELTIER TO 63C
		case 6:
			// Turn ON LED
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

			// Peltier DAC at full power 5V
			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, 1023);

			// Operation complete, but maintain flag ? (Not maintained)
			// ACK and BT string end to Firefly
			bufftx[0] = 'f';
			HAL_UART_Transmit_IT(&huart4, bufftx, 1);
			pxe_ffy(end);
			ttn_cmd = 0;

			// Turn OFF LED
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

			// check this - why turning LED off again?

			// Send to DAC
			tempdac = 600;
			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, tempdac);

			if (count2>20){
				// Turn off LED
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			}
			HAL_Delay(500);

		 stop:  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);		// Turn off LED

		break;

		// ARRAY READOUT
		case 7:

			temperr_prev /= 3;

			if(nframe == 0 || nframe == 1) {
				pid_true = 0;
			}

			// Set optimal Vref
			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, vref);

			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, 0);

			while(ttn_cmd == 7){
				if(ttn_cmd==0){
					break;
				}
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);		// Turn on LED

				// Vref sweep
				if (nframe==6 || nframe==7 || nframe==8) {
					if (vref<900) {
						//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, vref+20);
					}
					else {
						//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, vref-20);
					}
				}
				else if(nframe>8){
					//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, vref);
				}

				count = 0;

				temp_avg = 0;
				if (nframe==1){
					n_pixel_temp = 0;
				}
				for(col_now=0; col_now<col_max; col_now++){
					ttn_col(col_now);

					for(row_now=0; row_now<row_max; row_now++){
					ttn_row(row_now);
						// Chemical px
						// Change pixel
						// Read and clear RAM Vs value
						ttn_ramread();
						vs = read;
						// Intermediate calib Vs of active px
						if(vs!=0 && vs!=1023){
								ttn_pxvo(vs);

								if(read<(vreg-tol) && vs<1020){

									while(read<(vreg-tol) && vs<1020){
										vs++;
										ttn_pxvo(vs);
									}

									vs--;
								}
								else if(read>(vreg+tol) && vs>4){

									while(read>(vreg+tol) && vs>4){
										vs--;
										ttn_pxvo(vs);
									}
									vs++;
								}
						}

						// Store Vs in RAM
						ttn_ramwrite(vs);

						// If temp pixel
						if((col_now%3==1) && (row_now%3==1) && (row_now>27) && (row_now<51) && (col_now>30)){
							temp_avg = temp_avg + vs;

							if (nframe==1){
								n_pixel_temp=vs;
							}
						}

						// Send value to Firefly
						pxe_ffy(vs);
					}
				}

				// Send Vref value
				pxe_ffy(vref);

				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);		// Turn off LED

				tempavg = 0;
				temp_avg = temp_avg/72;
				n_pixel_temp = temp_avg;
				pxe_ffy(temp_avg);

				// Firefly end frame
				pxe_ffy(end);

				if (nframe>10) {

					if(nframe <60) {
						// New DAC value
						tempdac = tempmax;
					}
					else if(nframe <200) {
						// New DAC value
						tempdac = temp0;

						if (nframe == 199) {
							tempref_dyn = temp_avg;
						}
					}
					else {
						// Error
						temperr = - temp_avg + tempref_dyn;

						// Proportional
						tempP = tempPkr*temperr;
						//if(tempP>tempmax-temp0){tempP=tempmax-temp0;}
						//else if(tempP<tempmin-temp0){tempP=tempmin-temp0;}
						//if (tempP < 0) {
						//	tempP = 2*tempP;
						//}

						// Integral
						tempI = tempPir*(temperr+temperr_prev);
						//if(tempI>tempmax-temp0){tempI=tempmax-temp0;}
						//else if(tempI<tempmin-temp0){tempI=tempmin-temp0;}

						// Deriative
						tempD = tempPd*(temperr-temperr_prevD);
						//if(tempD>tempmax-temp0){tempD=tempmax-temp0;}
						//else if(tempD<tempmin-temp0){tempD=tempmin-temp0;}

						// New DAC value
						tempdac = temp0 + tempP + tempD;// + tempI + tempD ;
						if(tempdac>tempmax){tempdac=tempmax;}
						else if(tempdac<tempmin){tempdac=tempmin;}
						temperr_prev += temperr;
						temperr_prevD = temperr;

					}

					tempdac=0;

					// Send to DAC - why tempdac if 0
					//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, tempdac);
				}

				nframe++;
			}

			// Operation complete, deassert flag
			ttn_cmd = 0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);		// Turn off LED

		break;
	  }
  }
	return 0;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void spi_tx(void){
  // Set SS pin low to activate
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  // Transmit instruction over SPI
  if(HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&master_tx[0], (uint8_t*)&master_rx[0], 1, 1000)!=HAL_OK){
    Error_Handler();
  }

  // Check finished
  while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){
  }

  // Set SS pin high to deactivate
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

/* Reset TTN and define durations */
static void ttn_init(const uint16_t rst_dur, const uint16_t pwm_max){
	uint16_t rst_ttn = 8192;
	uint16_t rst_dur_seq = 34048 + rst_dur;
	uint16_t pwm_max_seq = 34304 + pwm_max;

  // Reset TTN
  master_tx[0] = rst_ttn;
  spi_tx();
  HAL_Delay(100);

  // Define reset duration of pixel before measurement
  master_tx[0] = rst_dur_seq;
  spi_tx();

  // Define PWM max width
  master_tx[0] = pwm_max_seq;
  spi_tx();
}

/* Access row and col of TTN */
static void ttn_rowcol(const uint16_t row, const uint16_t col){
	uint16_t row_seq = 49152 + row*16;
	uint16_t col_seq = 51200 + col*32;

  // Access Row
  master_tx[0] = row_seq;
  spi_tx();
  HAL_Delay(1);
  //__delay_us(20);

  // Access Col
  master_tx[0] = col_seq;
  spi_tx();
  HAL_Delay(1);
  //__delay_us(20);
}

/* Write to RAM of TTN */
static void ttn_ramwrite(const uint16_t vs){
  uint16_t wram = 56320 + vs;

	// Store Vs value in RAM
  master_tx[0] = wram;

  spi_tx();
  HAL_Delay(1);
  //__delay_us(15);
}

static void ttn_ramread(){
	// Read Vs value in RAM
	master_tx[0] = 0xD800;

	// Discard the previous value
	spi_tx();
  HAL_Delay(1);
	//__delay_us(10);

  // Write 0 to RAM
	master_tx[0] = 56320;

	// Read this value
	spi_tx();

	tmp = master_rx[0];
  HAL_Delay(1);
  // __delay_us(10);

	read = (tmp >> 6);
}

/* Send command to read temp pixel with Vs value */
static void ttn_pxvo(const uint16_t vs){
  uint16_t rtemp = 53248 + vs;

  master_tx[0] = rtemp;
  spi_tx();
  HAL_Delay(1);
  //__delay_us(50);

	// Write 0 to RAM
	master_tx[0] = 56320;

	// Read temp value
  spi_tx();
  read = master_rx[0];
  HAL_Delay(1);
  //__delay_us(15);
}

static void pxe_ffy(int data){
	//Transmit via Bluetooth to Android phone
	// Shift MSByte
	bufftx[0] = (char)(data >> 8);
	HAL_UART_Transmit_IT(&huart4, bufftx, 1);
  //	__delay_us(10);
	// Shift LSByte
	bufftx[0] = (char)data;
	HAL_UART_Transmit_IT(&huart4, bufftx, 1);
//	__delay_us(10);
}

static void ttn_col(const uint16_t col){
	uint16_t col_seq = 51200 + col*32;

  master_tx[0] = col_seq;
	spi_tx();

	//tmp = SPI1BUFL;
	//__delay_us(10);
  //__delay_us(20);
}

static void ttn_row(const uint16_t row){
	uint16_t row_seq = 49152 + row*16;

	master_tx[0] = row_seq;
  spi_tx();
	//tmp = SPI1BUFL;
	//__delay_us(10);
	//__delay_us(20);
}

static void ttn_pxvoRef(const uint16_t vs){
	// Send command to read pixel with Vs value
  uint16_t rtemp = 53248 + vs;

	// Discard the previous value
  master_tx[0] = rtemp;
  spi_tx();

	HAL_Delay(50);
  //__delay_us(50);

  // Write 0 to RAM
	master_tx[0] = 56320;

	// Read temp value
  spi_tx();
  read = master_rx[0];
  HAL_Delay(1);

	//__delay_us(15);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	/*if(buffrx[0] == 'a'){
		HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
	}
	if(buffrx[0] == 'b'){
		HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
	}
	HAL_UART_Receive_IT(&huart4, buffrx, 1);*/

	//LATBbits.LATB2 = 1;		// Turn on LED
	/*if (U1STAbits.OERR == 1) // Clear RX overrun flag
	{
		U1STAbits.OERR = 0;
	}
	else
	{
			while(U1STAbits.URXDA)
		{*/
			if(ttn_cmd == 0){
				btrx = buffrx[0];
				switch(btrx){
					case 'Q': ttn_cmd = 1;
					break;
					case 'R': ttn_cmd = 2;
					break;
					case 'T': ttn_cmd = 3;
					break;
					case 'U': ttn_cmd = 4;
					break;
					case 'S': count = 0;
								bufftx[0] = 's';
								HAL_UART_Transmit_IT(&huart4, bufftx, 1);
								pxe_ffy(end);
								ttn_cmd = 9;
					break;
					case 'V': count = 0;
								ttn_cmd = 10;
					break;
					case 'W': ttn_cmd = 7;
					break;
					case 'C': ttn_cmd = 8;
					break;
					}
			}
			else if(ttn_cmd == 6){
				btrx = buffrx[0];
				if(btrx == 'W'){
					ttn_cmd = 7;
				}
				else if(btrx == 'C'){
					ttn_cmd = 8;
				}
			}
			else if(ttn_cmd == 7){
				btrx = buffrx[0];
				if(btrx == 'X'){
					ttn_cmd = 0;
				}
			}
			else if(ttn_cmd == 9){
				btrx = buffrx[0];
				if(btrx == 0xFF){
					ttn_cmd = 0;
				}
				else{
					tempfootprint[count] = btrx;
					count++;
					if(count == 988){
						count = 0;
						ttn_cmd = 5;
					}
				}
			}
			else if(ttn_cmd == 10){
				btrx = buffrx[0];
				tau63[count] = btrx;
				count++;
				if(count == 2){
					count = 0;
					ttn_cmd = 6;
				}
			}
			HAL_UART_Receive_IT(&huart4, buffrx, 1);
//	LATBbits.LATB2 = 0;		// Turn off LED
	//IFS0bits.U1RXIF = 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
