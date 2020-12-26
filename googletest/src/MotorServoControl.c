#include "StateLed.h"

uint8_t MotorServoControl(uint8_t CanReceiveData[])
{
	uint32_t A=500;
	uint32_t B=2500;
	uint32_t speed = 1;
	uint32_t pwm_value=0;

	MotorSpeed_t mMotorSpeed;
	// Can Receive Message from  LedControl Unit
	mMotorSpeed.id = CanReceiveData[0]; //50
	if(mMotorSpeed.id != 50)
		return -1;
	mMotorSpeed.data = CanReceiveData[1];

	// CAN Transmit Message to MaxMinRangeControl Unit
	//CanTransData[0] = mMotorSpeed.id;
	//CanTransData[1] = mMotorSpeed.data;
	

	switch(mMotorSpeed.data)
	{
		case 0:
		{
			//__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm_value);
			
			
			return 0;
			 break;
		}
		case 1:
		{
			for(pwm_value=A; pwm_value<B; pwm_value++)
			{
				  //Send pwm value
			//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, pwm_value);

				//HAL_Delay(5);
			}

			for(pwm_value=B; pwm_value>A; pwm_value--)
			{
				  //Send pwm value
			//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, pwm_value);
				//HAL_Delay(5);
			}
			return 1;
			break;
		}
		case 2:
		{
			for(pwm_value=A; pwm_value<B; pwm_value++)
			{
				//Send pwm value
			//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm_value);
				//HAL_Delay(0.25);
			}

			for(pwm_value=B; pwm_value>A; pwm_value--)
			{
					//Send pwm value
			//	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm_value);
				//HAL_Delay(0.25);
			}
			return 2;
			break;
		}

	}


}
