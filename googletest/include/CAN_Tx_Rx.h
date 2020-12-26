#ifndef CAN_ADD_TX_MESSAGE_H
#define CAN_ADD_TX_MESSAGE_H

#include "StateLed.h"
uint8_t CanTransData[8];

uint8_t mLedOrange;
uint8_t mLedBlue;
uint8_t mLedRed;
uint8_t mLedGreen;

	MxMinMeasure_t mMeasurement_var = {0};
	MxMinMeasure_t mMaxRange_var = {0};
    MxMinMeasure_t mMinRange_var = {0};

    MxMinMeasure_t* mMeasurement_ptr = &mMeasurement_var;
    MxMinMeasure_t* mMaxRange_ptr = &mMaxRange_var;
    MxMinMeasure_t* mMinRange_ptr = &mMinRange_var;
int CanReceiveData_From_User(uint8_t CanReceiveData[]);

uint8_t HAL_CAN_AddTxMessage(uint8_t CanTransData[]);


#endif