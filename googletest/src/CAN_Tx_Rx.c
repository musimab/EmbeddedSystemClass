#include "StateLed.h"
#include "CAN_Tx_Rx.h"

int CanReceiveData_From_User(uint8_t CanReceiveData[])
{

	MxMinMeasure_t mMeasurement_var = {0};
	MxMinMeasure_t mMaxRange_var = {0};
    MxMinMeasure_t mMinRange_var = {0};

    MxMinMeasure_t* mMeasurement_ptr = &mMeasurement_var;
    MxMinMeasure_t* mMaxRange_ptr = &mMaxRange_var;
    MxMinMeasure_t* mMinRange_ptr = &mMinRange_var;

    STATE_t state = ERROR_S;

    MotorSpeed_t mMotorSpeed = {0};
	 
	mMeasurement_ptr->id   = CanReceiveData[0];
	if(mMeasurement_ptr->id != 70)
		return -1;
	mMeasurement_ptr->data = CanReceiveData[1];

	mMaxRange_ptr->id   = CanReceiveData[2];
	if(mMaxRange_ptr->id != 80)
		return -1;
	mMaxRange_ptr->data = CanReceiveData[3];

	mMinRange_ptr->id   = CanReceiveData[4];
	if(mMinRange_ptr->id != 90)
		return -1;
	mMinRange_ptr->data = CanReceiveData[5];


    if(mMaxRange_ptr->data == 0 || mMinRange_ptr->data == 0)
      state = ERROR_S; // State ERROR
    else if(mMeasurement_ptr->data <= mMinRange_ptr->data)
      state = HIGHSEQURITY; // State HIGHSEQURITY
    else if(mMeasurement_ptr->data > mMinRange_ptr->data && mMeasurement_ptr->data <= mMaxRange_ptr->data)
      state = LOWSEQURITY; // State LOWSEQURITY
    else if(mMeasurement_ptr->data >= mMaxRange_ptr->data)
      state = NO_OBJECT; // State NO_OBJECT
    
    
    
    switch(state)
      {
		  case ERROR_S:
		  {
		    	  
		    	  mLedOrange =1;
                  mLedRed = 0;
                  mLedGreen =0;
                  mLedBlue = 0;
		    	  mMotorSpeed.data = 0;
		    	  mMotorSpeed.id = 50;
		    	  CanTransData[1]= mMotorSpeed.data; // mMotorSpeed =0
		    	  CanTransData[0]= mMotorSpeed.id; // mMotorSpeed =0
		    	  //HAL_CAN_AddTxMessage(CanTransData);// Send mMotorSpeed to MotorServoControl unit

		    	if((mMeasurement_ptr->data <= mMinRange_ptr->data) && (mMaxRange_ptr->data != 0 && mMinRange_ptr->data != 0)) //# [T1.4]
				    state = HIGHSEQURITY;
				else if((mMeasurement_ptr->data > mMinRange_ptr->data) && (mMeasurement_ptr->data <= mMaxRange_ptr->data) &&  (mMaxRange_ptr->data != 0 && mMinRange_ptr->data != 0)) //#[T1.3]
					state = LOWSEQURITY;
				else if((mMeasurement_ptr->data >= mMaxRange_ptr->data) && (mMaxRange_ptr->data != 0 && mMinRange_ptr->data != 0)) //# [T1.1]
				    state = NO_OBJECT;
				else
					state =ERROR_S;
		    return state;
		    break;
		  }

		  case HIGHSEQURITY:
		  {
		      mLedOrange =0;
              mLedRed = 1;
              mLedGreen =0;
              mLedBlue = 0;              
		      mMotorSpeed.data = 0;
		   	  mMotorSpeed.id = 50;
		   	  CanTransData[0]= mMotorSpeed.id; // mMotorSpeed = 0
		   	  CanTransData[1]= mMotorSpeed.data;
		   	  state = HIGHSEQURITY;
			  //HAL_CAN_AddTxMessage(CanTransData);// Send mMotorSpeed to MotorServoControl unit
			  if((mMeasurement_ptr->data > mMinRange_ptr->data) && (mMeasurement_ptr->data <= mMaxRange_ptr->data)) //#[T2.2]
 				  state =LOWSEQURITY;
			  return state;
			  break;
		  }

		  case LOWSEQURITY:
		  {
		      mLedOrange =0;
              mLedRed = 0;
              mLedGreen =0;
              mLedBlue = 1;   
		      mMotorSpeed.data = 2;
		      mMotorSpeed.id = 50;
		      CanTransData[0]= mMotorSpeed.id;  // mMotorSpeed id
		      CanTransData[1]= mMotorSpeed.data;// mMotorSpeed =2
		      state = LOWSEQURITY;
			 // HAL_CAN_AddTxMessage(CanTransData);// Send mMotorSpeed to MotorServoControl unit
		      return state;
			  break;
		  }

		  case NO_OBJECT:
		  {
		      mLedOrange =0;
              mLedRed = 0;
              mLedGreen =1;
              mLedBlue = 0;   
		      mMotorSpeed.data = 1;
		      mMotorSpeed.id = 50;
		      CanTransData[0]= mMotorSpeed.id;
		      CanTransData[1]= mMotorSpeed.data; // mMotorSpeed =1
			  //HAL_CAN_AddTxMessage(CanTransData);// Send mMotorSpeed to MotorServoControl unit
			  state = NO_OBJECT;
			  return state;
			  break;
		   }
		}


}