
#include <gtest/gtest.h>
#include "CAN_Tx_Rx.c"
#include "StateLed.h"
#include "MotorServoControl.c"
#include "ledcontrol.c"

uint8_t CanReceiveData[8];

using namespace std;






 TEST(StateTest, H_Range_Seq) 
    { 
    
    uint8_t Measurement=0;
        while(Measurement <=5)
        {
            CanReceiveData[0]=70; // Measurement id
            CanReceiveData[1]= Measurement;  // Measurement
            CanReceiveData[2]=80; // MaxRange id
            CanReceiveData[3]=10; // MaxRange data
            CanReceiveData[4]=90; // MinRange id
            CanReceiveData[5]=5;  // MinRange data

            ASSERT_EQ(HIGHSEQURITY, CanReceiveData_From_User(CanReceiveData));
            ASSERT_NE(ERROR_S, CanReceiveData_From_User(CanReceiveData));
            ASSERT_NE(NO_OBJECT, CanReceiveData_From_User(CanReceiveData));
            ASSERT_NE(LOWSEQURITY, CanReceiveData_From_User(CanReceiveData));
            Measurement++;
        }


    }


 TEST(StateTest, L_Range_Seq) 
    { 
    
    uint8_t Measurement=6;
        while(Measurement > 5 && Measurement <=10)
        {
            CanReceiveData[0]=70; // Measurement id
            CanReceiveData[1]= Measurement;  // Measurement
            CanReceiveData[2]=80; // MaxRange id
            CanReceiveData[3]=10; // MaxRange data
            CanReceiveData[4]=90; // MinRange id
            CanReceiveData[5]=5;  // MinRange data


            ASSERT_EQ(LOWSEQURITY, CanReceiveData_From_User(CanReceiveData));
            ASSERT_NE(ERROR_S, CanReceiveData_From_User(CanReceiveData));
            ASSERT_NE(NO_OBJECT, CanReceiveData_From_User(CanReceiveData));
            ASSERT_NE(HIGHSEQURITY, CanReceiveData_From_User(CanReceiveData));
            Measurement++;
        }


    }


 TEST(StateTest, NO_Range_Seq) 
    { 
    
    uint8_t Measurement=11;
        while(Measurement >10)
        {
            CanReceiveData[0]=70; // Measurement id
            CanReceiveData[1]= Measurement;  // Measurement
            CanReceiveData[2]=80; // MaxRange id
            CanReceiveData[3]=10; // MaxRange data
            CanReceiveData[4]=90; // MinRange id
            CanReceiveData[5]=5;  // MinRange data


            ASSERT_EQ(NO_OBJECT, CanReceiveData_From_User(CanReceiveData));
            ASSERT_NE(ERROR_S, CanReceiveData_From_User(CanReceiveData));
            ASSERT_NE(LOWSEQURITY, CanReceiveData_From_User(CanReceiveData));
            ASSERT_NE(HIGHSEQURITY, CanReceiveData_From_User(CanReceiveData));
            Measurement++;
        }


    }



TEST(StateTest, Error_State_Max_0_Min_0) 
    { 
    
    CanReceiveData[0]=70; // Measurement id
    CanReceiveData[1]=43; // Measurement
    CanReceiveData[2]=80; // MaxRange id
    CanReceiveData[3]=0; // MaxRange data
    CanReceiveData[4]=90; // MinRange id
    CanReceiveData[5]=0; // MinRange data

    ASSERT_EQ(ERROR_S, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(NO_OBJECT, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(LOWSEQURITY, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(HIGHSEQURITY, CanReceiveData_From_User(CanReceiveData));

    }

TEST(StateTest, Error_State_Min_0) 
    { 
    
    CanReceiveData[0]=70; // Measurement id
    CanReceiveData[1]=43; // Measurement
    CanReceiveData[2]=80; // MaxRange id
    CanReceiveData[3]=10; // MaxRange data
    CanReceiveData[4]=90; // MinRange id
    CanReceiveData[5]=0; // MinRange data

    ASSERT_EQ(ERROR_S, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(NO_OBJECT, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(LOWSEQURITY, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(HIGHSEQURITY, CanReceiveData_From_User(CanReceiveData));

    }

TEST(StateTest, Error_State_Max_0) 
    { 
    
    CanReceiveData[0]=70; // Measurement id
    CanReceiveData[1]=43; // Measurement
    CanReceiveData[2]=80; // MaxRange id
    CanReceiveData[3]=0;  // MaxRange data
    CanReceiveData[4]=90; // MinRange id
    CanReceiveData[5]=35; // MinRange data

    ASSERT_EQ(ERROR_S, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(NO_OBJECT, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(LOWSEQURITY, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(HIGHSEQURITY, CanReceiveData_From_User(CanReceiveData));

    }


 TEST(StateTest, H_sequrity_State) 
    { 
    
    CanReceiveData[0]=70; // Measurement id
    CanReceiveData[1]=5;  // Measurement
    CanReceiveData[2]=80; // MaxRange id
    CanReceiveData[3]=10; // MaxRange data
    CanReceiveData[4]=90; // MinRange id
    CanReceiveData[5]=5;  // MinRange data

    ASSERT_EQ(HIGHSEQURITY, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(ERROR_S, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(NO_OBJECT, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(LOWSEQURITY, CanReceiveData_From_User(CanReceiveData));

    }

 TEST(StateTest, H_State) 
    { 
    
    CanReceiveData[0]=70; // Measurement id
    CanReceiveData[1]=5;  // Measurement
    CanReceiveData[2]=80; // MaxRange id
    CanReceiveData[3]=50; // MaxRange data
    CanReceiveData[4]=90; // MinRange id
    CanReceiveData[5]=40;  // MinRange data

    ASSERT_EQ(HIGHSEQURITY, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(ERROR_S, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(NO_OBJECT, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(LOWSEQURITY, CanReceiveData_From_User(CanReceiveData));

    }

 TEST(StateTest, L_sequrity_State) 
    { 
    
    CanReceiveData[0]=70; // Measurement id
    CanReceiveData[1]=10; // Measurement
    CanReceiveData[2]=80; // MaxRange id
    CanReceiveData[3]=10; // MaxRange data
    CanReceiveData[4]=90; // MinRange id
    CanReceiveData[5]=5;  // MinRange data

    ASSERT_EQ(LOWSEQURITY, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(HIGHSEQURITY, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(ERROR_S, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(NO_OBJECT, CanReceiveData_From_User(CanReceiveData));

    }

 TEST(StateTest, Noobject_State) 
    { 
    
    CanReceiveData[0]=70; // Measurement id
    CanReceiveData[1]=15; // Measurement
    CanReceiveData[2]=80; // MaxRange id
    CanReceiveData[3]=10; // MaxRange data
    CanReceiveData[4]=90; // MinRange id
    CanReceiveData[5]=5;  // MinRange data

    ASSERT_EQ(NO_OBJECT, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(LOWSEQURITY, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(HIGHSEQURITY, CanReceiveData_From_User(CanReceiveData));
    ASSERT_NE(ERROR_S, CanReceiveData_From_User(CanReceiveData));
    }

 TEST(MotorSpeedTest, Speed_0) 
    { 

    CanReceiveData[0]=50; // MotorSpeed_id
    CanReceiveData[1]=0; // Speed_Value

    ASSERT_EQ(0, MotorServoControl(CanReceiveData));

    }

 TEST(MotorSpeedTest, Speed_1) 
    { 
    
    CanReceiveData[0]=50; // MotorSpeed_id
    CanReceiveData[1]=1; // Speed_Value

    ASSERT_EQ(1, MotorServoControl(CanReceiveData));

    }

 TEST(MotorSpeedTest, Speed_2) 
    { 
    
    CanReceiveData[0]=50; // MotorSpeed_id
    CanReceiveData[1]=2; // Speed_Value

    ASSERT_EQ(2, MotorServoControl(CanReceiveData));
    ASSERT_NE(0, MotorServoControl(CanReceiveData));
    ASSERT_NE(1, MotorServoControl(CanReceiveData));

    }

     TEST(LedControlTest, LedOrange_True) 
    { 
    
    CanReceiveData[0]=70; // Measurement id
    CanReceiveData[1]=10; // Measurement
    CanReceiveData[2]=80; // MaxRange id
    CanReceiveData[3]=10; // MaxRange data
    CanReceiveData[4]=90; // MinRange id
    CanReceiveData[5]=0;  // MinRange data

    ASSERT_TRUE(ledControlUnit(CanReceiveData));

    }


     TEST(LedControlTest, LedBlue_True) 
    { 
    
    CanReceiveData[0]=70; // Measurement id
    CanReceiveData[1]=10; // Measurement
    CanReceiveData[2]=80; // MaxRange id
    CanReceiveData[3]=10; // MaxRange data
    CanReceiveData[4]=90; // MinRange id
    CanReceiveData[5]=5;  // MinRange data

    ASSERT_TRUE(ledControlUnit(CanReceiveData));

    }

     TEST(LedControlTest, LedRed_True) 
    { 
    
    CanReceiveData[0]=70; // Measurement id
    CanReceiveData[1]=5; // Measurement
    CanReceiveData[2]=80; // MaxRange id
    CanReceiveData[3]=10; // MaxRange data
    CanReceiveData[4]=90; // MinRange id
    CanReceiveData[5]=5;  // MinRange data

    ASSERT_TRUE(ledControlUnit(CanReceiveData));

    }

     TEST(LedControlTest, LedGreen_True) 
    { 
    
    CanReceiveData[0]=70; // Measurement id
    CanReceiveData[1]=15; // Measurement
    CanReceiveData[2]=80; // MaxRange id
    CanReceiveData[3]=10; // MaxRange data
    CanReceiveData[4]=90; // MinRange id
    CanReceiveData[5]=5;  // MinRange data

    ASSERT_TRUE(ledControlUnit(CanReceiveData));

    }

int main(int argc, char **argv) 
    {

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    }
