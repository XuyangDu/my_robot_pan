/**
  * @brief  RS_powerControl A0 command
  * @param  uint8_t Motor_ID 
  * @param  int16_t powerControl
  */
void RS_powerControl(uint8_t Motor_ID, int16_t powerControl)
{
  uint8_t TxData[8] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA0;
  TxData[2] = Motor_ID;
  TxData[3] = 2;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&powerControl));
  TxData[6] = *((uint8_t *)(&powerControl)+1);
  TxData[7] = Checksumcrc(TxData,5,2);
        
  RS_T_EN();
  HAL_UART_Transmit(&huart5, TxData, 8, 10);
  RS_R_EN();

}

/**
  * @brief  RS_TorgueControl A1 command
  * @param  uint8_t Motor_ID 
  * @param  int32_t iqControl
  */
void RS_TorgueControl(uint8_t Motor_ID, int32_t iqControl)
{
  uint8_t TxData[8] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA1;
  TxData[2] = Motor_ID;
  TxData[3] = 2;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&iqControl));
  TxData[6] = *((uint8_t *)(&iqControl)+1);
  TxData[7] = Checksumcrc(TxData,5,2);
  
  RS_T_EN();
  HAL_UART_Transmit(&huart5, TxData, 8, 10);
  RS_R_EN();
}

/**
  * @brief  RS_speedControl A2 command
  * @param  uint8_t Motor_ID 
  * @param  int32_t speedControl
  */
void RS_speedControl(uint8_t Motor_ID, int32_t speedControl)
{
  uint8_t TxData[10] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA2;
  TxData[2] = Motor_ID;
  TxData[3] = 4;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&speedControl));
  TxData[6] = *((uint8_t *)(&speedControl)+1);
  TxData[7] = *((uint8_t *)(&speedControl)+2);
  TxData[8] = *((uint8_t *)(&speedControl)+3);
  TxData[9] = Checksumcrc(TxData,5,4);
        
  RS_T_EN();
  HAL_UART_Transmit(&huart5, TxData, 10, 10);
  RS_R_EN();
}

/**
  * @brief  RS_angleControl_1 A3 command
  * @param  uint8_t Motor_ID 
  * @param  int64_t angleControl_1
  */
void RS_angleControl_1(uint8_t Motor_ID, int64_t angleControl_1)
{
  uint8_t TxData[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA3;
  TxData[2] = Motor_ID;
  TxData[3] = 8;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&angleControl_1)); //*(uint8_t *)(&angleControl);
  TxData[6] = *((uint8_t *)(&angleControl_1)+1);
  TxData[7] = *((uint8_t *)(&angleControl_1)+2);
  TxData[8] = *((uint8_t *)(&angleControl_1)+3);
  TxData[9] = *((uint8_t *)(&angleControl_1)+4);
  TxData[10] = *((uint8_t *)(&angleControl_1)+5);
  TxData[11] = *((uint8_t *)(&angleControl_1)+6);
  TxData[12] = *((uint8_t *)(&angleControl_1)+7);
  TxData[13] = Checksumcrc(TxData,5,8);
        
  HAL_UART_Transmit(&huart5, TxData, 14, 10);
  
}

/**
  * @brief  RS_angleControl_2 A4 command
  * @param  uint8_t Motor_ID   
  * @param  int64_t angleControl_1
  * @param  uint32_t maxSpeed 
  */
void RS_angleControl_2(uint8_t Motor_ID, int64_t angleControl_1, uint32_t maxSpeed)
{
  uint8_t TxData[18] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA4;
  TxData[2] = Motor_ID;
  TxData[3] = 12;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&angleControl_1));
  TxData[6] = *((uint8_t *)(&angleControl_1)+1);
  TxData[7] = *((uint8_t *)(&angleControl_1)+2);
  TxData[8] = *((uint8_t *)(&angleControl_1)+3);
  TxData[9] = *((uint8_t *)(&angleControl_1)+4);
  TxData[10] = *((uint8_t *)(&angleControl_1)+5);
  TxData[11] = *((uint8_t *)(&angleControl_1)+6);
  TxData[12] = *((uint8_t *)(&angleControl_1)+7);
  TxData[13] = *((uint8_t *)(&maxSpeed));
  TxData[14] = *((uint8_t *)(&maxSpeed)+1);
  TxData[15] = *((uint8_t *)(&maxSpeed)+2);
  TxData[16] = *((uint8_t *)(&maxSpeed)+3);
  TxData[17] = Checksumcrc(TxData,5,12);
        
  RS_T_EN();
  HAL_UART_Transmit(&huart5, TxData, 18, 10);
  RS_R_EN();

}
/**
  * @brief  single loop1_RS_angleControl_3 A5 command
  * @param  uint8_t Motor_ID   
  * @param  uint8_t spinDirection 
  * @param  int16_t angleControl_1
  */
void RS_angleControl_3(uint8_t Motor_ID, uint8_t spinDirection, int16_t angleControl_1)
{
  uint8_t TxData[10] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA5;
  TxData[2] = Motor_ID;
  TxData[3] = 4;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = spinDirection;
  TxData[6] = *((uint8_t *)(&angleControl_1));
  TxData[7] = *((uint8_t *)(&angleControl_1)+1);
  TxData[8] = 0;
  TxData[9] = Checksumcrc(TxData,5,4);
  
  RS_T_EN();
  HAL_UART_Transmit(&huart5, TxData, 10, 10);
  RS_R_EN();
}
/**
  * @brief  single loop2_RS_angleControl_4 A6 command
  * @param  uint8_t Motor_ID   
  * @param  uint8_t spinDirection 
  * @param  int16_t angleControl_1
  * @param uint32_t maxSpeed
  */
void RS_angleControl_4(uint8_t Motor_ID, uint8_t spinDirection, uint16_t angleControl_1, uint32_t maxSpeed)
{
  uint8_t TxData[14] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA6;
  TxData[2] = Motor_ID;
  TxData[3] = 8;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = spinDirection;
  TxData[6] = *((uint8_t *)(&angleControl_1));
  TxData[7] = *((uint8_t *)(&angleControl_1)+1);
  TxData[8] = 0;
  TxData[9] = *((uint8_t *)(&maxSpeed));
  TxData[10] = *((uint8_t *)(&maxSpeed)+1);
  TxData[11] = *((uint8_t *)(&maxSpeed)+2);
  TxData[12] = *((uint8_t *)(&maxSpeed)+3);
  TxData[13] = Checksumcrc(TxData,5,8);
  
  RS_T_EN();
  HAL_UART_Transmit(&huart5, TxData, 14, 10);
  RS_R_EN();
  /*
  uint8_t i=0;
  for (i=0;i<14;i++)
  printf("buf%d=%#X\r\n",i,TxData[i]);
  */
}
/**
  * @brief  angleIncrement1_RS_angleControl_5 A7 command
  * @param  uint8_t Motor_ID     
  * @param uint32_t angleIncrement
  */ 
void RS_angleControl_5(uint8_t Motor_ID,  int32_t angleIncrement)
{
  uint8_t TxData[10] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA7;
  TxData[2] = Motor_ID;
  TxData[3] = 4;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&angleIncrement));
  TxData[6] = *((uint8_t *)(&angleIncrement)+1);
  TxData[7] = *((uint8_t *)(&angleIncrement)+2);
  TxData[8] = *((uint8_t *)(&angleIncrement)+3);
  TxData[9] = Checksumcrc(TxData,5,4);
        
  RS_T_EN();
  HAL_UART_Transmit(&huart5, TxData, 10, 10);
  RS_R_EN();
}
/**
  * @brief  angleIncrement2_RS_angleControl_6 A8 command
  * @param  uint8_t Motor_ID     
  * @param uint32_t angleIncrement
  * @param  uint32_t maxSpeed  
  */ 
void RS_angleControl_6(uint8_t Motor_ID,  int32_t angleIncrement, uint32_t maxSpeed)
{
  uint8_t TxData[14] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0xA8;
  TxData[2] = Motor_ID;
  TxData[3] = 8;
  TxData[4] = Checksumcrc(TxData,0,4);
  TxData[5] = *((uint8_t *)(&angleIncrement));
  TxData[6] = *((uint8_t *)(&angleIncrement)+1);
  TxData[7] = *((uint8_t *)(&angleIncrement)+2);
  TxData[8] = *((uint8_t *)(&angleIncrement)+3);
  TxData[9] = *((uint8_t *)(&maxSpeed));
  TxData[10] = *((uint8_t *)(&maxSpeed)+1);
  TxData[11] = *((uint8_t *)(&maxSpeed)+2);
  TxData[12] = *((uint8_t *)(&maxSpeed)+3);
  TxData[13] = Checksumcrc(TxData,5,8);
   
  RS_T_EN();
  HAL_UART_Transmit(&huart5, TxData, 14, 10);
  RS_R_EN();
}
/**
  * @brief  RS_Motor_Off  command
  * @param  uint8_t Motor_ID     
  */ 
void RS_Motor_Off(uint8_t Motor_ID)
{
  uint8_t TxData[5] = {0};
  
  TxData[0] = 0x3E;
  TxData[1] = 0x80;
  TxData[2] = Motor_ID;
  TxData[3] = 4;
  TxData[4] = Checksumcrc(TxData,0,4);
        
  RS_T_EN();
  HAL_UART_Transmit(&huart5, TxData, 5, 10);
  RS_R_EN();

}

