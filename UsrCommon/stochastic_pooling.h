/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2025 xxx,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : stochastic_pooling.h  
  ** @Description   : None
  **--------------------------------------------------------------------------**
  ** @Author        : Depressed	  
  ** @Version       : v1.0				
  ** @Creat Date    : 2025-04-09  
  **--------------------------------------------------------------------------**
  ** @Modfier       : None
  ** @Version       : None
  ** @Modify Date   : None
  ** @Description   : None
  **--------------------------------------------------------------------------**
  ** @Function List : None
  **--------------------------------------------------------------------------**
  ** @Attention     : None
  **--------------------------------------------------------------------------**
  ******************************************************************************
  *
 **/
 
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __STOCHASTIC_POOLING_H_
#define __STOCHASTIC_POOLING_H_
float** allocate_matrix(int rows, int cols);
void free_matrix(float **matrix, int rows);

void weighted_avg_pooling_variable(float **input, int rows, int cols,
                                 int row_pool, int col_pool, 
                                 float **output);
void weighted_avg_pooling_variable_temp(float **input, int rows, int cols,
                                 int row_pool, int col_pool, 
                                 float **output, float temperature);
void mixed_pooling(float **input, int rows, int cols,
                 int row_pool, int col_pool, 
                 float **output, float alpha);
int test_mean_pooling(void);
int test_weighted_avg_pooling(void);
int test_weighted_avg_pooling_tempera(void);
#endif


/******************************** END OF FILE *********************************/


 

