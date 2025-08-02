/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2025 YKT,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : stochastic_pooling.c  
  ** @Brief         : None
  **--------------------------------------------------------------------------**
  ** @Author Data   : Depressed 2025-04-09
  ** @Version       : v1.0				
  **--------------------------------------------------------------------------**
  ** @Modfier Data  : None
  ** @Version       : None
  ** @Description   : None
  **--------------------------------------------------------------------------**
  ** @Function List : None
  **--------------------------------------------------------------------------**
  ** @Attention     : None
  **--------------------------------------------------------------------------**
  ******************************************************************************
  *
 **/
#include "stochastic_pooling.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>  // 用于指数计算
#include <time.h>  // 用于随机数生成

// 分配二维数组
float** allocate_matrix(int rows, int cols) {
    float **matrix = (float**)malloc(rows * sizeof(float*));
    for (int i = 0; i < rows; i++) {
        matrix[i] = (float*)malloc(cols * sizeof(float));
    }
    return matrix;
}

// 释放二维数组
void free_matrix(float **matrix, int rows) {
    for (int i = 0; i < rows; i++) {
        free(matrix[i]);
    }
    free(matrix);
}

// 打印矩阵
void print_matrix(float **matrix, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            printf("%6.2f ", matrix[i][j]);
        }
        printf("\n");
    }
}


// 行列不同比例缩小函数
void resize_matrix(float **input, int rows, int cols,
                  int row_scale, int col_scale, 
                  float **output) {
    int new_rows = rows / row_scale;
    int new_cols = cols / col_scale;
    
    for (int i = 0; i < new_rows; i++) {
        for (int j = 0; j < new_cols; j++) {
            float sum = 0.0;
            int count = 0;
            
            // 计算当前块的行列范围
            int row_start = i * row_scale;
            int row_end = (i + 1) * row_scale;
            int col_start = j * col_scale;
            int col_end = (j + 1) * col_scale;
            
            // 确保不越界
            if (row_end > rows) row_end = rows;
            if (col_end > cols) col_end = cols;
            
            // 计算块内平均值
            for (int x = row_start; x < row_end; x++) {
                for (int y = col_start; y < col_end; y++) {
                    sum += input[x][y];
                    count++;
                }
            }
            output[i][j] = sum / count;
        }
    }
}
//缩小比例除不尽自动处理									
void flexible_resize(float **input, int rows, int cols,
                    int new_rows, int new_cols,
                    float **output) {
    float row_stride = (float)rows / new_rows;
    float col_stride = (float)cols / new_cols;
    
    for (int i = 0; i < new_rows; i++) {
        for (int j = 0; j < new_cols; j++) {
            // 计算输入矩阵中的对应区域
            int row_start = (int)(i * row_stride);
            int row_end = (int)((i + 1) * row_stride);
            int col_start = (int)(j * col_stride);
            int col_end = (int)((j + 1) * col_stride);
            
            // 确保不越界
            if (row_end > rows) row_end = rows;
            if (col_end > cols) col_end = cols;
            
            // 计算平均值
            float sum = 0.0;
            int count = 0;
            
            for (int x = row_start; x < row_end; x++) {
                for (int y = col_start; y < col_end; y++) {
                    sum += input[x][y];
                    count++;
                }
            }
            
            output[i][j] = sum / count;
        }
    }
}
// 用随机值初始化矩阵
void init_random_matrix(float **matrix, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // 生成0-99之间的随机浮点数
            matrix[i][j] = (float)(rand() % 100);
        }
    }
}
										// 计算softmax权重
// 计算softmax权重
void compute_weights(float *window, int size, float *weights) {
    float sum = 0.0f;
    float max_val = window[0];
    
    // 找到最大值（用于数值稳定性）
    for (int i = 1; i < size; i++) {
        if (window[i] > max_val) {
            max_val = window[i];
        }
    }
    
    // 计算指数和
    for (int i = 0; i < size; i++) {
        weights[i] = expf(window[i] - max_val);
        sum += weights[i];
    }
    
    // 归一化权重
    for (int i = 0; i < size; i++) {
        weights[i] /= sum;
    }
}
// 修改compute_weights函数，添加temperature参数
void compute_weights_temp(float *window, int size, float *weights, float temperature) {
    float sum = 0.0f;
    float max_val = window[0];
    
    // 找到最大值
    for (int i = 1; i < size; i++) {
        if (window[i] > max_val) {
            max_val = window[i];
        }
    }
    
    // 计算带温度参数的指数和
    for (int i = 0; i < size; i++) {
        weights[i] = expf((window[i] - max_val) / temperature);  // 温度参数控制权重分布
        sum += weights[i];
    }
    
    // 归一化权重
    for (int i = 0; i < size; i++) {
        weights[i] /= sum;
    }
}
void compute_weights_softened(float *window, int size, float *weights) {
    float sum = 0.0f;
    float max_val = window[0];
    
    // 找到最大值
    for (int i = 1; i < size; i++) {
        if (window[i] > max_val) {
            max_val = window[i];
        }
    }
    
    // 使用平方根压缩差异
    for (int i = 0; i < size; i++) {
        float normalized = (window[i] - max_val) / (max_val + 1e-6f);  // 防止除以0
        weights[i] = expf(sqrtf(fabsf(normalized)) * (normalized > 0 ? 1 : -1));
        sum += weights[i];
    }
    
    // 归一化权重
    for (int i = 0; i < size; i++) {
        weights[i] /= sum;
    }
}
// 行列不同比例的加权平均池化函数
void weighted_avg_pooling_variable(float **input, int rows, int cols,
                                 int row_pool, int col_pool, 
                                 float **output) {
    int new_rows = rows / row_pool;
    int new_cols = cols / col_pool;
    
    // 计算最大可能的窗口大小
    int max_window_size = row_pool * col_pool;
    float *window = (float*)malloc(max_window_size * sizeof(float));
    float *weights = (float*)malloc(max_window_size * sizeof(float));
    
    for (int i = 0; i < new_rows; i++) {
        for (int j = 0; j < new_cols; j++) {
            // 计算当前池化窗口的位置
            int row_start = i * row_pool;
            int row_end = row_start + row_pool;
            int col_start = j * col_pool;
            int col_end = col_start + col_pool;
            
            // 确保不越界
            if (row_end > rows) row_end = rows;
            if (col_end > cols) col_end = cols;
            
            // 提取窗口数据
            int window_size = 0;
            for (int x = row_start; x < row_end; x++) {
                for (int y = col_start; y < col_end; y++) {
                    window[window_size++] = input[x][y];
                }
            }
            
            // 计算权重（使用softmax，数值大的权重大）
            compute_weights(window, window_size, weights);
            
            // 计算加权平均值
            float weighted_sum = 0.0f;
            for (int k = 0; k < window_size; k++) {
                weighted_sum += window[k] * weights[k];
            }
            
            output[i][j] = weighted_sum;
        }
    }
    
    free(window);
    free(weights);
}
void weighted_avg_pooling_variable_temp(float **input, int rows, int cols,
                                 int row_pool, int col_pool, 
                                 float **output, float temperature) {
    int new_rows = rows / row_pool;
    int new_cols = cols / col_pool;
    int max_window_size = row_pool * col_pool;
    
    float *window = (float*)malloc(max_window_size * sizeof(float));
    float *weights = (float*)malloc(max_window_size * sizeof(float));
    
    for (int i = 0; i < new_rows; i++) {
        for (int j = 0; j < new_cols; j++) {
            int row_start = i * row_pool;
            int row_end = row_start + row_pool;
            int col_start = j * col_pool;
            int col_end = col_start + col_pool;
            
            if (row_end > rows) row_end = rows;
            if (col_end > cols) col_end = cols;
            
            int window_size = 0;
            for (int x = row_start; x < row_end; x++) {
                for (int y = col_start; y < col_end; y++) {
                    window[window_size++] = input[x][y];
                }
            }
            
            compute_weights_temp(window, window_size, weights, temperature);
            
            float weighted_sum = 0.0f;
            for (int k = 0; k < window_size; k++) {
                weighted_sum += window[k] * weights[k];
            }
            
            output[i][j] = weighted_sum;
        }
    }
    
    free(window);
    free(weights);
}
void mixed_pooling(float **input, int rows, int cols,
                 int row_pool, int col_pool, 
                 float **output, float alpha) {
    // alpha控制加权平均和简单平均的混合比例 (0.0-1.0)
    
    int new_rows = rows / row_pool;
    int new_cols = cols / col_pool;
    
    for (int i = 0; i < new_rows; i++) {
        for (int j = 0; j < new_cols; j++) {
            // 计算窗口边界
            int row_start = i * row_pool;
            int row_end = row_start + row_pool;
            int col_start = j * col_pool;
            int col_end = col_start + col_pool;
            
            // 确保不越界
            if (row_end > rows) row_end = rows;
            if (col_end > cols) col_end = cols;
            
            // 首先计算窗口内的最大值和简单平均
            float max_val = input[row_start][col_start];
            float simple_avg = 0.0f;
            int count = 0;
            
            for (int x = row_start; x < row_end; x++) {
                for (int y = col_start; y < col_end; y++) {
                    float val = input[x][y];
                    simple_avg += val;
                    count++;
                    
                    if (val > max_val) {
                        max_val = val;
                    }
                }
            }
            simple_avg /= count;
            
            // 计算加权平均（使用相对权重）
            float weighted_avg = 0.0f;
            float sum_weights = 0.0f;
            
            for (int x = row_start; x < row_end; x++) {
                for (int y = col_start; y < col_end; y++) {
                    // 使用相对权重（值/max_val）
                    float weight = input[x][y] / (max_val + 1e-6f);  // 加小量防止除以0
                    weighted_avg += input[x][y] * weight;
                    sum_weights += weight;
                }
            }
            weighted_avg /= sum_weights;
            
            // 混合两种结果
            output[i][j] = alpha * weighted_avg + (1 - alpha) * simple_avg;
        }
    }
}
												 
float inin[6][8];
float outout[6][8];
int test_mean_pooling(void) {
    // 原始数组大小
    int rows = 6, cols = 8;
    
    // 行列缩小比例 (行缩小2倍，列缩小4倍)
    int row_scale = 2, col_scale = 4;
    
    // 计算缩小后的大小
    int new_rows = rows / row_scale;
    int new_cols = cols / col_scale;
    
    // 分配内存
    float **input = allocate_matrix(rows, cols);
    float **output = allocate_matrix(new_rows, new_cols);
    
    // 初始化输入数组 (0-47)
    printf("Original matrix (%dx%d):\n", rows, cols);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            input[i][j] = i * cols + j;
					  inin[i][j] = i * cols + j;
            printf("%6.1f ", input[i][j]);
        }
        printf("\n");
    }
    
    // 执行缩小操作
    resize_matrix(input, rows, cols, row_scale, col_scale, output);
    
		 for (int i = 0; i < new_rows; i++) {
        for (int j = 0; j < new_cols; j++) {
					  outout[i][j] = output[i][j];
        }
        printf("\n");
    }
		
    // 打印结果
    printf("\nDownsampled matrix (%dx%d, row_scale=%d, col_scale=%d):\n", 
           new_rows, new_cols, row_scale, col_scale);
    print_matrix(output, new_rows, new_cols);
    
    // 释放内存
    free_matrix(input, rows);
    free_matrix(output, new_rows);
    
    return 0;
}
int test_weighted_avg_pooling(void) {
    // 初始化随机数种子
//    srand(time(NULL));
    srand(1);
    // 设置矩阵大小
    int rows = 6, cols = 8;
    
    // 设置行列不同的池化比例
    int row_pool = 2;  // 行方向缩小2倍
    int col_pool = 4;  // 列方向缩小5倍
    
    // 计算池化后的大小
    int new_rows = rows / row_pool;
    int new_cols = cols / col_pool;
    
    // 分配内存
    float **input = allocate_matrix(rows, cols);
    float **output = allocate_matrix(new_rows, new_cols);
    
    // 用随机值初始化输入矩阵
//    init_random_matrix(input, rows, cols);
		for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
					  input[i][j] = i * cols + j;
					  inin[i][j] = input[i][j];
        }
        printf("\n");
    }
    // 打印输入矩阵
    printf("Original matrix (%dx%d):\n", rows, cols);
    print_matrix(input, rows, cols);
    
    // 执行加权平均池化
    weighted_avg_pooling_variable(input, rows, cols, row_pool, col_pool, output);
   for (int i = 0; i < new_rows; i++) {
        for (int j = 0; j < new_cols; j++) {
					  outout[i][j] = output[i][j];
        }
        printf("\n");
    }
    // 打印输出矩阵
    printf("\nWeighted average pooled matrix (%dx%d, row_pool=%d, col_pool=%d):\n", 
           new_rows, new_cols, row_pool, col_pool);
    print_matrix(output, new_rows, new_cols);
    
    // 释放内存
    free_matrix(input, rows);
    free_matrix(output, new_rows);
    
    return 0;
}

int test_weighted_avg_pooling_tempera(void) {
    //srand(time(NULL));
    srand(1);
    int rows = 8, cols = 8;
    int row_pool = 2, col_pool = 2;
    float temperature = 2.0f;  // 温度参数越大，权重分布越平缓
    
    int new_rows = rows / row_pool;
    int new_cols = cols / col_pool;
    
    float **input = allocate_matrix(rows, cols);
    float **output = allocate_matrix(new_rows, new_cols);
    
    //    init_random_matrix(input, rows, cols);
		for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
					  input[i][j] = i * cols + j;
					  inin[i][j] = input[i][j];
        }
        printf("\n");
    }
    
    printf("Original matrix (%dx%d):\n", rows, cols);
    print_matrix(input, rows, cols);
    
    weighted_avg_pooling_variable_temp(input, rows, cols, row_pool, col_pool, output, temperature);
    for (int i = 0; i < new_rows; i++) {
        for (int j = 0; j < new_cols; j++) {
					  outout[i][j] = output[i][j];
        }
        printf("\n");
    }
    printf("\nWeighted average pooled matrix (temperature=%.1f):\n", temperature);
    print_matrix(output, new_rows, new_cols);
    
    free_matrix(input, rows);
    free_matrix(output, new_rows);
    
    return 0;
}
