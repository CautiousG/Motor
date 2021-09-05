/*
 * Matrix: C code implementation for basic matrix operation
 */

#ifndef __MATRIX_H
#define __MATRIX_H


#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct
{
    /* 缁村害 */
    int rows;
    int cols;
    /* 鍐呭 */
    float** data;
} Matrix;

Matrix alloc_matrix (int rows, int cols); /* 涓虹煩闃靛垎閰嶅垵濮嬬┖闂� */
//void free_matrix (Matrix m); /* 閲婃斁绌洪棿 */
void set_matrix (Matrix m, ...); /* 鍒濆鍖栫煩闃� */
void set_identity_matrix (Matrix m); /* 杞崲涓哄崟鍏冪煩闃� */
void copy_matrix (Matrix source, Matrix destination); /* 澶嶅埗鐭╅樀 */
void add_matrix (Matrix a, Matrix b, Matrix c); /* 鐭╅樀鐩稿姞 c = a + b */
void subtract_matrix (Matrix a, Matrix b, Matrix c); /* 鐭╅樀鐩稿噺 */
void subtract_from_identity_matrix (Matrix a); /* 鐢ㄥ崟鍏冪煩闃靛噺鍘昏鐭╅樀 */
void multiply_matrix (Matrix a, Matrix b, Matrix c); /* 鐭╅樀鐩镐箻 c = a*b */
//void multiply_matrix (Matrix *a, Matrix *b, Matrix *c); /* 鐭╅樀鐩镐箻 c = a*b */
void multiply_by_transpose_matrix (Matrix a, Matrix b, Matrix c); /* 涔樹互涓�涓煩闃电殑杞疆鐭╅樀. */
void transpose_matrix (Matrix input, Matrix output); /* 鐭╅樀杞疆 */
void scale_matrix (Matrix m, float scalar); /* 鐭╅樀涔樹互涓�涓郴鏁� */
void swap_rows (Matrix m, int r1, int r2); /* 浜ゆ崲鐭╅樀鐨勪袱琛� */
void scale_row (Matrix m, int r, float scalar); /* 鐭╅樀鏌愯涔樹互涓�涓郴鏁� */
void shear_row (Matrix m, int r1, int r2, float scalar); /* Add scalar * row r2 to row r1. */
int destructive_invert_matrix (Matrix input, Matrix output); /* 鐭╅樀姹傞�� */


// inline void multiply_matrix (Matrix a, Matrix b, Matrix c)
// {
    // int i;
    // int j;
    // int k;

    // for (i = 0; i < c.rows; ++i)
    // {
        // for (j = 0; j < c.cols; ++j)
        // {
            // c.data[i][j] = 0.0;

            // for (k = 0; k < a.cols; ++k)
            // {
                // c.data[i][j] += a.data[i][k] * b.data[k][j];
            // }
        // }
    // }
// }

// inline void copy_matrix (Matrix source, Matrix destination)
// {
    // int i;
    // int j;
    
    // for (i = 0; i < source.rows; ++i)
    // {
        // for (j = 0; j < source.cols; ++j)
        // {
            // destination.data[i][j] = source.data[i][j];
        // }
    // }
// }

#endif
