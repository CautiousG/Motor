/* Matrix math. */

#include "matrix.h"


Matrix alloc_matrix (int rows, int cols)
{
    Matrix m;
    int i;
    int j;
    m.rows = rows;
    m.cols = cols;
    m.data = (float**) malloc (sizeof (float*) * m.rows);
    
    for (i = 0; i < m.rows; ++i)
    {
        m.data[i] = (float*) malloc (sizeof (float) * m.cols);
        
        for (j = 0; j < m.cols; ++j)
        {
            m.data[i][j] = 0.0;
        }
    }
    
    return m;
}

// void free_matrix (Matrix m)
// {
    // int i;
    
    // for (i = 0; i < m.rows; ++i)
    // {
        // free (m.data[i]);
    // }
    
    // free (m.data);
// }

void set_matrix (Matrix m, ...)
{
    va_list ap;
    int i, j;
    va_start (ap, m);
    
    for (i = 0; i < m.rows; ++i)
    {
        for (j = 0; j < m.cols; ++j)
        {
            m.data[i][j] = va_arg (ap, float);
        }
    }
    
    va_end (ap);
}

void set_identity_matrix (Matrix m)
{
    int i;
    int j;
    
    for (i = 0; i < m.rows; ++i)
    {
        for (j = 0; j < m.cols; ++j)
        {
            if (i == j)
            {
                m.data[i][j] = 1.0;
            }
            
            else
            {
                m.data[i][j] = 0.0;
            }
        }
    }
}

void copy_matrix (Matrix source, Matrix destination)
{
    int i;
    int j;
    
    for (i = 0; i < source.rows; ++i)
    {
        for (j = 0; j < source.cols; ++j)
        {
            destination.data[i][j] = source.data[i][j];
        }
    }
}

void add_matrix (Matrix a, Matrix b, Matrix c)
{
    int i;
    int j;
    
    for (i = 0; i < a.rows; ++i)
    {
        for (j = 0; j < a.cols; ++j)
        {
            c.data[i][j] = a.data[i][j] + b.data[i][j];
        }
    }
}

void subtract_matrix (Matrix a, Matrix b, Matrix c)
{
    int i;
    int j;
    
    for (i = 0; i < a.rows; ++i)
    {
        for (j = 0; j < a.cols; ++j)
        {
            c.data[i][j] = a.data[i][j] - b.data[i][j];
        }
    }
}

void subtract_from_identity_matrix (Matrix a)
{
    int i;
    int j;
    
    for (i = 0; i < a.rows; ++i)
    {
        for (j = 0; j < a.cols; ++j)
        {
            if (i == j)
            {
                a.data[i][j] = 1.0 - a.data[i][j];
            }
            
            else
            {
                a.data[i][j] = 0.0 - a.data[i][j];
            }
        }
    }
}

void multiply_matrix (Matrix a, Matrix b, Matrix c)
{
    int i;
    int j;
    int k;
    
    for (i = 0; i < c.rows; ++i)
    {
        for (j = 0; j < c.cols; ++j)
        {
            c.data[i][j] = 0.0;
            
            for (k = 0; k < a.cols; ++k)
            {
                c.data[i][j] += a.data[i][k] * b.data[k][j];
            }
        }
    }
}

// void multiply_matrix (Matrix *a, Matrix *b, Matrix *c)
// {
    // int i;
    // int j;
    // int k;
 	// Matrix aa;
	// Matrix bb;
	// Matrix cc;   
	// aa = *a;
	// bb = *b;
	// cc = *c;
	
    // for (i = 0; i < cc.rows; ++i)
    // {
        // for (j = 0; j < cc.cols; ++j)
        // {
            // cc.data[i][j] = 0.0;
            
            // for (k = 0; k < aa.cols; ++k)
            // {
                // cc.data[i][j] += aa.data[i][k] * bb.data[k][j];
            // }
        // }
    // }
// }

void multiply_by_transpose_matrix (Matrix a, Matrix b, Matrix c)
{
    int i;
    int j;
    int k;
    
    for (i = 0; i < c.rows; ++i)
    {
        for (j = 0; j < c.cols; ++j)
        {
            c.data[i][j] = 0.0;
            
            for (k = 0; k < a.cols; ++k)
            {
                c.data[i][j] += a.data[i][k] * b.data[j][k];
            }
        }
    }
}

void transpose_matrix (Matrix input, Matrix output)
{
    int i;
    int j;
    //int k;
    
    for (i = 0; i < input.rows; ++i)
    {
        for (j = 0; j < input.cols; ++j)
        {
            output.data[j][i] = input.data[i][j];
        }
    }
}

void scale_matrix (Matrix m, float scalar)
{
    int i;
    int j;
    
    for (i = 0; i < m.rows; ++i)
    {
        for (j = 0; j < m.cols; ++j)
        {
            m.data[i][j] *= scalar;
        }
    }
}

void swap_rows (Matrix m, int r1, int r2)
{
    float *tmp;
    tmp = m.data[r1];
    m.data[r1] = m.data[r2];
    m.data[r2] = tmp;
}

void scale_row (Matrix m, int r, float scalar)
{
    int i;
    
    for (i = 0; i < m.cols; ++i)
    {
        m.data[r][i] *= scalar;
    }
}

void shear_row (Matrix m, int r1, int r2, float scalar)
{
    int i;
    
    for (i = 0; i < m.cols; ++i)
    {
        m.data[r1][i] += scalar * m.data[r2][i];
    }
}

int destructive_invert_matrix (Matrix input, Matrix output)
{
    int i;
    int j;
    int r;
    float scalar;
    float shear_needed;

    set_identity_matrix (output);
    
    
    for (i = 0; i < input.rows; ++i)
    {
        if (input.data[i][i] == 0.0)
        {
            for (r = i + 1; r < input.rows; ++r)
            {
                if (input.data[r][i] != 0.0)
                {
                    break;
                }
            }
            
            if (r == input.rows)
            {
                return 0;
            }
            
            swap_rows (input, i, r);
            swap_rows (output, i, r);
        }
        
        scalar = 1.0 / input.data[i][i];
        scale_row (input, i, scalar);
        scale_row (output, i, scalar);
        
        for (j = 0; j < input.rows; ++j)
        {
            if (i == j)
            {
                continue;
            }
            
            shear_needed = -input.data[j][i];
            shear_row (input, j, i, shear_needed);
            shear_row (output, j, i, shear_needed);
        }
    }
    
    return 1;
}


