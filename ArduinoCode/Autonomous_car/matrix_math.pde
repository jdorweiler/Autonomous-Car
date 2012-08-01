unsigned int matrix_error = 0;

void Vector_Cross_Product(float C[3], float A[3], float B[3])
{
    C[0] = (A[1] * B[2]) - (A[2] * B[1]);
    C[1] = (A[2] * B[0]) - (A[0] * B[2]);
    C[2] = (A[0] * B[1]) - (A[1] * B[0]);
  
    return;
}

void Vector_Scale(float C[3], float A[3], float b)
{
    for (int m = 0; m < 3; m++)
        C[m] = A[m] * b;
        
    return;
}

float Vector_Dot_Product(float A[3], float B[3])
{
    float result = 0.0;

    for (int i = 0; i < 3; i++) {
        result += A[i] * B[i];
    }
    
    return result;
}

void Vector_Add(float C[3], float A[3], float B[3])
{
    for (int m = 0; m < 3; m++)
        C[m] = A[m] + B[m];
        
    return;
}

void Vector_Add(float C[3][3], float A[3][3], float B[3][3])
{
    for (int m = 0; m < 3; m++)
        for (int n = 0; n < 3; n++)
            C[m][n] = A[m][n] + B[m][n];
}

void Matrix_Add(float C[3][3], float A[3][3], float B[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
           C[i][j] = A[i][j] + B[i][j];
        }
    }
}

void Matrix_Add(int n, int m, float *C, float *A, float *B)
{
    for (int i = 0; i < n*m; i++) {
       C[i] = A[i] + B[i];
    }
}

void Matrix_Subtract(int n, int m, float *C, float *A, float *B)
{
    for (int i = 0; i < n*m; i++) {
       C[i] = A[i] - B[i];
    }
}



// grabbed from MatrixMath library for Arduino
// http://arduino.cc/playground/Code/MatrixMath
// E.g., the equivalent Octave script:
//   A=[x; y; z];
//   B=[xx xy xz; yx yy yz; zx xy zz]; 
//   C=A*B;
// Would be called like this:
//   Matrix_Mulitply(1, 3, 3, C, A, B);
//
void Matrix_Multiply(int m, int p, int n, float *C, float *A, float *B)
{
    // A = input matrix (m x p)
    // B = input matrix (p x n)
    // m = number of rows in A
    // p = number of columns in A = number of rows in B
    // n = number of columns in B
    // C = output matrix = A*B (m x n)
    for (int i=0; i < m; i++) {
        for(int j=0; j < n; j++) {
            C[n*i+j] = 0;
            for (int k=0; k < p; k++) {
                C[i*n+j] += A[i*p+k] * B[k*n+j];
            }
        }
    }
        
    return;
}

void Matrix_Multiply(float C[3][3], float A[3][3], float B[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i][j] = 0;
            for (int k = 0; k < 3; k++) {
               C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}


void Matrix_Transpose(int n, int m, float *C, float *A)
{
    for (int i=0; i < n; i++) {
        for (int j=0; j < m; j++) {
            C[j*n+i] = A[i*m+j];
        }
    }
}

#define fabs(x) (((x) < 0) ? -x : x)

// grabbed from MatrixMath library for Arduino
// http://arduino.cc/playground/Code/MatrixMath
//Matrix Inversion Routine
// * This function inverts a matrix based on the Gauss Jordan method.
// * Specifically, it uses partial pivoting to improve numeric stability.
// * The algorithm is drawn from those presented in 
//     NUMERICAL RECIPES: The Art of Scientific Computing.
// * NOTE: The argument is ALSO the result matrix, meaning the input matrix is REPLACED
void Matrix_Inverse(int n, float *A)
{
    // A = input matrix AND result matrix
    // n = number of rows = number of columns in A (n x n)
    int pivrow=0;   // keeps track of current pivot row
    int k,i,j;        // k: overall index along diagonal; i: row index; j: col index
    int pivrows[n]; // keeps track of rows swaps to undo at end
    float tmp;        // used for finding max value and making column swaps
    
    for (k = 0; k < n; k++) {
        // find pivot row, the row with biggest entry in current column
        tmp = 0;
        for (i = k; i < n; i++) {
            if (fabs(A[i*n+k]) >= tmp) {    // 'Avoid using other functions inside abs()?'
                tmp = fabs(A[i*n+k]);
                pivrow = i;
            }
        }
        
        // check for singular matrix
        if (A[pivrow*n+k] == 0.0f) {
            //matrix_error |= SINGULAR_MATRIX;
            fprintf(stdout, "Inversion failed due to singular matrix");
            return;
        }
        
        // Execute pivot (row swap) if needed
        if (pivrow != k) {
            // swap row k with pivrow
            for (j = 0; j < n; j++) {
                tmp = A[k*n+j];
                A[k*n+j] = A[pivrow*n+j];
                A[pivrow*n+j] = tmp;
            }
        }
        pivrows[k] = pivrow;    // record row swap (even if no swap happened)
        
        tmp = 1.0f/A[k*n+k];    // invert pivot element
        A[k*n+k] = 1.0f;        // This element of input matrix becomes result matrix
        
        // Perform row reduction (divide every element by pivot)
        for (j = 0; j < n; j++) {
            A[k*n+j] = A[k*n+j]*tmp;
        }
        
        // Now eliminate all other entries in this column
        for (i = 0; i < n; i++) {
            if (i != k) {
                tmp = A[i*n+k];
                A[i*n+k] = 0.0f;  // The other place where in matrix becomes result mat
                for (j = 0; j < n; j++) {
                    A[i*n+j] = A[i*n+j] - A[k*n+j]*tmp;
                }
            }
        }
    }
    
    // Done, now need to undo pivot row swaps by doing column swaps in reverse order
    for (k = n-1; k >= 0; k--) {
        if (pivrows[k] != k) {
            for (i = 0; i < n; i++) {
                tmp = A[i*n+k];
                A[i*n+k] = A[i*n+pivrows[k]];
                A[i*n+pivrows[k]] = tmp;
            }
        }
    }
    return;
}


void Matrix_Copy(int n, int m, float *C, float *A)
{
    for (int i=0; i < n*m; i++)
        C[i] = A[i];
}

void Matrix_print(int n, int m, float *A, const char *name)
{
    fprintf(stdout, "%s=[", name);
    for (int i=0; i < n; i++) {
        for (int j=0; j < m; j++) {
            fprintf(stdout, "%5.5f", A[i*m+j]);
            if (j < m-1) fprintf(stdout, ", ");
        }
        if (i < n-1) fprintf(stdout, "; ");
    }
    fprintf(stdout, "]\n");
}  


void Vector_Print(float A[3], const char *name)
{
    fprintf(stdout, "%s=[ ", name);
    for (int i=0; i < 3; i++)
        fprintf(stdout, "%5.5f ", A[i]);
    fprintf(stdout, "]\n");
    
    return;
}


