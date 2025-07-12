#include "Matrix.h"

void Matrix_init(Matrix * S,uint16_t nRows,uint16_t nColumns,float * pData){
   /* Assign Number of Rows */
   S->numRows = nRows;
   /* Assign Number of Columns */
   S->numCols = nColumns;
   /* Assign Data pointer */
   S->pData = pData;
 }

res_status Matrix_add(const Matrix * pSrcA,const Matrix * pSrcB,Matrix * pDst){
   float *pIn1 = pSrcA->pData;                /* input data matrix pointer A  */
   float *pIn2 = pSrcB->pData;                /* input data matrix pointer B  */
   float *pOut = pDst->pData;                 /* output data matrix pointer   */
   uint32_t numSamples;                           /* total number of elements in the matrix  */
   uint32_t blkCnt;                               /* loop counters */
   numSamples = (uint32_t) pSrcA->numRows * pSrcA->numCols;
   blkCnt = numSamples;
   while (blkCnt > 0U){
         /* C(m,n) = A(m,n) + B(m,n) */
         /* Add and then store the results in the destination buffer. */
         *pOut++ = (*pIn1++) + (*pIn2++);

         /* Decrement the loop counter */
         blkCnt--;
   }
   return (MATH_SUCCESS);
 }

 res_status Matrix_sub(const Matrix * pSrcA,const Matrix * pSrcB,Matrix * pDst){
   float *pIn1 = pSrcA->pData;                /* input data matrix pointer A */
   float *pIn2 = pSrcB->pData;                /* input data matrix pointer B */
   float *pOut = pDst->pData;                 /* output data matrix pointer  */
   uint32_t numSamples;                       /* total number of elements in the matrix  */
   uint32_t blkCnt;                           /* loop counters */
   numSamples = (uint32_t) pSrcA->numRows * pSrcA->numCols;
   blkCnt = numSamples;
   while (blkCnt > 0U){
    /* C(m,n) = A(m,n) - B(m,n) */
    /* Subtract and then store the results in the destination buffer. */
    *pOut++ = (*pIn1++) - (*pIn2++);
    /* Decrement the loop counter */
    blkCnt--;
   }
   return (MATH_SUCCESS);
 }

 res_status Matrix_mult(const Matrix * pSrcA,const Matrix* pSrcB, Matrix * pDst){
   float *pIn1 = pSrcA->pData;                /* input data matrix pointer A */
   float *pIn2 = pSrcB->pData;                /* input data matrix pointer B */
   float *pInA = pSrcA->pData;                /* input data matrix pointer A  */
   float *pOut = pDst->pData;                 /* output data matrix pointer */
   float *px;                                 /* Temporary output data matrix pointer */
   float sum;                                 /* Accumulator */
   uint16_t numRowsA = pSrcA->numRows;            /* number of rows of input matrix A */
   uint16_t numColsB = pSrcB->numCols;            /* number of columns of input matrix B */
   uint16_t numColsA = pSrcA->numCols;            /* number of columns of input matrix A */
   float *pInB = pSrcB->pData;                /* input data matrix pointer B */
   uint16_t col, i = 0U, row = numRowsA, colCnt;  /* loop counters */
   do{
                            /* Output pointer is set to starting address of the row being processed */
     px = pOut + i;         /* For every row wise process, the column loop counter is to be initiated */
     col = numColsB;        /* For every row wise process, the pIn2 pointer is set to the starting address of the pSrcB data */
     pIn2 = pSrcB->pData;   /* column loop */
     do
     {
       /* Set the variable sum, that acts as accumulator, to zero */
       sum = 0.0f;          /* Initialize the pointer pIn1 to point to the starting address of the row being processed */
       pIn1 = pInA;         /* Matrix A columns number of MAC operations are to be performed */
       colCnt = numColsA;
       while (colCnt > 0U)
       {
         /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
         sum += *pIn1++ * (*pIn2);
         pIn2 += numColsB;
         colCnt--;          /* Decrement the loop counter */
       }
       *px++ = sum;         /* Store the result in the destination buffer */
       col--;               /* Decrement the column loop counter */
       pIn2 = pInB + (numColsB - col); /* Update the pointer pIn2 to point to the  starting address of the next column */

     } while (col > 0U);
     i = i + numColsB;
     pInA = pInA + numColsA;    /* Update the pointer pInA to point to the  starting address of the next row */
     row--;                     /* Decrement the row loop counter */
   } while (row > 0U);
   return (MATH_SUCCESS);
 }

 res_status Matrix_trans(const Matrix * pSrc,Matrix * pDst){
    float *pIn = pSrc->pData;                  /* input data matrix pointer */
    float *pOut = pDst->pData;                 /* output data matrix pointer */
    float *px;                                 /* Temporary output data matrix pointer */
    uint16_t nRows = pSrc->numRows;                /* number of rows */
    uint16_t nColumns = pSrc->numCols;             /* number of columns */
    uint16_t col, i = 0U, row = nRows;             /* loop counters */
    do{

        px = pOut + i;  /* The pointer px is set to starting address of the column being processed */
        col = nColumns;    /* Initialize column loop counter */
         while (col > 0U){
            *px = *pIn++; /* Read and store the input element in the destination */
            px += nRows; /* Update the pointer px to point to the next row of the transposed matrix */
            col--; /* Decrement the column loop counter */
          }
         i++;
         row--; /* Decrement the row loop counter */

        } while (row > 0U);
    return (MATH_SUCCESS);
 }


res_status Matrix_inverse(const Matrix * pSrc,Matrix * pDst){
      float *pIn = pSrc->pData;                  /* input data matrix pointer */
      float *pOut = pDst->pData;                 /* output data matrix pointer */
      float *pInT1, *pInT2;                      /* Temporary input data matrix pointer */
      float *pOutT1, *pOutT2;                    /* Temporary output data matrix pointer */
      float *pPivotRowIn, *pPRT_in, *pPivotRowDst, *pPRT_pDst;  /* Temporary input and output data matrix pointer */
      uint32_t numRows = pSrc->numRows;              /* Number of rows in the matrix  */
      uint32_t numCols = pSrc->numCols;              /* Number of Cols in the matrix  */
      float Xchg, in = 0.0f;                     /* Temporary input values  */
      uint32_t i, rowCnt, flag = 0U, j, loopCnt, k, l;      /* loop counters */
      /*--------------------------------------------------------------------------------------------------------------
          * Matrix Inverse can be solved using elementary row operations.
          *
          *  Gauss-Jordan Method:
          *
          *     1. First combine the identity matrix and the input matrix separated by a bar to form an
          *        augmented matrix as follows:
          *                      _  _          _     _      _   _         _         _
          *                     |  |  a11  a12  | | | 1   0  |   |       |  X11 X12  |
          *                     |  |            | | |        |   |   =   |           |
          *                     |_ |_ a21  a22 _| | |_0   1 _|  _|       |_ X21 X21 _|
          *
          *      2. In our implementation, pDst Matrix is used as identity matrix.
          *
          *      3. Begin with the first row. Let i = 1.
          *
          *      4. Check to see if the pivot for row i is zero.
          *         The pivot is the element of the main diagonal that is on the current row.
          *         For instance, if working with row i, then the pivot element is aii.
          *         If the pivot is zero, exchange that row with a row below it that does not
          *         contain a zero in column i. If this is not possible, then an inverse
          *         to that matrix does not exist.
          *
          *      5. Divide every element of row i by the pivot.
          *
          *      6. For every row below and  row i, replace that row with the sum of that row and
          *         a multiple of row i so that each new element in column i below row i is zero.
          *
          *      7. Move to the next row and column and repeat steps 2 through 5 until you have zeros
          *         for every element below and above the main diagonal.
          *
          *      8. Now an identical matrix is formed to the left of the bar(input matrix, src).
          *         Therefore, the matrix to the right of the bar is our solution(dst matrix, dst).
          *----------------------------------------------------------------------------------------------------------------*/

         pOutT1 = pOut;  /* Working pointer for destination matrix */
         rowCnt = numRows; /* Loop over the number of rows */
         while (rowCnt > 0U){ /* Making the destination matrix as identity matrix */
           j = numRows - rowCnt; /* Writing all zeroes in lower triangle of the destination matrix */
           while (j > 0U){
             *pOutT1++ = 0.0f;
             j--;
           }

           *pOutT1++ = 1.0f; /* Writing all ones in the diagonal of the destination matrix */
           j = rowCnt - 1U;  /* Writing all zeroes in upper triangle of the destination matrix */
           while (j > 0U) {
             *pOutT1++ = 0.0f;
             j--;
           }
           rowCnt--;        /* Decrement the loop counter */
         }
         loopCnt = numCols; /* Loop over the number of columns of the input matrix. All the elements in each column are processed by the row operations */
         l = 0U; /* Index modifier to navigate through the columns */
         while (loopCnt > 0U) {
           /* Check if the pivot element is zero..
            * If it is zero then interchange the row with non zero row below.
            * If there is no non zero element to replace in the rows below,
            * then the matrix is Singular. */
           pInT1 = pIn + (l * numCols);          /* Working pointer for the input matrix that points to the pivot element of the particular row  */
           pOutT1 = pOut + (l * numCols);       /* Working pointer for the destination matrix that points to the pivot element of the particular row  */
           in = *pInT1;                         /* Temporary variable to hold the pivot value */
           k = 1U;                              /* Destination pointer modifier */
           if (*pInT1 == 0.0f) {                 /* Check if the pivot element is zero */
             for (i = (l + 1U); i < numRows; i++) { /* Loop over the number rows present below */
               pInT2 = pInT1 + (numCols * l); /* Update the input and destination pointers */
               pOutT2 = pOutT1 + (numCols * k);
               if (*pInT2 != 0.0f) {    /* Check if there is a non zero pivot element to replace in the rows below */
                 /* Loop over number of columns
                  * to the right of the pilot element */
                 for (j = 0U; j < (numCols - l); j++) {  /* Exchange the row elements of the input matrix */
                   Xchg = *pInT2;
                   *pInT2++ = *pInT1;
                   *pInT1++ = Xchg;
                 }
                 for (j = 0U; j < numCols; j++){
                   Xchg = *pOutT2;
                   *pOutT2++ = *pOutT1;
                   *pOutT1++ = Xchg;
                 }
                 flag = 1U;  /* Flag to indicate whether exchange is done or not */
                 break;  /* Break after exchange is done */
               }
               k++;  /* Update the destination pointer modifier */
             }
           }
           pPivotRowIn = pIn + (l * numCols);  /* Points to the pivot row of input and destination matrices */
           pPivotRowDst = pOut + (l * numCols);
           pInT1 = pPivotRowIn;                /* Temporary pointers to the pivot row pointers */
           pOutT1 = pPivotRowDst;
           in = *(pIn + (l * numCols));        /* Pivot element of the row */
           for (j = 0U; j < (numCols - l); j++) {   /* Loop over number of columns to the right of the pilot element */
                  *pInT1 = *pInT1 / in;  /* Divide each element of the row of the input matrix by the pivot element */
                  pInT1++;
          }
          for (j = 0U; j < numCols; j++) {
            *pOutT1 = *pOutT1 / in;  /* Divide each element of the row of the destination matrix by the pivot element */
            pOutT1++;
          }

          /* Replace the rows with the sum of that row and a multiple of row i
           * so that each new element in column i above row i is zero.*/
           /* Temporary pointers for input and destination matrices */
           pInT1 = pIn;
           pOutT1 = pOut;
           for (i = 0U; i < numRows; i++){
             /* Check for the pivot element */
            if (i == l){
             pInT1 += numCols - l; /* If the processing element is the pivot element, only the columns to the right are to be processed */
             pOutT1 += numCols;
             }
             else{
             in = *pInT1;  /* Element of the reference row */
             /* Working pointers for input and destination pivot rows */
             pPRT_in = pPivotRowIn;
             pPRT_pDst = pPivotRowDst;
           for (j = 0U; j < (numCols - l); j++){ /* Loop over the number of columns to the right of the pivot element, to replace the elements in the input matrix */
               *pInT1 = *pInT1 - (in * *pPRT_in++); /* Replace the element by the sum of that row and a multiple of the reference row  */
               pInT1++;
           }
           for (j = 0U; j < numCols; j++){  /* Loop over the number of columns to replace the elements in the destination matrix */
                *pOutT1 = *pOutT1 - (in * *pPRT_pDst++); /* Replace the element by the sum of that row and a multiple of the reference row  */
                pOutT1++;
              }
           }
           pInT1 = pInT1 + l; /* Increment the temporary input pointer */
         }
         pIn++;      /* Increment the input pointer */
         loopCnt--; /* Decrement the loop counter */
         l++; /* Increment the index modifier */
       }
  return (MATH_SUCCESS);
}



