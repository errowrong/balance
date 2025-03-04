#include "martix.h"

void mat_init_f32(matrix_instance_f32* S, uint16_t nRows, uint16_t nColumns, float* pData)
{
    /* Assign Number of Rows */
    S->numRows = nRows;

    /* Assign Number of Columns */
    S->numCols = nColumns;

    /* Assign Data pointer */
    S->pData = pData;
}

arm_status mat_add_f32(const matrix_instance_f32* pSrcA, const matrix_instance_f32* pSrcB, matrix_instance_f32* pDst)
{
    float* pInA = pSrcA->pData;                /* input data matrix pointer A */
    float* pInB = pSrcB->pData;                /* input data matrix pointer B */
    float* pOut = pDst->pData;                 /* output data matrix pointer */

    uint32_t numSamples;                           /* total number of elements in the matrix */
    uint32_t blkCnt;                               /* loop counters */
    arm_status status;                             /* status of matrix addition */


    {
        /* Total number of samples in input matrix */
        numSamples = (uint32_t)pSrcA->numRows * pSrcA->numCols;

#if defined (ARM_MATH_LOOPUNROLL)

        /* Loop unrolling: Compute 4 outputs at a time */
        blkCnt = numSamples >> 2U;

        while (blkCnt > 0U)
        {
            /* C(m,n) = A(m,n) + B(m,n) */

            /* Add and store result in destination buffer. */
            *pOut++ = *pInA++ + *pInB++;

            *pOut++ = *pInA++ + *pInB++;

            *pOut++ = *pInA++ + *pInB++;

            *pOut++ = *pInA++ + *pInB++;

            /* Decrement loop counter */
            blkCnt--;
        }

        /* Loop unrolling: Compute remaining outputs */
        blkCnt = numSamples % 0x4U;

#else

        /* Initialize blkCnt with number of samples */
        blkCnt = numSamples;

#endif /* #if defined (ARM_MATH_LOOPUNROLL) */

        while (blkCnt > 0U)
        {
            /* C(m,n) = A(m,n) + B(m,n) */

            /* Add and store result in destination buffer. */
            *pOut++ = *pInA++ + *pInB++;

            /* Decrement loop counter */
            blkCnt--;
        }

        /* Set status as ARM_MATH_SUCCESS */
        status = ARM_MATH_SUCCESS;
    }

    /* Return to application */
    return (status);
}

arm_status mat_sub_f32(const matrix_instance_f32* pSrcA, const matrix_instance_f32* pSrcB, matrix_instance_f32* pDst)
{
    float* pInA = pSrcA->pData;                /* input data matrix pointer A */
    float* pInB = pSrcB->pData;                /* input data matrix pointer B */
    float* pOut = pDst->pData;                 /* output data matrix pointer */

    uint32_t numSamples;                           /* total number of elements in the matrix */
    uint32_t blkCnt;                               /* loop counters */
    arm_status status;                             /* status of matrix subtraction */

#ifdef ARM_MATH_MATRIX_CHECK

  /* Check for matrix mismatch condition */
    if ((pSrcA->numRows != pSrcB->numRows) ||
        (pSrcA->numCols != pSrcB->numCols) ||
        (pSrcA->numRows != pDst->numRows) ||
        (pSrcA->numCols != pDst->numCols))
    {
        /* Set status as ARM_MATH_SIZE_MISMATCH */
        status = ARM_MATH_SIZE_MISMATCH;
    }
    else

#endif /* #ifdef ARM_MATH_MATRIX_CHECK */

    {
        /* Total number of samples in input matrix */
        numSamples = (uint32_t)pSrcA->numRows * pSrcA->numCols;

#if defined (ARM_MATH_LOOPUNROLL)

        /* Loop unrolling: Compute 4 outputs at a time */
        blkCnt = numSamples >> 2U;

        while (blkCnt > 0U)
        {
            /* C(m,n) = A(m,n) - B(m,n) */

            /* Subtract and store result in destination buffer. */
            *pOut++ = (*pInA++) - (*pInB++);
            *pOut++ = (*pInA++) - (*pInB++);
            *pOut++ = (*pInA++) - (*pInB++);
            *pOut++ = (*pInA++) - (*pInB++);

            /* Decrement loop counter */
            blkCnt--;
        }

        /* Loop unrolling: Compute remaining outputs */
        blkCnt = numSamples % 0x4U;

#else

        /* Initialize blkCnt with number of samples */
        blkCnt = numSamples;

#endif /* #if defined (ARM_MATH_LOOPUNROLL) */

        while (blkCnt > 0U)
        {
            /* C(m,n) = A(m,n) - B(m,n) */

            /* Subtract and store result in destination buffer. */
            *pOut++ = (*pInA++) - (*pInB++);

            /* Decrement loop counter */
            blkCnt--;
        }

        /* Set status as ARM_MATH_SUCCESS */
        status = ARM_MATH_SUCCESS;
    }

    /* Return to application */
    return (status);
}

arm_status mat_mult_f32(const matrix_instance_f32* pSrcA, const matrix_instance_f32* pSrcB, matrix_instance_f32* pDst)
{
    float* pIn1 = pSrcA->pData;                /* Input data matrix pointer A */
    float* pIn2 = pSrcB->pData;                /* Input data matrix pointer B */
    float* pInA = pSrcA->pData;                /* Input data matrix pointer A */
    float* pInB = pSrcB->pData;                /* Input data matrix pointer B */
    float* pOut = pDst->pData;                 /* Output data matrix pointer */
    float* px;                                 /* Temporary output data matrix pointer */
    float sum;                                 /* Accumulator */
    uint16_t numRowsA = pSrcA->numRows;            /* Number of rows of input matrix A */
    uint16_t numColsB = pSrcB->numCols;            /* Number of columns of input matrix B */
    uint16_t numColsA = pSrcA->numCols;            /* Number of columns of input matrix A */
    uint32_t col, i = 0U, row = numRowsA, colCnt;  /* Loop counters */
    arm_status status;                             /* Status of matrix multiplication */

#ifdef ARM_MATH_MATRIX_CHECK

  /* Check for matrix mismatch condition */
    if ((pSrcA->numCols != pSrcB->numRows) ||
        (pSrcA->numRows != pDst->numRows) ||
        (pSrcB->numCols != pDst->numCols))
    {
        /* Set status as ARM_MATH_SIZE_MISMATCH */
        status = ARM_MATH_SIZE_MISMATCH;
    }
    else

#endif /* #ifdef ARM_MATH_MATRIX_CHECK */

    {
        /* The following loop performs the dot-product of each row in pSrcA with each column in pSrcB */
        /* row loop */
        do
        {
            /* Output pointer is set to starting address of row being processed */
            px = pOut + i;

            /* For every row wise process, column loop counter is to be initiated */
            col = numColsB;

            /* For every row wise process, pIn2 pointer is set to starting address of pSrcB data */
            pIn2 = pSrcB->pData;

            /* column loop */
            do
            {
                /* Set the variable sum, that acts as accumulator, to zero */
                sum = 0.0f;

                /* Initialize pointer pIn1 to point to starting address of column being processed */
                pIn1 = pInA;

#if defined (ARM_MATH_LOOPUNROLL)

                /* Loop unrolling: Compute 4 MACs at a time. */
                colCnt = numColsA >> 2U;

                /* matrix multiplication */
                while (colCnt > 0U)
                {
                    /* c(m,p) = a(m,1) * b(1,p) + a(m,2) * b(2,p) + .... + a(m,n) * b(n,p) */

                    /* Perform the multiply-accumulates */
                    sum += *pIn1++ * *pIn2;
                    pIn2 += numColsB;

                    sum += *pIn1++ * *pIn2;
                    pIn2 += numColsB;

                    sum += *pIn1++ * *pIn2;
                    pIn2 += numColsB;

                    sum += *pIn1++ * *pIn2;
                    pIn2 += numColsB;

                    /* Decrement loop counter */
                    colCnt--;
                }

                /* Loop unrolling: Compute remaining MACs */
                colCnt = numColsA % 0x4U;

#else

                /* Initialize cntCnt with number of columns */
                colCnt = numColsA;

#endif /* #if defined (ARM_MATH_LOOPUNROLL) */

                while (colCnt > 0U)
                {
                    /* c(m,p) = a(m,1) * b(1,p) + a(m,2) * b(2,p) + .... + a(m,n) * b(n,p) */

                    /* Perform the multiply-accumulates */
                    sum += *pIn1++ * *pIn2;
                    pIn2 += numColsB;

                    /* Decrement loop counter */
                    colCnt--;
                }

                /* Store result in destination buffer */
                *px++ = sum;

                /* Decrement column loop counter */
                col--;

                /* Update pointer pIn2 to point to starting address of next column */
                pIn2 = pInB + (numColsB - col);

            } while (col > 0U);

            /* Update pointer pInA to point to starting address of next row */
            i = i + numColsB;
            pInA = pInA + numColsA;

            /* Decrement row loop counter */
            row--;

        } while (row > 0U);

        /* Set status as ARM_MATH_SUCCESS */
        status = ARM_MATH_SUCCESS;
    }

    /* Return to application */
    return (status);
}

arm_status mat_trans_f32(const matrix_instance_f32* pSrc, matrix_instance_f32* pDst)
{
    float* pIn = pSrc->pData;                  /* input data matrix pointer */
    float* pOut = pDst->pData;                 /* output data matrix pointer */
    float* px;                                 /* Temporary output data matrix pointer */
    uint16_t nRows = pSrc->numRows;                /* number of rows */
    uint16_t nCols = pSrc->numCols;                /* number of columns */
    uint32_t col, row = nRows, i = 0U;             /* Loop counters */
    arm_status status;                             /* status of matrix transpose */

#ifdef ARM_MATH_MATRIX_CHECK

  /* Check for matrix mismatch condition */
    if ((pSrc->numRows != pDst->numCols) ||
        (pSrc->numCols != pDst->numRows))
    {
        /* Set status as ARM_MATH_SIZE_MISMATCH */
        status = ARM_MATH_SIZE_MISMATCH;
    }
    else

#endif /* #ifdef ARM_MATH_MATRIX_CHECK */

    {
        /* Matrix transpose by exchanging the rows with columns */
        /* row loop */
        do
        {
            /* Pointer px is set to starting address of column being processed */
            px = pOut + i;

#if defined (ARM_MATH_LOOPUNROLL)

            /* Loop unrolling: Compute 4 outputs at a time */
            col = nCols >> 2U;

            while (col > 0U)        /* column loop */
            {
                /* Read and store input element in destination */
                *px = *pIn++;
                /* Update pointer px to point to next row of transposed matrix */
                px += nRows;

                *px = *pIn++;
                px += nRows;

                *px = *pIn++;
                px += nRows;

                *px = *pIn++;
                px += nRows;

                /* Decrement column loop counter */
                col--;
            }

            /* Loop unrolling: Compute remaining outputs */
            col = nCols % 0x4U;

#else

            /* Initialize col with number of samples */
            col = nCols;

#endif /* #if defined (ARM_MATH_LOOPUNROLL) */

            while (col > 0U)
            {
                /* Read and store input element in destination */
                *px = *pIn++;

                /* Update pointer px to point to next row of transposed matrix */
                px += nRows;

                /* Decrement column loop counter */
                col--;
            }

            i++;

            /* Decrement row loop counter */
            row--;

        } while (row > 0U);          /* row loop end */

        /* Set status as ARM_MATH_SUCCESS */
        status = ARM_MATH_SUCCESS;
    }

    /* Return to application */
    return (status);
}

arm_status mat_inverse_f32(    const matrix_instance_f32* pSrc,    matrix_instance_f32* pDst)
{
    float* pIn = pSrc->pData;                  /* input data matrix pointer */
    float* pOut = pDst->pData;                 /* output data matrix pointer */

    float* pTmp;
    uint32_t numRows = pSrc->numRows;              /* Number of rows in the matrix  */
    uint32_t numCols = pSrc->numCols;              /* Number of Cols in the matrix  */


    float pivot = 0.0f, newPivot = 0.0f;                /* Temporary input values  */
    uint32_t selectedRow, pivotRow, i, rowNb, rowCnt, flag = 0U, j, column;      /* loop counters */
    arm_status status;                             /* status of matrix inverse */

#ifdef ARM_MATH_MATRIX_CHECK

  /* Check for matrix mismatch condition */
    if ((pSrc->numRows != pSrc->numCols) ||
        (pDst->numRows != pDst->numCols) ||
        (pSrc->numRows != pDst->numRows))
    {
        /* Set status as ARM_MATH_SIZE_MISMATCH */
        status = ARM_MATH_SIZE_MISMATCH;
    }
    else

#endif /* #ifdef ARM_MATH_MATRIX_CHECK */

    {
        /*--------------------------------------------------------------------------------------------------------------
         * Matrix Inverse can be solved using elementary row operations.
         *
         *  Gauss-Jordan Method:
         *
         *      1. First combine the identity matrix and the input matrix separated by a bar to form an
         *        augmented matrix as follows:
         *                      _                  _         _         _
         *                     |  a11  a12 | 1   0  |       |  X11 X12  |
         *                     |           |        |   =   |           |
         *                     |_ a21  a22 | 0   1 _|       |_ X21 X21 _|
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
         *      8. Now an identical matrix is formed to the left of the bar(input matrix, pSrc).
         *         Therefore, the matrix to the right of the bar is our solution(pDst matrix, pDst).
         *----------------------------------------------------------------------------------------------------------------*/

         /* Working pointer for destination matrix */
        pTmp = pOut;

        /* Loop over the number of rows */
        rowCnt = numRows;

        /* Making the destination matrix as identity matrix */
        while (rowCnt > 0U)
        {
            /* Writing all zeroes in lower triangle of the destination matrix */
            j = numRows - rowCnt;
            while (j > 0U)
            {
                *pTmp++ = 0.0f;
                j--;
            }

            /* Writing all ones in the diagonal of the destination matrix */
            *pTmp++ = 1.0f;

            /* Writing all zeroes in upper triangle of the destination matrix */
            j = rowCnt - 1U;
            while (j > 0U)
            {
                *pTmp++ = 0.0f;
                j--;
            }

            /* Decrement loop counter */
            rowCnt--;
        }

        /* Loop over the number of columns of the input matrix.
           All the elements in each column are processed by the row operations */

           /* Index modifier to navigate through the columns */
        for (column = 0U; column < numCols; column++)
        {
            /* Check if the pivot element is zero..
             * If it is zero then interchange the row with non zero row below.
             * If there is no non zero element to replace in the rows below,
             * then the matrix is Singular. */

            pivotRow = column;

            /* Temporary variable to hold the pivot value */
            pTmp = ELEM(pSrc, column, column);
            pivot = *pTmp;
            selectedRow = column;

            /* Find maximum pivot in column */

              /* Loop over the number rows present below */

            for (rowNb = column + 1; rowNb < numRows; rowNb++)
            {
                /* Update the input and destination pointers */
                pTmp = ELEM(pSrc, rowNb, column);
                newPivot = *pTmp;
                if (fabsf(newPivot) > fabsf(pivot))
                {
                    selectedRow = rowNb;
                    pivot = newPivot;
                }
            }

            /* Check if there is a non zero pivot element to
             * replace in the rows below */
            if ((pivot != 0.0f) && (selectedRow != column))
            {

                SWAP_ROWS_F32(pSrc, column, pivotRow, selectedRow);
                SWAP_ROWS_F32(pDst, 0, pivotRow, selectedRow);


                /* Flag to indicate whether exchange is done or not */
                flag = 1U;
            }





            /* Update the status if the matrix is singular */
            if ((flag != 1U) && (pivot == 0.0f))
            {
                return ARM_MATH_SINGULAR;
            }


            /* Pivot element of the row */
            pivot = 1.0f / pivot;

            SCALE_ROW_F32(pSrc, column, pivot, pivotRow);
            SCALE_ROW_F32(pDst, 0, pivot, pivotRow);


            /* Replace the rows with the sum of that row and a multiple of row i
             * so that each new element in column i above row i is zero.*/

            rowNb = 0;
            for (; rowNb < pivotRow; rowNb++)
            {
                pTmp = ELEM(pSrc, rowNb, column);
                pivot = *pTmp;

                MAS_ROW_F32(column, pSrc, rowNb, pivot, pSrc, pivotRow);
                MAS_ROW_F32(0, pDst, rowNb, pivot, pDst, pivotRow);


            }

            for (rowNb = pivotRow + 1; rowNb < numRows; rowNb++)
            {
                pTmp = ELEM(pSrc, rowNb, column);
                pivot = *pTmp;

                MAS_ROW_F32(column, pSrc, rowNb, pivot, pSrc, pivotRow);
                MAS_ROW_F32(0, pDst, rowNb, pivot, pDst, pivotRow);

            }

        }

        /* Set status as ARM_MATH_SUCCESS */
        status = ARM_MATH_SUCCESS;

        if ((flag != 1U) && (pivot == 0.0f))
        {
            pIn = pSrc->pData;
            for (i = 0; i < numRows * numCols; i++)
            {
                if (pIn[i] != 0.0f)
                    break;
            }

            if (i == numRows * numCols)
                status = ARM_MATH_SINGULAR;
        }
    }

    /* Return to application */
    return (status);
}
