/** @file miniblas.h
 * KFCore
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief Minimal generic BLAS implementation
 *
 * Note: all matrices are stored in column-major order.
 *
 * @{ */

/******************************************************************************
 * SYSTEM INCLUDE FILES
 ******************************************************************************/

/******************************************************************************
 * PROJECT INCLUDE FILES
 ******************************************************************************/
#include "types.h"

/******************************************************************************
 * DEFINES
 ******************************************************************************/

/******************************************************************************
 * TYPEDEFS
 ******************************************************************************/

/******************************************************************************
 * LOCAL DATA DEFINITIONS
 ******************************************************************************/

/******************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 ******************************************************************************/

/******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/
#ifndef GNCUTILS_EKF_KFCORE_MINIBLAS_H
#define GNCUTILS_EKF_KFCORE_MINIBLAS_H

namespace warpos {

    int lsame_(const char* a, const char* b);

    int strsm_(const char* side, const char* uplo, const char* transa, const char* diag, int* m,
              int* n, floating_point* alpha, const floating_point* a, int* lda, floating_point* b, int* ldb);

    int sgemm_(char* transa, char* transb, int* m, int* n, int* k, floating_point* alpha, floating_point* a, int* lda,
              floating_point* b, int* ldb, floating_point* beta, floating_point* c__, int* ldc);

    int ssyrk_(char* uplo, char* trans, int* n, int* k, floating_point* alpha, floating_point* a, int* lda,
              floating_point* beta, floating_point* c__, int* ldc);

    int ssymm_(char* side, char* uplo, int* m, int* n, floating_point* alpha, floating_point* a, int* lda, floating_point* b,
              int* ldb, floating_point* beta, floating_point* c__, int* ldc);

    int strmm_(const char* side, const char* uplo, const char* transa, const char* diag, int* m,
               int* n, floating_point* alpha, floating_point* a, int* lda, floating_point* b, int* ldb);

}
#endif

/* @} */
