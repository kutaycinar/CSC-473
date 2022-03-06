#include <stdio.h>

#include "jama_lu.h"

using namespace std ;

main()
{
        // example of solving a linear system A*X = b
        Array2D< double > A(2,2) ;
        A[0][0] = 1 ;
        A[1][0] = 2 ;
        A[0][1] = 1 ;
        A[1][1] = 1 ;

        Array1D< double > b(2)  ;
        b[0] = 3 ;
        b[1] = 2 ;

        JAMA::LU<double>  lu(A);
        Array1D< double > x(2)  ;
        x = lu.solve(b) ;

        printf("x = %lf %lf\n", x[0], x[1]) ;

        // example of inverting a matrix
        Array2D<double> I(2,2) ;
        Array2D<double> invA(2,2) ;
        // set the indentity I
        I[0][0] = 1 ;
        I[1][0] = 0 ;
        I[0][1] = 0 ;
        I[1][1] = 1 ;

        // solve A*invA = I
        JAMA::LU<double> invertMatrix(A) ;
        invA = invertMatrix.solve(I) ;
        printf("%lf %lf\n", invA[0][0], invA[0][1]) ;
        printf("%lf %lf\n", invA[1][0], invA[1][1]) ;

        return 0;
}
