/* THIS IS A SIMPLE TEST PROGRAM */

#include "linear.h"
#include "basic.h"

int main(int argc, char *argv[])
{
	int n = 27, p=18;
	int i,j;

	// initialize A
	Matd A (n,p,0.0);

	srand(time(NULL));

	for (i=0;i<n;i++) {
		for (j=0;j<n;j++) {
			double r = rand()/100;
			A[i][j] = r*30.0;
			printf("%.4f ",r);
		}
		printf("\n");
	}
	printf("\n");

	// solve A
	Vecd x = linearSolver(A);

	// print solution
	printf("solution for x: \n");
	for (i=0;i<p;i++)
		printf("%.4f ",x[i]);
	printf("\n");

	// check solution against random unit vectors
	printf("checking solution...\n");
	for (i=0;i<10000;i++) {

		Vecd xc(p,0.0);

		for(j=0;j<p;j++) {
			xc[j] = rand()/100;
		}

		xc = norm(xc);

		assert(len(A*xc) > len(A*x));

	}
	printf("done.\n");

	return 0;

}
