#include <stdlib.h>
#include <math.h>
#include "stat.h"

namespace EDOGMapping {
	double pf_ran_gaussian(double sigma)
	{
		double x1, x2, w, r;

		do
		{
			do { r = drand48(); } while (r==0.0);
			x1 = 2.0 * r - 1.0;
			do { r = drand48(); } while (r==0.0);
			x2 = 2.0 * r - 1.0;
			w = x1*x1 + x2*x2;
		} while(w > 1.0 || w==0.0);
			
		return(sigma * x2 * sqrt(-2.0*log(w)/w));
	}
	
	double sampleGaussian(double sigma, unsigned long int S)
	{
		if (S!=0)
			srand48(S);
		if (sigma==0)
			return 0;
		return pf_ran_gaussian (sigma);
	}

} // end namespace
