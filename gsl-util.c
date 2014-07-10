/** File: gsl-util.c
	Description: implement the GSL utility functions
	Date: 11/06/2009
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
*/

#include "common.h"

#include "gsl-util.h"

//#include <math.h> //for tgamma()
#include <gsl/gsl_sf_gamma.h> //gsl_sf_gamma()
#include <gsl/gsl_randist.h> //the functions for random variates and probability density functions 

#include <stdlib.h> //strtoll() - convert a string to a long integer

void GSL_test()
{ //test function for gsl library
/*     double cdf = 0; */
/*     double X = 0.03, sigma = 1; */

/*     cdf = gsl_cdf_gaussian_P(X, sigma); */
/*     printf("gsl_test(): P(x<=X)=%.4f where X=%.4f and sigma=%.4f\n", cdf, X, sigma);   */

    struct H_params params = {17.882, 2.235, 1000}; //(vehicle_mean_speed, vehicle_speed_standard_deviation, road_length)
    //struct H_params params = {100, 20};
    //struct H_params params = {0, 1};

    double interval_start = 11.176; //minimum speed: interval's start; note that this interval start should be non-negative
    double interval_end = 24.587; //maximum speed: interval's end
    //double interval_start = 0; //interval's start; note that this interval start should be non-negative
    //double interval_end = 120; //interval's end
    //double interval_end = INF; //interval's end
    //double interval_end = 0.1; //interval's end
    double result = 0;

    double mean = 0; //mean travel time
    double second_moment = 0; //second moment of travel time
    double variance = 0; //travel time variance
    double standard_deviation = 0; //travel time standard deviation

    result = GSL_function_integral(&params, interval_start, interval_end);
    printf("result=%.4f\n", (float)result);
    
    /** compute the mean and standard deviation of travel time for the road length */
    mean = GSL_function_integral_for_mean(&params, interval_start, interval_end);
    second_moment = GSL_function_integral_for_second_moment(&params, interval_start, interval_end);
    variance = second_moment - pow(mean, 2);
    standard_deviation = sqrt(variance);

    printf("mean=%.4f and standard_deviation=%.4f\n", (float)mean, (float)standard_deviation);
}

void GSL_test_for_gamma()
{ //test function for gsl library for Gamma distribution
/*     double cdf = 0; */
/*     double X = 0.03, sigma = 1; */

/*     cdf = gsl_cdf_gaussian_P(X, sigma); */
/*     printf("gsl_test(): P(x<=X)=%.4f where X=%.4f and sigma=%.4f\n", cdf, X, sigma);   */

    struct H_params params = {17.882, 2.235, 1000}; //(vehicle_mean_speed, vehicle_speed_standard_deviation, road_length)
    //struct H_params params = {100, 20};
    //struct H_params params = {0, 1};

    struct H_params params_for_gamma = {0, 0, 0}; //(mean_travel_time, travel_time_standard_deviation, road_length)

    double interval_start = 11.176; //minimum speed: interval's start; note that this interval start should be non-negative
    double interval_end = 24.587; //maximum speed: interval's end
    //double interval_start = 0; //interval's start; note that this interval start should be non-negative
    //double interval_end = 120; //interval's end
    //double interval_end = INF; //interval's end
    //double interval_end = 0.1; //interval's end
    double result = 0;

    double mean = 0; //mean travel time
    double second_moment = 0; //second moment of travel time
    double variance = 0; //travel time variance
    double standard_deviation = 0; //travel time standard deviation

    double probability = 0; //probabilty for the given interval

    //result = GSL_function_integral(&params, interval_start, interval_end);
    //printf("result=%.4f\n", (float)result);
    
    /** compute the mean and standard deviation of travel time for the road length */
    mean = GSL_function_integral_for_mean(&params, interval_start, interval_end);
    second_moment = GSL_function_integral_for_second_moment(&params, interval_start, interval_end);
    variance = second_moment - pow(mean, 2);
    standard_deviation = sqrt(variance);

    printf("mean=%.4f and standard_deviation=%.4f\n", (float)mean, (float)standard_deviation);

    /* set up the parameters for Gamma distribution */
    params_for_gamma.mu = mean;
    params_for_gamma.sigma = standard_deviation;
    params_for_gamma.length = params.length;

    /* compute the probability for the given interval */
    interval_start = 0;
    interval_end = params_for_gamma.mu;

    probability = GSL_function_integral_for_gamma(&params_for_gamma, interval_start, interval_end);

    printf("probability=%.4f for the interval [%.4f, %.4f]\n", (float)probability, (float)interval_start, (float)interval_end);
}

double H(double y, void *params)
{ //compute the probability for a Gaussian random variable
    double H = 0;
    struct H_params *p = (struct H_params*)params;
    double mu = p->mu;
    double sigma = p->sigma;

    //H = sigma*(gsl_cdf_gaussian_P((y-mu)/sigma, sigma) - gsl_cdf_gaussian_P(-mu/sigma, sigma));
    //H = gsl_ran_gaussian_pdf((y-mu)/sigma, 1);
    H = gsl_ran_gaussian_pdf(0, 1);
    //H = 1/sqrt(2*PI*sigma*sigma) * exp(-y*y / (2*sigma*sigma)); //Gaussian pdf
    H = 1/sqrt(2*PI*sigma*sigma) * exp(-pow(y-mu, 2) / (2*sigma*sigma)); //Gaussian pdf

    return H;
}

double H_for_gamma(double y, void *params)
{ //compute the probability for a Gamma random variable
    double H = 0;
    struct H_params *p = (struct H_params*)params;
    double mu = p->mu;
    double sigma = p->sigma;

    double absolute_mu = 0; //absolute mu, that is, absolute mean
    double absolute_sigma = 0; //absolute sigma, that is, absolute sigma
    double kappa = 0; //parameter kappa for Gamma distribution
    double theta = 0; //parameter theta for Gamma distribution
    double gamma_value = 0; //gamma function value

    //H = sigma*(gsl_cdf_gaussian_P((y-mu)/sigma, sigma) - gsl_cdf_gaussian_P(-mu/sigma, sigma));
    //gsl_ran_gaussian_pdf(y, sigma);
    //H = 1/sqrt(2*PI*sigma*sigma) * exp(-y*y / (2*sigma*sigma)); //Gaussian pdf
    //H = 1/sqrt(2*PI*sigma*sigma) * exp(-pow(y-mu, 2) / (2*sigma*sigma)); //Gaussian pdf

    /** perform the CDF difference */
    /* check the validity of mu */
    absolute_mu = fabs(mu);
    if(absolute_mu <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("GSL_function_integral_for_gamma(): Error: params->mu is zero\n", (float)absolute_mu);
        exit(1);
    }

    /* check the validity of sigma */
    absolute_sigma = fabs(sigma);
    if(absolute_sigma <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("GSL_function_integral_for_gamma(): Error: params->sigma is zero\n", (float)absolute_sigma);
        exit(1);
    }

    /* compute the parameter theta */
    theta = pow(absolute_sigma,2) / absolute_mu;

    /* compute the parameter kappa */
    kappa = absolute_mu / theta;

    /* compute the PDF for y */
    //H = gsl_ran_gamma_pdf(y, kappa, theta); //Gamma pdf
    //gamma_value = gamma(kappa); //logarithm of the gamma function
    //gamma_value = tgamma(kappa); //true the gamma function
 
    /* @Note that kappa_v(>=GSL::Sf::GAMMA_XMAX = 171.0)  makes gamma function return Infinite, so we need to use the implementation of the PDF of the Gamma function from GSL instead of making our own built gamma function-based PDF
     */
    //gamma_value = gsl_sf_gamma(kappa); //true the gamma function
    //H = pow(y, kappa-1) * exp(-y/theta) / (gamma_value * pow(theta, kappa));

    H = gsl_ran_gamma_pdf(y, kappa, theta); //the PDF of the Gamma distribution

    return H;
}

double H_for_mean(double y, void *params)
{ //compute the portion of the mean of a Gaussian random variable
    double H = 0;
    struct H_params *p = (struct H_params*)params;
    double mu = p->mu;
    double sigma = p->sigma;
    double length = p->length;

    if(y < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("H_for_mean(): Error: y(=%.2f) should be greater than %e\n", (float)y, ERROR_TOLERANCE_FOR_REAL_ARITHMETIC);
        exit(1);
    }
    
    H = length/y * 1/sqrt(2*PI*sigma*sigma) * exp(-pow(y-mu, 2) / (2*sigma*sigma)); //value * Gaussian pdf

    return H;
}

double H_for_second_moment(double y, void *params)
{ //compute the portion of the second moment of a Gaussian random variable
    double H = 0;
    struct H_params *p = (struct H_params*)params;
    double mu = p->mu;
    double sigma = p->sigma;
    double length = p->length;

    if(y < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("H_for_second_moment(): Error: y(=%.2f) should be greater than %e\n", (float)y, ERROR_TOLERANCE_FOR_REAL_ARITHMETIC);
        exit(1);
    }
    
    H = pow(length,2)/pow(y,2) * 1/sqrt(2*PI*sigma*sigma) * exp(-pow(y-mu, 2) / (2*sigma*sigma)); //value * Gaussian pdf

    return H;
}

double GSL_function_integral(struct H_params *params, double interval_start, double interval_end)
//double Function_Integral((double) (*h)(double, double, double), struct H_params *params, double interval_start, double interval_end)
{ //compute the integral of function h for the interval between int_start and int_end
    double result = 0;
    double error = 0;
    size_t limit = MAXIMUM_SUBINTERVAL_NUNBER; //maximum number of subintervals
    gsl_integration_workspace *w = NULL; //work space for integration
    gsl_function F; //gsl function structure
   
    double integral = 0; //integration result
    double y = interval_end; //upper bound for integration
    double mu = params->mu;
    double sigma = params->sigma;

    F.function = &H; //probability function to calculate the probability for argument y
    F.params = params; //parameters

    /* allocate a workspace sufficient to hold n double precision intervals, their integration results and error estimates. */
    w = gsl_integration_workspace_alloc(limit);

    /* perform the integration for the function F for the interval (interval_start, interval_end) */
    gsl_integration_qags(&F, interval_start, interval_end, 0, 1e-7, limit, w, &result, &error);

    //integral = gsl_cdf_gaussian_P((y-mu)/sigma, sigma) - gsl_cdf_gaussian_P(-mu/sigma, sigma);
    integral = gsl_cdf_gaussian_P((y-mu)/sigma, 1) - gsl_cdf_gaussian_P(-mu/sigma, 1);

    /* free the memory for the workspace */
    gsl_integration_workspace_free(w);

    return result;
}


double GSL_function_integral_for_gamma(struct H_params *params, double interval_start, double interval_end)
{ //compute the integral of function h for the interval between int_start and int_end for Gamma distribution
    double result = 0;
    double error = 0;
    size_t limit = MAXIMUM_SUBINTERVAL_NUNBER; //maximum number of subintervals
    gsl_integration_workspace *w = NULL; //work space for integration
    gsl_function F; //gsl function structure
   
    double integral = 0; //integration result
    double y = interval_end; //upper bound for integration
    double absolute_mu = 0; //absolute mu, that is, absolute mean
    double absolute_sigma = 0; //absolute sigma, that is, absolute sigma
    double kappa = 0; //parameter kappa for Gamma distribution
    double theta = 0; //parameter theta for Gamma distribution

    double cdf_for_interval_start = 0; //CDF for the interval start
    double cdf_for_interval_end = 0; //CDF for the interval end

    F.function = &H_for_gamma; //probability function to calculate the probability for argument y
    F.params = params; //parameters

    /* allocate a workspace sufficient to hold n double precision intervals, their integration results and error estimates. */
    w = gsl_integration_workspace_alloc(limit);

    /** perform the integration for the function F for the interval (interval_start, interval_end) */
    gsl_integration_qags(&F, interval_start, interval_end, 0, 1e-7, limit, w, &result, &error);

    /** perform the CDF difference */
    /* check the validity of mu */
    absolute_mu = fabs(params->mu);
    if(absolute_mu <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("GSL_function_integral_for_gamma(): Error: params->mu is zero\n", (float)absolute_mu);
        exit(1);
    }

    /* check the validity of sigma */
    absolute_sigma = fabs(params->sigma);
    if(absolute_sigma <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("GSL_function_integral_for_gamma(): Error: params->sigma is zero\n", (float)absolute_sigma);
        exit(1);
    }

    /* compute the parameter theta */
    theta = pow(absolute_sigma,2) / absolute_mu;

    /* compute the parameter kappa */
    kappa = absolute_mu / theta;

    cdf_for_interval_start = gsl_cdf_gamma_P(interval_start, kappa, theta);
    cdf_for_interval_end = gsl_cdf_gamma_P(interval_end, kappa, theta);

    integral = cdf_for_interval_end - cdf_for_interval_start;

    /* print the computation results of two methods */
    printf("gsl_integration_qags()'s result=%.3f\n", (float)result);
    printf("cdf difference =%.3f\n", (float)integral);

    /* free the memory for the workspace */
    gsl_integration_workspace_free(w);

    return result;
}

double GSL_function_integral_for_mean(struct H_params *params, double interval_start, double interval_end)
{ //for the mean, compute the integral of function H_for_mean for the interval between int_start and int_end
    double result = 0;
    double error = 0;
    size_t limit = MAXIMUM_SUBINTERVAL_NUNBER; //maximum number of subintervals
    gsl_integration_workspace *w = NULL; //work space for integration
    gsl_function F; //gsl function structure
   
    F.function = &H_for_mean; //probability function to calculate the mean for argument y
    F.params = params; //parameters

    /* allocate a workspace sufficient to hold n double precision intervals, their integration results and error estimates. */
    w = gsl_integration_workspace_alloc(limit);

    /* perform the integration for the function F for the interval (interval_start, interval_end) */
    gsl_integration_qags(&F, interval_start, interval_end, 0, 1e-7, limit, w, &result, &error);

    /* free the memory for the workspace */
    gsl_integration_workspace_free(w);

    return result;
}

double GSL_function_integral_for_second_moment(struct H_params *params, double interval_start, double interval_end)
{ //for the second moment, compute the integral of function H_for_second_moment for the interval between int_start and int_end
    double result = 0;
    double error = 0;
    size_t limit = MAXIMUM_SUBINTERVAL_NUNBER; //maximum number of subintervals
    gsl_integration_workspace *w = NULL; //work space for integration
    gsl_function F; //gsl function structure
   
    F.function = &H_for_second_moment; //probability function to calculate the second moment for argument y
    F.params = params; //parameters

    /* allocate a workspace sufficient to hold n double precision intervals, their integration results and error estimates. */
    w = gsl_integration_workspace_alloc(limit);

    /* perform the integration for the function F for the interval (interval_start, interval_end) */
    gsl_integration_qags(&F, interval_start, interval_end, 0, 1e-7, limit, w, &result, &error);

    /* free the memory for the workspace */
    gsl_integration_workspace_free(w);

    return result;
}

/** Vanet functions for delivery probability **/
/** Probability Function using Gaussian Distribution */
double GSL_Vanet_Delivery_Probability_For_Gaussian_Distribution(double mu_p, double sigma_p, double mu_v, double sigma_v, double interval_start, double interval_end)
{ //compute the delivery probability for two Gaussian random variables: (i) Packet delay of N(mu_p, sigma_p) and (ii) Vehicle delay of N(mu_v, sigma_v)
    double P = 0; //delivery probability

    vanet_opt_params_t params = {mu_p, sigma_p, mu_v, sigma_v};
    double result = 0;

    P = GSL_Vanet_Function_Integral_For_Gaussian_Distribution(&params, interval_start, interval_end);
    //result = Function_Integral(&H, &params, interval_start, interval_end);
    
    printf("delivery probability P=%.4f\n", (float)P);
    
    return P;
}

double  GSL_Vanet_Function_Integral_For_Gaussian_Distribution(vanet_opt_params_t *params, double interval_start, double interval_end)
{ //compute the integral of function GSL_Vanet_Probability_Func() for the interval between interval_start and interval_end using the Gaussian distribution
    double result = 0;
    double error = 0;
    size_t limit = MAXIMUM_SUBINTERVAL_NUNBER; //maximum number of subintervals
    gsl_integration_workspace *w = NULL; //work space for integration
    gsl_function F; //gsl function structure

    F.function = &GSL_Vanet_Probability_Function_For_Gaussian_Distribution; //Vanet probability function to calculate the delivery probability for argument y using the Gaussian Distribution
    F.params = params; //parameters

    /* allocate a workspace sufficient to hold n double precision intervals, their integration results and error estimates. */
    w = gsl_integration_workspace_alloc(limit);

    /* perform the integration for the function F for the interval (interval_start, interval_end) */
    gsl_integration_qags(&F, interval_start, interval_end, 0, 1e-7, limit, w, &result, &error);

    /* free the memory for the workspace */
    gsl_integration_workspace_free(w);

    return result;
}

double GSL_Vanet_Probability_Function_For_Gaussian_Distribution(double y, void *params)
{ //compute the probability for two Gaussian random variables: (i) Packet delay and (ii) Vehicle delay
    double f1 = 0; //probability for vehicle delay probability
    double f2 = 0;  //probability for packet delay probability
    double f = 0; //probability that the packet arrives earlier than the vehicle for the time y
    vanet_opt_params_t *p = (vanet_opt_params_t*)params;
    double mu_p = p->mu_p;; //packet delay average
    double sigma_p = p->sigma_p; //packet delay standard deviation
    double mu_v = p->mu_v;; //vehicle delay average
    double sigma_v = p->sigma_v; //vehicle delay standard deviation

    /* check whether the parameter sigma_v are valid for the calculation */
    if(sigma_v <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        f1 = 0;
    }
    else
    {
        f1 = 1/sqrt(2*PI*sigma_v*sigma_v) * exp(-pow(y-mu_v, 2) / (2*sigma_v*sigma_v)); //probability for vehicle delay probability
    }

    /* check whether the parameter sigma_v are valid for the calculation */
    if(sigma_p <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        f1 = 0;
    }
    else
    {
        f2 = gsl_cdf_gaussian_P((y-mu_p)/sigma_p, 1) - gsl_cdf_gaussian_P(-mu_p/sigma_p, 1); //probability for packet delay probability
    }

    /* compute the joint-probability */
    f = f1*f2;

    return f;
}

/** Probability Function using Gamma Distribution */
double GSL_Vanet_Delivery_Probability_For_Gamma_Distribution(double mu_p, double sigma_p, double mu_v, double sigma_v, double interval_start, double interval_end)
{ //compute the delivery probability for two Gamma random variables: (i) Packet delay of Gamma distribution for (mu_p, sigma_p) and (ii) Vehicle delay of Gamma distribution for (mu_v, sigma_v)
    double P = 0; //delivery probability

    vanet_opt_params_t params = {mu_p, sigma_p, mu_v, sigma_v};
    double result = 0;

#if 1 /* [ */
	P = GSL_Vanet_Function_Integral_For_Gamma_Distribution_v2(&params, interval_start, interval_end);
#else
    P = GSL_Vanet_Function_Integral_For_Gamma_Distribution(&params, interval_start, interval_end);
#endif /* ] */

#ifdef __DEBUG_LEVEL_GSL_FUNCTION_PROBABILITY__
    printf("delivery probability P=%.4f\n", (float)P);
#endif
    
    return P;
}

double  GSL_Vanet_Function_Integral_For_Gamma_Distribution(vanet_opt_params_t *params, double interval_start, double interval_end)
{ //compute the integral of function GSL_Vanet_Probability_Func_For_Gamma_Distribution() for the interval between interval_start and interval_end using the Gamma distribution
    double result = 0;
    double error = 0;
	int status = 0; //return-value for gsl_integration_qags()
    size_t limit = MAXIMUM_SUBINTERVAL_NUNBER; //maximum number of subintervals
    gsl_integration_workspace *w = NULL; //work space for integration
    gsl_function F; //gsl function structure

    F.function = &GSL_Vanet_Probability_Function_For_Gamma_Distribution; //Vanet probability function to calculate the delivery probability for argument y using the Gamma Distribution
    F.params = params; //parameters

    /* allocate a workspace sufficient to hold n double precision intervals, their integration results and error estimates. */
    w = gsl_integration_workspace_alloc(limit);

    /* perform the integration for the function F for the interval (interval_start, interval_end) */
#ifdef __DEBUG_LEVEL_GSL_FUNCTION__
    printf("GSL_Vanet_Function_Integral_For_Gamma_Distribution(): (interval_start,interval_end = (%.4f, %.4f), (mu_p, sigma_p) = (%.4f, %.4f), (mu_v, sigma_v) = (%.4f, %.4f)\n", (float)interval_start, (float)interval_end, (float)params->mu_p, (float)params->sigma_p, (float)params->mu_v, (float)params->sigma_v);
#endif
    status = gsl_integration_qags(&F, interval_start, interval_end, 0, 1e-7, limit, w, &result, &error);

    /* free the memory for the workspace */
    gsl_integration_workspace_free(w);

    return result;
}

double  GSL_Vanet_Function_Integral_For_Gamma_Distribution_v2(vanet_opt_params_t *params, double interval_start, double interval_end)
{ //compute the integral of function GSL_Vanet_Probability_Func_For_Gamma_Distribution() for the interval between interval_start and interval_end using the Gamma distribution along with GSL error handling
    double result = 0;
    double error = 0;
    size_t limit = MAXIMUM_SUBINTERVAL_NUNBER; //maximum number of subintervals
    gsl_integration_workspace *w = NULL; //work space for integration
    gsl_function F; //gsl function structure

#if 1 /* [ */
	gsl_error_handler_t *old_handler = gsl_set_error_handler_off(); //switch off default error handler, store old error handler in old_handler
	int status = 1; //status after the call to gsl_integration_qags()
	double relerr = 1e-7; //relative error
#endif /* ] */

    F.function = &GSL_Vanet_Probability_Function_For_Gamma_Distribution; //Vanet probability function to calculate the delivery probability for argument y using the Gamma Distribution
    F.params = params; //parameters

    /* allocate a workspace sufficient to hold n double precision intervals, their integration results and error estimates. */
    w = gsl_integration_workspace_alloc(limit);

    /* perform the integration for the function F for the interval (interval_start, interval_end) */
#ifdef __DEBUG_LEVEL_GSL_FUNCTION__
    printf("GSL_Vanet_Function_Integral_For_Gamma_Distribution(): (interval_start,interval_end = (%.4f, %.4f), (mu_p, sigma_p) = (%.4f, %.4f), (mu_v, sigma_v) = (%.4f, %.4f)\n", (float)interval_start, (float)interval_end, (float)params->mu_p, (float)params->sigma_p, (float)params->mu_v, (float)params->sigma_v);
#endif

#if 0 /* [ */
    gsl_integration_qags(&F, interval_start, interval_end, 0, 1e-7, limit, w, &result, &error);
#else
	status = gsl_integration_qags(&F, interval_start, interval_end, 0, relerr, limit, w, &result, &error);
	if(status)
	{
		result = 0;
	}

	gsl_set_error_handler(old_handler); //restore the old error handler
#endif /* ] */

    /* free the memory for the workspace */
    gsl_integration_workspace_free(w);

    return result;
}

double GSL_Vanet_Probability_Function_For_Gamma_Distribution(double y, void *params)
{ //compute the probability for two Gamma random variables: (i) Packet delay and (ii) Vehicle delay
    
    double absolute_y = 0; //absolute value of y
    
    double f1 = 0; //probability for vehicle delay probability
    double f2 = 0;  //probability for packet delay probability
    double f = 0; //probability that the packet arrives earlier than the vehicle for the time y
    vanet_opt_params_t *p = (vanet_opt_params_t*)params;
    double mu_p = p->mu_p; //packet delay average
    double sigma_p = p->sigma_p; //packet delay standard deviation
    double mu_v = p->mu_v; //vehicle delay average
    double sigma_v = p->sigma_v; //vehicle delay standard deviation
    
    /* parameters for packet delay of the Gamma distribution */
    double absolute_mu_p = 0; //absolute mu, that is, absolute mean for packet delay
    double absolute_sigma_p = 0; //absolute sigma, that is, absolute sigma for packet delay
    double kappa_p = 0; //parameter kappa for Gamma distribution for packet delay
    double theta_p = 0; //parameter theta for Gamma distribution for packet delay
    double gamma_value_p = 0; //gamma function value for packet delay

    /* parameters for vehicle delay of the Gamma distribution */
    double absolute_mu_v = 0; //absolute mu, that is, absolute mean for vehicle delay
    double absolute_sigma_v = 0; //absolute sigma, that is, absolute sigma for vehicle delay
    double kappa_v = 0; //parameter kappa for Gamma distribution for vehicle delay
    double theta_v = 0; //parameter theta for Gamma distribution for vehicle delay
    double gamma_value_v = 0; //gamma function value for packet delay

    /** check the validity of y */
    absolute_y = fabs(y);
    if(absolute_y <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("GSL_Vanet_Probability_Function_For_Gamma_Distribution(): Error: absolute_y(=%.4f) is zero\n", (float)absolute_y);

        f = 0;
        return f;
    }   

    /** compute the parameters of the Gamma distribution for vehicle delay */
    /* check the validity of mu_v */
    absolute_mu_v = fabs(mu_v);
    if(absolute_mu_v <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
#if 0 /* [ */
        printf("GSL_Vanet_Probability_Function_For_Gamma_Distribution(): Error: absolute_mu_v(=%.4f) is zero\n", (float)absolute_mu_v);
#endif /* ] */
        f = 0;
        return f;
    }

    /* check the validity of sigma_v */
    absolute_sigma_v = fabs(sigma_v);
    if(absolute_sigma_v <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
#if 0 /* [ */
        printf("GSL_Vanet_Probability_Function_For_Gamma_Distribution(): Error: absolute_sigma_v(=%.4f) is zero\n", (float)absolute_sigma_v);
#endif /* ] */

        f = 0;
        return f;
    }

    /** compute the parameter theta_v */
    theta_v = pow(absolute_sigma_v,2) / absolute_mu_v;

    /* check the validity of theta_v */
    if(theta_v <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("GSL_Vanet_Probability_Function_For_Gamma_Distribution(): Error: theta_v(=%.4f) is zero\n", (float)theta_v);

        f = 0;
        return f;
    }

    /** compute the parameter kappa_v */
    kappa_v = absolute_mu_v / theta_v;

    /* check the validity of kappa_v */
    if(kappa_v <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("GSL_Vanet_Probability_Function_For_Gamma_Distribution(): Error: kappa_v(=%.4f) is zero\n", (float)kappa_v);

        f = 0;
        return f;
    }
    /*
    else if(kappa_v > GAMMA_XMAX)
    {
        //printf("GSL_Vanet_Probability_Function_For_Gamma_Distribution(): Error: kappa_v(=%.4f) is greater than GAMMA_XMAX(%.4f)\n", (float)kappa_v, (float)GAMMA_XMAX);

        f = 0;
        return f;
    }
    */
    /** compute the PDF for y given kappa_v and theta_v */
    /* @Note that kappa_v(>=GSL::Sf::GAMMA_XMAX = 171.0)  makes gamma function return Infinite, so we need to use the implementation of the PDF of the Gamma function from GSL instead of making our own built gamma function-based PDF
     */
    //gamma_value_v = gsl_sf_gamma(kappa_v); //true the gamma function; 
    //f2 = pow(y, kappa_v-1) * exp(-y/theta_v) / (gamma_value_v * pow(theta_v, kappa_v));

    f2 = gsl_ran_gamma_pdf(y, kappa_v, theta_v); //the PDF of the Gamma distribution

    ////////////////////////////////////////////

    /** compute the parameters of the Gamma distribution for packet delay */
    /* check the validity of mu_p */
    absolute_mu_p = fabs(mu_p);
    if(absolute_mu_p <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("GSL_Vanet_Probability_Function_For_Gamma_Distribution(): Error: absolute_mu_p(=%.4f) is zero\n", (float)absolute_mu_p);
        
        f1 = 1; //CDF is 1
    }

    /* check the validity of sigma */
    absolute_sigma_p = fabs(sigma_p);
    if(absolute_sigma_p <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("GSL_Vanet_Probability_Function_For_Gamma_Distribution(): Error: absolute_sigma_p(=%.4f) is zero\n", (float)absolute_sigma_p);
        
        f1 = 1; //CDF is 1
    }

    /** compute the parameter theta_p */
    theta_p = pow(absolute_sigma_p,2) / absolute_mu_p;

    /* check the validity of theta_p */
    if(theta_p <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
#if 0 /* [ */ //Date: 5/6/2014
        printf("GSL_Vanet_Probability_Function_For_Gamma_Distribution(): Error: theta_p(=%.4f) is zero\n", (float)theta_p);
#endif /* ] */

        f = 0;
        return f;
    }

    /** compute the parameter kappa_p */
    kappa_p = absolute_mu_p / theta_p;

    /* check the validity of kappa_p */
    if(kappa_p <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("GSL_Vanet_Probability_Function_For_Gamma_Distribution(): Error: kappa_p(=%.4f) is zero\n", (float)kappa_p);

        f = 0;
        return f;
    }
    /*
    else if(kappa_p > GAMMA_XMAX)
    {
        //printf("GSL_Vanet_Probability_Function_For_Gamma_Distribution(): Error: kappa_p(=%.4f) is greater than GAMMA_XMAX(%.4f)\n", (float)kappa_p, (float)GAMMA_XMAX);

        f = 0;
        return f;
    }
    */

    /** compute the CDF for y given kappa_p and theta_p */
    f1 = gsl_cdf_gamma_P(y, kappa_p, theta_p);
    
    ////////////////////////////////////////////

    /** compute the joint-probability */
    f = f1*f2;

    return f;
}

//////////////////////////////////////////////////////////////////////////////////////
void GSL_Vanet_Compute_TravelTime_And_Deviation(parameter_t *param, double road_length, double *travel_time, double *travel_time_deviation)
{ //compute the mean and standard deviation of the travel time for road_length given the mean and standard deviation of the vehicle speed

    struct H_params params = {param->vehicle_speed, param->vehicle_speed_standard_deviation, road_length}; //(vehicle_mean_speed, vehicle_speed_standard_deviation, road_length)

    double interval_start = param->vehicle_minimum_speed; //minimum speed: interval's start; note that this interval start should be non-negative
    double interval_end = param->vehicle_maximum_speed; //maximum speed: interval's end
    double result = 0;

    double mean = 0; //mean travel time
    double second_moment = 0; //second moment of travel time
    double variance = 0; //travel time variance
    double standard_deviation = 0; //travel time standard deviation

    /** compute the mean and standard deviation of travel time for the road length */
    mean = GSL_function_integral_for_mean(&params, interval_start, interval_end);
    second_moment = GSL_function_integral_for_second_moment(&params, interval_start, interval_end);
    variance = second_moment - pow(mean, 2);
    standard_deviation = sqrt(variance);

    //printf("mean=%.4f and standard_deviation=%.4f\n", (float)mean, (float)standard_deviation);

    /** set the mean and standard deviation of the travle time */
    *travel_time = mean;
    *travel_time_deviation = standard_deviation;
}

void GSL_Vanet_Compute_TravelTime_And_Deviation_By_VehicleActualSpeed(parameter_t *param, double road_length, double *travel_time, double *travel_time_deviation, double vehicle_speed, double vehicle_speed_standard_deviation)
{ //compute the mean and standard deviation of the travel time for road_length given the mean and standard deviation of vehicle's actual speed rather than param's vehicle speed

    struct H_params params = {vehicle_speed, vehicle_speed_standard_deviation, road_length}; //(vehicle's mean_speed, vehicle's speed_standard_deviation, road_length)
    double interval_start = param->vehicle_minimum_speed; //minimum speed: interval's start; note that this interval start should be non-negative
    double interval_end = param->vehicle_maximum_speed; //maximum speed: interval's end
    double result = 0;

    double mean = 0; //mean travel time
    double second_moment = 0; //second moment of travel time
    double variance = 0; //travel time variance
    double standard_deviation = 0; //travel time standard deviation

    /** compute the mean and standard deviation of travel time for the road length */
    mean = GSL_function_integral_for_mean(&params, interval_start, interval_end);
    second_moment = GSL_function_integral_for_second_moment(&params, interval_start, interval_end);
    variance = second_moment - pow(mean, 2);
    standard_deviation = sqrt(variance);

    //printf("mean=%.4f and standard_deviation=%.4f\n", (float)mean, (float)standard_deviation);

    /** set the mean and standard deviation of the travle time */
    *travel_time = mean;
    *travel_time_deviation = standard_deviation;
}

//////////////////////////////////////////////////////////////////////////////////////////
/** GSL Probability Functions using Gamma Distribution for TPD */
double GSL_TPD_Encounter_Probability_For_Gamma_Distribution(double mu_y, 
		double sigma_y, 
		double mu_x, 
		double sigma_x, 
		double interval_start, 
		double interval_end, 
		double link_delay_a, 
		double link_delay_b)
{ //compute the delivery probability for two Gamma random variables: (i) Vehicle_b's travel delay of Gamma distribution for (mu_y, sigma_y) and (ii) Vehicle_a's travel delay of Gamma distribution for (mu_x, sigma_x) along with Vehicle_a's link_delay_a on the edge (n_i, n_j) and Vehicle_b's link_delay_b on the edge (n_j, n_i).

    double P = 0; //delivery probability
    tpd_opt_params_t params = {mu_y, sigma_y, mu_x, sigma_x, link_delay_a, link_delay_b};
    double result = 0;

    P = GSL_TPD_Function_Integral_For_Gamma_Distribution(&params, interval_start, interval_end, link_delay_a, link_delay_b);

#ifdef __DEBUG_LEVEL_GSL_FUNCTION_PROBABILITY__
    printf("encounter probability P=%.4f\n", (float)P);
#endif
    
    return P;
}

double  GSL_TPD_Function_Integral_For_Gamma_Distribution(tpd_opt_params_t *params, 
		double interval_start, 
		double interval_end, 
		double link_delay_a, 
		double link_delay_b)
{ //compute the integral of function GSL_TPD_Probability_Func_For_Gamma_Distribution() for the interval between interval_start and interval_end using the Gamma distribution along with link_delay_a and link_delay_b.
    double result = 0;
    double error = 0;
    size_t limit = MAXIMUM_SUBINTERVAL_NUNBER; //maximum number of subintervals
    gsl_integration_workspace *w = NULL; //work space for integration
    gsl_function F; //gsl function structure

#if 1 /* [ */
	gsl_error_handler_t *old_handler = gsl_set_error_handler_off(); //switch off default error handler, store old error handler in old_handler
	int status = 1; //status after the call to gsl_integration_qags()
	//double relerr = 0.01; //relative error
	double relerr = 1e-7; //relative error
#endif /* ] */

    F.function = &GSL_TPD_Probability_Function_For_Gamma_Distribution; //TPD probability function to calculate the encounter probability for argument y using the Gamma Distribution
    F.params = params; //parameters

    /* allocate a workspace sufficient to hold n double precision intervals, their integration results and error estimates. */
    w = gsl_integration_workspace_alloc(limit);

    /* perform the integration for the function F for the interval (interval_start, interval_end) */
#ifdef __DEBUG_LEVEL_GSL_FUNCTION__
    printf("GSL_TPD_Function_Integral_For_Gamma_Distribution(): (interval_start,interval_end = (%.4f, %.4f), (mu_y, sigma_y) = (%.4f, %.4f), (mu_x, sigma_x) = (%.4f, %.4f)\n", (float)interval_start, (float)interval_end, (float)params->mu_y, (float)params->sigma_y, (float)params->mu_x, (float)params->sigma_x);
#endif

#if 0 /* [ */	
    gsl_integration_qags(&F, interval_start, interval_end, 0, 1e-7, limit, w, &result, &error);
#else
#if 1 /* [[ */
	status = gsl_integration_qags(&F, interval_start, interval_end, 0, relerr, limit, w, &result, &error);
	if(status)
	{
		result = 0;
	}

	gsl_set_error_handler(old_handler); //restore the old error handler
#else
	while(status)
	{
		status = gsl_integration_qags(&F, interval_start, interval_end, 0, relerr, limit, w, &result, &error);
		relerr *= 1.1; //increased tolerance (relative error)
		if(status)
		{
#if 0 /* [ */
			printf("%s:%d Increased tolerance=%.2f\n",
					__FUNCTION__, __LINE__,
					relerr);
#endif /* ] */

			result = 0;
			break;
		}
	}

	gsl_set_error_handler(old_handler); //restore the old error handler
#endif /* ]] */
#endif /* ] */

    /* free the memory for the workspace */
    gsl_integration_workspace_free(w);

    return result;
}

double GSL_TPD_Probability_Function_For_Gamma_Distribution(double x, 
		void *params)
{ //compute the encounter probability for two Gamma random variables: (i) Vehicle_a's travel delay and (ii) Vehicle_b's travel delay
    
    double absolute_x = 0; //absolute value of y
    
    double f1 = 0; //probability for Vehicle_b's travel delay probability
	double f1_upper = 0; //f1 value for upper-bound
	double f1_lower = 0; //f1 value for lower-bound

    double f2 = 0;  //probability for Vehicle_a's travel delay probability
    double f = 0; //probability that Vehicle_a and Vehicle_b encounter in the edge (n_i, n_j) for the time x
    tpd_opt_params_t *p = (tpd_opt_params_t*)params;
    double mu_y = p->mu_y; //Vehicle_b's travel delay average
    double sigma_y = p->sigma_y; //Vehicle_b's travel delay standard deviation
    double mu_x = p->mu_x; //Vehicle_a's travel delay average
    double sigma_x = p->sigma_x; //Vehicle_a's travel delay standard deviation
    
    /* parameters for Vehicle_b's travel delay of the Gamma distribution */
    double absolute_mu_y = 0; //absolute mu, that is, absolute mean for travel delay
    double absolute_sigma_y = 0; //absolute sigma, that is, absolute sigma for travel delay
    double kappa_y = 0; //parameter kappa for Gamma distribution for travel delay
    double theta_y = 0; //parameter theta for Gamma distribution for travel delay
    double gamma_value_y = 0; //gamma function value for travel delay

    /* parameters for Vehicle_a's travel delay of the Gamma distribution */
    double absolute_mu_x = 0; //absolute mu, that is, absolute mean for travel delay
    double absolute_sigma_x = 0; //absolute sigma, that is, absolute sigma for travel delay
    double kappa_x = 0; //parameter kappa for Gamma distribution for travel delay
    double theta_x = 0; //parameter theta for Gamma distribution for travel delay
    double gamma_value_x = 0; //gamma function value for travel delay

	/* link delays for the road segments (n_i, n_j) and (n_j, n_i) */
	double link_delay_a = p->link_delay_a; /* Vehicle_a's link delay on (n_i, n_j) */
	double link_delay_b = p->link_delay_b; /* Vehicle_b's link delay on (n_j, n_i) */

    /** check the validity of x */
    absolute_x = fabs(x);
    if(absolute_x <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
#if GSL_ERROR_DISPLAY_FLAG
        printf("GSL_TPD_Probability_Function_For_Gamma_Distribution(): Error: absolute_x(=%.4f) is zero\n", (float)absolute_x);
#endif
        f = 0;
        return f;
    }   

    /** compute the parameters of the Gamma distribution for Vehicle_a's travel delay */
    /* check the validity of mu_x */
    absolute_mu_x = fabs(mu_x);
    if(absolute_mu_x <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
#if GSL_ERROR_DISPLAY_FLAG 
        printf("GSL_TPD_Probability_Function_For_Gamma_Distribution(): Error: absolute_mu_x(=%.4f) is zero\n", (float)absolute_mu_x);
#endif 
        f = 0;
        return f;
    }

    /* check the validity of sigma_x */
    absolute_sigma_x = fabs(sigma_x);
    if(absolute_sigma_x <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
#if GSL_ERROR_DISPLAY_FLAG 
        printf("GSL_TPD_Probability_Function_For_Gamma_Distribution(): Error: absolute_sigma_x(=%.4f) is zero\n", (float)absolute_sigma_x);
#endif 
        f = 0;
        return f;
    }

    /** compute the parameter theta_x */
    theta_x = pow(absolute_sigma_x,2) / absolute_mu_x;

    /* check the validity of theta_x */
    if(theta_x <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
#if GSL_ERROR_DISPLAY_FLAG		
        printf("GSL_TPD_Probability_Function_For_Gamma_Distribution(): Error: theta_x(=%.4f) is zero\n", (float)theta_x);
#endif
        f = 0;
        return f;
    }

    /** compute the parameter kappa_x */
    kappa_x = absolute_mu_x / theta_x;

    /* check the validity of kappa_x */
    if(kappa_x <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
#if GSL_ERROR_DISPLAY_FLAG		
        printf("GSL_TPD_Probability_Function_For_Gamma_Distribution(): Error: kappa_x(=%.4f) is zero\n", (float)kappa_x);
#endif
        f = 0;
        return f;
    }

    f2 = gsl_ran_gamma_pdf(x, kappa_x, theta_x); //the PDF of Vehicle_a's travel delay with the Gamma distribution

    ////////////////////////////////////////////

    /** compute the parameters of the Gamma distribution for Vehicle_b's travel delay */
    /* check the validity of mu_y */
    absolute_mu_y = fabs(mu_y);
    if(absolute_mu_y <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
#if GSL_ERROR_DISPLAY_FLAG
        printf("GSL_TPD_Probability_Function_For_Gamma_Distribution(): Error: absolute_mu_y(=%.4f) is zero\n", (float)absolute_mu_y);
#endif  

#if 0 /* [ */
        f1 = 1; //CDF is 1
#else
		f = 0;
		return f;
#endif /* ] */
    }

    /* check the validity of sigma */
    absolute_sigma_y = fabs(sigma_y);
    if(absolute_sigma_y <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
#if GSL_ERROR_DISPLAY_FLAG
        printf("GSL_TPD_Probability_Function_For_Gamma_Distribution(): Error: absolute_sigma_y(=%.4f) is zero\n", (float)absolute_sigma_y);
#endif

#if 0 /* [ */       
        f1 = 1; //CDF is 1
#else
		f = 0;
		return f;
#endif /* ] */
    }

    /** compute the parameter theta_y */
    theta_y = pow(absolute_sigma_y,2) / absolute_mu_y;

    /* check the validity of theta_y */
    if(theta_y <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
#if GSL_ERROR_DISPLAY_FLAG
        printf("GSL_TPD_Probability_Function_For_Gamma_Distribution(): Error: theta_y(=%.4f) is zero\n", (float)theta_y);
#endif
        f = 0;
        return f;
    }

    /** compute the parameter kappa_y */
    kappa_y = absolute_mu_y / theta_y;

    /* check the validity of kappa_y */
    if(kappa_y <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
#if GSL_ERROR_DISPLAY_FLAG
        printf("GSL_TPD_Probability_Function_For_Gamma_Distribution(): Error: kappa_y(=%.4f) is zero\n", (float)kappa_y);
#endif
        f = 0;
        return f;
    }

    /** compute the CDF for x given kappa_y and theta_y */
	f1_upper = gsl_cdf_gamma_P(x+link_delay_a+link_delay_b, kappa_y, theta_y);
	f1_lower = gsl_cdf_gamma_P(x, kappa_y, theta_y);

	/* check the validity of f1_upper and f1_lower */
	if(f1_upper < f1_lower)
	{
#if GSL_ERROR_DISPLAY_FLAG
        printf("GSL_TPD_Probability_Function_For_Gamma_Distribution(): Error: f1_upper(=%.4f) is less than f1_lower(=%.4f)\n", (float)f1_upper, (float)f1_lower);
#endif
        f = 0;
        return f;

	}

    f1 = f1_upper - f1_lower; //computer f1 with the difference between f1_upper and f1_lower
    
    ////////////////////////////////////////////

    /** compute the joint-probability */
    f = f1*f2;

    return f;
}

//////////////////////////////////////////////////////////////////////////////////////
