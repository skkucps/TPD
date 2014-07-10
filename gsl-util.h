/** File: gsl-util.h
	Description: specify the GSL utility functions
	Date: 11/06/2009
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
*/

#ifndef __GSL_UTIL_H__
#define __GSL_UTIL_H__

#include <gsl/gsl_cdf.h>
#include <gsl/gsl_integration.h>

/** Macro constants */
//#define MAXIMUM_SUBINTERVAL_NUNBER 1000 
#define MAXIMUM_SUBINTERVAL_NUNBER 10000
//#define MAXIMUM_SUBINTERVAL_NUNBER 1000000
//maximum number of subintervals

//the maximum value of kappa in the Gamma distribution whose Gamma() does not make an overflow
#define GAMMA_XMAX 171.0

/** structure of the parameter list of function H */
struct H_params 
{ 
    double mu; //mean
    double sigma; //standard deviation
    double length; //length of road segment
};

/* Vanet Optimization Parameters */
typedef struct _vanet_opt_params_t 
{ 
    double mu_p; //packet delay average
    double sigma_p; //packet delay standard deviation

    double mu_v; //vehicle delay average
    double sigma_v; //vehicle delay standard deviation
} vanet_opt_params_t;

typedef struct _tpd_opt_params_t 
{
	/* Let (n_i, n_j) be the encountered edge for Vehicle_a and Vehicle_b.
	 * Assume that Vehicle_a is approaching n_i toward n_j and Vehicle_b is
	 * approaching n_j toward n_i.
	 * */
    double mu_y; //Vehicle_b's travel delay average on the edge (n_j, n_i)
    double sigma_y; //Vehicle_b's travel delay standard deviation on the edge (n_j, n_i)

    double mu_x; //Vehicle_a's travel delay average on the edge (n_i, n_j)
    double sigma_x; //Vehicle_a's travel delay standard deviation on the edge (n_i, n_j)

	double link_delay_a; //link delay of Vehicle_a on the edge (n_i, n_j)
	double link_delay_b; //link delay of Vehicle_b on the edge (n_j, n_i)
} tpd_opt_params_t;

/** function predeclarations */

void GSL_test();
//test function for gsl library

void GSL_test_for_gamma();
//test function for gsl library for Gamma distribution

double H(double y, void *params);
//compute the probability for a Gaussian random variable

double H_for_gamma(double y, void *params);
//compute the probability for a Gamma random variable

double H_for_mean(double y, void *params);
//compute the portion of the mean of a Gaussian random variable

double H_for_second_moment(double y, void *params);
//compute the portion of the second moment of a Gaussian random variable

double GSL_function_integral(struct H_params *params, double interval_start, double interval_end);
//double Function_Integral((double) (*h)(double, void*), struct H_params *params, double interval_start, double interval_end);
//compute the integral of function h for the interval between int_start and int_end

double GSL_function_integral_for_gamma(struct H_params *params, double interval_start, double interval_end);
//compute the integral of function h for the interval between int_start and int_end for Gamma distribution

double GSL_function_integral_for_mean(struct H_params *params, double interval_start, double interval_end);
//for the mean, compute the integral of function H_for_mean for the interval between int_start and int_end

double GSL_function_integral_for_second_moment(struct H_params *params, double interval_start, double interval_end);
//for the second moment, compute the integral of function H_for_second_moment for the interval between int_start and int_end

/** Probability Function using Gaussian Distribution */
double GSL_Vanet_Delivery_Probability_For_Gaussian_Distribution(double mu_p, double sigma_p, double mu_v, double sigma_v, double interval_start, double interval_end);
//compute the delivery probability for two Gaussian random variables: (i) Packet delay of N(mu_p, sigma_p) and (ii) Vehicle delay of N(mu_v, sigma_v)

double GSL_Vanet_Probability_Function_For_Gaussian_Distribution(double y, void *params);
//compute the probability for two Gaussian random variables: (i) Packet delay and (ii) Vehicle delay

double GSL_Vanet_Function_Integral_For_Gaussian_Distribution(vanet_opt_params_t *params, double interval_start, double interval_end);
//compute the integral of function GSL_Vanet_Probability_Func_For_Gaussian_Distribution() for the interval between interval_start and interval_end using the Gaussian distribution

/** Probability Function using Gamma Distribution */
double GSL_Vanet_Delivery_Probability_For_Gamma_Distribution(double mu_p, double sigma_p, double mu_v, double sigma_v, double interval_start, double interval_end);
//compute the delivery probability for two Gamma random variables: (i) Packet delay of Gamma distribution for (mu_p, sigma_p) and (ii) Vehicle delay of Gamma distribution for (mu_v, sigma_v)

double GSL_Vanet_Probability_Function_For_Gamma_Distribution(double y, void *params);
//compute the probability for two Gamma random variables: (i) Packet delay and (ii) Vehicle delay

double GSL_Vanet_Function_Integral_For_Gamma_Distribution(vanet_opt_params_t *params, double interval_start, double interval_end);
//compute the integral of function GSL_Vanet_Probability_Func_For_Gamma_Distribution() for the interval between interval_start and interval_end using the Gamma distribution

double  GSL_Vanet_Function_Integral_For_Gamma_Distribution_v2(vanet_opt_params_t *params, double interval_start, double interval_end);
//compute the integral of function GSL_Vanet_Probability_Func_For_Gamma_Distribution() for the interval between interval_start and interval_end using the Gamma distribution along with GSL error handling

//////////////////////////////////////////////////////////////////////////////////////
void GSL_Vanet_Compute_TravelTime_And_Deviation(parameter_t *param, double road_length, double *travel_time, double *travel_time_deviation);
//compute the mean and standard deviation of the travel time for road_length given the mean and standard deviation of the vehicle speed

void GSL_Vanet_Compute_TravelTime_And_Deviation_By_VehicleActualSpeed(parameter_t *param, double road_length, double *travel_time, double *travel_time_deviation, double vehicle_speed, double vehicle_speed_standard_deviation);
//compute the mean and standard deviation of the travel time for road_length given the mean and standard deviation of vehicle's actual speed rather than param's vehicle speed

/** GSL Probability Functions using Gamma Distribution for TPD */
double GSL_TPD_Encounter_Probability_For_Gamma_Distribution(double mu_y, 
		double sigma_y, 
		double mu_x, 
		double sigma_x, 
		double interval_start, 
		double interval_end, 
		double link_delay_a, 
		double link_delay_b);
//compute the delivery probability for two Gamma random variables: (i) Vehicle_b's travel delay of Gamma distribution for (mu_y, sigma_y) and (ii) Vehicle_a's travel delay of Gamma distribution for (mu_x, sigma_x) along with Vehicle_a's link_delay_a on the edge (n_i, n_j) and Vehicle_b's link_delay_b on the edge (n_j, n_i). 

double  GSL_TPD_Function_Integral_For_Gamma_Distribution(tpd_opt_params_t *params, 
		double interval_start, 
		double interval_end, 
		double link_delay_a, 
		double link_delay_b);
//compute the integral of function GSL_TPD_Probability_Func_For_Gamma_Distribution() for the interval between interval_start and interval_end using the Gamma distribution along with link_delay_a and link_delay_b.

double GSL_TPD_Probability_Function_For_Gamma_Distribution(double x, 
		void *params); //compute the encounter probability for two Gamma random variables: (i) Vehicle_a's travel delay and (ii) Vehicle_b's travel delay

#endif
