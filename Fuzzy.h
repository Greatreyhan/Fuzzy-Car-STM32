/*
 * Fuzzy.h
 *
 *  Created on: Feb 2, 2022
 *      Author: Maulana Reyhan Savero
 */
#ifndef FUZZY_H_
#define FUZZY_H_

#include "main.h"

#include <stdbool.h>

typedef struct{
	double max;
	double med;
	double min;
}Fuzzy_input_t;

typedef struct{
	double max;
	double med;
	double min;
}Fuzzy_output_t;

typedef struct{
	double fuzz_min;
	double fuzz_med;
	double fuzz_max;
}Fuzzy_fuzzyfication_t;

typedef struct{
	double defuz1;
	double defuz2;
	double defuz3;
	double defuz4;
}Fuzzy_defuz_t;

typedef enum{
	FUZZY_MIN_TO_MAX = 0x01U,
	FUZZY_MAX_TO_MAX = 0X02U
}Fuzzy_rules_t;

void fuzzy_set_membership_input(Fuzzy_input_t* input, double min, double med, double max);
void fuzzy_set_membership_output(Fuzzy_output_t* output, double min, double med, double max);
void fuzzy_fuzfication_input(Fuzzy_input_t* input, Fuzzy_fuzzyfication_t* fuzz_fic, double data);
void fuzzy_fuzfication_output(Fuzzy_output_t* output, Fuzzy_fuzzyfication_t* fuzz_fic, double data);
void fuzzy_logic_rule(Fuzzy_output_t* output, Fuzzy_fuzzyfication_t* fuzz_fic, Fuzzy_defuz_t* defuz, Fuzzy_rules_t rules);
double fuzzy_defuz(Fuzzy_defuz_t* defuz,Fuzzy_fuzzyfication_t* fuzz_fic);
#endif