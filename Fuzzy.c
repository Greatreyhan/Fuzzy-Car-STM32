/*
 * Fuzzy.c
 *
 *  Created on: Feb 2, 2022
 *      Author: Maulana Reyhan Savero
 */

#include "Fuzzy.h"

void fuzzy_set_membership_input(Fuzzy_input_t* input, double min, double med, double max){
	input->min = min;
	input->med = med;
	input->max = max;
}

void fuzzy_set_membership_output(Fuzzy_output_t* output, double min, double med, double max){
	output->min = min;
	output->med = med;
	output->max = max;
}

void fuzzy_fuzfication_input(Fuzzy_input_t* input, Fuzzy_fuzzyfication_t* fuzz_fic, double data){
	
	// Get fuzzyfication output for minimum membership -> fuzz_min
	if(data < input->min) fuzz_fic->fuzz_min = 1;
	else if((data >= input->min) && (data <= input->med)) fuzz_fic->fuzz_min = (input->med - data)/(input->med - input->min);
	else if(data > input->med) fuzz_fic-> fuzz_min = 0;
	
	// Get fuzzyfication output for median membership -> fuzz_med
	if(data < input->min) fuzz_fic->fuzz_med = 0;
	else if((data >= input->min) && (data <= input->med)) fuzz_fic->fuzz_med = (data-input->min)/(input->med - input->min);
	else if((data >= input->med) && (data <= input->max)) fuzz_fic->fuzz_med = (input->max - data)/(input->med - input->min);
	else if(data > input->max) fuzz_fic->fuzz_med = 0;
	
	// Get fuzzyfication output for maximum membership -> fuzz_max
	if(data < input->med) fuzz_fic->fuzz_max = 0;
	else if((data >= input->med) && (data <= input->max)) fuzz_fic->fuzz_max = (data-input->med)/(input->med - input->min);
	else if(data > input->max) fuzz_fic->fuzz_max = 1;
}

void fuzzy_fuzfication_output(Fuzzy_output_t* output, Fuzzy_fuzzyfication_t* fuzz_fic, double data){
	
	// Get fuzzyfication output for minimum membership -> fuzz_min
	if(data < output->min) fuzz_fic->fuzz_min = 1;
	else if(data >= output->min && data <= output->med) fuzz_fic->fuzz_min = (output->med - data)/(output->med - output->min);
	else if(data > output->med) fuzz_fic-> fuzz_min = 0;
	
	// Get fuzzyfication output for median membership -> fuzz_med
	if(data < output->min) fuzz_fic->fuzz_med = 0;
	else if(data >= output->min && data <= output->med) fuzz_fic->fuzz_med = (data-output->min)/(output->med - output->min);
	else if(data >= output->med && data <= output->max) fuzz_fic->fuzz_med = (output->med - data)/(output->med - output->min);
	else if(data > output->max) fuzz_fic->fuzz_med = 0;
	
	// Get fuzzyfication output for maximum membership -> fuzz_max
	if(data < output->med) fuzz_fic->fuzz_max = 0;
	else if(data <= output->med && data >= output->max) fuzz_fic->fuzz_max = (data-output->med)/(output->med - output->min);
	else if(data > output->max) fuzz_fic->fuzz_max = 1;
}

void fuzzy_logic_rule(Fuzzy_output_t* output, Fuzzy_fuzzyfication_t* fuzz_fic, Fuzzy_defuz_t* defuz, Fuzzy_rules_t rules){
	if(rules == 0x01){
		defuz->defuz1 = output->med - (fuzz_fic->fuzz_min*(output->med-output->min));
		defuz->defuz2 = output->min + (fuzz_fic->fuzz_med*(output->med-output->min));
		defuz->defuz3 = output->max - (fuzz_fic->fuzz_med*(output->med-output->min));
		defuz->defuz4 = output->med + (fuzz_fic->fuzz_max*(output->med-output->min));
	}
	else if(rules == 0x02){
		defuz->defuz1 = output->med - (fuzz_fic->fuzz_max*(output->med-output->min));
		defuz->defuz2 = output->min + (fuzz_fic->fuzz_med*(output->med-output->min));
		defuz->defuz3 = output->max - (fuzz_fic->fuzz_med*(output->med-output->min));
		defuz->defuz4 = output->med + (fuzz_fic->fuzz_min*(output->med-output->min));
	}
}

double fuzzy_defuz(Fuzzy_defuz_t* defuz,Fuzzy_fuzzyfication_t* fuzz_fic){
	double output = ((defuz->defuz1*fuzz_fic->fuzz_min)+(defuz->defuz2*fuzz_fic->fuzz_med)+(defuz->defuz3*fuzz_fic->fuzz_med)+(defuz->defuz4*fuzz_fic->fuzz_max))/(fuzz_fic->fuzz_min+2*fuzz_fic->fuzz_med+fuzz_fic->fuzz_max);
	return output;
}
