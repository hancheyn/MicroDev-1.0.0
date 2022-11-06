/*
 * setup.c
 *
 *  Created on: May 2, 2022
 *      Author: nathan
 */

#include "setup.h"


//Basic Delay
void delay(int n) {
	int i;
	for(; n > 0; n--) {
		for(i =0; i < 3195; i++);
	}

}
