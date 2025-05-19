/*
 * defines.h
 *
 *  Created on: May 16, 2025
 *      Author: witol
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#include <xc.h>
#include "button.h"
#define TRUE          1
#define FALSE         0

// ToDo zmienić porty
#define BUTTON_AUTO_R       PORTBbits.RB1
#define BUTTON_AUTO_DIR     TRISBbits.TRISB1
#define BUTTON_AUTO_PULLUP  WPUBbits.WPUB1



#endif /* DEFINES_H_ */
