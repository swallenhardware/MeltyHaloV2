/*
 * initializeAnimations.c
 *
 *  Created on: Feb 24, 2019
 *      Author: swallen
 */

#include <animations.h>

uint32_t lastUpdateTime = 0;

Frame_P animationHead, currentFrame, safeHead, idleHead, preJukeHead, jukeHead, driveHead, spinHead;

Frame_P createBlankFrame(uint16_t duration) {
	Frame_P f = malloc(sizeof(Frame));
	for(int i=0; i<7; i++) {
		f->led[i].red = 0x00;
		f->led[i].green = 0x00;
		f->led[i].blue = 0x00;
	}
	f->duration = duration;
	f->next = NULL;
	return f;
}

Frame_P createMonocolorFrame(uint8_t leds, uint8_t red, uint8_t green, uint8_t blue, uint16_t duration) {
	Frame_P f = malloc(sizeof(Frame));
	for(int i=0; i<7; i++) {
		if((1 << i) & leds) {
			f->led[i].red = red;
			f->led[i].green = green;
			f->led[i].blue = blue;
		} else {
			f->led[i].red = 0;
			f->led[i].green = 0;
			f->led[i].blue = 0;
		}
	}
	f->duration = duration;
	f->next = NULL;
	return f;
}

Frame_P addBlankFrame(Frame_P tail, uint16_t duration) {
	tail->next = createBlankFrame(duration);
	return tail->next;
}

Frame_P addMonocolorFrame(Frame_P tail, uint8_t leds, uint8_t red, uint8_t green, uint8_t blue, uint16_t duration) {
	tail->next = createMonocolorFrame(leds, red, green, blue, duration);
	return tail->next;
}

//This creates a frame where the selected leds are colored with a foreground color, and all others are colored with a background color
Frame_P addForegroundBackgroundFrame(Frame_P tail, uint8_t leds, uint8_t fr, uint8_t fg, uint8_t fb, uint16_t duration, uint8_t br, uint8_t bg, uint8_t bb) {
	tail->next = createMonocolorFrame(leds, fr, fg, fb, duration);
	for(int i=0; i<7; i++) {
		if((1 << i) & ~leds) {
			tail->next->led[i].red = br;
			tail->next->led[i].green = bg;
			tail->next->led[i].blue = bb;
		}
	}

	return tail->next;
}

void initAnimations() {
	LED_init();
	initSafeAnimation();
	initIdleAnimation();
	initPreJukeAnimation();
	initJukeAnimation();
	initDriveAnimation();
	initSpinAnimation();
}

void setAnimation(uint8_t state) {
	switch(state) {
	default:
	case STATE_SAFE:
		animationHead = safeHead;
		break;
	case STATE_IDLE:
		animationHead = idleHead;
		break;
	case STATE_PREJUKE:
		animationHead = preJukeHead;
		break;
	case STATE_JUKE:
		animationHead = jukeHead;
		break;
	case STATE_DRIVE:
		animationHead = driveHead;
		break;
	case STATE_SPIN:
		animationHead = spinHead;
		break;
	}

	if(state != STATE_SPIN) {
		currentFrame = animationHead;
		updateLEDs(currentFrame);
		lastUpdateTime = HAL_GetTick();
	}
}

void runTimeAnimation() {
	//check if it's time to update the frame
	if(HAL_GetTick() - currentFrame->duration > lastUpdateTime) {
		//if the next frame is null, loop back to the beginning. Otherwise show the next frame
		if(currentFrame->next == NULL) {
			currentFrame = animationHead;
		} else {
			currentFrame = currentFrame->next;
		}
		lastUpdateTime = HAL_GetTick();
		updateLEDs(currentFrame);
	}
}

void runPOVAnimation(int16_t heading) {
	//sanity-check our input
	heading %= 360;
	if(heading < 0) heading += 360;

	//make sure we're at the correct position in the POV image
	Frame_P f = animationHead;
	uint16_t index = 0;
	while(1) {
		index += f->duration;
		if(index > heading) {
			if(f != currentFrame) {
				currentFrame = f;
				updateLEDs(f);
			}
			break;
		} else if(f->next != NULL) {
			f = f->next;
		} else {
			break;
		}
	}
}

void initSafeAnimation() {
	Frame_P next;
	safeHead = createMonocolorFrame(	0b00001000, 0x3F, 0x00, 0x00, 500);
	next = addMonocolorFrame(safeHead, 	0b00011100, 0x3F, 0x00, 0x00, 500);
	next = addMonocolorFrame(next,	 	0b00111110, 0x3F, 0x00, 0x00, 500);
	next = addMonocolorFrame(next,	 	0b01111111, 0x3F, 0x00, 0x00, 500);
	next = addMonocolorFrame(next,	 	0b00111110, 0x3F, 0x00, 0x00, 500);
	next = addMonocolorFrame(next,	 	0b00011100, 0x3F, 0x00, 0x00, 500);
}

void initIdleAnimation() {
	Frame_P next;
	idleHead = createMonocolorFrame(	0b00001000, 0x3F, 0x3F, 0x00, 500);
	next = addMonocolorFrame(idleHead, 	0b00011100, 0x3F, 0x3F, 0x00, 500);
	next = addMonocolorFrame(next,	 	0b00111110, 0x3F, 0x3F, 0x00, 500);
	next = addMonocolorFrame(next,	 	0b01111111, 0x3F, 0x3F, 0x00, 500);
	next = addMonocolorFrame(next,	 	0b00111110, 0x3F, 0x3F, 0x00, 500);
	next = addMonocolorFrame(next,	 	0b00011100, 0x3F, 0x3F, 0x00, 500);
}

void initPreJukeAnimation() {
	Frame_P next;
	preJukeHead = createMonocolorFrame(		0b00001000, 0x3F, 0x3F, 0x00, 500);
	next = addMonocolorFrame(preJukeHead, 	0b00011100, 0x3F, 0x00, 0x00, 500);
	next = addMonocolorFrame(next,	 		0b00111110, 0x3F, 0x3F, 0x00, 500);
	next = addMonocolorFrame(next,	 		0b01111111, 0x3F, 0x3F, 0x00, 500);
	next = addMonocolorFrame(next,	 		0b00111110, 0x3F, 0x3F, 0x00, 500);
	next = addMonocolorFrame(next,	 		0b00011100, 0x3F, 0x00, 0x00, 500);
}

void initJukeAnimation() {
	Frame_P next;
	jukeHead = createMonocolorFrame(	0b00001000, 0x00, 0x3F, 0x00, 100);
	next = addMonocolorFrame(jukeHead, 	0b01111111, 0x00, 0x3F, 0x00, 100);
	next = addMonocolorFrame(next,	 	0b00111110, 0x00, 0x3F, 0x00, 100);
	next = addMonocolorFrame(next,	 	0b00011100, 0x00, 0x3F, 0x00, 100);
	next = addMonocolorFrame(next,	 	0b00111110, 0x00, 0x3F, 0x00, 100);
	next = addMonocolorFrame(next,	 	0b01111111, 0x00, 0x3F, 0x00, 100);
}

void initDriveAnimation() {
	Frame_P next;
	driveHead = createMonocolorFrame(	0b00001000, 0x00, 0x3F, 0x00, 100);
	next = addMonocolorFrame(driveHead, 0b00011100, 0x00, 0x3F, 0x00, 100);
	next = addMonocolorFrame(next,	 	0b00111110, 0x00, 0x3F, 0x00, 100);
	next = addMonocolorFrame(next,	 	0b01111111, 0x00, 0x3F, 0x00, 100);
	next = addMonocolorFrame(next,	 	0b00111110, 0x00, 0x3F, 0x00, 100);
	next = addMonocolorFrame(next,	 	0b00011100, 0x00, 0x3F, 0x00, 100);
}

void initSpinAnimation() {
	Frame_P next;
	spinHead = createMonocolorFrame(	0b00001000, 0xFF, 0x00, 0x00, 5);
	next = addMonocolorFrame(spinHead, 	0b00010100, 0xFF, 0x00, 0x00, 5);
	next = addMonocolorFrame(next, 		0b00100010, 0xFF, 0x00, 0x00, 5);
	next = addMonocolorFrame(next, 		0b01000001, 0xFF, 0x00, 0x00, 5);
	for(int i=0; i<6; i++) {
		next = addMonocolorFrame(next, 	0b01000001, 0xFF, 0x00, 0x00, 5);
		next = addMonocolorFrame(next, 	0b01001001, 0xFF, 0x00, 0x00, 5);
		next = addMonocolorFrame(next, 	0b01010101, 0xFF, 0x00, 0x00, 5);
		next = addMonocolorFrame(next, 	0b01100011, 0xFF, 0x00, 0x00, 5);
	}

	uint8_t segThickness = 3;

	next = addBlankFrame(next, 5);//this HALO section is 80 degrees long
	next = addMonocolorFrame(next, 		0b00111110, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00001000, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00001000, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00001000, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00111110, 0x00, 0xFF, 0x00, segThickness);
	next = addBlankFrame(next, 3);
	next = addMonocolorFrame(next, 		0b00111100, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00001010, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00001010, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00001010, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00111100, 0x00, 0xFF, 0x00, segThickness);
	next = addBlankFrame(next, 3);
	next = addMonocolorFrame(next, 		0b00111100, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00100000, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00100000, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00100000, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00100000, 0x00, 0xFF, 0x00, segThickness);
	next = addBlankFrame(next, 3);
	next = addMonocolorFrame(next, 		0b00011100, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00100010, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00100010, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00100010, 0x00, 0xFF, 0x00, segThickness);
	next = addMonocolorFrame(next, 		0b00011100, 0x00, 0xFF, 0x00, segThickness);
	next = addBlankFrame(next, 6);

	for(int i=0; i<6; i++) {
		next = addMonocolorFrame(next, 	0b01100011, 0x00, 0x00, 0xFF, 5);
		next = addMonocolorFrame(next, 	0b01010101, 0x00, 0x00, 0xFF, 5);
		next = addMonocolorFrame(next, 	0b01001001, 0x00, 0x00, 0xFF, 5);
		next = addMonocolorFrame(next, 	0b01000001, 0x00, 0x00, 0xFF, 5);
	}
	next = addMonocolorFrame(next, 		0b01000001, 0x00, 0x00, 0xFF, 5);
	next = addMonocolorFrame(next, 		0b00100010, 0x00, 0x00, 0xFF, 5);
	next = addMonocolorFrame(next, 		0b00010100, 0x00, 0x00, 0xFF, 5);
	next = addMonocolorFrame(next, 		0b00001000, 0x00, 0x00, 0xFF, 5);
}
