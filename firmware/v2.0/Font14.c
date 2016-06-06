
#include <avr/pgmspace.h>
#include <avr/io.h>
#include "font14.h"

static const uint8_t image_data_Font_big_0x4b[12] PROGMEM = {
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x60,
	0x60
};
//static const tImage Font_big_0x4b = { image_data_Font_big_0x4b, 4, 12};

static const uint8_t image_data_Font_big_0x60[12] PROGMEM = {
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x78,
	0x00,
	0x00,
	0x00,
	0x00
};
//static const tImage Font_big_0x60 = { image_data_Font_big_0x60, 6, 12};

static const uint8_t image_data_Font_big_0xf0[12] PROGMEM = {
	0x3c,
	0x66,
	0xc3,
	0xc3,
	0xc3,
	0xc3,
	0xc3,
	0xc3,
	0xc3,
	0xc3,
	0x66,
	0x3c
};
//static const tImage Font_big_0xf0 = { image_data_Font_big_0xf0, 9, 12};

static const uint8_t image_data_Font_big_0xf1[12] PROGMEM = {
	0x0c,
	0x1c,
	0x7c,
	0x0c,
	0x0c,
	0x0c,
	0x0c,
	0x0c,
	0x0c,
	0x0c,
	0x0c,
	0x0c
};
//static const tImage Font_big_0xf1 = { image_data_Font_big_0xf1, 9, 12};

static const uint8_t image_data_Font_big_0xf2[12] PROGMEM = {
	0x3c,
	0x66,
	0xc3,
	0xc3,
	0x03,
	0x06,
	0x0c,
	0x18,
	0x30,
	0x60,
	0xc0,
	0xff
};
//static const tImage Font_big_0xf2 = { image_data_Font_big_0xf2, 9, 12};

static const uint8_t image_data_Font_big_0xf3[12] PROGMEM = {
	0x3c,
	0x66,
	0xc3,
	0xc3,
	0x06,
	0x1c,
	0x06,
	0x03,
	0xc3,
	0xc3,
	0x66,
	0x3c
};
//static const tImage Font_big_0xf3 = { image_data_Font_big_0xf3, 9, 12};

static const uint8_t image_data_Font_big_0xf4[12] PROGMEM = {
	0x06,
	0x0e,
	0x1e,
	0x16,
	0x36,
	0x26,
	0x66,
	0xc6,
	0xff,
	0x06,
	0x06,
	0x06
};
//static const tImage Font_big_0xf4 = { image_data_Font_big_0xf4, 9, 12};

static const uint8_t image_data_Font_big_0xf5[12] PROGMEM = {
	0x7f,
	0x60,
	0x60,
	0xc0,
	0xfc,
	0xc6,
	0x03,
	0x03,
	0x03,
	0xc3,
	0xc6,
	0x7c
};
//static const tImage Font_big_0xf5 = { image_data_Font_big_0xf5, 9, 12};

static const uint8_t image_data_Font_big_0xf6[12] PROGMEM = {
	0x3e,
	0x63,
	0x41,
	0xc0,
	0xdc,
	0xe6,
	0xc3,
	0xc3,
	0xc3,
	0xc3,
	0x66,
	0x3c
};
//static const tImage Font_big_0xf6 = { image_data_Font_big_0xf6, 9, 12};

static const uint8_t image_data_Font_big_0xf7[12] PROGMEM = {
	0xff,
	0x03,
	0x06,
	0x06,
	0x0c,
	0x0c,
	0x18,
	0x18,
	0x18,
	0x30,
	0x30,
	0x30
};
//static const tImage Font_big_0xf7 = { image_data_Font_big_0xf7, 9, 12};

static const uint8_t image_data_Font_big_0xf8[12] PROGMEM = {
	0x3c,
	0x66,
	0xc3,
	0xc3,
	0x66,
	0x3c,
	0x66,
	0xc3,
	0xc3,
	0xc3,
	0x66,
	0x3c
};
//static const tImage Font_big_0xf8 = { image_data_Font_big_0xf8, 9, 12};

static const uint8_t image_data_Font_big_0xf9[12] PROGMEM = {
	0x3c,
	0x66,
	0xc3,
	0xc3,
	0xc3,
	0xc3,
	0x67,
	0x3b,
	0x03,
	0x82,
	0xc6,
	0x7c
};
//static const tImage Font_big_0xf9 = { image_data_Font_big_0xf9, 9, 12};

//  --- SYMBOLS ---

// batt0
static const uint8_t image_data_batt0[7] PROGMEM = {
	0xfc, 
	0x82, 
	0x83, 
	//0x83, 
	//0x83, 
	0x83, 
	0x83, 
	0x82, 
	0xfc
};

// batt1
//static const uint8_t image_data_batt1[18] PROGMEM = {
//0xff, 0xf0,
//0x80, 0x10,
//0xb0, 0x18,
//0xb0, 0x18,
//0xb0, 0x18,
//0xb0, 0x18,
//0xb0, 0x18,
//0x80, 0x10,
//0xff, 0xf0
//};
// batt 2
//static const uint8_t image_data_batt2[18] PROGMEM = {
//0xff, 0xf0,
//0x80, 0x10,
//0xb6, 0x18,
//0xb6, 0x18,
//0xb6, 0x18,
//0xb6, 0x18,
//0xb6, 0x18,
//0x80, 0x10,
//0xff, 0xf0
//};

// batt 3
//static const uint8_t image_data_batt3[18] PROGMEM = {
//0xff, 0xf0,
//0x80, 0x10,
//0xb6, 0xd8,
//0xb6, 0xd8,
//0xb6, 0xd8,
//0xb6, 0xd8,
//0xb6, 0xd8,
//0x80, 0x10,
//0xff, 0xf0
//};

// UVI

static const uint8_t image_data_Image_uvi[12] PROGMEM = {
	0x8a, 0x24,
	0x8a, 0x24,
	0x89, 0x44,
	0x89, 0x44,
	0x88, 0x84,
	0x70, 0x84
};

// UVIh

static const uint8_t image_data_Image_uvih[18] PROGMEM = {
	0x8a, 0x24, 0x88,
	0x8a, 0x24, 0x88,
	0x89, 0x44, 0xf8,
	0x89, 0x44, 0x88,
	0x88, 0x84, 0x88,
	0x70, 0x84, 0x88
};

// �C
static const uint8_t image_data_Image_c[6] PROGMEM = {
	0xce,
	0xd1,
	0x10,
	0x10,
	0x11,
	0x0e
};

// mbar
static const uint8_t image_data_Image_mbar[24] PROGMEM = {
	0xc6, 0xf0, 0x8f, 0x00,
	0xc6, 0x88, 0x88, 0x80,
	0xaa, 0xf1, 0x48, 0x80,
	0xaa, 0x89, 0xcf, 0x00,
	0x92, 0x8a, 0x28, 0x80,
	0x92, 0xf2, 0x28, 0x80
};

// rh%

static const uint8_t image_data_Image_rh[12] PROGMEM = {
	0xf2, 0x4c,
	0x8a, 0x4d,
	0x8b, 0xc2,
	0xf2, 0x44,
	0x8a, 0x4b,
	0x8a, 0x43
};

// ----------------- corners

static const uint8_t image_data_corner_T_R[3] PROGMEM = {
	0xff,
	0x03,
	0x03
};

static const uint8_t image_data_corner_T_L[3] PROGMEM = {
	0xff,
	0xc0,
	0xc0
};

static const uint8_t image_data_corner_B_L[3] PROGMEM = {
	0xc0,
	0xc0,
	0xff
};

static const uint8_t image_data_corner_B_R[3] PROGMEM = {
	0x03,
	0x03,
	0xff
};

//static const uint8_t image_data_corner_T_R[5] PROGMEM = {
	//0xf8,
	//0xf8,
	//0x18,
	//0x18,
	//0x18
//};
//
//static const uint8_t image_data_corner_T_L[5] PROGMEM = {
	//0xf8,
	//0xf8,
	//0xc0,
	//0xc0,
	//0xc0
//};
//
//static const uint8_t image_data_corner_B_L[5] PROGMEM = {
	//0xc0,
	//0xc0,
	//0xc0,
	//0xf8,
	//0xf8
//};
//
//static const uint8_t image_data_corner_B_R[5] PROGMEM = {
	//0x18,
	//0x18,
	//0x18,
	//0xf8,
	//0xf8
//};

// BAR

static const uint8_t image_data_bar[3] PROGMEM = {
	0xf0,
	0xf0,
	0xf0
};

// ---------------

const tChar Font14_array[] PROGMEM = {

	// character: '-'
	{ image_data_Font_big_0x60, 6},//, 12},

	// character: '.'
	{ image_data_Font_big_0x4b, 4},//, 12},

	// character: '0'
	{ image_data_Font_big_0xf0, 8},//, 12},

	// character: '1'
	{ image_data_Font_big_0xf1, 8},//, 12},

	// character: '2'
	{ image_data_Font_big_0xf2, 8},//, 12},

	// character: '3'
	{ image_data_Font_big_0xf3, 8},//, 12},

	// character: '4'
	{ image_data_Font_big_0xf4, 8},//, 12},

	// character: '5'
	{ image_data_Font_big_0xf5, 8},//, 12},

	// character: '6'
	{ image_data_Font_big_0xf6, 8},//, 12},

	// character: '7'
	{ image_data_Font_big_0xf7, 8},//, 12},

	// character: '8'
	{ image_data_Font_big_0xf8, 8},//, 12},

	// character: '9'
	{ image_data_Font_big_0xf9, 8},//, 12},

	// BATT - id12
	
	{ image_data_batt0, 8},//, 7},
	//{ image_data_batt1, 13, 9},
	//{ image_data_batt2, 13, 9},
	//{ image_data_batt3, 13, 9},
	
	//  texts - id16
	
	{ image_data_Image_uvi, 15},//, 6},
	{ image_data_Image_c, 8},//, 6},
	{ image_data_Image_mbar, 25},//, 6},
	{ image_data_Image_rh, 16},//, 6},

	// corners - id20

	{ image_data_corner_T_L, 8},//, 3},
	{ image_data_corner_T_R, 8},//, 3},
	{ image_data_corner_B_L, 8},//, 3},
	{ image_data_corner_B_R, 8},//, 3},
	
	// BAR - id24

	{ image_data_bar, 4},//, 3},
	
	// id25 - uvih
	{ image_data_Image_uvih, 21},//, 6},

};
