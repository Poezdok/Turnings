#include "main.h"
#include "ws2812b.h"

#define REAR_LIGHTS_COUNT 24

#define REAR_LEFT_OUTSIDE 0
#define REAR_LEFT_INSIDE 7

#define REAR_STOP_LEFT 8
#define REAR_STOP_RIGHT 15

#define REAR_RIGHT_INSIDE 16
#define REAR_RIGHT_OUTSIDE 23

#define TURN_COUNT REAR_LEFT_INSIDE - REAR_LEFT_OUTSIDE

#define TURN_COLOR_R 255
#define TURN_COLOR_G 112
#define TURN_COLOR_B 0

#define STOP_COLOR_R 255
#define STOP_COLOR_G 0
#define STOP_COLOR_B 0


void Lights_Init(void);

void Turn_Off(void);

void Turn_On_Left(void);
void Turn_On_Right(void);
void Turn_On_Both(void);

void Stop_On(void);
void Stop_Off(void);




//void Set_Turns_Level(uint8_t Level);
