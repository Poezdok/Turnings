
#include "Lights.h"
//#include "stdbool.h"

//static uint8_t Turns_Level = 255;
static uint8_t Stop_On_Level = 255;
static uint8_t Stop_Off_Level = 63;
static int32_t level;
static uint32_t pix;
static uint32_t led_count = 8;
//static bool Turned_On = false;
//bool Need_To_Turn_Off = false;

typedef enum Mode {
    Left,
    Right,
    Both,
    None
} Mode;


static enum Mode current_mode = None;

void Lights_By_Timer(void);
static void Right_Light(void);
static void Left_Light(void);
static void Both_Light(void);
static void Set_Stop_Level(uint8_t Level);

void Turn_On_Left(void){
  
    if(current_mode != Left){
        pix = 0;
        Turn_Off();
        current_mode = Left;
        Lights_By_Timer();
    }
           
}

void Turn_On_Right(void){
    if(current_mode != Right){
        pix = 0;
        Turn_Off();
        current_mode = Right;
        Lights_By_Timer();
    }
        
}


void Turn_On_Both(void){
    if(current_mode != Both){
        pix = 0;
        Turn_Off();
        current_mode = Both;
        Lights_By_Timer();
    }
        
}



void Turn_Off(void){

    for(int32_t i = 0; i < led_count; i++){
        setPixel(0,0,0, i + 16);
        setPixel(0,0,0, led_count - i -1);        
    }

    RefreshMatrix();
    pix = 0;
    level = 0;

}




void Lights_By_Timer(void){
    //enum Mode current_mode;
   
    switch(current_mode){
        case Left: Left_Light();
            break;
        case Right: Right_Light();
            break;
        case Both: Both_Light();
            break;
        case None:
            break;
        default:
            break;
        
        
    }
    
    
}


static void Left_Light(void){
    
    if (pix < led_count){
        if(level <= 255){
            setPixel(TURN_COLOR_R*level/255, TURN_COLOR_G*level/255, TURN_COLOR_B*level/255, pix + 16);
            RefreshMatrix();
            level += 5;
        } else {
            pix++;
            level = 0;
            Left_Light();
        }
    } else if(pix == led_count){
       if(level <= 255){
            level += 1;
       } else {
           level = 255;
           pix++;
           Left_Light();
       }
    } else if(pix == led_count + 1){
        if(level >= 0){
            for(int32_t i = 0; i < led_count; i++){
                setPixel(TURN_COLOR_R*level/255, TURN_COLOR_G*level/255, TURN_COLOR_B*level/255, i + 16);
            }
            RefreshMatrix();
            level -= 5;
        } else{
            pix = 0;
            level = 0;
            Left_Light();
        } 
    }
}

static void Right_Light(void){
    
    if (pix < led_count){
        if(level <= 255){
            setPixel(TURN_COLOR_R*level/255, TURN_COLOR_G*level/255, TURN_COLOR_B*level/255, led_count - pix -1);
            RefreshMatrix();
            level += 5;
        } else {
            pix++;
            level = 0;
            Right_Light();
        }
    } else if(pix == led_count){
       if(level <= 255){
            level += 1;
       } else {
           level = 255;
           pix++;
           Right_Light();
       }
    } else if(pix == led_count + 1){
        if(level >= 0){
            for(int32_t i = 0; i < led_count; i++){
                setPixel(TURN_COLOR_R*level/255, TURN_COLOR_G*level/255, TURN_COLOR_B*level/255, led_count - i -1);
                
            }
            RefreshMatrix();
            level -= 5;
        } else{
            pix = 0;
            level = 0;
            Right_Light();
        } 
    }
        
        
        
    
    
}


static void Both_Light(void){
    
    if (pix < led_count){
        if(level <= 255){
            setPixel(TURN_COLOR_R*level/255, TURN_COLOR_G*level/255, TURN_COLOR_B*level/255, pix + 16);
            setPixel(TURN_COLOR_R*level/255, TURN_COLOR_G*level/255, TURN_COLOR_B*level/255, led_count - pix -1);
            RefreshMatrix();
            level += 5;
        } else {
            pix++;
            level = 0;
            Left_Light();
        }
    } else if(pix == led_count){
       if(level <= 255){
            level += 1;
       } else {
           level = 255;
           pix++;
           Left_Light();
       }
    } else if(pix == led_count + 1){
        if(level >= 0){
            for(int32_t i = 0; i < led_count; i++){
                setPixel(TURN_COLOR_R*level/255, TURN_COLOR_G*level/255, TURN_COLOR_B*level/255, i + 16);
                setPixel(TURN_COLOR_R*level/255, TURN_COLOR_G*level/255, TURN_COLOR_B*level/255, led_count - i -1);
            }
            RefreshMatrix();
            level -= 5;
        } else{
            pix = 0;
            level = 0;
            Left_Light();
        } 
    }
}


void Lights_Init(void){

    LL_TIM_EnableCounter(TIM4);
    LL_TIM_EnableIT_UPDATE(TIM4);
	WS2812Init();
	Set_Stop_Level(25);
}

void Stop_On(void){
    Set_Stop_Level(Stop_On_Level);
}

void Stop_Off(void){
    Set_Stop_Level(Stop_Off_Level);
}


static void Set_Stop_Level(uint8_t Level){
	for (uint8_t pix = REAR_STOP_LEFT; pix <= REAR_STOP_RIGHT; pix++){
		setPixel(STOP_COLOR_R*Level/255, STOP_COLOR_G*Level/255, STOP_COLOR_B*Level/255, pix);
	}
	RefreshMatrix();
}


/*
void LeftOnes(void){
	for (uint16_t pix = 0; pix <= TURN_COUNT; pix ++){
			for(uint16_t level = 0; level <= 255; level++){
				setPixel(TURN_COLOR_R * (Turns_Level * level)/(255 * 255), TURN_COLOR_G * level/255, TURN_COLOR_B * level/255, REAR_LEFT_INSIDE - pix);
//				setPixel(TURN_COLOR_R * (Turns_Level * level)/(255 * 255), TURN_COLOR_G * level/255, TURN_COLOR_B * level/255, REAR_RIGHT_INSIDE + pix);
				RefreshMatrix();
				for(uint16_t delay = 0; delay <= 1800; delay++);
			}
		}
	LL_mDelay(100);
	for(uint16_t level = 256; level > 0; level--){
		for (uint16_t pix = 0; pix <= TURN_COUNT; pix ++){
			setPixel(TURN_COLOR_R * (Turns_Level * (level-1))/(255 * 255), TURN_COLOR_G * (level-1)/255, TURN_COLOR_B * (level-1)/255, REAR_LEFT_INSIDE - pix);
//			setPixel(TURN_COLOR_R * (Turns_Level * (level-1))/(255 * 255), TURN_COLOR_G * (level-1)/255, TURN_COLOR_B * (level-1)/255, REAR_RIGHT_INSIDE + pix);
		}
	

		RefreshMatrix();
		for(uint16_t delay = 0; delay <= 3600; delay++);
		}
		for (uint16_t pix = 0; pix <= TURN_COUNT; pix ++){
	setPixel(0,0,0, REAR_LEFT_INSIDE - pix);
	};
		RefreshMatrix();
}


void RightOnes(void){
	for (uint16_t pix = 0; pix <= TURN_COUNT; pix ++){
			for(uint16_t level = 0; level <= 255; level+= 10){
//				setPixel(TURN_COLOR_R * (Turns_Level * level)/(255 * 255), TURN_COLOR_G * level/255, TURN_COLOR_B * level/255, REAR_LEFT_INSIDE - pix);
				setPixel(TURN_COLOR_R * (Turns_Level * level)/(255 * 255), TURN_COLOR_G * level/255, TURN_COLOR_B * level/255, REAR_RIGHT_INSIDE + pix);
				RefreshMatrix();
//				for(uint16_t delay = 0; delay <= 1800; delay++);
			LL_mDelay(1);
			}
	}
	LL_mDelay(100);
	for(uint16_t level = 256; level > 0; level--){
		for (uint16_t pix = 0; pix <= TURN_COUNT; pix ++){
//			setPixel(TURN_COLOR_R * (Turns_Level * (level-1))/(255 * 255), TURN_COLOR_G * (level-1)/255, TURN_COLOR_B * (level-1)/255, REAR_LEFT_INSIDE - pix);
			setPixel(TURN_COLOR_R * (Turns_Level * (level-1))/(255 * 255), TURN_COLOR_G * (level-1)/255, TURN_COLOR_B * (level-1)/255, REAR_RIGHT_INSIDE + pix);
		}
		RefreshMatrix();
		for(uint16_t delay = 0; delay <= 3600; delay++);
		}
}



void TwoOnes(void){
    
    
    
	
	
	
	


	for (uint16_t pix = 0; pix <= TURN_COUNT; pix ++){
			for(uint16_t level = 0; level <= Turns_Level; level++){
				setPixel(TURN_COLOR_R * level/255, TURN_COLOR_G * level/255, TURN_COLOR_B * level/255, REAR_LEFT_INSIDE - pix);
				setPixel(TURN_COLOR_R * level/255, TURN_COLOR_G * level/255, TURN_COLOR_B * level/255, REAR_RIGHT_INSIDE + pix);
				RefreshMatrix();
				for(uint16_t delay = 0; delay <= 1800; delay++);
			}
	}
	LL_mDelay(100);
	for(uint16_t level = Turns_Level; level > 0; level--){
		for (uint16_t pix = 0; pix <= TURN_COUNT; pix ++){
			setPixel(TURN_COLOR_R * (level-1)/255, TURN_COLOR_G * (level-1)/255, TURN_COLOR_B * (level-1)/255, REAR_LEFT_INSIDE - pix);
			setPixel(TURN_COLOR_R * (level-1)/255, TURN_COLOR_G * (level-1)/255, TURN_COLOR_B * (level-1)/255, REAR_RIGHT_INSIDE + pix);
		}
		RefreshMatrix();
		for(uint16_t delay = 0; delay <= 3600; delay++);
		}
	
}
*/

