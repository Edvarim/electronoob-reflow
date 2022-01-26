/*//             ---Have in mind---
 *This code is a first version for the reflow hot plate project.
 *Tutorial here: http://electronoobs.com/eng_arduino_tut161.php
 *Please be careful working with "High Voltage" and double check everything. 
 *Only use this project with supervision, never leave it plugged.
 *You can use this code at your own risk. I don't offer any guarantee that you 
 *will get the same results as I did and you might have to adjsut some PID values */

#include <thermistor.h>             //Downlaod it here: http://electronoobs.com/eng_arduino_thermistor.php
thermistor therm1(A0,0);            //The 3950 Thermistor conencted on A0 
//LCD config
#include <Wire.h>                   //Included by Arduino IDE
#include <LiquidCrystal_I2C.h>      //Downlaod it here: http://electronoobs.com/eng_arduino_liq_crystal.php
LiquidCrystal_I2C lcd(0x27,16,2);   //Define LCD address as 0x27. Also try 0x3f if it doesn't work. 

//Inputs and Outputs
const int c_but_1 = 12;
const int c_but_2 = 11;
const int c_but_3 = 10; 
const int c_but_4 = 9;
const int c_SSR = 3;
const int c_buzzer = 6;
const int c_Thermistor_PIN = A0;

const int c_max_modes = 3;                              //For now, we only work with 1 mode...

const String c_ModeNames[c_max_modes] = {"Mode 1", "EasyPrint Pb", "Desolder"};

const int c_preheat_setpoint[c_max_modes] = {140, 155, 155};
const int c_soak_setpoint[c_max_modes] = {150, 160, 160};
const int c_reflow_setpoint[c_max_modes] = {200, 220, 220};
const int c_preheat_duration[c_max_modes] = {90, 90, 90};
const int c_soak_duration[c_max_modes] = {20, 60, 60};
const int c_reflow_duration[c_max_modes] = {20, 75, 180};

const float c_cooldown_temp = 40;                       //When is ok to touch the plate
const float refresh_rate = 500;                       //LCD refresh rate. You can change this if you want
const float pid_refresh_rate  = 50;                   //PID Refresh rate

//Variables
unsigned int millis_before, millis_before_2;    //We use these to create the loop refresh rate
unsigned int millis_now = 0;

float seconds = 0;                              //Variable used to store the elapsed time                   
int running_mode = 0;                           //We store the running selected mode here
int selected_mode = 0;                          //Selected mode for the menu

bool but_3_state = true;                        //Store the state of the button (HIGH OR LOW)
bool but_4_state  =true;                        //Store the state of the button (HIGH OR LOW)
float temperature = 0;                          //Store the temperature value here

float temp_setpoint = 0;                        //Used for PID control
float pwm_value = 0;         ///Keista is 255                 //The SSR is OFF with HIGH, so 255 PWM would turn OFF the SSR


/////////////////////PID VARIABLES///////////////////////
/////////////////////////////////////////////////////////
const float Kp = 2;               //Mine was 2
const float Ki = 0.0025;          //Mine was 0.0025
const float Kd = 9;               //Mine was 9

/////////////////////////////////////////////////////////

void setup() {
  //Define the pins as outputs or inputs
  pinMode(c_SSR, OUTPUT);
  digitalWrite(c_SSR, LOW);   //Keista is High     //Make sure we start with the SSR OFF (is off with HIGH)
  pinMode(c_buzzer, OUTPUT); 
  digitalWrite(c_buzzer, LOW);  
  pinMode(c_but_1, INPUT_PULLUP);
  pinMode(c_but_2, INPUT_PULLUP);
  pinMode(c_but_3, INPUT_PULLUP);
  pinMode(c_but_4, INPUT_PULLUP);
  pinMode(c_Thermistor_PIN, INPUT);

  lcd.init();                     //Init the LCD
  lcd.backlight();              //Activate backlight   
  Serial.begin(9600);
  tone(c_buzzer, 1800, 200);     
  millis_before = millis();
  millis_now = millis();
}

void loop() {
  millis_now = millis();
  if(millis_now - millis_before_2 > pid_refresh_rate){    //Refresh rate of the PID
    millis_before_2 = millis(); 
    
    temperature = therm1.analog2temp();

    if(running_mode > 0 && running_mode < 10){   
      if(temperature < c_preheat_setpoint[running_mode - 1]){
        temp_setpoint = seconds*1.666;                    //Reach 150ºC till 90s (150/90=1.666)
      }  
        
      if(temperature > c_preheat_setpoint[running_mode - 1] && seconds < (c_preheat_duration[running_mode - 1] + c_soak_duration[running_mode - 1])){
        temp_setpoint = c_soak_setpoint[running_mode - 1];               
      }   
        
      else if(seconds > (c_preheat_duration[running_mode - 1] + c_soak_duration[running_mode - 1]) && seconds < (c_preheat_duration[running_mode - 1] + c_soak_duration[running_mode - 1] + c_reflow_duration[running_mode - 1])){
        temp_setpoint = c_reflow_setpoint[running_mode - 1];                 
      } 
       
      SetPWMValue();
      
      if(seconds > (c_preheat_duration[running_mode - 1] + c_soak_duration[running_mode - 1] + c_reflow_duration[running_mode - 1])){
        digitalWrite(c_SSR, LOW);   // pakesita is high         //With HIGH the SSR is OFF
        temp_setpoint = 0;
        running_mode = 10;                  //Cooldown mode        
      }     
    }//End of running_mode = 2


    //Mode 10 is between reflow and cooldown
    if(running_mode == 10){
      lcd.clear();
      lcd.setCursor(0,1);     
      lcd.print("    COMPLETE    ");
      tone(c_buzzer, 1800, 1000);    
      seconds = 0;              //Reset timer
      running_mode = 11;
      delay(3000);
    }    
  }//End of > millis_before_2 (Refresh rate of the PID code)
  

  
  millis_now = millis();
  if(millis_now - millis_before > refresh_rate){          //Refresh rate of prntiong on the LCD
    millis_before = millis();   
    seconds = seconds + (refresh_rate/1000);              //We count time in seconds
    

    //Mode 0 is with SSR OFF (we can selcet mode with buttons)
    if(running_mode == 0){ 
      digitalWrite(c_SSR, LOW);  //Pakeista is High      //With HIGH the SSR is OFF
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print("T: ");
      lcd.print(temperature,1);   
      lcd.setCursor(9,0);      
      lcd.print("SSR OFF"); 
       
      lcd.setCursor(0,1);  
      if(selected_mode == 0){
        lcd.print("Select Mode");     
      }
      else if(selected_mode == 1){
        lcd.print(c_ModeNames[selected_mode - 1]);     
      }
      else if(selected_mode == 2){
        lcd.print(c_ModeNames[selected_mode - 1]);     
      }
      else if(selected_mode == 3){
        lcd.print(c_ModeNames[selected_mode - 1]);     
      }
      
      
    }//End of running_mode = 0

     //Mode 11 is cooldown. SSR is OFF
     else if(running_mode == 11){ 
      if(temperature < c_cooldown_temp){
        running_mode = 0; 
        tone(c_buzzer, 1000, 100); 
      }
      digitalWrite(c_SSR, LOW);  //pakeista is high      //With HIGH the SSR is OFF 
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print("T: ");
      lcd.print(temperature,1);   
      lcd.setCursor(9,0);      
      lcd.print("SSR OFF"); 
       
      lcd.setCursor(0,1);       
      lcd.print("    COOLDOWN    ");  
    }//end of running_mode == 11

    //Mode 1 is the PID runnind with selected mode 1
    else if(running_mode > 0 && running_mode < 10){            
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print("T: ");
      lcd.print(temperature,1);  
      lcd.setCursor(9,0);       
      lcd.print("SSR ON"); 
       
      lcd.setCursor(0,1); 
      lcd.print("S");  lcd.print(temp_setpoint,0); 
      lcd.setCursor(5,1);     
      lcd.print("PWM");  lcd.print(pwm_value,0); 
      lcd.setCursor(12,1); 
      lcd.print(seconds,0);  
      lcd.print("s");         
    }//End of running_mode == 1
    
    
  }


  
  ///////////////////////Button detection////////////////////////////
  ///////////////////////////////////////////////////////////////////
  if(!digitalRead(c_but_3) && but_3_state){
    but_3_state = false;
    selected_mode ++;   
    tone(c_buzzer, 2300, 40);  
    if(selected_mode > c_max_modes){
      selected_mode = 0;
    }
    delay(150);
  }
  else if(digitalRead(c_but_3) && !but_3_state){
    but_3_state = true;
  }

  
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  if(!digitalRead(c_but_4) && but_4_state){
    if(running_mode > 0 && running_mode < 10){
      digitalWrite(c_SSR, LOW);  //pakeista is high      //With HIGH the SSR is OFF
      running_mode = 0;
      selected_mode = 0; 
      tone(c_buzzer, 2500, 150);
      delay(130);
      tone(c_buzzer, 2200, 150);
      delay(130);
      tone(c_buzzer, 2000, 150);
      delay(130);
    }
    
    but_4_state = false;
    if(selected_mode == 0){
      running_mode = 0;
    }
    else if(selected_mode > 0 && selected_mode < 10){
      running_mode = selected_mode;
      PlayAndResetTimer();
    }
  }
  else if(digitalRead(c_but_4) && !but_4_state){
    but_4_state = true;
  }
}

void SetPWMValue()
{
  float PID_Output = 0;
  float PID_P, PID_I, PID_D;
  float PID_ERROR, PREV_ERROR;
  float MIN_PID_VALUE = 0;
  float MAX_PID_VALUE = 180;                      //Max PID value. You can change this. 

  PID_ERROR = temp_setpoint - temperature;
  PID_P = Kp*PID_ERROR;
  PID_I = PID_I+(Ki*PID_ERROR);      
  PID_D = Kd * (PID_ERROR-PREV_ERROR);
  PID_Output = PID_P + PID_I + PID_D;
  //Define maximun PID values
  if(PID_Output > MAX_PID_VALUE){
    PID_Output = MAX_PID_VALUE;
  }
  else if (PID_Output < MIN_PID_VALUE){
    PID_Output = MIN_PID_VALUE;
  }
  //Since the SSR is ON with LOW, we invert the pwm singal
  pwm_value = PID_Output; //pakeista is 255 - PID_Output
  
  analogWrite(c_SSR,pwm_value);           //We change the Duty Cycle applied to the SSR
  
  PREV_ERROR = PID_ERROR;
}

void PlayAndResetTimer()
{
  tone(c_buzzer, 2000, 150);
  delay(130);
  tone(c_buzzer, 2200, 150);
  delay(130);
  tone(c_buzzer, 2400, 150);
  delay(130);
  seconds = 0;                    //Reset timer
}
