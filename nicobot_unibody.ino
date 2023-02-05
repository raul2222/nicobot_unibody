/* 
PRACTICA 1: IMPLEMENTACION DE UN CONTROLADOR PARA UN MOTOR DE DC
*/
// DECLARACIONES //////////////////////////////////////////////////////////////////////////


#define NOMBRE_PRAC "NICOBOT UNIBODY"
#define VERSION_SW "0.1"
#define ACTIVA_DEBUG

#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "global.h"
#include "extras.h"
#include "keyinput.h"
#include "1-loopcontr.h"
#include "log.h"
#include "setup.h"
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = 0;

/*
LOOP ---- NO USAR ------------------------------------------------------------------- 
*/
void loop() {

    if(Serial.available() > 0){

       lastMotorCommand = millis();
       String str = Serial.readStringUntil('\r');
       if (str.indexOf("e") == 0 ) {
            Serial.print(ang_cnt); 
            Serial.print(" "); 
            Serial.println(ang_cnt2);
            Serial.flush();
        }

        if (str.indexOf("u") == 0 ) {
            Serial.println("OK"); 
            Serial.flush();
        }
          
        if (str.indexOf("r") == 0 ) {
            //ang_cnt=0;
            //ang_cnt2=0;
            //reset contador encoder
            Serial.println("OK"); 
            Serial.flush();
        }
                
        if (str.indexOf("m") == 0 ) {
            Serial2.println(str);
            str.replace("m", "");
            int i1 = str.indexOf(" ");

            String firstValue = str.substring(0, i1);
            String second = str.substring(i1 + 1);
            setpoint = firstValue.toFloat();
            setpoint2 = second.toFloat();
            Serial.println("OK"); 
            Serial.flush();
        }
          

    }
    if (millis()>(AUTO_STOP_INTERVAL + lastMotorCommand) ){
          setpoint = 0;
          setpoint2 = 0;
    }
}

// FUNCIONES ////////////////////////////////////////////////////////////////////////
// Funcion excitacion del motor con PWM

void excita_motor(float v_motor){
    
    // Obtención de la dirección
    if(v_motor > 0){
        direccion = 1;
    }else{
        direccion = 0;
    }
    
    if(direccion_ant != direccion){  //("Cambio de sentido");
    }
    if(v_motor > 0){    //Serial.println("Hacia adelante");
        digitalWrite(PWM_f, 1); // el pin de direccion
    }
    if(v_motor < 0){    //("Hacia atras");
        v_motor = abs(v_motor); // valor en positivo del voltaje el cambio de direccion lo hacen las variables
        digitalWrite(PWM_f, 0);
    }

    direccion_ant = direccion;
  
  	// Calcula y limita el valor de configuración del PWM
    dutyCycle = (int) ((v_motor * 255)/12);
    // El valor de excitación debe estar entro 0 y PWM_Max
    if(dutyCycle >= PWM_Max){
        dutyCycle = PWM_Max;
    }
    if(dutyCycle <= 0){
        dutyCycle = 0;
    }
  	
  	// Excitacion del motor con PWM
  	ledcWrite(0, dutyCycle);
    //dacWrite(26,dutyCycle);
}  
void excita_motor2(float v_motor){
    
    // Obtención de la dirección
    if(v_motor > 0){
        direccion2 = 1;
    }else{
        direccion2 = 0;
    }
    
    if(direccion_ant2 != direccion2){  //("Cambio de sentido");
    }
    if(v_motor > 0){    //Serial.println("Hacia adelante");
        digitalWrite(PWM_f2, 0); // el pin de direccion
    }
    if(v_motor < 0){    //("Hacia atras");
        v_motor = abs(v_motor); // valor en positivo del voltaje el cambio de direccion lo hacen las variables
        digitalWrite(PWM_f2, 1);
    }

    direccion_ant2 = direccion2;
  
  	// Calcula y limita el valor de configuración del PWM
    dutyCycle2 = (int) ((v_motor * 255)/12);
    // El valor de excitación debe estar entro 0 y PWM_Max
    if(dutyCycle2 >= PWM_Max){
        dutyCycle2 = PWM_Max;
    }
    if(dutyCycle2 <= 0){
        dutyCycle2 = 0;
    }
  	
  	// Excitacion del motor con PWM
  	ledcWrite(1, dutyCycle2);
    //dacWrite(25,dutyCycle2);
}  
