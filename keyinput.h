


void task_serial(void* arg)
{
  
  while (1) {
    if(Serial.available() > 0){

       lastMotorCommand = millis();
       String str = Serial.readStringUntil('\r');
       Serial2.println(str);
       if (str.indexOf("e") == 0 ) {
            Serial.print(ang_cnt); 
            Serial.print(" "); 
            Serial.println(ang_cnt2);
            Serial.flush();
            Serial2.print(ang_cnt); 
            Serial2.print(" "); 
            Serial2.println(ang_cnt2);
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
            //Serial2.println(str);
            str.replace("m", "");
            int i1 = str.indexOf(" ");

            String firstValue = str.substring(0, i1);
            String second = str.substring(i1 + 1);
            setpoint = firstValue.toFloat();
            setpoint2 = second.toFloat();
            Serial.println("OK"); 
            Serial.flush();
            Serial2.println(setpoint);
            Serial2.println(setpoint2);

        }

        if (millis() > (AUTO_STOP_INTERVAL + lastMotorCommand) ){
          setpoint = 0;
          setpoint2 = 0;
        }
          

    }
    if (millis() > (AUTO_STOP_INTERVAL + lastMotorCommand) ){
          setpoint = 0;
          setpoint2 = 0;
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}


/* 
Tarea de configuración de parámetros  #####################################################################
*/
void task_config(void *pvParameter) {
  char ini_char = '0';

  while(1) { 
    // Detectar caracter enviado
    if(Serial2.available() > 0){
        String str = Serial2.readStringUntil('\n');
        ini_char = str[0];
        if(str.indexOf("V") == 0 or str.indexOf("v") == 0){
            str.replace("V","");str.replace("v","");
            /*
            pwm_volt = str.toFloat();
            Serial.print("Voltaje motor = ");
            Serial.println(pwm_volt, 2);
            //ref_val = pwm_volt; estabamos usando esta forma
            */
            //puesta_a_cero();
            
            
            ACTIVA_P1C_MED_ANG = 0;
            //init_eeprom();
            #ifdef ACTIVA_DEBUG
            Serial2.print("----Modo Velocidad----");
            #endif
            
      }
      if (str.indexOf("R") == 0  or str.indexOf("r") == 0) {
            str.replace("R", "");str.replace("r", "");str.replace(",",".");
            setpoint = str.toFloat();
            setpoint2 = setpoint;
            if( ACTIVA_P1C_MED_ANG == 1 ){
              #ifdef ACTIVA_DEBUG
                Serial2.print("Angulo= ");
                Serial2.print(setpoint, 2);
                Serial2.println("º");
              #endif
            }else{

                
                #ifdef ACTIVA_DEBUG
                Serial2.print("Velocidad= ");
                Serial2.print(setpoint);
                Serial2.println("rps");
                #endif
            }

         }
      if(str.indexOf("A") == 0 or str.indexOf("a") == 0){
            str.replace("A", "");str.replace("a", "");
            ACTIVA_P1C_MED_ANG = 1;
            volt_max = 3.0;
            clean();   //init_eeprom();
            #ifdef ACTIVA_DEBUG
            Serial2.println("----Modo Angulo----");
            #endif
      }

      if(str.indexOf("Z") == 0 or str.indexOf("z") == 0){
            str.replace("Z", "");str.replace("z", "");
             start_stop = 1;
             #ifdef ACTIVA_DEBUG
             Serial2.println("--START--");
             #endif
             
      }
      
      
      if(str.indexOf("S") == 0 or str.indexOf("s") == 0){
          str.replace("S", "");str.replace("s", "");
          if(str == "1"){
              start_stop = 1;
              
              #ifdef ACTIVA_DEBUG
              Serial2.println("--START--");
              #endif
          }
          else{
              start_stop = 0;
              #ifdef ACTIVA_DEBUG
              Serial2.println("--STOP--");
              #endif
          }
          clean();
      }
      if(str.indexOf("P") == 0 or str.indexOf("p") == 0  ){
          str.replace("P",""); str.replace("p","");str.replace(",",".");
          //K_p = str.toFloat();
          Kp = str.toFloat();
          Serial.println(Kp);
         
          
      }
      if(str.indexOf("I") == 0 or str.indexOf("i") == 0){
          str.replace("I","");str.replace("i","");str.replace(",",".");
          //T_i = str.toFloat();
          Ki = str.toFloat();
          Serial.println(Ki);
         
        
      }
      if(str.indexOf("D") == 0 or str.indexOf("d") == 0){
          str.replace("D","");str.replace("d","");str.replace(",",".");
          //T_d = str.toFloat();
          Kd = str.toFloat();
          Serial.println(Kd);
          
      }


    }


    // Activacion de la tarea cada 0.1s
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}
