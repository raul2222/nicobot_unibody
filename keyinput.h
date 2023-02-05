/* 
Tarea de configuración de parámetros  #####################################################################
*/
void task_config(void *pvParameter) {
  char ini_char = '0';

  while(1) { 
    // Detectar caracter enviado
    if(Serial.available() > 0){
        String str = Serial.readStringUntil('\n');
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
            Serial.print("----Modo Velocidad----");
            #endif
            
      }
      if (str.indexOf("R") == 0  or str.indexOf("r") == 0) {
            str.replace("R", "");str.replace("r", "");str.replace(",",".");
            setpoint = str.toFloat();
            setpoint2 = setpoint;
            if( ACTIVA_P1C_MED_ANG == 1 ){
              #ifdef ACTIVA_DEBUG
                Serial.print("Angulo= ");
                Serial.print(setpoint, 2);
                Serial.println("º");
              #endif
            }else{

                
                #ifdef ACTIVA_DEBUG
                Serial.print("Velocidad= ");
                Serial.print(setpoint);
                Serial.println("rps");
                #endif
            }

         }
      if(str.indexOf("A") == 0 or str.indexOf("a") == 0){
            str.replace("A", "");str.replace("a", "");
            ACTIVA_P1C_MED_ANG = 1;
            volt_max = 3.0;
            clean();   //init_eeprom();
            #ifdef ACTIVA_DEBUG
            Serial.println("----Modo Angulo----");
            #endif
      }

      if(str.indexOf("Z") == 0 or str.indexOf("z") == 0){
            str.replace("Z", "");str.replace("z", "");
             start_stop = 1;
             #ifdef ACTIVA_DEBUG
             Serial.println("--START--");
             #endif
             
      }
      
      
      if(str.indexOf("S") == 0 or str.indexOf("s") == 0){
          str.replace("S", "");str.replace("s", "");
          if(str == "1"){
              start_stop = 1;
              
              #ifdef ACTIVA_DEBUG
              Serial.println("--START--");
              #endif
          }
          else{
              start_stop = 0;
              #ifdef ACTIVA_DEBUG
              Serial.println("--STOP--");
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
