
/*
 Tarea task_loopcontr #####################################################################
*/

void task_loopcontr2(void* arg) {

  while(1) {    
    
      if(start_stop2 == 1){

          Akpi2=Kp2+(Ki2*dt2);
          Akp2 = -Kp2;
          A0d2 = Kd2/dt2;
          A1d2 = (-2.0)*(Kd2/dt2);
          A2d2 = Kd2/dt2;
          Kd2 == 0 ? tau2 = 0 : tau2 = Kd2/(Kp2*N2); // IIR filter time constant  
          isinf(dt2 / (2*tau2)) ? alpha2 = 0 : alpha2 = dt / (2*tau2);

          if (ACTIVA_P1C_MED_ANG2 == 0){
            // Calculo rps
            v_medida2 = (ang_cnt2 * 2.0 * PI) / flancos;
            da2 = v_medida2 - anterior2;
            anterior2 = v_medida2;
            v_medida2 = da2 / (BLOQUEO_TAREA_LOOPCONTR_MS / 1000.0); // rad/s
            //v_medida2 = v_medida2 / (2.0 * PI); // rps
          } else {
             v_medida2 = (ang_cnt2 * 360.0) / flancos;  // Calculo de angulo
              //no_error_motor_break();
          }

          error_22 = error_12;
          error_12 = error_02;
          error_02 = setpoint2 - v_medida2;
          // PI
          output2 = output2+(Akpi2*error_02)+(Akp2*error_12);
          // Filtered D
          if(alpha2 !=0) {
            d12 = d02;
            d02 = (A0d2 * error_02) + (A1d2 * error_12) + (A2d2 * error_22);
            fd12 = fd02;
            fd02 = ((alpha2) / (alpha2 + 1)) * (d02 + d12) - ((alpha2 - 1) / (alpha2 + 1)) * fd12;
            output2 = output2 + fd02;  
          }

          if (abs(output2) > volt_max and output2 > 0) output2 = volt_max ; // min voltage value for dc-motor
          if (abs(output2) > volt_max and output2 < 0) output2 = -volt_max ;

          if (ACTIVA_P1C_MED_ANG2 == 1) {
            no_error_motor_break2();
          } else {
            if (abs(output2) < volt_min and setpoint2 > 0) output2 = volt_min ; 
            if (abs(output2) < volt_min and setpoint2 < 0) output2 = -volt_min ;
          }
          if( setpoint2 == 0) {
            clean2();
          } else {
            excita_motor2(output2);
          }
      } else {
          //clean();
      }
      // Activacion de la tarea cada 0.01s
      vTaskDelay(BLOQUEO_TAREA_LOOPCONTR_MS / portTICK_PERIOD_MS);
  }
}

void clean(){
  excita_motor(0);
  output = 0;
}

void clean2(){
  excita_motor2(0);
  output2 = 0;
}
/*
 Tarea task_enc #####################################################################
*/

void task_enc(void* arg) {
  uint8_t r ;
  uint8_t anterior_enc ;
  while(1){
    // Espera a leer los datos de la cola
    if (xQueueReceive( cola_enc , &r ,(TickType_t) portMAX_DELAY) == pdTRUE){
        if(r != anterior_enc){
                
            if(output >= 0){
              ang_cnt++;
            } else {
              ang_cnt--;
            }      
        }
        anterior_enc = r;
    } else {
        printf("Error de lectura de la cola cola_enc \n");
    }
  }
}

void task_enc2(void* arg) {
  uint8_t r2 ;
  uint8_t anterior_enc ;
  while(1){
    // Espera a leer los datos de la cola
    if (xQueueReceive( cola_enc2 , &r2 ,(TickType_t) portMAX_DELAY) == pdTRUE){
        if(r2 != anterior_enc){
                
            if(output2 >= 0){
              ang_cnt2++;
            } else {
              ang_cnt2--;
            }
                  
        }
        anterior_enc = r2;
    } else {
        printf("Error de lectura de la cola cola_enc \n");
    }
  }
}
