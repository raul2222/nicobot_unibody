
void task_loopcontr(void* arg) {

  while(1) {    
    if(start_stop == true){
        Akpi=Kp+(Ki*dt);
        Akp = -Kp;
        A0d = Kd/dt;
        A1d = (-2.0)*(Kd/dt);
        A2d = Kd/dt;
        Kd == 0 ? tau = 0 : tau = Kd/(Kp*N); // IIR filter time constant  
        isinf(dt / (2*tau)) ? alpha = 0 : alpha = dt / (2*tau);

        Akpi2=Kp2+(Ki2*dt);
        Akp2 = -Kp2;
        A0d2 = Kd2/dt;
        A1d2 = (-2.0)*(Kd2/dt);
        A2d2 = Kd2/dt;
        Kd2 == 0 ? tau2 = 0 : tau2 = Kd2/(Kp2*N2); // IIR filter time constant  
        isinf(dt / (2*tau2)) ? alpha2 = 0 : alpha2 = dt / (2*tau2);

        if (ACTIVA_P1C_MED_ANG == 0){
           // rps_vel
            v_medida = (ang_cnt * 2.0 * PI) / flancos;
            da = v_medida - anterior;
            anterior = v_medida;
            v_medida = da / (BLOQUEO_TAREA_LOOPCONTR_MS / 1000.0); // rad/s
            //v_medida = v_medida / (2.0 * PI); // rps
            //v_medida = v_medida * radius_wheel; // m/s
        } else {
            v_medida = (ang_cnt * 360.0) / flancos;  // Calculo de angulo
        }

        if (ACTIVA_P1C_MED_ANG2 == 0){
           // rps_vel
            v_medida2 = (ang_cnt2 * 2.0 * PI) / flancos;
            da2 = v_medida2 - anterior2;
            anterior2 = v_medida2;
            v_medida2 = da2 / (BLOQUEO_TAREA_LOOPCONTR_MS / 1000.0); // rad/s
            //v_medida = v_medida / (2.0 * PI); // rps
            //v_medida = v_medida * radius_wheel; // m/s
        } else {
            v_medida2 = (ang_cnt2 * 360.0) / flancos;  // Calculo de angulo
        }

        error_2 = error_1;
        error_1 = error_0;
        error_0 = setpoint - v_medida;
       // PI
        output = output+(Akpi*error_0)+(Akp*error_1);
       // Filtered D
       if(alpha !=0) {
          d1 = d0;
          d0 = (A0d * error_0) + (A1d * error_1) + (A2d * error_2);
          fd1 = fd0;
          fd0 = ((alpha) / (alpha + 1)) * (d0 + d1) - ((alpha - 1) / (alpha + 1)) * fd1;
          output = output + fd0;  
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

       if (abs(output) > volt_max and output > 0) output = volt_max ; // min voltage value for dc-motor
       if (abs(output) < volt_min and setpoint > 0) output = volt_min ;  // min voltage value for dc-motor 
       if (abs(output) > volt_max and output < 0) output = -volt_max ;
       if (abs(output) < volt_min and setpoint < 0) output = -volt_min ;
       if(setpoint == 0) {
          excita_motor(0);
       } else{ 
          excita_motor(output);
              
       } 

       /*if (abs(output2) > volt_max and output2 > 0) output2 = volt_max ; // min voltage value for dc-motor
       if (abs(output2) < volt_min and setpoint2 > 0) output2 = volt_min ;  // min voltage value for dc-motor 
       if (abs(output2) > volt_max and output2 < 0) output2 = -volt_max ;
       if (abs(output2) < volt_min and setpoint2 < 0) output2 = -volt_min ;*/
       if(setpoint2 == 0) {
          excita_motor2(0);
       } else{ 
         
          excita_motor2(output2);          
       } 


       // Activacion de la tarea cada 0.01s
      vTaskDelay(BLOQUEO_TAREA_LOOPCONTR_MS / portTICK_PERIOD_MS);
    }
  }
}


void clean(){
  excita_motor(0);
  error_2 = 0; 
  error_1 = 0;
  error_0 = 0;
  output = 0;
  v_medida = 0;
  da = 0;
  anterior = 0;
  d1 = 0;
  d0 = 0;
  fd1 = 0;
  fd0 = 0;
  ang_cnt = 0;
}

/*
 Tarea task_enc #####################################################################
*/

void task_enc(void* arg) {
  uint8_t r ;
  uint8_t anterior ;
  while(1){
    // Espera a leer los datos de la cola
    if (xQueueReceive( cola_enc , &r ,(TickType_t) portMAX_DELAY) == pdTRUE){
        if(r != anterior){
                
            if(output >= 0){
              ang_cnt++;
            } else {
              ang_cnt--;
            }
                  
        }
        anterior = r;
    } else {
      printf("Error de lectura de la cola cola_enc \n");
    }
 }
}

void task_enc2(void* arg) {
  uint8_t r2 ;
  uint8_t anterior2 ;
  while(1){
    // Espera a leer los datos de la cola
    if (xQueueReceive( cola_enc , &r2 ,(TickType_t) portMAX_DELAY) == pdTRUE){
        if(r2 != anterior2){
                
            if(output2 >= 0){
              ang_cnt2++;
            } else {
              ang_cnt2--;
            }
                  
        }
        anterior2 = r2;
    } else {
      printf("Error de lectura de la cola cola_enc \n");
    }
 }
}


/*



// Tarea del lazo principal del controlador modo angulo ###############################################
void proceso_angulo(){
  
  v_medida = (ang_cnt * 360.0) / 1668.0;  // Calculo de angulo
  //no_error_motor_break();
  error = ref_val - v_medida;
       
  // FIX DIRECTION ON ANGLE MOVEMENT
  if(error_anterior > 0 and error < 0 or error_anterior < 0 and error > 0){
    reset_params();                // se ha pasado
  }
        
  // ****** Control proporcional ****** //
              
  V_p = error * K_p;
        
  // ****** Control proporcional integral ****** //
        
  if(T_i == 0){
    V_i = 0;
  } else{
    V_i = error * (K_p / T_i) * (BLOQUEO_TAREA_LOOPCONTR_MS / 1000.0) + V_i_anterior - V_i_windup;  
    if(windup_bool){
      V_i = V_i_anterior;      
    }
  }
              
  V_i_anterior = V_i;
              
  // ****** Control proporcional derivativo ****** //
  if(T_d == 0){
    V_d = 0;
  } else{
    if(fd_bool){
      fd = (alpha/(1+alpha))*((1/alpha)*error + fd_anterior);
      V_d = ((K_p * T_d) / (BLOQUEO_TAREA_LOOPCONTR_MS / 1000.0)) * (fd - fd_anterior);
      fd_anterior = fd;
    }else{
      V_d = ((K_p * T_d) / (BLOQUEO_TAREA_LOOPCONTR_MS / 1000.0)) * (error - error_anterior);
    }       
  }
  error_anterior = error;    
}

// Tarea del lazo principal del controlador modo RPS ###############################################
void proceso_rpm(){
  
    // Calculo de angulo
    v_medida = (ang_cnt * 2.0 * PI) / 1668;
    da = v_medida - anterior;
    anterior = v_medida;
    v_medida = da / 0.01; // rad/s
    v_medida = v_medida / (2.0 * PI); // rps
 
    //EL ORDEN
    
    error = ref_val - v_medida;
    no_error_motor_break_vel();        
 
    // ****** Control proporcional ****** //
    V_p = error * K_p;
              
    // ****** Control proporcional integral ****** //
    if(T_i == 0){
        V_i = 0;
    }else{
        V_i = error * (K_p / T_i) * (BLOQUEO_TAREA_LOOPCONTR_MS / 1000.0) + V_i_anterior - V_i_windup;
        if(windup_bool){
            V_i = V_i_anterior;      
        }
    }
    V_i_anterior = V_i;

    // ****** Control proporcional derivativo ****** //
    if(T_d == 0){
        V_d = 0;
    } else{
        if(fd_bool){
            fd = (alpha/(1+alpha))*((1/alpha)*error + fd_anterior);
            V_d = ((K_p * T_d) / (BLOQUEO_TAREA_LOOPCONTR_MS / 1000.0)) * (fd - fd_anterior);
            fd_anterior = fd;
        }else{
            V_d = ((K_p * T_d) / (BLOQUEO_TAREA_LOOPCONTR_MS / 1000.0)) * (error - error_anterior);
        }       
    }
              
    error_anterior = error;
    
}


*/
