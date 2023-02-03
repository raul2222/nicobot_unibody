/*
  Tarea imprimir por terminal del controlador  #####################################################################
*/



void task_medidas(void* arg){
  int contador =0;
  float tv_medida=0;
  float toutput = 0;
  int limite=1;
  while (1) {



    if (start_stop == 1) {  

      if (contador ==limite){
       // Mostrar medidas de angulo y velocidad del motor
        Serial.print("D: ");
        Serial.print(dutyCycle2);
        Serial.print(" C: ");
        Serial.print(ang_cnt2);

        if ( ACTIVA_P1C_MED_ANG2 == 1 ) { // Medida de angulo
          a_medida2 = (ang_cnt2 * 360) / flancos;
          Serial.print(" M: ");
          Serial.print(a_medida2);
          Serial.print(" A: ");
          Serial.print(setpoint2, 2);

        } else { // Medida de velocidad
          Serial.print(" Med: ");
          //Serial.print(v_medida);
          Serial.print(tv_medida/limite);
          Serial.print(", Ref: ");
          Serial.print(setpoint2, 2);
        }
        Serial.print(", V: ");
      //Serial.print(output, 2);
        Serial.print(toutput/limite, 3);

        Serial.print(" Kp:");
        Serial.print(Kp2, 3);
        Serial.print(" Ki:");
        Serial.print(Ki2, 3);
        Serial.print(" Kd:");
        Serial.print(Kd2, 3);
        Serial.println(" ");
        tv_medida=0;
        toutput=0;
        contador=0;
      } else {
        //Acumular
                    
        tv_medida = tv_medida + v_medida2;
        toutput = toutput + output2;
        contador++;
      }
    }

   vTaskDelay(BLOQUEO_TAREA_MEDIDA_MS / portTICK_PERIOD_MS);
  }
}



void task_adc(void* arg)
{
  while (1) {
    /*
    if (start_stop == 1) {
        adc0 = ads.readADC_SingleEnded(0);
        //current = (adc0 * 0.1875)/1000;
        current = ads.computeVolts(adc0)/0.5;

    } else {

    }
*/
    // Activacion de la tarea cada 1s
    vTaskDelay(BLOQUEO_TAREA_LOOPCONTR_MS / portTICK_PERIOD_MS);
  }

}
