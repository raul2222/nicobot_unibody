
/*
SET UP -----------------------------------------------------------------------------------
*/
void setup() {
  // Configuracion puerto serie
  config_sp();
  digitalRead(A_enc_pin);

  // Configuracion PWM
  config_PWM();

  // Configuracion PWM
  //config_ADC();

  // Crear cola_enc
  cola_enc2 = xQueueCreate(TAM_COLA_I, TAM_MSG_I);
  if(cola_enc2 == NULL){
      Serial.println("Error en creacion de cola_enc2");
      exit(-1);
  }

  cola_enc = xQueueCreate(TAM_COLA_I, TAM_MSG_I);
  if(cola_enc == NULL){
      Serial.println("Error en creacion de cola_enc");
      exit(-1);
  }

   //Crear la tarea task_enc
  if(xTaskCreatePinnedToCore( task_enc , "task_enc", 2048, NULL, 3, NULL,1) != pdPASS){
      Serial.println("Error en creacion tarea task_enc");
      exit(-1);
  }

  if(xTaskCreatePinnedToCore( task_enc2 , "task_enc2", 2048, NULL, 3, NULL,0) != pdPASS){
      Serial.println("Error en creacion tarea task_enc");
      exit(-1);
  }

  // Crear la tarea task_config
  if(xTaskCreatePinnedToCore( task_config , "task_config", 2048, NULL, 1, NULL,0) != pdPASS){
      Serial.println("Error en creacion tarea task_config");
      exit(-1);
  }

  // Crear la tarea task_loopcontr
  if(xTaskCreatePinnedToCore( task_loopcontr , "task_loopcontr", 2048, NULL, 2, NULL,1) != pdPASS){
      Serial.println("Error en creacion tarea task_loopcontr");
      exit(-1);
  }

  if(xTaskCreatePinnedToCore( task_loopcontr2 , "task_loopcontr2", 2048, NULL, 2, NULL,0) != pdPASS){
      Serial.println("Error en creacion tarea task_loopcontr");
      exit(-1);
  }


  if(xTaskCreatePinnedToCore( task_adc , "task_adc", 2048, NULL, 3, NULL,0) != pdPASS){
      Serial.println("Error en creacion tarea task_medidas");
      exit(-1);
  }


   // Crear la tarea task_medidas
  if(xTaskCreatePinnedToCore( task_medidas , "task_medidas", 2048, NULL, 3, NULL,0) != pdPASS){
      Serial.println("Error en creacion tarea task_medidas");
      exit(-1);
  }

  // Crear la tarea task_adc


  // Configuracion del encoder
  config_enc();


  Kp=0.6;
  Ki=0.8;
  Kd=0.01;
  N = 2;
  
  flancos = 834.0;
  Kp2=0.6;
  Ki2=0.8;
  Kd2=0.01;
  N2=2;
  
  ACTIVA_P1C_MED_ANG2 == 0;
  ACTIVA_P1C_MED_ANG == 0;
  setpoint = 0;
  setpoint2 = 0;
  start_stop=1;
  start_stop2=1;
}

/*
 RUTINAS ATENCION INTERRUPCIONES ########################################################################
*/
/* 
 Rutina de atención a interrupción ISC_enc --------------------------------------------
*/
void IRAM_ATTR ISR_enc() {
  // Lee las salidas del Encoder    
  uint8_t a = digitalRead(A_enc_pin);
  //uint8_t b = digitalRead(B_enc_pin);
  
  r = a;
  // Enviar los bytes a la cola 
  if (xQueueSendFromISR( cola_enc , &r ,NULL) != pdTRUE)
  {
      printf("Error de escritura en la cola cola_enc \n");
  }
}

void IRAM_ATTR ISR_enc2() {
  // Lee las salidas del Encoder    
  uint8_t a = digitalRead(A_enc_pin2);
  //uint8_t b = digitalRead(B_enc_pin);
  
  r2 = a;
  // Enviar los bytes a la cola 
  if (xQueueSendFromISR( cola_enc2 , &r2 ,NULL) != pdTRUE)
  {
      printf("Error de escritura en la cola cola_enc \n");
  }
}

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del adc
////////////////////////////////////////////////////////////////////////////////////

void config_ADC(){
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
  /*if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }*/
}  

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del encoder
////////////////////////////////////////////////////////////////////////////////////

void config_enc(){
    // Configuracion de pines del encoder
    pinMode(A_enc_pin, INPUT);
    pinMode(A_enc_pin2, INPUT);
    // Configuracion interrupcion
    attachInterrupt(A_enc_pin, ISR_enc, CHANGE);
    attachInterrupt(A_enc_pin2, ISR_enc2, CHANGE);
} 

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del PWM
////////////////////////////////////////////////////////////////////////////////////

void config_PWM(){
    // Configuracion de pines de control PWM
    pinMode(PWM_f, OUTPUT); 
    pinMode(PWM_f2, OUTPUT); 
    pinMode(2, OUTPUT);
    // Configuracion LED PWM 
    ledcSetup(pwmChannel, pwmfreq, pwmresolution);
    ledcSetup(pwmChannel2, pwmfreq, pwmresolution);
    // Asignar el controlador PWM al GPIO
    ledcAttachPin(PWM_Pin, 0);
    ledcAttachPin(PWM_Pin2, 1);
}  

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del puerto serie
////////////////////////////////////////////////////////////////////////////////////
void config_sp(){
  Serial.begin(115200);

}  


////////////////////////////////////////////////////////////////////////////////////
// Funcion de interpolacion LUT de Velocidad-Voltaje
////////////////////////////////////////////////////////////////////////////////////

float interpola_vel_vol_lut(float x) {
    // Buscar el valor superior más pequeño del array
    int8_t i = 0;
    if ( x >= Vel_LUT[LONG_LUT - 2] ) {i = LONG_LUT - 2;}
    else {while ( x > Vel_LUT[i+1] ) i++;}
  
    // Guardar valor superior e inferior
    float xL = Vel_LUT[i];
    float yL = Vol_LUT[i];
    float xR = Vel_LUT[i+1];
    float yR = Vol_LUT[i+1];
  
    // Interpolar
    float dydx = ( yR - yL ) / ( xR - xL );
  
    return yL + dydx * ( x - xL );
}
