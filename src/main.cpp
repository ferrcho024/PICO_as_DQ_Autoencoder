#include <Arduino.h>
#include <stdio.h>
//#include <stdlib.h>

#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "littlefs-lib/pico_hal.h"

/*
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#inclqude "freertos/queue.h"
*/




#include "parameters.h"
//#include "file_func.h"
#include "connectivity.h"
#include "dimensions.h"
#include "ntp_client.h" // NTP 
#include "mqtt.h"

 
//extern char* in_txt;
extern PubSubClient client;
extern String in_txt;
extern bool callback;

int listSize;  // Tamaño de la lista que almacena los values
int startline; // Inicializa la lectura desde la línea 0
int siataValue; // Contador para extraer el valor de la estación SIATA

bool ban = true;
int frec = 1000; // Espacio de tiempo entre los values que llegan (en milisegundos)
float values_df[60];
float values_nova[60];
float value_siata;


// Declaración de una cola
//QueueHandle_t queue_df;
//QueueHandle_t queue_nova;





void task1() {
  int cont = 0; // Variable de conteo de datos recibidos.
  float queue_df[60];
  float queue_nova[60];

  char c = getchar();

  if (pico_mount((c | ' ') == 'y') != LFS_ERR_OK) {
        Serial.printf("Error mounting FS\n");
    }else{
      Serial.printf("Se montó\n");
    }

  while(true){
    delay(frec/4);

    if (cont > 59){
      cont = 0;
      memcpy(values_df, queue_df, sizeof(queue_df));
      memcpy(values_nova, queue_nova, sizeof(queue_nova));
      ban = false;
    }

    if (callback){  

      //   Encontrar la posición de la primera coma
      int comaPos1 = in_txt.indexOf(',');

      // Extraer el token antes de la coma
      String df_value = in_txt.substring(0, comaPos1);

      int comaPos2 = in_txt.indexOf(',', comaPos1+1);
      String nova_value = in_txt.substring(comaPos1+1,comaPos2);
      
      in_txt = in_txt.substring(comaPos2 + 1);
      value_siata = in_txt.toFloat();


    //   if (token != ID){
      
      printf("Valor DF: %s\n",df_value);
      printf("Valor NOVA: %s\n",nova_value);
      printf("Valor SIATA: %s\n",in_txt);
      printf("%d\n",cont);
      printf("\n");
    
      //value_to_list(queue_df, df_value, cont);
      //value_to_list(queue_nova, nova_value, cont);   

      callback = false;
      cont++;
    }
  }
}

void task2() {
  float dimen[24][10];
  while (true) {

    delay(frec/2);

    if(!ban){
      printf("Tarea 2 ejecutándose en el núcleo 1 %d\n", ban);
      ban = true;
      listSize = sizeof(values_nova)/4;

      float p_com_df = completeness(values_df, listSize);
      printf("********** Completeness DF: %.5f\n", p_com_df);
      
      float p_com_nova = completeness(values_nova, listSize);
      printf("********** Completeness NOVA: %.5f\n", p_com_nova);

      float uncer = uncertainty(values_df, values_nova, listSize);
      printf("********** Uncertainty: %.5f\n", uncer);

      float p_df = precision(values_df, listSize);
      printf("********** Precision DF: %.5f\n", p_df);

      float p_nova = precision(values_nova, listSize);
      printf("********** Precision NOVA: %.5f\n", p_nova);
      
      float a_df = accuracy(values_df, value_siata, listSize);
      printf("********** Accuracy DF: %.5f\n", a_df);

      float a_nova = accuracy(values_nova, value_siata, listSize);
      printf("********** Accuracy NOVA: %.5f\n", a_nova);

      float concor = PearsonCorrelation(values_df, values_nova, listSize);
      printf("********** Concordance: %.5f\n", concor);

      float* valuesFusioned = plausability(p_com_df, p_com_nova, p_df, p_nova, a_df, a_nova, values_df, values_nova, listSize);
      float fusion = calculateMean(valuesFusioned, listSize);
      printf("********** Value Fusioned: %.5f\n", fusion);

      float DQIndex = DQ_Index(valuesFusioned, uncer, concor, value_siata, listSize);
      printf("********** DQ Index: %.5f\n", DQIndex);

      char mqtt_msg[50];
      sprintf(mqtt_msg, "%s,%.5f,distancia,%.5f",ID,fusion,DQIndex);
      client.publish(TOPIC.c_str(), mqtt_msg);

      char resultString[50];
      sprintf(resultString, "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f",p_com_df,p_com_nova,p_df,p_nova,a_df,a_nova,uncer,concor,fusion,DQIndex);
      //writeFile(dimensions, resultString);
      dimen[siataValue%24][0] = p_com_df;
      dimen[siataValue%24][1] = p_com_nova;
      dimen[siataValue%24][2] = p_df;
      dimen[siataValue%24][3] = p_nova;
      dimen[siataValue%24][4] = a_df;
      dimen[siataValue%24][5] = a_nova;
      dimen[siataValue%24][6] = uncer;
      dimen[siataValue%24][7] = concor;
      dimen[siataValue%24][8] = fusion;
      dimen[siataValue%24][9] = DQIndex;

      //read_data_from_file("/spiffs/data.txt"); // Lee el archivo con formato de hora y valor float
    }

  }

    //printf("************ Free Memory task 2: %u bytes\n", esp_get_free_heap_size());

}

void setup() {
  //stdio_init_all();
  Serial.begin(115200);

  // Configurar y conectar WiFi
  ConnectToWiFi();

  // Obtener hora de servidor NTP y actualizar el RTC
  run_ntp_test();
  
  //mountLittleFS();
  //createFile(data); // Archivo de memoria permanente 
  //createFile(dimensions); // Archivo que almacena las métricas cada hora 
  //writeFile(dimensions, "hora,comp_df,comp_nova,prec_df,prec_nova,acc_df,acc_nova,uncer,concor");
  //writeFile(data, "fechaHora,pm25df,pm25nova");
  createMQTTClient();
  //connectMQTTBroker();

  multicore_launch_core1(task2);
  task1();

  // Crea dos tareas y las asigna a diferentes núcleos
  //xTaskCreatePinnedToCore(task1, "Task1", 10000, NULL, 1, NULL, 0);
  //xTaskCreatePinnedToCore(task2, "Task2", 10000, NULL, 1, NULL, 1);

  // Iniciar el planificador de tareas
  //vTaskStartScheduler();
}

void loop() {
  reconnectMQTTClient();
  client.loop();
  delay(1000);
}