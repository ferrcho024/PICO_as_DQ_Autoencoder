#include <Arduino.h>
#include <stdio.h>
//#include <stdlib.h>

#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
//#include "littlefs-lib/pico_hal.h"

// Import TensorFlow stuff - Autoencoder
//#include <TensorFlowLite_ESP32.h>
#include "modelo_df.h" // Autoencoder
#include "tensorflow/lite/micro/kernels/micro_ops.h"
//#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
//#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
//#include "tensorflow/lite/micro/all_ops_resolver.h"
//#include "tensorflow/lite/micro/system_setup.h"
//#include "tensorflow/lite/schema/schema_generated.h"

#include "parameters.h"
//#include "file_func.h"
#include "connectivity.h"
#include "dimensions.h"
#include "ntp_client.h" // NTP 
#include "mqtt.h"

// We need our utils functions for calculating MEAN, SD and Normalization
extern "C" {
#include "utils.h"
};

// Set to 1 to output debug info to Serial, 0 otherwise
#define DEBUG 1

// Settings Autoencoder
constexpr float THRESHOLD = 0.3500242427984803;    // Any MSE over this is an anomaly
constexpr float MEAN_TRAINING = 26.403898673843077;    // Mean of the training process
constexpr float STD_TRAINING = 10.86128076630132;    // Standard Desviation of the training process
constexpr int WAIT_TIME = 1000;       // ms between sample sets

// Settings DQ 
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

// TFLite globals, used for compatibility with Arduino-style sketches - Autoencoder
namespace {
  tflite::ErrorReporter* error_reporter = nullptr;
  const tflite::Model* model = nullptr;
  tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor* model_input = nullptr;
  TfLiteTensor* model_output = nullptr;

  // Create an area of memory to use for input, output, and other TensorFlow
  // arrays. You'll need to adjust this by combiling, running, and looking
  // for errors.
  constexpr int kTensorArenaSize = 6 * 1024;
  uint8_t tensor_arena[kTensorArenaSize];
} // namespace

void task1() {
  int cont = 0; // Variable de conteo de datos recibidos.
  float queue_df[60];
  float queue_nova[60];

  /*
  char c = getchar();

  if (pico_mount((c | ' ') == 'y') != LFS_ERR_OK) {
        Serial.printf("Error mounting FS\n");
    }else{
      Serial.printf("Se montó\n");
    }
  */

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

      // Autoencoder
      TfLiteStatus invoke_status;

      //size_t size = sizeof(read_data) / sizeof(read_data[0]);

      float* input_data = normalize_data(values_df, listSize, MEAN_TRAINING, STD_TRAINING);

      // Copiar los datos al tensor de entrada del modelo
      for (int i = 0; i < listSize; i++) {
          float value = input_data[i];
          model_input->data.f[0] = value;

          /*
          Serial.println("\nValores ingresados al modelo");
          for (int pos = 0; pos < listSize; pos++) {
            Serial.println(model_input->data.f[pos]);
          }
          */

          // Run inference
          invoke_status = interpreter->Invoke();
          if (invoke_status != kTfLiteOk) {
            error_reporter->Report("Invoke failed on input");
          }

          // Read predicted y value from output buffer (tensor)
          float acum = 0;
          Serial.println();
          

          //Serial.println("\nValores output después de ejecutado el modelo 1");
          // Reshaping the array for compatibility with 1D model
          for (int pos = 0; pos < listSize*16; pos+=4) {
            //Serial.println(model_output->data.f[pos]);

            acum += model_output->data.f[pos];
          }

          float pred_vals = acum/4;

          /*
          Serial.println("\nValores output después de ejecutado el modelo 2");
          Serial.println(pred_vals);
          */

          #if DEBUG
            Serial.println("Inference result: ");
            String msg = "Is " + String(value,2) + " an Outlier?: ";
            Serial.print(msg);
            float mae_loss = fabs(pred_vals - value);
            if (mae_loss > THRESHOLD){
              Serial.println("YES");
              Serial.println("****** OUTLIER ******");
              Serial.print("INPUT DATA: ");
              Serial.println(values_df[i]);
              Serial.print("MAE: ");
              Serial.println(mae_loss);
              Serial.println();
            }
            else{
              Serial.println("NO");
            }           
          #endif
      }



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


  // Autoeoncoder
  // Set up logging (will report to Serial, even within TFLite functions) - Autoencoder
  //static tflite::ErrorReporter error_reporter;
  //static tflite::MicroErrorReporter micro_error_reporter;
  //error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure - Autoencoder
  model = tflite::GetModel(modelo_df);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report("Model version does not match Schema");
    while(1);
  }


  // With all Ops Resolver
  //static tflite::AllOpsResolver micro_mutable_op_resolver;

  // Pull in only needed operations (should match NN layers) - Autoencoder
  // Available ops:
  //  https://github.com/tensorflow/tensorflow/blob/master/tensorflow/lite/micro/kernels/micro_ops.h
  // Based on https://colab.research.google.com/github/tensorflow/tensorflow/blob/master/tensorflow/lite/g3doc/guide/model_analyzer.ipynb#scrollTo=_jkg6UNtdz8c
  static tflite::MicroMutableOpResolver<7> micro_mutable_op_resolver;
  micro_mutable_op_resolver.AddConv2D();
  micro_mutable_op_resolver.AddTransposeConv();
  micro_mutable_op_resolver.AddStridedSlice();
  micro_mutable_op_resolver.AddShape();
  micro_mutable_op_resolver.AddPack();
  micro_mutable_op_resolver.AddDequantize();
  micro_mutable_op_resolver.AddQuantize();
  
  // Build an interpreter to run the model - Autoencoder
  static tflite::MicroInterpreter static_interpreter(
    model, micro_mutable_op_resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors - Autoencoder
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    error_reporter->Report("AllocateTensors() failed");
    while(1);
  }

  // Assign model input and output buffers (tensors) to pointers - Autoencoder
  model_input = interpreter->input(0);
  model_output = interpreter->output(0);



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