/***************************************************************
* TCLAB - Control PID v0.2
* Desarrollado por Garikoitz Martínez [garikoitz.info] [01/2023]
* https://garikoitz.info/blog/?p=1923
***************************************************************/
/***************************************************************
* Librerías
***************************************************************/
#include <PID_v1.h>
/***************************************************************
* Variables
***************************************************************/
int T1pin = 0;
int T2pin = 2;
int H1pin = 3;
int H2pin = 5;
float T1_PV = 0.0;
float T2_PV = 0.0; 
float T1_SP = 0.0;
float T2_SP = 0.0;
int T1_OP = 0;
int T2_OP = 0;
int T1SPMAX = 75;
int T2SPMAX = 75;
int T1OPMAX = 190;
int T2OPMAX = 190;
const long baud = 9600;       // serial baud rate
unsigned long previousMillis = 0;
int Ts = 50; //Sample time in ms
int contador = 0;
int n = 15; //Promedio lecturas
boolean newData = false;       
const char sp = ' ';           
const char nl = '\n';
// global variables
char Buffer[64]; 
int buffer_index = 0; 
String cmd;  
float val;
//PID
double SetpointT1, InputT1, OutputT1, SetpointT2, InputT2, OutputT2;
double KcT1=13.70, KiT1=0.14, KdT1=0.0; //ZN10
double KcT2=21.82, KiT2=0.18, KdT2=0.0; //IAE-SP
PID PIDT1(&InputT1, &OutputT1, &SetpointT1, KcT1, KiT1, KdT1, P_ON_E, DIRECT);    //PI-D
//PID PIDT1(&InputT1, &OutputT1, &SetpointT1, KcT1, KiT1, KdT1, P_ON_M, DIRECT);  //I-PD
PID PIDT2(&InputT2, &OutputT2, &SetpointT2, KcT2, KiT2, KdT2, P_ON_E, DIRECT);    //PI-D
//PID PIDT2(&InputT2, &OutputT2, &SetpointT2, Kc, Ki, Kd, P_ON_M, DIRECT);        //I-PD
/***************************************************************
* SINTONÍAS
* T1 -> K:0.380  T0:29.5   TP:100.5
* CC25      PI KcT1=08.29, KiT1=0.14, KdT1=0.0 
* CC10      PI KcT1=04.97, KiT1=0.07, KdT1=0.0 
* ZN25      PI KcT1=08.07, KiT1=0.08, KdT1=0.0  
* ZN10      PI KcT1=05.37, KiT1=0.05, KdT1=0.0  
* ITAE-C    PI KcT1=07.49, KiT1=0.12, KdT1=0.0 
* ITAE-SP   PI KcT1=04.74, KiT1=0.05, KdT1=0.0  
* IAE-C     PI KcT1=08.67, KiT1=0.12, KdT1=0.0 
* IAE-SP    PI KcT1=05.73, KiT1=0.05, KdT1=0.0  
* LAMBDA    PI KcT1=03.81, KiT1=0.04, KdT1=0.0      TF:40
* IMC       PI KcT1=04.73, KiT1=0.04, KdT1=0.0      TF:60
* SIMC      PI KcT1=04.44, KiT1=0.04, KdT1=0.0      TF:30
* IMP.SIMC  PI KcT1=04.18, KiT1=0.04, KdT1=0.0      TF:40
* ------------------------------------------------------
* T2 -> K:0.405   T0:20    TP:117 
* CC25      PI KcT1=12.45, KiT1=0.08, KdT1=0.0 
* CC10      PI KcT1=07.92, KiT1=0.13, KdT1=0.0 
* ZN25      PI KcT1=13.00, KiT1=0.19, KdT1=0.0 
* ZN10      PI KcT1=08.66, KiT1=0.13, KdT1=0.0 
* ITAE-C    PI KcT1=11.91, KiT1=0.23, KdT1=0.0 
* ITAE-SP   PI KcT1=07.30, KiT1=0.06, KdT1=0.0  
* IAE-C     PI KcT1=13.87, KiT1=0.25, KdT1=0.0 
* IAE-SP    PI KcT1=08.57, KiT1=0.07, KdT1=0.0 
* LAMBDA    PI KcT1=04.13, KiT1=0.04, KdT1=0.0      TF:50
* IMC       PI KcT1=06.02, KiT1=0.05, KdT1=0.0      TF:50
* SIMC      PI KcT1=05.78, KiT1=0.05, KdT1=0.0      TF:30
* IMP.SIMC  PI KcT1=06.11, KiT1=0.05, KdT1=0.0      TF:30
***************************************************************/
/***************************************************************
* SETUP
***************************************************************/
void setup() 
{ 
  analogReference(EXTERNAL);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.begin(baud);
  Serial.flush();
  //
  PIDT1.SetOutputLimits(0, 255);
  PIDT1.SetMode(MANUAL);
  PIDT2.SetOutputLimits(0, 255);
  PIDT2.SetMode(MANUAL);
  if (Ts < 100){
    PIDT1.SetSampleTime(Ts);
    PIDT2.SetSampleTime(Ts);
  }
    
}
/***************************************************************
* BUCLE PRINCIPAL
***************************************************************/
void loop() 
{
  if (millis() - previousMillis > Ts)
  {
    previousMillis = millis();
    //Lectura Temperaturas
    LecturaTTs();
    LeoCMD();
    ProcesoCMD();
    EjecutoCMD();
    //PID (Parámetros vía serial cmds)
    if (PIDT1.GetMode() == 1){//AUTO
        InputT1 = T1_PV;
        SetpointT1 = T1_SP;
        PIDT1.Compute();
        T1_OP = map(OutputT1, 0, 255, 0, 100); //PWM -> %
        analogWrite(H1pin,OutputT1);
     }else if (PIDT1.GetMode() == 0) {//MANUAL
        T1_SP = T1_PV;
     }
     if (PIDT2.GetMode() == 1){//AUTO
        InputT2 = T2_PV;
        SetpointT2 = T2_SP;
        PIDT2.Compute();
        T2_OP = map(OutputT2, 0, 255, 0, 100); //PWM -> %
        analogWrite(H2pin,OutputT2);
     }else if (PIDT2.GetMode() == 0) {//MANUAL
        T2_SP = T2_PV;
     }
     //Para Arduino COM Plotter
     Serial.print("#");        //Char inicio
     Serial.print(T1_SP,1);    //
     Serial.write(" ");        //separador
     Serial.print(T1_PV,1);    //
     Serial.write(" ");        //separador
     Serial.print(T1_OP);      //
     Serial.write(" ");        //separador
     Serial.print(T2_SP,1);    //
     Serial.write(" ");        //separador
     Serial.print(T2_PV,1);    //
     Serial.write(" ");        //separador
     Serial.print(T2_OP);      //
     Serial.println();
    //
  }//millis
}//loop
/***************************************************************
* FUNCIONES
***************************************************************/
double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void LeoCMD() {
  while (Serial && (Serial.available() > 0) && (newData == false)) {
    int byte = Serial.read();
    if ((byte != '\r') && (byte != nl) && (buffer_index < 64)) {
      Buffer[buffer_index] = byte;
      buffer_index++;
    }
    else {
      newData = true;
    }
  }   
}
void ProcesoCMD(void) {
  if (newData) {
    String read_ = String(Buffer);
    // separate command from associated data
    int idx = read_.indexOf(sp);
    cmd = read_.substring(0, idx);
    cmd.trim();
    cmd.toUpperCase();

    // extract data. toFloat() returns 0 on error
    String data = read_.substring(idx + 1);
    data.trim();
    val = data.toFloat();

    // reset parameter for next command
    memset(Buffer, 0, sizeof(Buffer));
    buffer_index = 0;
    newData = false;
  }
}
void EjecutoCMD(void) {
  if (cmd == "T1A") {
    PIDT1.SetMode(AUTOMATIC);
  }
  else if (cmd == "T1M") {
    PIDT1.SetMode(MANUAL);
  }
  else if (cmd == "T2A") {
    PIDT2.SetMode(AUTOMATIC);
  }
  else if (cmd == "T2M") {
    PIDT2.SetMode(MANUAL);
  }
  else if (cmd == "T1SP") {
    if (PIDT1.GetMode() == 1){
      if (val > T1SPMAX){
        T1_SP = T1SPMAX;
      }else{
        T1_SP = val;
      }
    }
  }
  else if (cmd == "T1OP") {
    if (PIDT1.GetMode() == 0){
      if (val > T1OPMAX){
        val = T1OPMAX;
      }
      T1_OP = val;
      val = map(val, 0, 100, 0, 255);
      analogWrite(H1pin,T1_OP);
    }
  }
  else if (cmd == "T2SP") {
    if (PIDT2.GetMode() == 1){
      if (val > T2SPMAX){
        T2_SP = T2SPMAX;
      }else{
        T2_SP = val;
      }
    }
  }
  else if (cmd == "T2OP") {
    if (PIDT2.GetMode() == 0){
      if (val > T2OPMAX){
        val = T2OPMAX;
      }
      T2_OP = val;
      val = map(val, 0, 100, 0, 255);
      analogWrite(H2pin,T2_OP);
    }
  }
  else if (cmd == "T1KC") {
    PIDT1.SetTunings(val,KiT1,KdT1);
  }
  else if (cmd == "T1KI") {
    PIDT1.SetTunings(KcT1,val,KdT1);
  }
  else if (cmd == "T1KD") {
    PIDT1.SetTunings(KcT1,KiT1,val);
  }
  else if (cmd == "T2KC") {
    PIDT2.SetTunings(val,KiT2,KdT2);
  }
  else if (cmd == "T2KI") {
    PIDT2.SetTunings(KcT2,val,KdT2);
  }
  else if (cmd == "T2KD") {
    PIDT2.SetTunings(KcT2,KiT2,val);
  }
  else if (cmd == "STT2") {
    StepTestT2();
  }
  else if (cmd == "STT1") {
    StepTestT1();
  }
  Serial.flush();
  cmd = "";
}
void LecturaTTs(void) {
  //Lectura Temperaturas
  for (int i = 0; i < n; i++) {   
    T1_PV += (analogRead(T1pin)* 0.322265625 - 50.0); // 3.3v AREF
    T2_PV += (analogRead(T2pin)* 0.322265625 - 50.0); // 3.3v AREF
    //T1_PV += (analogRead(T1pin)*4.88 - 500) / 10; //5v
    //T2_PV += (analogRead(T2pin)*4.88 - 500) / 10; //5v
  }
  T1_PV = T1_PV / float(n);
  T2_PV = T2_PV / float(n);
}
void StepTestT1(void) {
  PIDT2.SetMode(MANUAL);
  analogWrite(H2pin,0);
  PIDT1.SetMode(MANUAL);
  analogWrite(H1pin,0);
  while (T1_PV > 30){
    LecturaTTs(); 
    Serial.print(T1_PV,1);
    Serial.println();
    Serial.write("Esperando a T1_PV <= 30ºC");
    Serial.println();
  }
  for (int contador = 0; contador < 1850; contador++) {
      LecturaTTs(); 
      if (contador <=50){
         analogWrite(H1pin,0);
         T1_OP = 0;
      }else if (contador >=50 && contador <350){     
          analogWrite(H1pin,63);
          T1_OP = 63; //25%
      }else if (contador >=350 && contador <650){     
          analogWrite(H1pin,127);
          T1_OP = 127; //50%
      }else if (contador >=650 && contador <950){     
          analogWrite(H1pin,190);
          T1_OP = 190; //75%
      }else if (contador >=950 && contador <1250){     
          analogWrite(H1pin,127);
          T1_OP = 127;
      }else if (contador >=1250 && contador <1550){     
          analogWrite(H1pin,63);
          T1_OP = 63;
      }else if (contador >=1550 && contador <1850){     
          analogWrite(H1pin,0);
          T1_OP = 0;
      }
      if (1){
        //Debug serial Arduino COM Plotter
        Serial.print("#");        //Char inicio
        Serial.print(T2_PV,1);    //
        Serial.write(" ");        //separador
        Serial.print(T1_PV,1);    //
        Serial.write(" ");        //separador
        Serial.print(T1_OP);      //
        Serial.println();
      }
      delay(1000);
  }
}
void StepTestT2(void) {
  PIDT2.SetMode(MANUAL);
  analogWrite(H2pin,0);
  PIDT1.SetMode(MANUAL);
  analogWrite(H1pin,0);
  while (T2_PV > 30){
    LecturaTTs(); 
    Serial.print(T2_PV,1);
    Serial.println();
    Serial.write("Esperando a T2_PV <= 30ºC");
    Serial.println();
  }
  for (int contador = 0; contador < 1850; contador++) {
      LecturaTTs(); 
      if (contador <=50){
         analogWrite(H2pin,0);
         T2_OP = 0;
      }else if (contador >=50 && contador <350){     
          analogWrite(H2pin,63);
          T2_OP = 63; //25%
      }else if (contador >=350 && contador <650){     
          analogWrite(H2pin,127);
          T2_OP = 127; //50%
      }else if (contador >=650 && contador <950){     
          analogWrite(H2pin,190);
          T2_OP = 190; //75%
      }else if (contador >=950 && contador <1250){     
          analogWrite(H2pin,127);
          T2_OP = 127;
      }else if (contador >=1250 && contador <1550){     
          analogWrite(H2pin,63);
          T2_OP = 63;
      }else if (contador >=1550 && contador <1850){     
          analogWrite(H2pin,0);
          T2_OP = 0;
      }
      if (1){
        //Debug serial Arduino COM Plotter
        Serial.print("#");        //Char inicio
        Serial.print(T2_PV,1);    //
        Serial.write(" ");        //separador
        Serial.print(T2_PV,1);    //
        Serial.write(" ");        //separador
        Serial.print(T2_OP);      //
        Serial.println();
      }
      delay(1000);
  }
}
