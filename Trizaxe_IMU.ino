/*
   Trizaxe_IMU
   Реализация алгоритма БИНС на основе
   imu_MC_IS_12_GPS.m

   % Для вычисления размерных значений используются матрицы 3х4
   % данные акселерометра и гироскопа фильтруются ФНЧ 1-го порядка
   % Используется простой комплитментарный фильтр 1-го порядка
*/

#include <Wire.h>
#include "MPU.h"
#include "MPU9250.h"

// библиотека для работы с GPS устройством
#include <TroykaGPS.h>

// serial-порт к которому подключён GPS-модуль
#define GPS_SERIAL    Serial3

// создаём объект класса GPS и передаём в него объект Serial1 
GPS gps(GPS_SERIAL);

// задаём размер массива для времени, даты, широты и долготы
#define MAX_SIZE_MASS 16
// массив для хранения текущего времени
char strTime[MAX_SIZE_MASS];
// массив для хранения текущей даты
char strDate[MAX_SIZE_MASS];
// массив для хранения широты в градусах, минутах и секундах
char latitudeBase60[MAX_SIZE_MASS];
// массив для хранения долготы в градусах, минутах и секундах
char longitudeBase60[MAX_SIZE_MASS];

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;


//#define PI 3.14159265358979f
//const float PI = 3.14159265358979;

// Константы для IMU
const double Tper_LPF = 0.05; // сек Период времени фильтра низких частот, коэффициент K_LPF = (T/dt)/(1+T/dt)
const double Tper_CF = 2.0; // сек Период времени комплиментарного фильтра, коэффициент K_CF = (T/dt)/(1+T/dt)
const double da_max = 0.02; // ед g, дупустимое значение перегрузки, когда мы учитываем её в синтезе
const int n_to_mean_def = 200; // количество точек для начального осреднения
const int TimePeriod_IMU = 10000; // мкс, период вызова функции вычисления модуля GetPitchRoll

const int n_MNK = 10; // кол-во точек для определения производной методом МНК

// Матрица калибровки акселерометра, размерность 1g/2^15
// Эта матрица должна храниться в энергонезависимой памяти и загружаться при включении МК 
/* // Матрица с куба
const float A_Matr_Acel[3][4] = {{-4.6421858e-07,  -6.0852511e-05,   1.0520920e-08,   1.8578426e-02},
                                  {6.0558612e-05,    1.0690196e-07,   9.6723401e-07,  -1.0433303e-01},
                                 {-8.6043756e-07,  -9.5465967e-08,   6.0900436e-05,  -1.5037214e-02}};
*/
const double A_Matr_Acel[3][4] = {{-4.9578893e-07,  -6.0755360e-05,  -8.4210623e-07,  -3.9815803e-02},
                                  {7.7733384e-07,   1.7652318e-07,  -6.0090338e-05,  -9.6849819e-02},
                                 {-6.0814213e-05,   2.0752929e-06,  -9.8158582e-07,   1.4468826e-02}};                                 
// Матрица калибровки гироскопа, размерность 250/360/2^15
// Эта матрица должна храниться в энергонезависимой памяти и загружаться при включении МК           
/*   // Матрица с куба               
const float A_Matr_Gyro[3][4] = {{6.3973758e-26,    2.1192763e-05,  -1.9283581e-24,  -4.9253293e-04},
                                {-2.1192763e-05,  -3.6555749e-26,   7.7682672e-25,   1.0713353e-03},
                                 {7.7681788e-25,    1.9283573e-24,  -2.1192763e-05,  -3.3074209e-04}};
*/                                 
const double A_Matr_Gyro[3][4] = {{-6.9457310e-25,   2.1192763e-05,  -3.6217486e-23,  -1.2695645e-03},
                                {-2.9537883e-23,  -3.6263929e-23,   2.1192763e-05,   2.0351038e-02},
                                 {2.1192763e-05,  -6.8098741e-25,  -2.9207650e-23,   8.6487842e-04}};                                 
                            
// Инициализация глобальных переменнных IMU
boolean Mean_data_def = 1; // флаг указывающий на определение средних значений при включении МК
boolean InitData = 1; // Флаг указывающий на окончание получение начальных данных и их осреднения
boolean InitialSet = 0; // флаг указывающий на необходимость выполнения операции начального выставления блока IMU, определения его углов ориентации относительно объёкта

double A[3]; // единиц g,  вектор ориентации в системе координат платы после комплиментарного фильтра
double W[3]; // рад/сек, 3 компоненты размерной угловой скорости в системе координат платы после фильтра низких частот
double A_f[3]; // ед g, отфильтрованные после ФНЧ значения перегрузок в системе платы
double W_zero[3] = {0, 0, 0};

unsigned long t_mcs; //mcs текущее время в цикле GetPitchRoll
unsigned long Prev_Time_Period; // mcs время последнего принудительного вызова функции GetPitchRoll

double Pitch;  //рад, текущий дифферент Объекта
double Roll; //рад, текущее крен Объекта

double Pitch0 = 0.0;  //рад, Угол дифферента нулевого положения микросхемы IMU
double Roll0 = 0.0; //рад, Угол крена нулевого положения микросхемы IMU

double Pitch1 = 0.0; // рад Текущий Угол дифферента микросхемы IMU
double Roll1 =0.0; // рад Текущий Угол крена микросхемы IMU

// Инициализация глобальных переменнных работы с GPS
unsigned Azim10; // азимут, полученный с GPS (умножен на 10)
unsigned Veloc10; //knts скорость, полученная с GPS (умножен на 10)
unsigned long t_GPS_mcs; //mcs текущее время в цикле определения угловой скорости
float V_mean = 0.0; // м/с текущая отфильтрованная скорость
float W_Az = 0.0; // текущая угловая скорость

void setup() {
// serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // открываем Serial-соединение с GPS-модулем
  GPS_SERIAL.begin(115200);
  
  delay(1000);
  GPS_SERIAL.write("$PMTK220,100*2F\r\n$PMTK500,100,0,0,0,0*2A\r\n");
  delay(1000);
  GPS_SERIAL.write("$PMTK220,100*2F\r\n$PMTK500,100,0,0,0,0*2A\r\n");
  delay(1000);
  GPS_SERIAL.write("$PMTK220,100*2F\r\n$PMTK500,100,0,0,0,0*2A\r\n");
  
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  Serial.print("Status: ");
  Serial.println(status);

  i2cInit();
//  accelgyro.initialize();
  t_mcs = micros();
  Prev_Time_Period = t_mcs;
}

// Основной цикл
void loop() {

   // если пришли данные с GPS-модуля
  if (gps.available()) {
    // считываем данные и парсим
    gps.readParsing();
    // проверяем состояние GPS-модуля\
    
  }

  if (InitialSet) { // выполняется процедура начального выставления IMU
   Get_Pitch0_Roll0_of_IMU(0.0, 0.0); // входными данными являются углы ориентации объекта в этот момент
  } else // выполняется процедура определени углов ориентации
    if ((micros() - Prev_Time_Period) >= TimePeriod_IMU) {
      Prev_Time_Period = micros();
      GetPitchRoll();
    }
  
 //Test1(); 
}
//////////////////////////////////////////////////////////////////////
// Процедура вычисления углов дифферента и крена, фильтрации перегрузок
void GetPitchRoll() {
  int16_t Ar[3]; // Данные акселерометра, читаемые из модуля MPU
  int16_t Wr[3]; // Данные гироскопа, читаемые из модуля MPU
  int16_t Tempr; // Температура, читаемая их модуля MPU
  
  unsigned dt;
  static long A_mean[3] = {0, 0, 0}; // Средние значения вычисляемые при выставлении
  static long W_mean[3] = {0, 0, 0}; // Средние значения вычисляемые при выставлении
  static int n_init = 0; // количество шагов прошедших с инициализации
  long A_t[3];
  long W_t[3];
  double A_N; // Абсолютное значение перегрузки по данным акселерометра
  double A_N_P; // Абсолютное значение перегрузки по данным акселерометра
  double A_pred[3];
  unsigned long t0_mcs; //mcs предыдущее время в цикле GetPitchRoll
  double K_LPF; //коэффициент фильтра низких частот
  double K_CF; //коэффициент комплиментарного фильтра
  double A_Scall[3] = {0.0, 0.0, 0.0}; 
  double W_Scall[3] = {0.0, 0.0, 0.0}; 
  double A12;
  double A23;

  mpuRawData RawDataMPU; // создание объекта типа mpuRawData
  
  mpuGetRawData(&RawDataMPU); // Читаем данные из модля MPU6050
    Ar[0] = RawDataMPU.accX;
    Ar[1] = RawDataMPU.accY;
    Ar[2] = RawDataMPU.accZ;
    Wr[0] = RawDataMPU.gyroX;
    Wr[1] = RawDataMPU.gyroY;
    Wr[2] = RawDataMPU.gyroZ;
    Tempr = RawDataMPU.tempRaw;

  t0_mcs = t_mcs;
  t_mcs = micros();
  dt = (t_mcs - t0_mcs); // вычисление шага времени
  
  K_LPF = (Tper_LPF*1.0e6/dt)/(1+Tper_LPF*1.0e6/dt);  //Определяем значение коээфициента фильтра низких частот
  
  if (Mean_data_def) { // Если стоит флаг Mean_data_def то определяем начальный вектор ориентации и нули гироскопа путём осреднения
    if (n_init < n_to_mean_def) {
      for (int i = 0; i <= 2; i++) {
        A_mean[i] = A_mean[i] + Ar[i];
        W_mean[i] = W_mean[i] + Wr[i];
      } // for
      n_init = n_init + 1;
    } else {
      for (int i = 0; i <= 2; i++) {
        A_mean[i] = A_mean[i]/n_to_mean_def;
        W_mean[i] = W_mean[i]/n_to_mean_def;
      } // for
      Get_Scale_Gyro(W_mean, W_zero);
      Mean_data_def = 0; // заканчиваем выставление
      InitData = 1; // ставим флаг, что данные только что инициализированы
    } //
    
  } else {
    
    // Переводим данные в размерный вид
    if (InitData) { // если массивы с размерынми A и W только инициированы
      Get_Scale_Accel(A_mean, A_f); // вычисляем размерные значения акселерометра, когда вектора A и W только инициализированы
      Get_Scale_Gyro(W_mean, W); // вычисляем размерные значения гироскопа
      A_N = sqrt(A_f[0] * A_f[0] + A_f[1] * A_f[1] + A_f[2] * A_f[2]);     
      for (int i = 0; i <= 2; i++) {   
        A[i] = A_f[i]/A_N; // вектор ориентации определяется по начальному выставлению, по данным акселерометра
        W[i] = W[i] - W_zero[i];
      }   
      InitData = 0;
      Pitch1 = asin(-A[0]); //рад
      Roll1 = atan2(A[2], -A[1]); //рад
      //
    } else { // во всех остальных случаях фильтруем
      // делаем переприсвоение присвоение для согласования типов данных  
      for (int i = 0; i <= 2; i++) {
        A_t[i] = Ar[i];      
        W_t[i] = Wr[i]; ////////
      }
      Get_Scale_Accel(A_t, A_Scall);   
      Get_Scale_Gyro(W_t, W_Scall);      
      for (int i = 0; i <= 2; i++) { 
        A_f[i] = K_LPF * A_f[i] + (1-K_LPF) * A_Scall[i]; // Значение ускорения после ФНЧ
        W[i] = K_LPF * W[i] + (1-K_LPF) * (W_Scall[i]-W_zero[i]); // Значение угловых скоростей после ФНЧ    
      } // end of for 
    } // end of if InitData
    
    // Добавляем поправку на центробежное ускорение
    //A_f[1] = A_f[1] - W_Az/180*3.14*V_mean/9.81*sin(Roll1);
    //A_f[2] = A_f[2] - W_Az/180*3.14*V_mean/9.81*cos(Roll1); 
    
    A_N = sqrt(A_f[0] * A_f[0] + A_f[1] * A_f[1] + A_f[2] * A_f[2]); // модуль перегрузки
    if ((A_N >= 1.0 - da_max) && (A_N <= 1.0 + da_max)) {
//      K_CF = 1 - (Tper_CF*1.0e6/dt)/(1+Tper_CF*1.0e6/dt); // если перегрузка близка к 1, то учитываем включем её в комплиментарный филтр
      K_CF = 0.0;
    } else {
      K_CF = 0.0; // иначе просто интегрируем углы
    } // if
    
    // Предсказываем вектор ориентации по данным гироскопа и вектору ориентации, полученном на предыдущем шаге
    A_pred[0] = A[0] + (-1.0) * (W[1] * A[2] - W[2] * A[1]) * dt/1e6;
    A_pred[1] = A[1] - (-1.0) * (W[0] * A[2] - W[2] * A[0]) * dt/1e6;
    A_pred[2] = A[2] + (-1.0) * (W[0] * A[1] - W[1] * A[0]) * dt/1e6;
    //normalize3DVector(A_pred);
    
    A_N_P = sqrt(A_pred[0] * A_pred[0] + A_pred[1] * A_pred[1] + A_pred[2] * A_pred[2]);
    // Комплиментарный фильтр
    for (int i = 0; i <= 2; i++) {
      A[i] = (1-K_CF)*A_pred[i]/A_N_P + K_CF*A_f[i]/A_N;
    }
    
    A_N = sqrt(A[0] * A[0] + A[1] * A[1] + A[2] * A[2]); // модуль вектора ориентации
    // нормируем вектор ориентации
    for (int i = 0; i <= 2; i++) {
      A[i] = A[i]/A_N;
    }
    
    // Углы ориентации микросхемы IMU
    Pitch1 = asin(-A[0]); //рад
    Roll1 = atan2(A[2], -A[1]); //рад
    
    A12 = -cos(Pitch1)*cos(Roll0)*sin(Pitch0) + sin(Pitch1)*cos(Roll0)*cos(Pitch0);
    A23 = -cos(Roll1)*sin(Pitch1)*sin(Roll0)*sin(Pitch0) - cos(Roll1)*cos(Pitch1)*sin(Roll0)*cos(Pitch0) + sin(Roll1)*cos(Roll0);
    
    // Углы ориентации объекта
    Pitch = asin(A12); //рад
    Roll = asin(A23); //рад
    
  } // end of if (Calibrate)
  /*
    Serial.print(t_mcs);
    Serial.print(", ");
    Serial.print(Ar[0]);
    Serial.print(",");
    Serial.print(Ar[1]);
    Serial.print(",");
    Serial.print(Ar[2]);
    Serial.print(",");
    Serial.print(Wr[0]);
    Serial.print(",");
    Serial.print(Wr[1]);
    Serial.print(",");
    Serial.print(Wr[2]);
    Serial.print(",");
    Serial.print(Pitch1*180/PI);
    Serial.print(", ");
    Serial.print(Roll1*180/PI);
    Serial.println();  */  

    union Cnv
    {
      int16_t num;
      uint8_t dat[2];
    } cnv;

    union Cnv32
    {
      unsigned long num;
      uint8_t dat[4];
    } cnv32;

    const uint8_t transmit_len = 26;
    
    uint8_t data[transmit_len];

    cnv32.num = t_mcs;
    data[0] = 0xAB;
    data[1] = cnv32.dat[0];
    data[2] = cnv32.dat[1];
    data[3] = cnv32.dat[2];
    data[4] = cnv32.dat[3];
  
    cnv.num = Ar[0];
    data[5] = cnv.dat[0];
    data[6] = cnv.dat[1];

    cnv.num = Ar[1];
    data[7] = cnv.dat[0];
    data[8] = cnv.dat[1];

    cnv.num = Ar[2];
    data[9] = cnv.dat[0];
    data[10] = cnv.dat[1];

    cnv.num = Wr[0];
    data[11] = cnv.dat[0];
    data[12] = cnv.dat[1];

    cnv.num = Wr[1];
    data[13] = cnv.dat[0];
    data[14] = cnv.dat[1];

    cnv.num = Wr[2];
    data[15] = cnv.dat[0];
    data[16] = cnv.dat[1];

    int16_t p1 = (int16_t)(Pitch1*18000.0/PI);
    int16_t r1 = (int16_t)(Roll1*18000.0/PI);

    cnv.num = p1;
    data[17] = cnv.dat[0];
    data[18] = cnv.dat[1];

    cnv.num = r1;
    data[19] = cnv.dat[0];
    data[20] = cnv.dat[1];

    int i_speed = (int16_t)(gps.getSpeedKm()*100.0);
    int i_coarse = (int16_t)(gps.getCoarseDeg()*10.0);
    
    cnv.num = i_speed;
    data[21] = cnv.dat[0];
    data[22] = cnv.dat[1];

    cnv.num = i_coarse;
    data[23] = cnv.dat[0];
    data[24] = cnv.dat[1];

    data[transmit_len-1] = 0; 
    for (int i = 0; i < transmit_len-1; i++)
      data[transmit_len-1] ^= data[i];

    Serial.write(data, transmit_len); 
    
} // end GetPitchRoll()


/////////////////////////////////////////////////////////////
// Процедура начального выставления блока IMU. Определяются углы ориентации модуля IMU относительно объекта
// Процедура запускается из приложения телефона. Из приложения передаются углы ориентации объекта Pitch_IS и Roll_IS
// По умолчанию эти углы равны нулю
// Результатом действия процедуры являются значения Pitch0 и Roll0
void Get_Pitch0_Roll0_of_IMU(double Pitch_IS, double Roll_IS) {
  int16_t Ar[3]; // Данные акселерометра, читаемые из модуля
  int16_t Wr[3]; // Данные гироскопа, читаемые из модуля
  int16_t Tempr;

  int n_IS = 100; // количество точек для записи 
  long A_m[3]={0,0,0};
  double A_t[3]={0.0,0.0,0.0};
  double A012, A023;
  double A_N;
  
  mpuRawData RawDataMPU; // создание объекта типа mpuRawData

  // Считываем n_IS значений подряд
  for (int k=0; k<n_IS; k++) {
    mpuGetRawData(&RawDataMPU); // Читаем данные из модля MPU6050
    Ar[0] = RawDataMPU.accX;
    Ar[1] = RawDataMPU.accY;
    Ar[2] = RawDataMPU.accZ;
    //Wr[0] = RawDataMPU.gyroX;
    //Wr[1] = RawDataMPU.gyroY;
    //Wr[2] = RawDataMPU.gyroZ;
    //Tempr = RawDataMPU.tempRaw;
    for (int i = 0; i <= 2; i++) {
        A_m[i] = A_m[i] + Ar[i];
        //W_mean[i] = W_mean[i] + (long)Wr[i];
    } // for  
  }
  for (int i = 0; i <= 2; i++) {
        A_m[i] = A_m[i]/n_IS;
        //W_mean[i] = W_mean[i] + (long)Wr[i];
  } // for 
        
  Get_Scale_Accel(A_m, A_t);
  
  A_N = sqrt(A_t[0] * A_t[0] + A_t[1] * A_t[1] + A_t[2] * A_t[2]); // модуль вектора ориентации
    // нормируем вектор ориентации
    for (int i = 0; i <= 2; i++) {
      A_t[i] = A_t[i]/A_N;
    }

  Pitch1 = asin(-A_t[0]);
  Roll1 = atan2(A_t[2], -A_t[1]);

  A012 = cos(Pitch_IS)*sin(Pitch1) - cos(Roll_IS)*sin(Pitch_IS)*cos(Roll1)*cos(Pitch1) - sin(Roll_IS)*sin(Pitch_IS)*sin(Roll1)*cos(Pitch1); 
  A023 = cos(Roll_IS)*cos(Pitch_IS)*sin(Roll1) - sin(Roll_IS)*cos(Pitch_IS)*cos(Roll1);
    
  Pitch0 = asin(A012);
  Roll0 = asin(A023);            
} // end void Get_Pitch0_Roll0_of_IMU()

////////////////////////////////////////////////////////
void normalize3DVector(double * vector) {
  static double R;
  R = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
  vector[0] /= R;
  vector[1] /= R;
  vector[2] /= R;
}

////////////////////////////////////////////////////////
// Функция вычисления размерных значений ускорений. Данные выводятся в единицах g
void Get_Scale_Accel(long *A_vect, double *A_Sc) {
  double T[3] = {0.0, 0.0, 0.0};
  for (int i=0; i<=2; i++) {
    for (int j=0; j<=2; j++){
        T[i] = T[i] + A_Matr_Acel[i][j]*A_vect[j];   
    }
    T[i] = T[i] + A_Matr_Acel[i][3];
    A_Sc[i] = T[i];
  }
} // end of void Get_Scale_AccelS

/////////////////////////////////////////////////////
// Функция вычисления размерных значений угловой скорости. Данные выводятся в радианах
void Get_Scale_Gyro(long *W_vect, double *W_Sc) {
  double T[3] = {0.0, 0.0, 0.0};
  for (int i=0; i<=2; i++) {
    for (int j=0; j<=2; j++){
        T[i] = T[i] + A_Matr_Gyro[i][j]*W_vect[j];
    }
    T[i] = T[i] + A_Matr_Gyro[i][3];
    W_Sc[i] = T[i]*2*PI;
  }  
} // end of void Get_Scale_Gyro

////////////////////////////////////////////////////
// Функция вызывается при получении новых данных от GPS
void Omega_Y() {
  static unsigned Az_MNK[n_MNK]; //(Глобальная) массив с данными угла азимута для нахождения производной МНК длиной n_MNK
  static unsigned V_MNK[n_MNK]; //(Глобальная) массив с данными скорости для осреднения длиной n_MNK 
  static unsigned t_MNK[n_MNK]; //(Глобальная) массив с данными времени
  static int i_n_MNK = 0;
  
  static int First_Azim = 1;
  static unsigned Az_pred;
  static long delta360;

  float xt_yt;
  float xt;
  float yt;
  float xt2;

  t_GPS_mcs = micros();
  
  if (First_Azim <= n_MNK) { // Пропускаем первые N значений
        for (int j=0; j<n_MNK; j++){
            Az_MNK[j] = Azim10;
            V_MNK[j] = Veloc10;
            t_MNK[j] = t_GPS_mcs;
        }
        First_Azim = 11;
        Az_pred = Azim10;
  } else {       
        if (Azim10-Az_pred>3000){
            delta360 = delta360-3600;
        } else if (Az_pred-Azim10>3000){
            delta360 = delta360+3600;
        }
        Az_pred = Azim10;            
        Az_MNK[i_n_MNK] = Azim10+delta360;
        V_MNK[i_n_MNK] = Veloc10;
        t_MNK[i_n_MNK] = t_GPS_mcs;
        
        if (i_n_MNK == n_MNK-1){
            i_n_MNK = 0;
        } else {
            i_n_MNK = i_n_MNK +1;
        }
    
        xt_yt = 0;
        xt = 0;
        yt = 0;
        xt2 = 0;
        V_mean = 0;
        for (int j=0; j<n_MNK; j++){
            V_mean = V_mean+V_MNK[j]/10.0/1.852/n_MNK; // средняя скорость за период осреднения n_MNK
            xt_yt = xt_yt +t_MNK[j]*Az_MNK[j]/10;
            xt = xt + t_MNK[j];
            yt = yt + Az_MNK[j]/10;
            xt2 = xt2 + t_MNK[j]*t_MNK[j];
        }

        if (V_mean<1){ // если скорость меньше 1м/с, то считаем, что угловая скорость равна нулю
            W_Az = 0.0;
        } else {
            W_Az = (n_MNK*xt_yt-xt*yt)/(n_MNK*xt2 - xt*xt)/1e6; // угловая скорость град/мсек
        }
  }
  
}
