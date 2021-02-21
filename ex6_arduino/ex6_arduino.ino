
#include<Wire.h>
#define MAX_SAMPLES 30

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY;
float C = 261.63;
float G = 392.00;
int BuzzerPin = 2;
int samples[MAX_SAMPLES];
static int samples_counter = 0;
int start = 0;
int full = 0;
boolean first = false;
int running_avg_hpf = 0;
int running_avg_lpf = 0;
float lpf[51] = {0.000450512197862357, 0.00153912705777971, 0.00348497385302190, 0.00599355718918527, 0.00815343557605636, 0.00859295274071079, 0.00606548279898031, 0.000284675521385808, -0.00742297904630510, -0.0141538572037984, -0.0164274088286249, -0.0119240768534547, -0.00119316487527943,  0.0116848156034187, 0.0202132060951870, 0.0182984233594517, 0.00371824537758757, -0.0194511274008932, -0.0409901694894725,  -0.0477953197077678, -0.0292744267612076, 0.0173348835091749, 0.0842451800303790, 0.154606017278228, 0.208004389555266, 0.227913624795602, 0.208004389555266, 0.154606017278228, 0.0842451800303790, 0.0173348835091749, -0.0292744267612076, -0.0477953197077678, -0.0409901694894725, -0.0194511274008932, 0.00371824537758757, 0.0182984233594517, 0.0202132060951870, 0.0116848156034187, -0.00119316487527943, -0.0119240768534547, -0.0164274088286249, -0.0141538572037984, -0.00742297904630510, 0.000284675521385808, 0.00606548279898031, 0.00859295274071079, 0.00815343557605636, 0.00599355718918527, 0.00348497385302190, 0.00153912705777971, 0.000450512197862357};
float hpf[57] = {3.77692515066913e-05, -0.000285182221958107, 0.000684268575785428, -0.00139919734409437, 0.00245115623490109, -0.00383068450330604, 0.00542807613014502, -0.00702360721503492, 0.00829053905076281, -0.00883001551793346, 0.00823709097742149, -0.00619258196136876,  0.00256664214039379, 0.00248681546977868, -0.00846836583321612, 0.0145265955830660, -0.0195202289325947, 0.0221521747500979, -0.0211616671806140, 0.0155477711209896, -0.00478551567449822, -0.0110069395845336, 0.0309886327461735, -0.0536282871914877, 0.0768548547872441, -0.0983100400075222, 0.115666261773318, -0.126958949991253, 0.130876674420814, -0.126958949991253, 0.115666261773318, -0.0983100400075222, 0.0768548547872441, -0.0536282871914877, 0.0309886327461735, -0.0110069395845336, -0.00478551567449822, 0.0155477711209896, -0.0211616671806140, 0.0221521747500979, -0.0195202289325947, 0.0145265955830660, -0.00846836583321612, 0.00248681546977868, 0.00256664214039379, -0.00619258196136876, 0.00823709097742149, -0.00883001551793346, 0.00829053905076281, -0.00702360721503492, 0.00542807613014502, -0.00383068450330604, 0.00245115623490109, -0.00139919734409437, 0.000684268575785428, -0.000285182221958107, 3.77692515066913e-05};

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);  
  Serial.begin(9600);
}

void fill_samples(int new_sample)
{
  samples[samples_counter] = new_sample;
  samples_counter++;
}

void update_samples(int new_sample){
  if(full)
  {
    samples[start] = new_sample;
    start = (++start) % MAX_SAMPLES;
  }
  else
  {
    samples[samples_counter] = new_sample;
    samples_counter++;
    if(samples_counter == MAX_SAMPLES);
    {
      full = 1;
    }
  }
}

void convolve_hpf_and_avg(){
  running_avg_hpf = 0;
  for (int i = MAX_SAMPLES - 1; i >= 0; i-- )
  {
      for (int j = 0; j < samples_counter; j++ )
    {
      running_avg_hpf += abs(samples[(start + j) % MAX_SAMPLES] * hpf[i - j]);
    } 
  }
  if (running_avg_hpf < 0){
    running_avg_hpf = abs(running_avg_hpf);
  }
  running_avg_hpf = running_avg_hpf / samples_counter;
}

void convolve_lpf_and_avg(){
  running_avg_lpf = 0;
  for (int i = MAX_SAMPLES - 1; i >= 0; i-- )
  {
      for (int j = 0; j < samples_counter; j++ )
    {
      running_avg_lpf += abs(samples[(start + j) % MAX_SAMPLES] * lpf[i - j]);
    } 
  }
  if (running_avg_lpf < 0){
    running_avg_lpf = abs(running_avg_lpf);
  }
  running_avg_lpf = running_avg_lpf / samples_counter;
}

void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  update_samples(AcY);
  convolve_lpf_and_avg();
  convolve_hpf_and_avg();
  boolean gesture_flag = false;
  if ((running_avg_lpf > 7000) && (!gesture_flag)){
    gesture_flag = true;
    if (running_avg_hpf > 4000) {
      first = true;
      Serial.println(15);
      delay(1000);
    } 
  } else {
    Serial.println(0);
  }
  float avg_20 = 0;
  for (int i = 1; i < 20; i++) {
    avg_20 += samples[(start - i) % MAX_SAMPLES];
  }
  if ((avg_20 / 20) < 3000){
    gesture_flag = false;
    first = false;
  }
  delay(100);
}
