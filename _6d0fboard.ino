#include <Wire.h>
#include <ITG3200.h>
#include <math.h>
#include <Servo.h>

#define spiclk  52    // connect to ADXL CLK
#define spimiso 50  // connect to ADXL DO
#define spimosi 51  // connect to ADXL DI
#define spics   53    // connect to ADXL CS

#define ACC_SENS_2G   0.00390625//
#define S_RATE_DIV 0x63 //100Hz sampling rate

#define SERVO_PIN 9
#define M_PI 3.14159265359
#define P_FILTER_COEFF .98

Servo servo;

ITG3200 gyro = ITG3200();
float  gyr_x,gyr_y,gyr_z;
float  pitch     = 0;
float  gyr_pitch = 0;
float  acc_pitch = 0;
float  delta     = 0;
long   timer     = 0;

//ACCELEROMETER
float  acc_x,acc_y,acc_z; //in g
byte   raw_acc[8];  // raw accelerometer data storage
byte   spiread; //SPI buffer

void setup(void) {
  
  Serial.begin(9600);
  Wire.begin();   
  //servo.attach(SERVO_PIN);
  delay(100);  
  init_adxl();
  gyro.init(ITG3200_ADDR_AD0_LOW); 
  gyro.setSampleRateDiv(S_RATE_DIV);
  gyro.setFilterBW(BW005_SR1);
  gyro.zeroCalibrate(2500, 2);
  
}

void loop(void) { 
    if (gyro.isRawDataReady()) {
      delta = ((float)micros() - timer)/10000000.000f; //get delta
      timer = micros();
      //work still needs to be done to ensure that Gyroscope data and Accelerometer data are in agreement
      //with regards to time
        
      read_xyz(); //reads accelerometer; output in Gs   
      gyro.readGyro(&gyr_x,&gyr_y,&gyr_z);  // Reads calibrated values in deg/sec       
      
      gyr_pitch+= gyr_x*delta;   //integration
      acc_pitch = 4068*(atan(acc_y/(sqrt((acc_x*acc_x)+(acc_z*acc_z)))))/71;
      pitch = P_FILTER_COEFF*gyr_pitch + (1.0f-P_FILTER_COEFF)*acc_pitch; //Complementary filter
      //GYRO PITCH, ACCELEROMETER PITCH, COMP FILTER OUTPUT
      Serial.print(gyr_pitch);
      Serial.print(",");    
      Serial.print(acc_pitch);
      Serial.print(",");   
      Serial.println(pitch);  
  }  
}

/*SPI Functions*/
/**
 * Code c/o E-Gizmo
**/
void spi_out(byte spidat){
  byte bitnum=8;
 
    spiread=0;
    // start spi bit bang
    while(bitnum>0){
       
      pinMode(spiclk,OUTPUT);    // SPI CLK =0
      if((spidat & 0x80)!=0)
        pinMode(spimosi,INPUT);  // MOSI = 1 if MSB =1
        else
        pinMode(spimosi,OUTPUT);  // else MOSI = 0
      
      spidat=spidat<<1; 
      pinMode(spiclk,INPUT);  // SPI CLK = 1
      
      // read spi data
      spiread=spiread<<1;
      
      if(digitalRead(spimiso)==HIGH) spiread |= 0x01; // shift in a 1 if MISO is 1
 
      pinMode(spimosi,INPUT);  // reset MOSI to 1
      bitnum--; 
    }
}
/*  Initialize ADXL345 */
 
void  init_adxl(void){
  delay(250);
  pinMode(spics,OUTPUT);  // CS=0  
  
  //Write to register 0x31, DATA FORMAT
  spi_out(0x31);
  // uncomment your desired range
 // spi_out(0x0B); //full resolution, +/- 16g range
  //spi_out(0x0A); //full resolution, +/- 8g range
  //spi_out(0x09); //full resolution, +/- 4g range
   spi_out(0x08); //full resolution, +/- 2g range
  
  pinMode(spics,INPUT);  //CS HIGH
  
  delay(1);
    pinMode(spics,OUTPUT);  // CS=0   
  
  // Write to register 0x2d, POWER_CTL
  spi_out(0x2d);
  //set to measure mode
  spi_out(0x08);
  pinMode(spics,INPUT);  //CS HIGH
  
  delay(1);
}
void read_xyz(void){
  int i;
    pinMode(spics,OUTPUT);  // CS=0   
    
  //Set start address to 0x32
  //D7= 1 for read and D6=1 for sequential read
  spi_out(0xF2);
  // dump xyz content to array
  for(i=0;i<6;i++){
    spi_out(0x00);
    raw_acc[i]=spiread;
  }  
  // merge to convert to 16 bits
  acc_x=((int)(raw_acc[1]<<8)|(int)raw_acc[0])*ACC_SENS_2G;
  acc_y=((int)(raw_acc[3]<<8)|(int)raw_acc[2])*ACC_SENS_2G;
  acc_z=((int)(raw_acc[5]<<8)|(int)raw_acc[4])*ACC_SENS_2G;
  
  pinMode(spics,INPUT);  //CS HIGH
}
