//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// ARDUINO BATTERY CAPACITY TESTER
//Version-1.0
//by deba168,INDIA
//Dated : 04/09/2016
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

#include <Arduino.h>
//#include "U8glib.h"
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//#define MOSFET_Pin 2
#define Bat_Pin_1 A0
#define Res_Pin_1 A1
#define Bat_Pin_2 A2
#define Res_Pin_2 A3
#define Bat_Pin_3 A6
#define Res_Pin_3 A7
#define Buzzer_Pin 9
#define PWMOFF 0
#define PWMMAX 255
//U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE); // I2C / TWI
//float Capacity = 0.0;                       // Capacity in mAh
float Res_Value = 1.0; // Resistor Value in Ohm
float Vcc = 4.64;      // Voltage of Arduino 5V pin ( Mesured by Multimeter )
//float Current = 0.0;                        // Current in Amp
//float mA = 0;                               // Current in mA
//float Bat_Volt = 0.0;                       // Battery Voltage
//float Res_Volt = 0.0;                       // Voltage at higher end of the Resistor
float Bat_High = 4.3;             // Battery High Voltage
float Bat_Low = 2.5;              // Discharge Cut Off Voltage
unsigned long previousMillis = 0; // Previous time in ms
unsigned long millisPassed = 0;   // Current time in ms
float sample1 = 0;
//float sample2 = 0;
int x = 0;
int row = 0;
//const int Current [] = {0,110,210,300,390,490,580,680,770,870,960,1000};
//const byte PWM_Pin = 10;
//int PWM_Value = 0;
//float CRVAL = 1;

//**********************CELL CLASS ********************//
class Cell
{
private:
  uint8_t BatteryPin;
  uint8_t ResistorPin;
  float Capacity;
  float Current;
  float mA;
  uint8_t PWMPin;
  float CurrentLimit;
  float BattVolt;
  float ResVolt;
  uint8_t CellTagNumber;

public:
  uint8_t PWMvalue;
  static uint8_t NUMBER_OF_CELL;
  Cell(uint8_t Bat_pin, uint8_t Res_pin, uint8_t PWM_pin, float Curr_limit, uint8_t CellNumber)
  {
    PWMvalue = 0;
    Capacity = 0;
    Current = 0;
    mA = 0;
    BatteryPin = Bat_pin;
    ResistorPin = Res_pin;
    PWMPin = PWM_pin;
    CurrentLimit = Curr_limit;
    BattVolt = 0;
    ResVolt = 0;
    CellTagNumber = CellNumber;
  }
  ~Cell() {}

  uint8_t getBattPin()
  {
    return BatteryPin;
  }
  uint8_t getResPin()
  {
    return ResistorPin;
  }
  uint8_t getPWMPin()
  {
    return PWMPin;
  }
  float getCurrent()
  {
    return Current;
  }
  float getCapacity()
  {
    return Capacity;
  }
  float getmA()
  {
    return mA;
  }
  uint8_t getPWMpercent()
  {
    return (this->PWMvalue * 100 / 255);
  }
  void setBattVolt(float volt)
  {
    BattVolt = volt;
  }
  void setResVolt(float volt)
  {
    ResVolt = volt;
  }
  float getBattVolt()
  {
    return BattVolt;
  }
  float getResVolt()
  {
    return ResVolt;
  }
  void MeasureBattVolt()
  {
    sample1 = 0;
    for (int i = 0; i < 100; i++)
    {
      sample1 = sample1 + analogRead(BatteryPin); //read the voltage from the divider circuit
      delay(2);
    }
    sample1 = sample1 / 100;
    BattVolt = sample1 * 2 * Vcc / 1024.0;
  }
  void MeasureResVolt()
  {
    sample1 = 0;
    for (int i = 0; i < 100; i++)
    {
      sample1 = sample1 + analogRead(ResistorPin); //read the voltage from the divider circuit
      delay(2);
    }
    sample1 = sample1 / 100;
    ResVolt = sample1 * 2 * Vcc / 1024.0;
  }
  uint8_t getCellTagNumber()
  {
    return CellTagNumber;
  }
  void updateCapacity()
  {
    Current = ResVolt / Res_Value;
    mA = Current * 1000.0;
    Capacity = Capacity + mA * (millisPassed / 3600000.0); // 1 Hour = 3600000ms
  }
  void updatePWM()
  {
    if (Current <= CurrentLimit && PWMvalue < PWMMAX)
    {
      if (Current + 0.2 < CurrentLimit)
      {
        PWMvalue = PWMvalue + 10;
      }
      else
      {
        PWMvalue = PWMvalue + 1;
      }
    }
    if (Current > CurrentLimit)
    {
      if (Current > CurrentLimit + 0.2)
      {
        PWMvalue -= 5;
      }
      else
      {
        PWMvalue -= 1;
      }
    }
    analogWrite(PWMPin, PWMvalue);
  }
  void printinfoSerial()
  {
    Serial.print("BATT_VOLT(V):");
    Serial.println(BattVolt);
    Serial.print("RES_VOLT(V):");
    Serial.println(ResVolt);
    Serial.print("CAPACITY(mAh):");
    Serial.println(Capacity);
    Serial.print("Current(A):");
    Serial.println(Current);
    Serial.print("PWM(%):");
    Serial.println(PWMvalue * 100 / 256);
    Serial.println("");
  }
  void setPWMZeroCuttOff()
  {
    if (BattVolt < Bat_Low)
    {
      PWMvalue = 0;
      analogWrite(PWMPin, PWMvalue);
    }
  }
  void handler()
  {
    if (BattVolt > Bat_High && BattVolt < Bat_Low)
    {
      updateCapacity();
      updatePWM();
    }
  }
  void drawhandle()
  {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(25, 40);
    display.println("CELL:"+String(CellTagNumber));
    if (BattVolt < 1)
    {
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(10, 40); // set position
      display.println("No Battery!");
    }
    else if (BattVolt > Bat_High)
    {
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(25, 40); // set position
      display.println("High-V!");
    }
    else if (BattVolt < Bat_Low)
    {
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(25, 40); // set position
      display.println("Low-V!");
    }
    else if (BattVolt >= Bat_Low && BattVolt < Bat_High)
    {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 20); // set position
      display.println("Volt: " + String(BattVolt) + "V");
      display.setCursor(0,40);
      display.println("Curr: " + String(Current)+ "mA" );
      display.setCursor(0,40);
      display.println("mAh: " + String(Capacity));
      // u8g.drawStr(0, 20, "Volt: "); // put string of display at position X, Y
      // u8g.drawStr(0, 40, "Curr: ");
      // u8g.drawStr(0, 60, "mAh: ");
      // u8g.setPrintPos(58, 20); // set position
      // u8g.print(BattVolt, 2);  // display Battery Voltage in Volt
      // u8g.println("V");
      // u8g.setPrintPos(58, 40); // set position
      // u8g.print(mA, 0);        // display current in mA
      // u8g.println("mA");
      // u8g.setPrintPos(58, 60); // set position
      // u8g.print(Capacity, 1);  // display capacity in mAh
    }
  }
};

//************************ OLED Display Draw Function *******************************************************
// void draw(void)
// {
// u8g.setFont(u8g_font_fub14r); // select font
// if (Bat_Volt < 1)
// {
//   u8g.setPrintPos(10, 40); // set position
//   u8g.println("No Battery!");
// }
// else if (Bat_Volt > Bat_High)
// {
//   u8g.setPrintPos(25, 40); // set position
//   u8g.println("High-V!");
// }
// else if (Bat_Volt < Bat_Low)
// {
//   u8g.setPrintPos(25, 40); // set position
//   u8g.println("Low-V!");
// }
// else if (Bat_Volt >= Bat_Low && Bat_Volt < Bat_High)
// {

//   u8g.drawStr(0, 20, "Volt: "); // put string of display at position X, Y
//   u8g.drawStr(0, 40, "Curr: ");
//   u8g.drawStr(0, 60, "mAh: ");
//   u8g.setPrintPos(58, 20); // set position
//   u8g.print(Bat_Volt, 2);  // display Battery Voltage in Volt
//   u8g.println("V");
//   u8g.setPrintPos(58, 40); // set position
//   u8g.print(mA, 0);        // display current in mA
//   u8g.println("mA");
//   u8g.setPrintPos(58, 60); // set position
//   u8g.print(Capacity, 1);  // display capacity in mAh
// }

// }
//******************************Buzzer Beep Function *********************************************************
void beep(unsigned char delay_time)
{
  analogWrite(9, 20);         // PWM signal to generate beep tone
  delay(delay_time);          // wait for a delayms ms
  analogWrite(Buzzer_Pin, 0); // 0 turns it off
  delay(delay_time);          // wait for a delayms ms
}

//*******************************Setup Function ***************************************************************
uint8_t Cell::NUMBER_OF_CELL = 0;
Cell CELL_1(A0, A1, 10, 1, 1);
Cell CELL_2(A2, A3, 9, 1, 2);
Cell CELL_3(A6, A7, 11, 1, 3);
void setup()
{
  Serial.begin(9600);
  //pinMode(MOSFET_Pin, OUTPUT);
  //pinMode(Buzzer_Pin, OUTPUT);
  //digitalWrite(MOSFET_Pin, LOW); // MOSFET is off during the start
  Serial.println("CLEARDATA");
  Serial.println("LABEL,Time,Bat_Volt,capacity");
  pinMode(CELL_1.getPWMPin(), OUTPUT);
  pinMode(CELL_2.getPWMPin(), OUTPUT);
  pinMode(CELL_3.getPWMPin(), OUTPUT);
  digitalWrite(CELL_1.getPWMPin(),LOW);
  digitalWrite(CELL_2.getPWMPin(),LOW);
  digitalWrite(CELL_3.getPWMPin(),LOW);
  //Serial.println("Arduino Battery Capacity Tester v1.0");
  //Serial.println("BattVolt Current mAh");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(12, 25);
  display.print("BATT_CAP_TESTER");
  display.display();
}
//********************************Main Loop Function***********************************************************
void loop()
{
  // Vcc = readVcc()/1000.0; // Conevrrt mV to Volt

  // Voltage devider Out = Bat_Volt * R2/(R1+R2 ) // R1 =10K and R2 =10K

  //************ Measuring Battery Voltage ***********

  // for (int i = 0; i < 100; i++)
  // {
  //   sample1 = sample1 + analogRead(CELL_1.getBattPin()); //read the voltage from the divider circuit
  //   delay(2);
  // }
  // sample1 = sample1 / 100;
  // CELL_1.setBattVolt(sample1 * 2 * Vcc / 1024.0);
  CELL_1.MeasureBattVolt();
  CELL_2.MeasureBattVolt();
  CELL_3.MeasureBattVolt();

  // *********  Measuring Resistor Voltage ***********

  // for (int i = 0; i < 100; i++)
  // {
  //   sample2 = sample2 + analogRead(Res_Pin_1); //read the voltage from the divider circuit
  //   delay(2);
  // }
  // sample2 = sample2 / 100;
  // Res_Volt = sample2 * 2 * Vcc / 1024.0;
  CELL_1.MeasureResVolt();
  CELL_2.MeasureResVolt();
  CELL_3.MeasureResVolt();
  //********************* Checking the different conditions *************

  if (CELL_1.getBattVolt() > Bat_High || CELL_2.getBattVolt() > Bat_High || CELL_2.getBattVolt() > Bat_High)
  {
    if (CELL_1.getBattVolt() > Bat_High)
    {
      analogWrite(CELL_1.getPWMPin(), PWMOFF); // Turned Off the MOSFET // No discharge
    }
    else if (CELL_2.getBattVolt() > Bat_High)
    {
      analogWrite(CELL_2.getPWMPin(), PWMOFF); // Turned Off the MOSFET // No discharge
    }
    else if (CELL_3.getBattVolt() > Bat_High)
    {
      analogWrite(CELL_3.getPWMPin(), PWMOFF); // Turned Off the MOSFET // No discharge
    }
    Serial.println("Warning High-V! ");
    delay(1000);
  }
  else if (CELL_1.getBattVolt() < Bat_Low || CELL_2.getBattVolt() < Bat_Low || CELL_2.getBattVolt() < Bat_Low)
  {
    if (CELL_1.getBattVolt() < Bat_Low)
    {
      analogWrite(CELL_1.getPWMPin(), PWMOFF); // Turned Off the MOSFET // No discharge
    }
    else if (CELL_2.getBattVolt() < Bat_Low)
    {
      analogWrite(CELL_2.getPWMPin(), PWMOFF); // Turned Off the MOSFET // No discharge
    }
    else if (CELL_3.getBattVolt() < Bat_Low)
    {
      analogWrite(CELL_3.getPWMPin(), PWMOFF); // Turned Off the MOSFET // No discharge
    }
    Serial.println("Warning Low-V! ");
    delay(1000);
  }
  else if ((CELL_1.getBattVolt() > Bat_Low && CELL_1.getBattVolt() < Bat_High) || (CELL_3.getBattVolt() > Bat_Low && CELL_3.getBattVolt() < Bat_High) || (CELL_2.getBattVolt() > Bat_Low && CELL_2.getBattVolt() < Bat_High))
  { // Check if the battery voltage is within the safe limit
    //digitalWrite(MOSFET_Pin, HIGH);
    millisPassed = millis() - previousMillis;
    // mA = Current * 1000.0;
    // Capacity = Capacity + mA * (millisPassed / 3600000.0); // 1 Hour = 3600000ms
    CELL_1.handler();
    CELL_2.handler();
    CELL_3.handler();
    previousMillis = millis();
    CELL_1.printinfoSerial();
    CELL_2.printinfoSerial();
    CELL_3.printinfoSerial();
    row++;
    x++;
    delay(500);
  }
  CELL_1.setPWMZeroCuttOff();
  CELL_2.setPWMZeroCuttOff();
  CELL_3.setPWMZeroCuttOff();
  //*************************************************

  //**************************************************
  // u8g.firstPage();
  // do
  // {
  //   CELL_1.drawhandle();
  // } while (u8g.nextPage());
  CELL_1.drawhandle();
  CELL_2.drawhandle();
  CELL_3.drawhandle();
  //*************************************************
}
