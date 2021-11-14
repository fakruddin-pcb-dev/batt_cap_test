//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// ARDUINO BATTERY CAPACITY TESTER
//Version-3.1
//Auth:Fakruddin
//Dated : 14/11/2021
//All PINS are now configured
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
#define Bat_Pin_1 A7
#define Res_Pin_1 A6
#define PWM_Pin_1 10
#define Bat_Pin_2 A0
#define Res_Pin_2 A1
#define PWM_Pin_2 6
#define Bat_Pin_3 A2
#define Res_Pin_3 A3
#define PWM_Pin_3 3
#define PWMOFF 0
#define PWMMAX 255
float Res_Value = 1.0; // Resistor Value in Ohm
float Vcc = 4.64;      // Voltage of Arduino 5V pin ( Mesured by Multimeter )
float Bat_High = 4.3;  // Battery High Voltage
float Bat_Low = 2.5;   // Discharge Cut Off Voltage
float sample1 = 0;
int x = 0;
int row = 0;
unsigned long previousMillisprint = 0; // Previous time in ms
unsigned long millisPassedprint = 0;   // Current time in ms
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
  unsigned long previousMillis; // Previous time in ms
  unsigned long millisPassed;   // Current time in ms
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
    unsigned long previousMillis = 0; // Previous time in ms
    unsigned long millisPassed = 0;   // Current time in ms
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
    return (PWMvalue * 100 / 255);
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
  void setPWMZeroCuttOff()
  {
    if (BattVolt < Bat_Low && BattVolt > 1)
    {
      PWMvalue = PWMOFF;
      analogWrite(PWMPin, PWMvalue);
    }
  }
  void resetvalues()
  {
    PWMvalue = 0;
    Capacity = 0;
    Current = 0;
    mA = 0;
    BattVolt = 0;
    ResVolt = 0;
  }
  void handler()
  {
    if (BattVolt > Bat_High)
    {
      analogWrite(PWMPin, PWMOFF); // Turned Off the MOSFET // No discharge
      resetvalues();
    }
    else if (BattVolt < 1)
    {
      analogWrite(PWMPin, PWMOFF); // Turned Off the MOSFET // No discharge
      resetvalues();
    }
    else if (BattVolt > Bat_Low && BattVolt < Bat_High)
    {
      millisPassed = millis() - previousMillis;
      Current = ResVolt / Res_Value;
      mA = Current * 1000.0;
      Capacity = Capacity + mA * (millisPassed / 3600000.0); // 1 Hour = 3600000ms
      previousMillis = millis();
      row++;
      x++;
      //testcode
      if (Current <= CurrentLimit && PWMvalue < 255)
      {
        if (Current + 0.2 < CurrentLimit)
        {
          PWMvalue += 10;
        }
        else
        {
          PWMvalue += 1;
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
      //   delay(500);
    }
  }
  void drawhandle()
  {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(25, 40);
    display.println("CELL:" + String(CellTagNumber));
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
      display.setCursor(0, 40);
      display.println("Curr: " + String(Current) + "mA");
      display.setCursor(0, 40);
      display.println("mAh: " + String(Capacity));
    }
  }
  void printinfo()
  {
    if (BattVolt > Bat_High)
    {
      Serial.print("Warning High-V! Cell");
      Serial.println(getCellTagNumber());
    }
    else if (BattVolt > 1)
    {
      if (BattVolt < Bat_Low && BattVolt > 1)
      {
        Serial.print("Cell :");
        Serial.print(getCellTagNumber());
        Serial.print(" ");
        Serial.println("Cuttoff");
      }
      Serial.print("Cell :");
      Serial.println(getCellTagNumber());
      Serial.print("BATT_VOLT(V):");
      Serial.println(BattVolt);
      Serial.print("RES_VOLT(V):");
      Serial.println(ResVolt);
      Serial.print("CAPACITY(mAh):");
      Serial.println(Capacity);
      Serial.print("Current(A):");
      Serial.println(Current);
      Serial.print("PWM:");
      Serial.print("Value(%):");
      Serial.println(PWMvalue * 100 / 256);
      Serial.println("");
    }
    else if (BattVolt < 1)
    {
      Serial.print("No Cell! Slot");
      Serial.println(getCellTagNumber());
      Serial.print("Reseting values Slot");
      Serial.println(getCellTagNumber());
    }
  }
};

//******************************Buzzer Beep Function *********************************************************
// void beep(unsigned char delay_time)
// {
//   analogWrite(9, 20);         // PWM signal to generate beep tone
//   delay(delay_time);          // wait for a delayms ms
//   analogWrite(Buzzer_Pin, 0); // 0 turns it off
//   delay(delay_time);          // wait for a delayms ms
// }

//*******************************Setup Function ***************************************************************
uint8_t Cell::NUMBER_OF_CELL = 0;
Cell CELL_1(Bat_Pin_1, Res_Pin_1, PWM_Pin_1, 1, 1);
Cell CELL_2(Bat_Pin_2, Res_Pin_2, PWM_Pin_2, 1, 2);
Cell CELL_3(Bat_Pin_3, Res_Pin_3, PWM_Pin_3, 1, 3);
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
  CELL_1.MeasureBattVolt();
  CELL_2.MeasureBattVolt();
  CELL_3.MeasureBattVolt();

  // *********  Measuring Resistor Voltage ***********
  CELL_1.MeasureResVolt();
  CELL_2.MeasureResVolt();
  CELL_3.MeasureResVolt();
  //********************* Checking the different conditions *************
  CELL_1.handler();
  CELL_2.handler();
  CELL_3.handler();
  CELL_1.setPWMZeroCuttOff();
  CELL_2.setPWMZeroCuttOff();
  CELL_3.setPWMZeroCuttOff();
  millisPassedprint = millis();
  if (millisPassedprint - previousMillisprint > 2000)
  {
    previousMillisprint = millis();
    CELL_1.printinfo();
    CELL_2.printinfo();
    CELL_3.printinfo();
  }

  //*************************************************

  //**************************************************
  //CELL_1.drawhandle();
  //CELL_2.drawhandle();
  //CELL_3.drawhandle();
  //*************************************************
}
