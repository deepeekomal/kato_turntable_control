   
/*
  Indexed turntable altered for Arduino Mega
  based on Clement Chan <ahyanchan@gmail.com>
  revised by Dheerendra Prasad <ntrains.org>
  Used on
    Kato electric Turntable 20-283

  History:
    21/12/2014 - First Release
    04/08/2018 - Second Release modified to add 16 button keypad for more reliability and FRAM for position memory.
    11/08/2018 - Current release modified  to count docks(tracks) from 1 -37 to match numbering on ESU ECoS
    07/16/2020 - This release was made to update the decoder interpreter interface, silencing serialdebug messages
    and using that channel to transmit current table position.
    12/29/2020 - Updated the command interpreter to read 1-36 incoming from decoder to desired track position, 99 as
                  180degree turn and 97 ccw and 98 cw single steps.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include <SevenSegment.h>;
#include <Keypad.h>;
#include <Adafruit_FRAM_I2C.h>
#include <Wire.h>;

// FRAM setup
Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();
uint16_t          framAddr = 0;
/*****Uncomment below to dump all of the memory to screen  SLOW!!!!  ***/
//#define dumpMemory

// Keypad Setup
const byte ROWS = 4; // Four Rows
const byte COLS = 4; // Four Columns
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'Z'},
  {'4', '5', '6', '<'},
  {'7', '8', '9', '>'},
  {'0', 'G', 'E', 'H'}
};
byte rowPins[ROWS] = {22, 24, 26, 28}; // Arduino pins connected to the row pins of the keypad
byte colPins[COLS] = {23, 25, 27, 29}; // Arduino pins connected to the column pins of the keypad
Keypad mykeypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );  // Keypad Library definition


uint8_t DIRECTIONS[4][4] = {
  {0, 1, 1, 0},
  {0, 0, 1, 1},
  {1, 0, 0, 1},
  {1, 1, 0, 0}
};

uint8_t REVERSE[] = {18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};

//OnewireKeypad <Print, 16 > Keypad(Serial, KEYS, 4, 4, A5, 4700, 1000 );

static unsigned long last_interrupt_time = 0;
const int rotateMotorPin1 = 10;
const int rotateMotorPin2 = 11;

const int lockMotorPin1 = 5;
const int lockMotorPin2 = 6;

const int controlSwitchPin = 2;

const int indicatorLEDPin = 13;
//int fwdpin = 50;
const int NumDock = 36;

int switchState = 1;             // the current reading from the input pin
int prevSwitchState = 1;
int lastSwitchState = LOW;   // the previous reading from the input pin
int writeAddress = 10;       //FRAM start address
char command;
char truntable_status;
char lockedFlag;

int currentPosition;
int moving = 0;

int locoStarted = 0;

volatile int controlSwitchTick = 0;
volatile int travelDirection = 0;
volatile boolean ignoreTick = true;
volatile boolean int_occured = false;

bool doneFlag = false;
String inString = "";    // string to hold input


/*
  Using a 7-segment display with the 74HC595 shift register
*/
int latchpin = A1;// connect to pin 12 on the '595
int clockpin = A2; // connect to pin 11 on the '595
int datapin = A0; // connect to pin 14 on the '595
int dccPosition;
int scanLinePin[2] = {A3, A4};

SevenSegment _7Segment(latchpin, clockpin, datapin, scanLinePin, 2, COMOMON_ANODE);

void readSwitchState()
{
  int_occured = true;

  if (ignoreTick)
    return;

  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 210ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 210)
  {
    if (travelDirection == 1)
    {
      controlSwitchTick += 1;
    }
    else
    {
      controlSwitchTick -= 1;
    }

    digitalWrite(indicatorLEDPin, LOW);
  }
  last_interrupt_time = interrupt_time;
}


void setup()
{
  Serial.begin(38400);
  Serial1.begin(38400);
  Serial.setTimeout(50);

  pinMode(controlSwitchPin, INPUT);
  digitalWrite(indicatorLEDPin, HIGH);

  attachInterrupt(0, readSwitchState, FALLING) ;

  pinMode(rotateMotorPin1, OUTPUT);
  pinMode(rotateMotorPin2, OUTPUT);

  pinMode(lockMotorPin1, OUTPUT);
  pinMode(lockMotorPin2, OUTPUT);

  pinMode(indicatorLEDPin, OUTPUT);
  //pinMode(A11, INPUT);           // set pin to input
  //digitalWrite(A11, LOW);       // turn on pullup resistors

  if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
    //Serial.println("Found I2C FRAM");
  } else {
    //Serial.println("I2C FRAM not identified ... check your connections?\r\n");
    //Serial.println("Will continue in case this processor doesn't support repeated start\r\n");
  }

  // Read the first byte
  uint8_t test = fram.read8(0x0);
  //Serial.print("Restarted "); Serial.print(test); Serial.println(" times");
  // Test write ++
  fram.write8(0x0, test + 1);

#  ifdef dumpMemory
  dumpMem();
#  endif
  currentPosition = readMem(writeAddress);
  _7Segment.setup();
  _7Segment.updateDisplay(currentPosition);
  lockedFlag = 0;
}

void CountingDockPosition()
{
  if (int_occured)
  {
    //Serial.println("interrupt occurs");

    int_occured = false;
  }

  if (controlSwitchTick != 0)
  {
//    Serial.print("Control Switch Tick: ");
//    Serial.print(controlSwitchTick);
//    Serial.print(" Previous Position: ");
//    Serial.print(currentPosition);

    digitalWrite(indicatorLEDPin, HIGH);
    currentPosition = readMem(writeAddress);
    currentPosition += controlSwitchTick;
    

    if (currentPosition >NumDock)
    {
      currentPosition = 1;
    }
    else if (currentPosition < 1)
    {
      currentPosition = NumDock;
    }

//    Serial.print(" Current Position: ");
//    Serial.print(currentPosition);
//    Serial.print(" Travel Direction: ");
//    Serial.println(travelDirection);
    writeMem(writeAddress, currentPosition);
    controlSwitchTick = 0;
  }
}

char GetKeyFromKeyPad()
{
  char key = ' ', tmp;
  int keyState;

  tmp = mykeypad.getKey();
  if (tmp)
  {
    keyState = mykeypad.getState();

    //value = Keypad.readPin();

    if (keyState == PRESSED)
    {
      key = tmp;

      //Serial.print(value);
//      Serial.print(" ");
//      Serial.println(key);
    }
  }

  return key;
}

void loop()
{
  CountingDockPosition();
  command = ' ';
  if (doneFlag == false)
    {
        Serial.println(currentPosition);
        delay(200);
        Serial.println(currentPosition);
        doneFlag = true;
    }
  
  if (Serial.available())
  {
    
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
    dccPosition = (inString.toInt());
    // clear the string for new input:
      inString = "";
      }
    if (dccPosition!= currentPosition){
       switch (dccPosition){
       case 97:
       dccPosition = currentPosition -1;
       dccCommand(dccPosition);
       break;
       
       case 98:
       dccPosition = currentPosition +1;
       dccCommand(dccPosition);
       break;
       
       case 99:
       dccPosition = currentPosition +18;
        if (dccPosition >36){ 
          dccPosition = dccPosition - 36;
          dccCommand(dccPosition);
        }
        else {
          dccCommand(dccPosition);
        }
        break;
        
        default:
        dccCommand(dccPosition);
        break;
        }
       }
        
    }
//      trackCounter= recCounter;
//      Serial1.println(trackCounter);

    else
  {
    command = GetKeyFromKeyPad();
  }


  ParseCommand(command);
  command = ' ';


  _7Segment.updateDisplay();
   Serial1.write(currentPosition);
  //Serial.print(currentPosition);
}

void ResetAutoFlags()
{
  truntable_status = 'm';
  prevSwitchState = 1;
}

int GetZone(int pIndex)
{
  int zone;

  if (pIndex >= 0 && pIndex < 9)
  {
    zone = 0;
  }
  else if (pIndex >= 9 && pIndex < 18)
  {
    zone = 1;
  }
  else if (pIndex >= 18 && pIndex < 27)
  {
    zone = 2;
  }
  else
  {
    zone = 3;
  }

  return zone;
}

void SearchPosition(int pSearchPosition)
{
  boolean findPos = false;
  int fromZone, toZone;

  if (pSearchPosition == currentPosition)
    return;

  fromZone = GetZone(currentPosition);
  toZone = GetZone(pSearchPosition);

//  Serial.print("From Zone: ");
//  Serial.print(fromZone);
//  Serial.print(" To Zone: ");
//  Serial.println(toZone);

  if (fromZone != toZone)
  {
    if (DIRECTIONS[fromZone][toZone])
    {
      RotateTurnTableClockwise();
    }
    else
    {
      RotateTurnTableCounterClockwise();
    }
  }
  else
  {
    if (pSearchPosition > currentPosition)
    {
      RotateTurnTableClockwise();
    }
    else
    {
      RotateTurnTableCounterClockwise();
    }
  }

  while (!findPos)
  {
    CountingDockPosition();

    if (currentPosition == pSearchPosition)
    {
      //Serial.println("Position found");
      DoStopAndDock();

      findPos = true;
      break;
    }

    _7Segment.updateDisplay();
  }
}

void DoStopAndDock()
{
  //Serial.println("Stop turn table");
  RotateTurnTableStop();

  //Serial.println("Apply lock");
  ApplyLock();
}

int GetNumberFromKeyPad(int &pDirection)
{
  char buffer[2];
  int i, number;
  char key;

  i = 1;
  buffer[0] = '0';
  buffer[1] = '0';

  number = 0;

  while (true)
  {
    key = GetKeyFromKeyPad();

    if (key >= '0' && key <= '9')
    {
      buffer[i] = key;

      number = (((buffer[1] - 0x30) * 10) + (buffer[0] - 0x30));

      i--;
      if (i < 0)
        i = 1;
    }

    if (key == 'E' || key == '>')
    {
      pDirection = 1;
      break;
    }
    else if (key == 'R' || key == '<')
    {
      pDirection = 0;
      break;
    }
    else if (key == 'G')
    {
      number = 99;

      _7Segment.updateDisplay(currentPosition);
      writeMem(writeAddress, currentPosition);
      break;
    }

    _7Segment.updateDisplay(number, true);
  }

  return number;
}
void dccCommand(int dccPosition){
  int i = 0;
  int searchPos;
  int dir = 1;
  byte inputBuffer[10];
     searchPos = dccPosition;
     if (searchPos > 0 && searchPos <=NumDock)
    {
      //Serial.print("Go to position :");
      //Serial.println(searchPos);

      _7Segment.updateDisplay(searchPos);
     

      ReleaseLock();

      if (dir == 0)
      {
        searchPos = REVERSE[searchPos];
      }
      SearchPosition(searchPos);
    }
    else
    {
      //Serial.println("Invalid position");
    }
   writeMem(writeAddress, currentPosition);
   Serial.println(currentPosition);
  }


void ParseCommand(int command)
{
  int i = 0;
  int searchPos;
  int dir = 1;
  byte inputBuffer[10];

  if (command == 'g' ||  command == 'G')
  {
    Serial.println("Go to mode");
    searchPos = GetNumberFromKeyPad(dir);

    if (searchPos > 0 && searchPos <=NumDock)
    {
      //Serial.print("Go to position :");
      //Serial.println(searchPos);

      _7Segment.updateDisplay(searchPos);


      ReleaseLock();

      if (dir == 0)
      {
        searchPos = REVERSE[searchPos];
      }
      SearchPosition(searchPos);
    }
    else
    {
      //Serial.println("Invalid position");
    }
  }
  else if (command == 's' || command == 'S')
  {
    //Serial.println("Rotate Turntable Stop");
    RotateTurnTableStop();
  }
  else if (command == '>')
  {
//    Serial.println("Rotate Turntable Trun Clockwise ");
//    Serial.print("Current Position: ");
//    Serial.print(currentPosition);
    ReleaseLock();
    searchPos = currentPosition + 1;
    if (searchPos > NumDock)
    {
      searchPos = 1;
    }

//    Serial.print("Go to : ");
//    Serial.println(searchPos);

    _7Segment.updateDisplay(searchPos);
    SearchPosition(searchPos);
    
  }
  else if (command == '<')
  {
//    Serial.println("Rotate Turntable Trun Counter Clockwise ");
//    Serial.print("Current Position: ");
//    Serial.print(currentPosition);
    ReleaseLock();
    searchPos = currentPosition - 1;
    if (searchPos < 1)
    {
      searchPos = NumDock;
    }

//    Serial.print("Go to : ");
//    Serial.println(searchPos);

    _7Segment.updateDisplay(searchPos);
    SearchPosition(searchPos);
  }
  else if (command == 'a' || command == 'A')
  {
    //Serial.println("Apply Lock ");
    ApplyLock();
  }
  else if (command == 'r' || command == 'R')
  {
    //Serial.println("Release Lock ");
    if (lockedFlag == 0)
    {
      ApplyLock();
    }
    else
    {
      ReleaseLock();
    }
  }
  else if (command == 'h' || command == 'H')
  {
    //Serial.println("Rotate Truntable home");
    ReleaseLock();
    RotateTurntableHome();
     _7Segment.updateDisplay(currentPosition);
  }
  else if (command == 'z' || command == 'Z')
  {
    //Serial.println("Reset Home");

    currentPosition = 1;

    _7Segment.updateDisplay(currentPosition);
    writeMem(writeAddress, currentPosition);
  }
writeMem(writeAddress, currentPosition);
Serial.println(currentPosition);
}
void RotateTurntableHome()
{
  SearchPosition(1);
}

void RotateTurnTableClockwise()
{
  //Serial.println("turn clockwise");
  if (!moving)
  {
    moving = 1;
    ignoreTick = true;

    analogWrite(rotateMotorPin1, 100);
    digitalWrite(rotateMotorPin2, 0);

    travelDirection = 1;

    _7Segment.delayAndDisplay(300);

    ignoreTick = false;
  }
}

void RotateTurnTableCounterClockwise()
{
  //Serial.println("turn counter clockwise");
  if (!moving)
  {
    moving = 1;
    ignoreTick = true;

    digitalWrite(rotateMotorPin1, 0);
    analogWrite(rotateMotorPin2, 100);

    travelDirection = 0;

    _7Segment.delayAndDisplay(300);
    ignoreTick = false;
  }
}

void RotateTurnTableStop()
{
  if (moving)
  {
    moving = 0;
    ignoreTick = true;

    digitalWrite(rotateMotorPin1, 0);
    digitalWrite(rotateMotorPin2, 0);

    controlSwitchTick = 0;
  }
}

void ApplyLock()
{
  if (!lockedFlag)
  {
    ignoreTick = true;

    digitalWrite(lockMotorPin1, 0);
    analogWrite(lockMotorPin2, 128);


    _7Segment.delayAndDisplay(550);

    digitalWrite(lockMotorPin1, 0);
    digitalWrite(lockMotorPin2, 0);

    lockedFlag = 1;
  }
}

void ReleaseLock()
{
  ignoreTick = true;

  analogWrite(lockMotorPin1, 128);
  digitalWrite(lockMotorPin2, 0);

  _7Segment.delayAndDisplay(550);

  digitalWrite(lockMotorPin1, 0);
  digitalWrite(lockMotorPin2, 0);

  lockedFlag = 0;
}
void writeMem(int address, long value) {
    int MSB = value / 256L;      // Break the value into 2 Byte-parts for storing
    int LSB = value % 256L;      // Above is MSB, remainder (Modulo) is LSB
    fram.write8(address, MSB);              // Store the value MSB at address add1
    fram.write8(address + 1, LSB);          // Store the value LSB at address add1 + 1
  }

int readMem(int address) {
    int MSB = fram.read8(address);           //Read the 2 bytes from memory
    int LSB = fram.read8(address + 1);
    int value = (256 * MSB) + LSB;
    return value;



  }

void dumpMem(){
      // dump the entire 32K of memory!
      uint8_t value;
      for (uint16_t a = 0; a < 32768; a++) {
       value = fram.read8(a);
       if ((a % 32) == 0) {
         Serial.print("\n 0x"); Serial.print(a, HEX); Serial.print(": ");
       }
       Serial.print("0x");
       if (value < 0x1)
         Serial.print('0');
       Serial.print(value, HEX); Serial.print(" ");
      }

}
