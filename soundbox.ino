
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

#define DEBUG
#define SPEAKER_VOLUME_0_30 5     // громкость динамика от 0 до 30
#define MP3_SERIAL_TIMEOUT_MS 500 // таймаут подключения по uart

// Define the possible states of the state machine
enum State {
  STATE_A,
  STATE_B,
  STATE_C,
  STATE_D
};

// Define the possible events that can trigger a transition
enum Event {
  EVENT_1,
  EVENT_2,
  EVENT_3,
  EVENT_4,
  EVENT_TIMER
};



SoftwareSerial mySoftwareSerial(10, 11);

DFRobotDFPlayerMini myDFPlayer;

void printDetail(uint8_t type, int value);

// Define the state machine
State currentState = STATE_A;

// Define a global variable to keep track of the elapsed time
unsigned long elapsedTime = 0;

// Define the timer interval, in milliseconds
const unsigned long TIMER_INTERVAL = 100;

void setup()
{
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);

  mySoftwareSerial.begin(9600);

#ifdef DEBUG
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
#endif
  
  if (!myDFPlayer.begin(mySoftwareSerial)) {  // подключение к плееру по soft uart

#ifdef DEBUG
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
#endif
    while(true);
  }

#ifdef DEBUG
  Serial.println(F("DFPlayer Mini online."));
#endif
  
  myDFPlayer.setTimeOut(MP3_SERIAL_TIMEOUT_MS); // таймаут
  myDFPlayer.volume(SPEAKER_VOLUME_0_30); // громкость
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL); // эквалайзер
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD); // подключение SD-карты
  
#if 0
  myDFPlayer.sleep();      // sleep
  myDFPlayer.reset();      // Reset the module
  myDFPlayer.enableDAC();  // Enable On-chip DAC
  myDFPlayer.disableDAC(); // Disable On-chip DAC
  myDFPlayer.outputSetting(true, 15); // output setting, enable the output and set the gain to 15
#endif
  

#if 0
  myDFPlayer.next();  //Play next mp3
  delay(1000);
  myDFPlayer.previous();  //Play previous mp3
  delay(1000);
  myDFPlayer.play(1);  //Play the first mp3
  delay(1000);
  myDFPlayer.loop(1);  //Loop the first mp3
  delay(1000);
  myDFPlayer.pause();  //pause the mp3
  delay(1000);
  myDFPlayer.start();  //start the mp3 from the pause
  delay(1000);
  myDFPlayer.playFolder(15, 4);  //play specific mp3 in SD:/15/004.mp3; Folder Name(1~99); File Name(1~255)
  delay(1000);
  myDFPlayer.enableLoopAll(); //loop all mp3 files.
  delay(1000);
  myDFPlayer.disableLoopAll(); //stop loop all mp3 files.
  delay(1000);
  myDFPlayer.playMp3Folder(4); //play specific mp3 in SD:/MP3/0004.mp3; File Name(0~65535)
  delay(1000);
  myDFPlayer.advertise(3); //advertise specific mp3 in SD:/ADVERT/0003.mp3; File Name(0~65535)
  delay(1000);
  myDFPlayer.stopAdvertise(); //stop advertise
  delay(1000);
  myDFPlayer.playLargeFolder(2, 999); //play specific mp3 in SD:/02/004.mp3; Folder Name(1~10); File Name(1~1000)
  delay(1000);
  myDFPlayer.loopFolder(5); //loop all mp3 files in folder SD:/05.
  delay(1000);
  myDFPlayer.randomAll(); //Random play all the mp3.
  delay(1000);
  myDFPlayer.enableLoop(); //enable loop.
  delay(1000);
  myDFPlayer.disableLoop(); //disable loop.
  delay(1000);
#endif

#ifdef DEBUG
  Serial.println(myDFPlayer.readState()); // состояние MP3 модуля
  Serial.println(myDFPlayer.readVolume()); // текущая громкость
  Serial.println(myDFPlayer.readEQ()); // текущие настройки эквалайзера
  Serial.println(myDFPlayer.readFileCounts()); // количество файлов на SD-карте
  Serial.println(myDFPlayer.readCurrentFileNumber()); // номер текущего файла
  Serial.println(myDFPlayer.readFileCountsInFolder(1)); // количество файлов в папке SD:/01
#endif
}

void loop()
{

  // Check for events and trigger transitions based on current state
  switch (currentState) {
    case STATE_A:
      if (eventHappened(EVENT_1)) {
        currentState = STATE_B;
      } else if (eventHappened(EVENT_2)) {
        currentState = STATE_C;
      }
      break;
    case STATE_B:
      if (eventHappened(EVENT_3)) {
        currentState = STATE_D;
      } else if (eventHappened(EVENT_4)) {
        currentState = STATE_A;
      }
      break;
    case STATE_C:
      if (eventHappened(EVENT_1)) {
        currentState = STATE_D;
      } else if (eventHappened(EVENT_4)) {
        currentState = STATE_A;
      }
      break;
    case STATE_D:
      if (eventHappened(EVENT_2)) {
        currentState = STATE_C;
      } else if (eventHappened(EVENT_3)) {
        currentState = STATE_B;
      }
      break;
  }

  // Perform actions based on the current state
  switch (currentState) {
    case STATE_A:
      // Perform actions for STATE_A here
      break;
    case STATE_B:
      // Perform actions for STATE_B here
      break;
    case STATE_C:
      // Perform actions for STATE_C here
      break;
    case STATE_D:
      // Perform actions for STATE_D here
      break;
  }
  static unsigned long timer = millis();
  
  if (millis() - timer > 500) {
    timer = millis();
    if (digitalRead(2)) {
      myDFPlayer.play(1);
    }
    if (digitalRead(3)) {
      myDFPlayer.play(2);
    }
    if (digitalRead(4)) {
      myDFPlayer.play(3);
    }
  }

#ifdef DEBUG
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); // мониторинг сообщений от плеера
  }
#endif
}

void printDetail(uint8_t type, int value)
{
  switch (type) {

    case TimeOut:
      Serial.println(F("Time Out!"));
      break;

    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;

    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;

    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;

    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;

    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;

    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;

    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;

    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
  
      switch (value) {

        case Busy:
          Serial.println(F("Card not found"));
          break;

        case Sleeping:
          Serial.println(F("Sleeping"));
          break;

        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;

        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;

        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;

        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;

        case Advertise:
          Serial.println(F("In Advertise"));
          break;

        default:
          break;
      }
      break;

    default:
      break;
  }
  
}

// Returns true if the specified event has happened, false otherwise
bool eventHappened(Event event) {
  // Check for the event based on the state of input pins or other sensors
  // You can use Arduino functions such as digitalRead() or analogRead() to
  // check for events based on the state of input pins or other sensors.
  // You can also use libraries or other functions to check for events based
  // on external conditions, such as a button press or a timer expiration.

  // Check for the timer event
  if (event == EVENT_TIMER) {
    // Check if the elapsed time is greater than or equal to the timer interval
    if (millis() - elapsedTime >= TIMER_INTERVAL) {
      // Reset the elapsed time
      elapsedTime = millis();
      // Return true to indicate that the event has happened
      return true;
    }
  }

  // Return false if the event has not happened
  return false;
}

