
#include "Arduino.h"
#include <avr/interrupt.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

#define DEBUG // вывод в консоль отладочных сообщений
#define SPEAKER_VOLUME_0_30 30 // громкость динамика от 0 до 30

// Период опроса входов (мкс)
#define EVENT_SWITCH_PERIOD_MCS 100000ul
const int16_t compare_val = (int16_t)(EVENT_SWITCH_PERIOD_MCS / 64. - .5);
#define MP3_SERIAL_TIMEOUT_MS 500 // таймаут подключения mp3-плеера по uart

#define SOFTWARE_SERIAL_RX_PIN 10 // пины для управления mp3-плеером по uart
#define SOFTWARE_SERIAL_TX_PIN 11

// Пины герконов
enum {
  SWITCH_1_PIN = 2,
  SWITCH_2_PIN,
  SWITCH_3_PIN,
};

#define SWITCH_NUM 3

// Глобальная переменная для отсчета времени до наступления события
unsigned long elapsedTime = 0;
// Интервал времени до наступления события
const unsigned long TIMER_INTERVAL = 100;

// Возможные состояния
typedef enum {
  STATE_START,
  FIGURE_1_PEND, // ожидание установки первой фигуры
  FIGURE_2_PEND, // ожидание установки второй фигуры
  FIGURE_3_PEND, // ожидание установки третьей фигуры
  STATE_DONE,
} STATE_t;

// Первоначальное состояние
volatile STATE_t state = STATE_START;

// Возможные события
typedef enum {
  EVENT_DEFAULT,    // событие по умолчанию
  EVENT_TIMER,      // начало отсчета до наступления события
  FIGURE_1_PLACED,  // фигура 1 установлена
  FIGURE_2_PLACED,  // фигура 2 установлена
  FIGURE_3_PLACED,  // фигура 3 установлена
  FIGURE_1_REMOVED, // фигура 1 вынута
  FIGURE_2_REMOVED, // фигура 2 вынута
  FIGURE_3_REMOVED, // фигура 3 вынута
  BOX_UNLOCKED,     // замок открылся  
  BOX_LOCKED,       // замок закрылся
  BOX_OPENED,       // крышка шкатулки открылась
  BOX_CLOSED,       // крышка шкатулки закрылась
} EVENT_t;

// Событие по умолчанию
volatile EVENT_t event = EVENT_DEFAULT;

SoftwareSerial mySoftwareSerial(SOFTWARE_SERIAL_RX_PIN, SOFTWARE_SERIAL_TX_PIN);
DFRobotDFPlayerMini myDFPlayer;

void tim_set(void);
void io_set(void);
bool eventHappened(EVENT_t event);
void printDetail(uint8_t type, int value);

void setup()
{
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
  // Переключение состояний
  switch (state) {

    case STATE_START:
      state = FIGURE_1_PEND;
      break;

    case FIGURE_1_PEND:
      if (eventHappened(FIGURE_1_PLACED)) {
        state = FIGURE_2_PEND;
      } else if (eventHappened(FIGURE_2_PLACED)) {
        state = STATE_START;
      } else if (eventHappened(FIGURE_3_PLACED)) {
        state = STATE_START;
      }
      break;

    case FIGURE_2_PEND:
      if (eventHappened(FIGURE_1_REMOVED)) {
        state = STATE_START;
      } else if (eventHappened(FIGURE_2_PLACED)) {
        state = FIGURE_3_PEND;
      }
      break;

    case FIGURE_3_PEND:
      if (eventHappened(FIGURE_1_REMOVED)) {
        state = STATE_START;
      } else if (eventHappened(FIGURE_2_REMOVED)) {
        state = STATE_START;
      } else if (eventHappened(FIGURE_3_PLACED)) {
        state = STATE_DONE;
      }
      break;

    case STATE_DONE:
      // Если все фигуры вынуты, перезапуск игры
      if (eventHappened(FIGURE_1_REMOVED) && eventHappened(FIGURE_2_REMOVED) && eventHappened(FIGURE_3_REMOVED)) {
        state = STATE_START;   
      }     
      break;
  }

  // Выполнение действий в зависимости от сотояния
  switch (state) {

    case STATE_START:
      break;

    case FIGURE_1_PEND:
      break;

    case FIGURE_2_PEND:
      break;

    case FIGURE_3_PEND:
      break;

    case STATE_DONE:
      break;
  }

#if 0
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
#endif

#ifdef DEBUG
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); // мониторинг сообщений от плеера
  }
#endif
}

// Инициализация Timer1
void tim1_set(void)
{
  cli();
  TCCR1A = 0;
  TCCR1B = 0;

  OCR1A = compare_val; // установка регистра совпадения

  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10); // /1024
  TCCR1B |= (1 << CS12);

  TIMSK1 |= (1 << OCIE1A);
  sei();
}

// Инициализация входов/выходов
void io_set(void)
{
  pinMode(SWITCH_1_PIN, INPUT);
  pinMode(SWITCH_2_PIN, INPUT);
  pinMode(SWITCH_3_PIN, INPUT);
}

// Возвращает true если событие случилось, иначе false
bool eventHappened(EVENT_t event)
{
  if (event == EVENT_TIMER) {
    if (millis() - elapsedTime >= TIMER_INTERVAL) {
      elapsedTime = millis();

      return true;
    }
  }

  return false;
}

// Вывод информации по mp3 модулю
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

// Вызов событий в прерывании по таймеру
ISR(TIMER1_COMPA_vect)
{
  // Предыдущие состояния герконов
  static bool switchStateShadow[SWITCH_NUM] = { LOW, LOW, LOW };

  // Текущие состояния герконов
  bool switchStateCurrent[SWITCH_NUM];

  // Чтение состояний герконов
  int pin = SWITCH_1_PIN;
  for (int i = 0; pin < SWITCH_NUM; i++) {
    switchStateCurrent[i] = (bool)digitalRead(pin);
    pin++;
  }
 
}
