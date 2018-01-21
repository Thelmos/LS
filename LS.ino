#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "lib/I2Cdev.h"
#include "lib/MPU6050.h"
#include "lib/DFPlayer_Mini_Mp3.h"
#include "lib/fsm.h"
#include "lib/OneButton.h"

#define DEBUG 0

#if DEBUG
#define DEBUG_PRINT( x )    Serial.print( x );
#define DEBUG_PRINTLN( x )  Serial.println( x );
#else
#define DEBUG_PRINT( x )
#define DEBUG_PRINTLN( x )
#endif

/***************** PIN ACCESS OPTIMIZATION MACROS *******************/
#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))
/********************************************************************/

//VIN measurement constants
#define PIN_VOLTIMETER_SWITCH A0
#define PIN_VOLTIMETER_INPUT  A1
#define LOW_BATT_VOLTAGE  7
#define CRIT_BATT_VOLTAGE 6.5

//Vibration motor
#define PIN_VIBRATION_SWITCH  3
#define PIN_VIBRATION_HIGH    200
#define PIN_VIBRATION_MED     80
#define PIN_VIBRATION_LOW     30

//Led string constants
#define PIN_STRING_1  5
#define PIN_STRING_2  6
#define PIN_STRING_3  9
#define PIN_STRING_4  10
#define PIN_STRING_5  11
#define STRING_COUNT  5

//Sound module values
#define PIN_PLAYER_TX   7
#define PIN_PLAYER_RX   8

//Number of sound fonts
#define SOUND_FONT_COUNT      3
//Sound sample font order
#define SOUND_SAMPLE_BOOT     0
#define SOUND_SAMPLE_POWERON  1
#define SOUND_SAMPLE_POWEROFF 2
#define SOUND_SAMPLE_HUM      3
#define SOUND_SAMPLE_SWING    4
#define SOUND_SAMPLE_CLASH    5
#define SOUND_SAMPLE_BLASTER  6
#define SOUND_SAMPLE_LOCKUP   7
#define SOUND_SAMPLE_FORCE    8

//Number of system samples
#define SOUND_SYSTEM_SAMPLE_COUNT       14
//System sounds index
#define SOUND_SYSTEM_READY              1
#define SOUND_SYSTEM_ENERGY_LOW         2
#define SOUND_SYSTEM_ENERGY_CRITICAL    3
#define SOUND_SYSTEM_OPTIONS_VOLUME     4
#define SOUND_SYSTEM_OPTIONS_VOLUME_OFF 5
#define SOUND_SYSTEM_OPTIONS_VOLUME_LOW 6
#define SOUND_SYSTEM_OPTIONS_VOLUME_MID 7
#define SOUND_SYSTEM_OPTIONS_VOLUME_MAX 8
#define SOUND_SYSTEM_OPTIONS_SOUND      9
#define SOUND_SYSTEM_OPTIONS_LASER      10
#define SOUND_SYSTEM_OPTIONS_LASER_STBL 11
#define SOUND_SYSTEM_OPTIONS_LASER_UNST 12
#define SOUND_SYSTEM_OPTIONS_LASER_SCAN 13
#define SOUND_SYSTEM_OPTIONS_SAVED      14

// Numbre of laser modes
#define LASER_MODE_COUNT 2

//Lasser ON/OFF speed (ms)
#define ONDELAY   30
#define OFFDELAY  50
//Lasser glow intensity
#define GLOW_HIGH    190
#define GLOW_MED     150
#define GLOW_LOW     100
//Lasser animation speed
#define GLOW_DELAY_MIN   80
#define GLOW_DELAY_MAX   150
//Lasser blaster duration
#define BLASTER_DELAY  150

//Button
#define BUTTON_PIN        4
#define BUTTON_PUSH_S     10
#define BUTTON_PUSH_L     500
#define BUTTON_PUSH_XL    2000
#define BUTTON_LED_PIN    2
#define BUTTON_ANIM_UFAST 300
#define BUTTON_ANIM_FAST  500
#define BUTTON_ANIM_MED   900
#define BUTTON_ANIM_SLOW  3000

//Events
#define EVT_NOP               0
#define EVT_BUTTON_SHORT      1
#define EVT_BUTTON_LONG_START 2
#define EVT_BUTTON_LONG_STOP  3
#define EVT_SWING             4
#define EVT_CLASH             5
#define EVT_BUTTON_DOUBLE     6

//Acelerometer/gyroscope constants
#define SWING_THR       35000
#define SWING_DECOUPLE  800
#define CLASH_THR       22000
#define CLASH_DECOUPLE  1000

#define VOLUME_LEVEL_COUNT  3
//Default config values
#define PREFERENCES_DEFAULT_VOLUME    2
#define PREFERENCES_DEFAULT_SOUNDFONT 0
#define PREFERENCES_DEFAULT_LASERMODE 0
#define PREFERENCES_ENDFLAG           0XCACA

 //User preferences
 struct UserPreferences {
  uint8_t volume     = PREFERENCES_DEFAULT_VOLUME;
  uint8_t soundFont  = PREFERENCES_DEFAULT_SOUNDFONT;
  uint8_t laserMode  = PREFERENCES_DEFAULT_LASERMODE;
  uint16_t endCheck  = PREFERENCES_ENDFLAG;
 } userPreferences;

//Device data estructures
const byte ledStringsPins[]={PIN_STRING_1,PIN_STRING_2,PIN_STRING_3,PIN_STRING_4,PIN_STRING_5};

//Acelerometer/Gyroscope
MPU6050 mpu;

//Sound player
SoftwareSerial DFPlayer(PIN_PLAYER_TX, PIN_PLAYER_RX);

//Button controller
OneButton button(BUTTON_PIN, true);

//lightsaber states
State STATE_OFF(&snd_off, &buttonLed_slowBlink, NULL);
State STATE_SETUPVOLUME(&setup_currentVolume, &buttonLed_fastBlink, NULL);
State STATE_SETUPSOUND(&setup_currentSoundFont, &buttonLed_fastBlink, NULL);
State STATE_SETUPLASER(&setup_currentLaserMode, &buttonLed_fastBlink, NULL);
State STATE_ON(&snd_humLoop, &blade_animation, NULL);
State STATE_CLASH(&blade_clash, NULL, NULL);
State STATE_SWING(&snd_swing, NULL, NULL);
State STATE_BLASTER(&snd_humLoop, &blade_animationLockup, NULL);
State STATE_SHOOT(&blade_blaster, NULL, NULL);
State STATE_LOCKUP(&snd_lockupLoop, &blade_lockup, NULL);
Fsm fsm(&STATE_OFF);

/**
 * Volume levels
 */
typedef struct {
  byte sample;
  byte level;
} VolumeLevel;
 
VolumeLevel volumeLevels[] = {
/*
  {
    SOUND_SYSTEM_OPTIONS_VOLUME_OFF,
    0
  },
*/
  {
    SOUND_SYSTEM_OPTIONS_VOLUME_LOW,
    15
  },
  {
    SOUND_SYSTEM_OPTIONS_VOLUME_MID,
    25
  },
  {
    SOUND_SYSTEM_OPTIONS_VOLUME_MAX,
    30
  }
};

/**
 * Laser modes
 */
typedef void (*FunctionPointer)(void);

typedef struct {
  byte laserModeSample;
  FunctionPointer laserOn;
  FunctionPointer laserClash;
  FunctionPointer laserBlaster;
  FunctionPointer laserLockup;
} LaserMode;

LaserMode laserModes[]={
  {
    SOUND_SYSTEM_OPTIONS_LASER_STBL,
    laserOn_Stable,         //Laser on animation
    laserClash_FullHigh,      //Laser clash animation
    laserBlaster_SegmentOff,  //Laser blaster animation
    laserLockup_Intermitent   //Laser lockup animation
  },
  {
    SOUND_SYSTEM_OPTIONS_LASER_UNST,
    laserOn_Unstable,           //Laser on animation
    laserClash_FullHigh,      //Laser clash animation
    laserBlaster_SegmentOff,  //Laser blaster animation
    laserLockup_Intermitent   //Laser lockup animation
  }
  
};

//INIT
void setup() {

  Serial.begin(115200);
  DEBUG_PRINTLN(">>>Init...")
  analogReference(DEFAULT);
  
  //Switch pin for voltage divider
  pinMode(PIN_VOLTIMETER_SWITCH, OUTPUT);
  digitalWrite(PIN_VOLTIMETER_SWITCH, LOW);

  //Vibration motor
  pinMode(PIN_VIBRATION_SWITCH, OUTPUT); //Button led
  digitalWrite(PIN_VIBRATION_SWITCH, LOW);

/**
 * Get user preferences from EEPROM
 */
  loadUserPreferences();

  DEBUG_PRINT("Laser/Sound config: ")
  DEBUG_PRINT(userPreferences.laserMode)
  DEBUG_PRINT("/")
  DEBUG_PRINTLN(userPreferences.soundFont)

/**
 *  DFPlayer init
 */
  DFPlayer.begin(9600);
  mp3_set_serial(DFPlayer);  //set softwareSerial for DFPlayer-mini mp3 module
  delay(50);
  mp3_stop();
  delay(50);
  mp3_set_EQ(3);
  delay(50);
  mp3_set_volume(volumeLevels[userPreferences.volume].level);

/**
 *   Button init
 */
  button.attachClick(buttonShort);
  button.attachDoubleClick(buttonDouble);
  button.attachLongPressStart(buttonLongStart);
  button.attachLongPressStop(buttonLongStop);
  pinMode(BUTTON_LED_PIN, OUTPUT); //Button led

/**
 * Led strings init
 */
   for(int i=0; i < STRING_COUNT; i++) {
    pinMode(ledStringsPins[i], OUTPUT);
   }

/**
 * MPU6050 init
 */
  DEBUG_PRINTLN("Initializing MPU6050 ...");
  mpu.initialize();
  delay(50);

#if DEBUG == 1
  //Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
#endif

  // Set internal sensor offsets (calibration)
  mpu.setXAccelOffset(-1195);
  mpu.setYAccelOffset(356);
  mpu.setZAccelOffset(828);
  mpu.setXGyroOffset(47);
  mpu.setYGyroOffset(-2);
  mpu.setZGyroOffset(-25);

/**
 * Configure state machine transitions
 */
  //Setup
  fsm.add_transition(&STATE_OFF, &STATE_SETUPVOLUME, EVT_BUTTON_LONG_START, &setup_volume);
  fsm.add_transition(&STATE_SETUPVOLUME, &STATE_SETUPVOLUME, EVT_BUTTON_SHORT, &setup_nextVolumeLevel);
  fsm.add_transition(&STATE_SETUPVOLUME, &STATE_OFF, EVT_BUTTON_LONG_START, &setup_save);
  fsm.add_transition(&STATE_SETUPVOLUME, &STATE_SETUPSOUND, EVT_BUTTON_DOUBLE, &setup_sound);
  fsm.add_transition(&STATE_SETUPSOUND, &STATE_SETUPSOUND, EVT_BUTTON_SHORT, &setup_nextSoundFont);
  fsm.add_transition(&STATE_SETUPSOUND, &STATE_OFF, EVT_BUTTON_LONG_START, &setup_save);
  fsm.add_transition(&STATE_SETUPSOUND, &STATE_SETUPLASER, EVT_BUTTON_DOUBLE, &setup_laser);
  fsm.add_transition(&STATE_SETUPLASER, &STATE_SETUPLASER, EVT_BUTTON_SHORT, &setup_nextLaserMode);
  fsm.add_transition(&STATE_SETUPLASER, &STATE_OFF, EVT_BUTTON_LONG_START, &setup_save);
  fsm.add_transition(&STATE_SETUPLASER, &STATE_SETUPVOLUME, EVT_BUTTON_DOUBLE, &setup_volume);

  //On/Off
  fsm.add_transition(&STATE_OFF, &STATE_ON, EVT_BUTTON_SHORT, &blade_powerOn);
  fsm.add_transition(&STATE_ON, &STATE_OFF, EVT_BUTTON_DOUBLE, &blade_powerOff);

  //Clash and Swing
  fsm.add_transition(&STATE_ON, &STATE_CLASH, EVT_CLASH, NULL);
  fsm.add_timed_transition(&STATE_CLASH, &STATE_ON, 500, NULL);
  fsm.add_transition(&STATE_ON, &STATE_SWING, EVT_SWING, NULL);
  fsm.add_timed_transition(&STATE_SWING, &STATE_ON, 600, NULL);
  //fsm.add_transition(&STATE_SWING, &STATE_CLASH, EVT_CLASH, NULL);

  //Blaster
  fsm.add_transition(&STATE_ON, &STATE_BLASTER, EVT_BUTTON_SHORT, NULL);
  fsm.add_transition(&STATE_BLASTER, &STATE_SHOOT, EVT_SWING, NULL);
  fsm.add_timed_transition(&STATE_SHOOT, &STATE_BLASTER, 750, NULL);
  fsm.add_transition(&STATE_BLASTER, &STATE_ON, EVT_BUTTON_SHORT, &buttonLed_on);

  //Lockup
  fsm.add_transition(&STATE_ON, &STATE_LOCKUP, EVT_BUTTON_LONG_START, NULL);
  fsm.add_transition(&STATE_LOCKUP, &STATE_ON, EVT_BUTTON_LONG_STOP, NULL);

  //Play ready sound if no battery warning found
  if (checkBatteryLevel()){
    mp3_play_physical(SOUND_SYSTEM_READY);
  }

  DEBUG_PRINTLN(">>>End init")
  delay(2000);
}

/**
 * Button events
 */
void buttonShort(){
  DEBUG_PRINTLN("Short click")
  fsm.trigger(EVT_BUTTON_SHORT);
}
void buttonDouble(){
  DEBUG_PRINTLN("Double click")
  fsm.trigger(EVT_BUTTON_DOUBLE);
}
void buttonLongStart(){
  DEBUG_PRINTLN("Long click start")
  fsm.trigger(EVT_BUTTON_LONG_START);
}
void buttonLongStop(){
  DEBUG_PRINTLN("Long click stop")
  fsm.trigger(EVT_BUTTON_LONG_STOP);
}

/**
 * Actions
 */
void snd_off(){
  DEBUG_PRINTLN("snd_off")
  mp3_stop();
}

void snd_humLoop(){
  DEBUG_PRINTLN("snd_humLoop")
  mp3_loop_play(getCurrentFontSampleIndex(SOUND_SAMPLE_HUM));
}

void snd_swing(){
  DEBUG_PRINTLN("snd_swing")
  //Start vibration
  analogWrite(PIN_VIBRATION_SWITCH, PIN_VIBRATION_HIGH);
  mp3_play_physical(getCurrentFontSampleIndex(SOUND_SAMPLE_SWING));
}

void snd_lockupLoop(){
  mp3_loop_play(getCurrentFontSampleIndex(SOUND_SAMPLE_LOCKUP));
}

/**
 * Blade actions
 */
void blade_animation(){
  laserModes[userPreferences.laserMode].laserOn();
}

void blade_animationLockup(){
  laserModes[userPreferences.laserMode].laserOn();
  buttonLed_ultraFastBlink();
}


void blade_clash(){
  DEBUG_PRINTLN("blade_clash")
  laserModes[userPreferences.laserMode].laserClash();
}

void blade_blaster(){
  DEBUG_PRINTLN("blade_blaster")
  laserModes[userPreferences.laserMode].laserBlaster();
}

void blade_lockup(){
  laserModes[userPreferences.laserMode].laserLockup();
}

void blade_powerOn() {
  DEBUG_PRINTLN("blade_powerOn")
  mp3_play_physical(getCurrentFontSampleIndex(SOUND_SAMPLE_POWERON));

  //Turn on button
  buttonLed_on();

  //Start vibration
  analogWrite(PIN_VIBRATION_SWITCH, PIN_VIBRATION_MED);

  //Turn on laser
  analogWrite(PIN_STRING_1, GLOW_HIGH+50);
  delay(ONDELAY+30);
  analogWrite(PIN_STRING_2, GLOW_HIGH);
  delay(ONDELAY);
  analogWrite(PIN_STRING_3, GLOW_HIGH);
  analogWrite(PIN_STRING_1, GLOW_HIGH);
  delay(ONDELAY-10);
  analogWrite(PIN_STRING_4, GLOW_HIGH);
  delay(ONDELAY);
  analogWrite(PIN_STRING_5, GLOW_HIGH);

  delay(1000);
}

void blade_powerOff() {
  DEBUG_PRINTLN("blade_powerOff")
  mp3_play_physical(getCurrentFontSampleIndex(SOUND_SAMPLE_POWEROFF));

  delay(200);

  //Stop vibration
  analogWrite(PIN_VIBRATION_SWITCH, LOW);

  //Turn off button
  buttonLed_off();

  //Turn off laser
  analogWrite(PIN_STRING_5, GLOW_MED);
  analogWrite(PIN_STRING_4, GLOW_MED);
  analogWrite(PIN_STRING_5, LOW);
  delay(OFFDELAY-10);
  analogWrite(PIN_STRING_3, GLOW_MED);
  analogWrite(PIN_STRING_4, LOW);
  delay(OFFDELAY-10);
  analogWrite(PIN_STRING_3, LOW);
  analogWrite(PIN_STRING_2, GLOW_MED);
  delay(OFFDELAY);
  analogWrite(PIN_STRING_2, LOW);
  analogWrite(PIN_STRING_1, GLOW_MED);
  delay(OFFDELAY);
  analogWrite(PIN_STRING_1, LOW);
  delay(500);
}

/**
 * Button actions
 */


void buttonLed_on(){
  digitalWrite(BUTTON_LED_PIN, HIGH);
}

void buttonLed_off(){
  digitalWrite(BUTTON_LED_PIN, LOW);
}

void buttonLed_slowGlow(){
  buttonGlow(millis(), BUTTON_ANIM_FAST);
}

void buttonLed_slowBlink(){
  buttonBlink(millis(), BUTTON_ANIM_SLOW);
}

void buttonLed_blink(){
  buttonBlink(millis(), BUTTON_ANIM_MED);
}

void buttonLed_fastBlink(){
  buttonBlink(millis(), BUTTON_ANIM_FAST);
}

void buttonLed_ultraFastBlink(){
  buttonBlink(millis(), BUTTON_ANIM_UFAST);
}

/**
 * Setup actions
 */

void setup_volume(){
  DEBUG_PRINTLN("setup_volume")
  mp3_play_physical(SOUND_SYSTEM_OPTIONS_VOLUME);
  delay(2000);
}

void setup_sound(){
  DEBUG_PRINTLN("setup_sound")
  mp3_play_physical(SOUND_SYSTEM_OPTIONS_SOUND);
  delay(2000);
}

void setup_laser(){
  DEBUG_PRINTLN("setup_laser")
  mp3_play_physical(SOUND_SYSTEM_OPTIONS_LASER);
  delay(2000);
}
 
void setup_currentVolume(){
   mp3_play_physical(volumeLevels[userPreferences.volume].sample);
}

void setup_currentSoundFont(){
  //TODO voz con la fuente actual
  mp3_play_physical(getCurrentFontSampleIndex(SOUND_SAMPLE_BOOT));
}

void setup_currentLaserMode(){
  //TODO voz con el modo actual de laser
  mp3_play_physical(laserModes[userPreferences.laserMode].laserModeSample);
}

void setup_nextVolumeLevel(){
  DEBUG_PRINTLN("setup_nextVolumeLevel")
  userPreferences.volume = (userPreferences.volume < (VOLUME_LEVEL_COUNT-1)) ? ++userPreferences.volume : 0;
  mp3_set_volume(volumeLevels[userPreferences.volume].level);
  delay(50);
}

void setup_nextSoundFont(){
  DEBUG_PRINTLN("setup_nextSoundFont")
  userPreferences.soundFont = (userPreferences.soundFont < (SOUND_FONT_COUNT-1)) ? ++userPreferences.soundFont : 0;
}

void setup_nextLaserMode(){
  DEBUG_PRINTLN("setup_nextLaserMode")
  userPreferences.laserMode = (userPreferences.laserMode < (LASER_MODE_COUNT-1)) ? ++userPreferences.laserMode : 0;
}

void setup_save(){
  DEBUG_PRINTLN("setup_save")
  mp3_play_physical(SOUND_SYSTEM_OPTIONS_SAVED);
  saveUserPreferences();
  delay(2000);
}

unsigned int loopTime = 0;

// the loop routine runs over and over again forever:
void loop() {
  //loopTime = millis();

  //Get input data from sensors
  button.tick();
  checkAcelerometer();

  //Update FSM state
  fsm.run_machine();

  /*
  loopTime = millis() - loopTime;
  Serial.println(loopTime);
  */
}

/**
 * Utility routines
 */
unsigned long sqrt32(unsigned long n){
  unsigned long c = 0x8000;
  unsigned long g = 0x8000;

  for(;;) {
    if(g*g > n){
      g ^= c;
    }
    c >>= 1;
    if(c == 0){
      return g;
    }
    g |= c;
  }
}

/**
 * Fire warning and critical battery level audio signals
 */
bool checkBatteryLevel() {

  bool batteryOk = false;
  
  //Turn on voltage divider
  digitalWrite(PIN_VOLTIMETER_SWITCH, HIGH);
  delay(50);
  //Read voltage
  float volts = (analogRead(PIN_VOLTIMETER_INPUT)*3*5)/1023.0;
  //Turn off voltage divider
  digitalWrite(PIN_VOLTIMETER_SWITCH, LOW);

  if (volts <= CRIT_BATT_VOLTAGE){
    mp3_play_physical(SOUND_SYSTEM_ENERGY_CRITICAL);
    DEBUG_PRINTLN("CRITICAL battery level")
  }else if (volts <= LOW_BATT_VOLTAGE){
    mp3_play_physical(SOUND_SYSTEM_ENERGY_LOW);
    DEBUG_PRINTLN("LOW battery level")
  }else{
    batteryOk = true;
  }

  return batteryOk;
}

void buttonBlink(unsigned long actualTime, int animationSpeed) {
  static unsigned long nextTime = 0;
  static byte ledStatus = LOW;

  if (actualTime > nextTime) {
    ledStatus = (ledStatus == LOW) ? HIGH : LOW;
    digitalWrite(BUTTON_LED_PIN, ledStatus);
    if (ledStatus == HIGH) {
      nextTime = actualTime + 80; //HIGH time
    } else {
      nextTime = actualTime + animationSpeed; //LOW time
    }
  }
}

#define BUTTON_GLOW_MAX  12
#define BUTTON_GLOW_MIN  1
#define BUTTON_GLOW_INC  1
void buttonGlow(unsigned long actualTime, int animationSpeed) {
  static uint16_t nextTime = 0;
  static uint8_t ledStatus = BUTTON_GLOW_MIN;
  static int8_t increment = BUTTON_GLOW_INC;

  if (actualTime > nextTime) {
    ledStatus += increment;
    if (ledStatus > BUTTON_GLOW_MAX - BUTTON_GLOW_INC){
      increment = -BUTTON_GLOW_INC;
    }else if (ledStatus < BUTTON_GLOW_MIN + BUTTON_GLOW_INC){
      increment = BUTTON_GLOW_INC;
      //aguantar más tiempo en nivel bajo
      animationSpeed += animationSpeed;
    }
    DEBUG_PRINTLN(ledStatus)
    analogWrite(BUTTON_LED_PIN, ledStatus);
    nextTime = actualTime + animationSpeed - (ledStatus*64);
  }
}

void checkAcelerometer() {
  static int16_t ax, ay, az;
  static int16_t gx, gy, gz;
  static unsigned long currRawGiro;
  static unsigned long lastRawAcel = 0;
  static unsigned long currRawAcel;
  static long currAcel;
  static unsigned int swingCicle = 0;
  static unsigned long swingIgnore = 0;
  static unsigned long clashIgnore = 0;
  static unsigned long now;

 // read raw accel/gyro measurements from device
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  currRawGiro = sqrt32(sq((long)gx) + sq((long)gy) + sq((long)gz));

  if (currRawGiro > SWING_THR) {

    swingCicle++;
    //Get acceleration vector modulus and calculate variation from previous one
    currRawAcel = sqrt32(sq((long)ax) + sq((long)ay) + sq((long)az));
    currAcel = currRawAcel - lastRawAcel;
    lastRawAcel = currRawAcel;

    now = millis();
    if (abs(currAcel) > CLASH_THR) {
      if (clashIgnore < now){
        Serial.println("CLASH>>> ");
        fsm.trigger(EVT_CLASH);
        swingCicle = 0;
        clashIgnore = now + CLASH_DECOUPLE;
        swingIgnore = now + SWING_DECOUPLE;
      }
    }else{
      if ((swingIgnore < now) && (swingCicle > 2)){
        Serial.println("SWING>>> ");
        fsm.trigger(EVT_SWING);
        swingCicle = 0;
        swingIgnore = now + SWING_DECOUPLE;
      }
    }
  }
}

/**
 * Configuration: User preferences
 */

inline void loadUserPreferences() {
  EEPROM.get(0, userPreferences);
  //If not correctly loaded fill with default values
  if (userPreferences.endCheck != PREFERENCES_ENDFLAG){
    userPreferences.volume     = PREFERENCES_DEFAULT_VOLUME; // 0 to 30
    userPreferences.soundFont  = PREFERENCES_DEFAULT_SOUNDFONT;
    userPreferences.laserMode = PREFERENCES_DEFAULT_LASERMODE;
    userPreferences.endCheck   = PREFERENCES_ENDFLAG;
  }
}

inline void saveUserPreferences() {
  EEPROM.put(0, userPreferences);
}

/**
 * Laser modes animations
 */

void laserOn_Stable(){
  static unsigned long nextTime = 0;
  //Glow animation
  unsigned long actualTime = millis();
  if (actualTime > nextTime) {
    for(int i=0; i < STRING_COUNT; i++) {
      //analogWrite(ledStringsPins[i], GLOW_HIGH);
      digitalHigh(i);
    }
    //Change vibration
    analogWrite(PIN_VIBRATION_SWITCH, PIN_VIBRATION_LOW * 2);
    nextTime = actualTime + 3000;
  }
}

void laserOn_Unstable(){
  static unsigned long nextTime = 0;
  static boolean changeVibration = true;

  //Glow animation
  unsigned long actualTime = millis();
  if (actualTime > nextTime) {
    for(int i=0; i < STRING_COUNT; i++) {
      analogWrite(ledStringsPins[i], random(GLOW_MED,GLOW_HIGH));
    }
    
    if (changeVibration){
      //Change vibration
      analogWrite(PIN_VIBRATION_SWITCH, random(PIN_VIBRATION_LOW, PIN_VIBRATION_MED));
    }

    nextTime = actualTime + random(GLOW_DELAY_MIN, GLOW_DELAY_MAX);
    changeVibration = !changeVibration;
  }
}

void laserClash_FullHigh(){
  DEBUG_PRINTLN("blade_clash")
  mp3_play_physical(getCurrentFontSampleIndex(SOUND_SAMPLE_CLASH));
  for(int i=0; i < STRING_COUNT; i++) {
      analogWrite(ledStringsPins[i], GLOW_HIGH);
  }
}

void laserBlaster_SegmentOff(){
  DEBUG_PRINTLN("blade_blaster")
  mp3_play_physical(getCurrentFontSampleIndex(SOUND_SAMPLE_BLASTER));
  analogWrite(ledStringsPins[random(0,5)], GLOW_LOW);
}

void laserLockup_Intermitent(){
  static unsigned long nextTime = 0;
  static byte bladeIntensity = GLOW_HIGH;

  //Glow animation
  unsigned long actualTime = millis();
  if (actualTime > nextTime) {
    for(int i=0; i < STRING_COUNT; i++) {
      analogWrite(ledStringsPins[i], bladeIntensity);
    }
    //Change vibration
    analogWrite(PIN_VIBRATION_SWITCH, random(PIN_VIBRATION_LOW, PIN_VIBRATION_HIGH));

    if (bladeIntensity == GLOW_HIGH){
      bladeIntensity = GLOW_LOW;
    }else{
      bladeIntensity = GLOW_HIGH;
    }
    nextTime = actualTime + random(GLOW_DELAY_MIN, GLOW_DELAY_MAX);
  }
}


/**
 * Sound fonts
 */

typedef struct {
  byte index;
  byte count;
} SoundSample;

typedef struct {
  SoundSample samples[9] ;
  byte sampleCount;
  unsigned int powerOnTime;
  unsigned int powerOffTime;
} SoundFont;

SoundFont soundFonts[SOUND_FONT_COUNT] = {
  {//Default by joe93barlow (http://www.freesound.org/people/joe93barlow/) 
    { //samples, each element: {index, count}
      {1,1},  //boot
      {2,1},  //powerOn
      {3,1},  //powerOff
      {4,1},  //hum
      {5,8},  //swing
      {13,3}, //clash
      {16,4}, //blaster
      {20,1}, //lockup
      {0,0}  //force
    },
    20,     //sample count
    500,    //power on time
    500     //power off time
  },
  {//Obsidian
    {//samples, each element: {index, count}
      {1,1},  //boot
      {2,1},  //powerOn
      {3,1},  //powerOff
      {4,1},  //hum
      {5,12}, //swing
      {17,12},//clash
      {29,4}, //blaster
      {33,1}, //lockup
      {34,1}  //force
    },
    34,     //sample count
    500,    //power on time
    500     //power off time
  },
  {//Sith
    {//samples, each element: {index, count}
      {1,1},  //boot
      {2,1},  //powerOn
      {3,1},  //powerOff
      {4,1},  //hum
      {5,8},  //swing
      {13,8}, //clash
      {21,1}, //blaster
      {22,1}, //lockup
      {23,1}  //force
    },
    23,     //sample count
    500,    //power on time
    500     //power off time
  }
};

// Get global index for a given current font local index
byte getCurrentFontSampleIndex(byte sample){
  byte soundIndex = 0;
  byte font = userPreferences.soundFont;
  
  DEBUG_PRINT("getCurrentFontSampleIndex(font/sample):")
  DEBUG_PRINT(font) DEBUG_PRINT("/") DEBUG_PRINTLN(sample)
  
  //Increment index for all banks before the selected one
  for (byte i = 0; i < SOUND_FONT_COUNT; i++){
    if (i == font) break;
    soundIndex += soundFonts[i].sampleCount;
  }

  //Increment index untill selected sample in current bank and add system sounds offset
  soundIndex += soundFonts[font].samples[sample].index + random (0, soundFonts[font].samples[sample].count-1) + SOUND_SYSTEM_SAMPLE_COUNT;
  
  DEBUG_PRINT("soundGlobalIndex:")
  DEBUG_PRINTLN(soundIndex)

  return soundIndex;
}

