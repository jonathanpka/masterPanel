///////////////////////////////////////////////////////////////////////////////////
/// @file masterPanel.ino
/// @brief Sketch for the Compressed Air Rocket Master Panel.
/// This sketch monitors the switches and buttons in the Master Panel, writes
/// data to the LCD, and communicates with the Remote Panel and Pad using Zigbee
/// @see LaunchPadPacket.h
///
/// LCD Layout
/// This section of documentation describes the complete layout of the LCD Screen
/// by the Master Panel program.  The LCD screen is 16 x 2 characters.
///
///                           Character Columns
///                                     111111
///                           0123456789012345
///           Line #1         aa S C R Ppp Ex
///           Line #2         bb LA RA RL LL
///
///   aa = pressure #1 and bb = pressure #2.  Both are 2 digit integers. \n
///   S = safety on. S is removed when safety is off \n
///   C = compressor on, C is removed when compressor is off \n
///   R = remote enabled, R is removed when remote is disabled. \n
///   Ppp is P12 (both pads enabled).  P1_ (only pad1 enabled). P_2 (only pad2
///        enabled) \n
///   Ex = Error and x is an integer from 0 to 9 (erro code) \n
///   LA = Local Arm, LA removed when local arm is off or not authorized. \n
///   RA = Remote Arm, RA removed when remote arm off or not authorized. \n
///   RL = Remote Launch, RL only on when remote launch is authorized and button
///        pressed. \n
///   LL = Local Launch, LL only on when local launch is authorized and button
///        pressed.
///////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <XBee.h>
#include <LaunchDataPacket.h>
///////////////////////////////////////////////////////////////////////////////////
/// @def NUM_CHAR
/// @brief Define the number of charcters per row in the LCD display
///////////////////////////////////////////////////////////////////////////////////
#define NUM_CHAR 15
#include <LCD4Bit_mod.h> 

///////////////////////////////////////////////////////////////////////////////////
/// @def _ArmLedBitMode_
/// @brief Define the Digital Pin # for the Arm Switch LED.
/// Master controller has an Arm Toggle switch with build-in LED.
/// This output does not control the LED.  Instead it supplies power to the LED
/// and switch combo.  LED will not turn on without power.  LED will turn on when
/// it has power and the toggle switch is on.
///
/// @def _ArmLedPort_
/// @brief Define the Digital Port the Arm Switch LED is connected to.
///
/// @def _ArmLedBit_
/// @brief Define the Port Bit # for the Arm Switch LED.
/// In Port D, the Port Bit # is equal to the Digital Pin #.
/// In Port B, the Port Bit # is equal to the Digital Pin # minus 8.
///
/// @def _ArmSwitchBitMode_
/// @brief Define the Digital Input # for the Arm Switch.
/// This input monitors the Master controller Arm Toggle switch (DPST).
/// 
/// @def _ArmSwitchPort_
/// @brief Define the Digital Port the Arm Switch is connected to.
// 
/// @def _ArmSwitchBit_
/// @brief Define the Port Bit # for the Arm Switch
/// In Port D, the Port Bit # is equal to the Digital Pin #.
/// In Port B, the Port Bit # is equal to the Digital Pin # minus 8.
// 
/// @def _LaunchLedBitMode_
/// @brief Define the Digital Output # for the Launch Button LED.
/// Master controller has a Launch Button with build in LED.  This pin turns
/// the built in LED on and off.  The button does not control the LED.
/// 
/// @def _LaunchLedPort_
/// @brief Define the Digital Port the Launch Button LED is connected to.
/// 
/// @def _LaunchLedBit_
/// @brief Define the Port Bit # for the Launch Button LED.
/// In Port D, the Port Bit # is equal to the Digital Pin #.
/// In Port B, the Port Bit # is equal to the Digital Pin # minus 8.
///
/// @def _LaunchButtonBitMode_
/// @breif Define the Digital Input # for the Launch Button.
///
/// @def _LaunchButtonPort_
/// @breif Define the Digital Port the Launch Button is connected to.
///
/// @def _LaunchButtonBit_
/// @breif Define the Port Bit # for the Launch Button.
/// In Port D, the Port Bit # is equal to the Digital Pin #.
/// In Port B, the Port Bit # is equal to the Digital Pin # minus 8.
///
/// @def _SafetyBitMode_
/// @brief Define the Digital Input # for the safety key.
///
/// @def _SafetyBitPort_
/// @brief Define the Digital Port the safety key is connected to.
///
/// @def _SafetyBit_
/// @brief Define the Port Bit # for the safety key.
/// In Port D, the Port Bit # is equal to the Digital Pin #.
/// In Port B, the Port Bit # is equal to the Digital Pin # minus 8.
///////////////////////////////////////////////////////////////////////////////////
#define _ArmLedBitMode_        2
#define _ArmLedPort_           PORTD
#define _ArmLedBit_            2

#define _ArmSwitchBitMode_     12
#define _ArmSwitchPort_        PORTB
#define _ArmSwitchBit_         4

#define _LaunchButtonBitMode_  11
#define _LaunchButtonPort_     PORTB
#define _LaunchButtonBit_      3

#define _LaunchLedBitMode_     3
#define _LaunchLedPort_        PORTD
#define _LaunchLedBit_         3 

#define _CompressorButtonMode_ 
#define _CompressorButtonPort_ 
#define _CompressorButtonBit_   

#define _SafetyBitMode_        5
#define _SafetyPort_           PORTD
#define _SafetyBit_            5

///////////////////////////////////////////////////////////////////////////////////
/// @def _AnalogButtons_
/// @brief Define the Analog Input # for the momentary push buttons.
/// Master controller has 3 momentary push button switches (SW1, SW2, and SW3).
/// All three of these switches have one terminal wired to a single resistor (R0).
/// Each switch has the other terminal wired to analog pin 0 and a seperate 
/// resistor.  SW1 is wired to R1, SW2 wired to R2, and SW3 wired to R3.  R1, R2, 
/// and R3 each form a voltage divider with R0 and the resistor values are chosen
/// to produce 3 distinct voltages at analog pin 0.   
///////////////////////////////////////////////////////////////////////////////////
#define _AnalogButtons_        0

///////////////////////////////////////////////////////////////////////////////////
/// @def _PressureScale1_
/// @brief Define the Analog Input # for POT1.
/// Master controller has 2 POTs used to calibrate the pressure readings displayed
/// on the LCD screen.  The center pin of the POT is connected to the analog Input.
/// @def _PressureScale2_
/// @brief Define the Analog Input # for POT2.
/// Master controller has 2 POTs used to calibrate the pressure readings displayed
/// on the LCD screen.  The center pin of the POT is connected to the analog Input.
///////////////////////////////////////////////////////////////////////////////////
#define _PressureScale1_       1
#define _PressureScale2_       2

///////////////////////////////////////////////////////////////////////////////////
/// @def _pulse_halfwidth_
/// @brief Define the 1/2 period of the pulse used to flash the LED in the launch
/// button.
///
/// @def _zigbee_timeout_
/// @brief Define the timeout for the zigbee communications.  Error occurs if
/// the time since the last communicaitons exceeds the timeout.
///////////////////////////////////////////////////////////////////////////////////
#define _pulse_halfwidth_       100
#define _zigbee_timeout_        1000



///////////////////////////////////////////////////////////////////////////////////
/// @brief Create object to control an LCD with 2 lines of text.  LCD controls bits
/// 4,5,6,7,8,9,and 10.  These bits cannot be used for input or output by anything
/// else unless you have knowledge of how the LCD shield and LCD library operate.
///////////////////////////////////////////////////////////////////////////////////
LCD4Bit_mod lcd = LCD4Bit_mod(2);

///////////////////////////////////////////////////////////////////////////////////
/// @brief Define the Analog values which correspond to the three momentary push
/// buttons installed in the Master Panel.  The buttons on the LCD shield are read
/// using the Analog pin #0.  We will not be using the buttons on the shield, but
/// we will use Analog pin #0 to read our buttons in the same way the Shield uses
/// Analog pin #0.  Define the array to be static const so the array is stored
/// in flash memeory and not RAM.
///////////////////////////////////////////////////////////////////////////////////
static const int  adc_key_val[5] ={30, 360, 760 };

///////////////////////////////////////////////////////////////////////////////////
/// @brief Define the number of buttons to be read using Analog pin #0.
///////////////////////////////////////////////////////////////////////////////////
static const int NUM_KEYS = 3;

///////////////////////////////////////////////////////////////////////////////////
/// @brief Store the Analog Input Value when reading the 3 push buttons.
///////////////////////////////////////////////////////////////////////////////////
int adc_key_in;

///////////////////////////////////////////////////////////////////////////////////
/// @brief Once we decypher the button pushed, store the button # here. Since there
/// are three buttons, the values are -1, 1, 2, and 3.  -1 is when no button is
/// pushed.
///////////////////////////////////////////////////////////////////////////////////
int key=-1;

///////////////////////////////////////////////////////////////////////////////////
/// @brief Store the previous button pushed.  When a button is pushed once, it will
/// be detected multiple times in a row.  By storing the previus button pushed, we
/// can detect the first moment a button is pushed and rule out subsequent
/// detections.
///////////////////////////////////////////////////////////////////////////////////
int oldkey=-1;

///////////////////////////////////////////////////////////////////////////////////
/// @brief Character string for storing integers to be displayed on the LCD screen.
///////////////////////////////////////////////////////////////////////////////////
char number[34];

///////////////////////////////////////////////////////////////////////////////////
/// @brief Store the Analog Input value for the pressure calibration POT #1.
/// POT #1 is inside the Master Panel, but it calibrates the pressure reading
/// obtained from launch pad #1.
///////////////////////////////////////////////////////////////////////////////////
int pressureCal1;

///////////////////////////////////////////////////////////////////////////////////
/// @brief Store the Analog Input value for the pressure calibration POT #2.
/// POT #2 is inside the Master Panel, but it calibrates the pressure reading
/// obtained from launch pad #2.
///////////////////////////////////////////////////////////////////////////////////
int pressureCal2;

///////////////////////////////////////////////////////////////////////////////////
/// @brief Store the pressure (in units of psi) from Launch Pad #1.
/// The pressure stored in the instance of LaunchDataPacket is the raw Analog
/// input reading.  pressureCal1 is the calibration values used to convert the
/// Analog input reading into units of psi.  pressure1 is the value to be
/// displayed on the LCD.
///////////////////////////////////////////////////////////////////////////////////
int pressure1;

///////////////////////////////////////////////////////////////////////////////////
/// @brief Store the pressure (in units of psi) from Launch Pad #2.
/// The pressure stored in the instance of LaunchDataPacket is the raw Analog
/// input reading.  pressureCal2 is the calibration values used to convert the
/// Analog input reading into units of psi.  pressure2 is the value to be
/// displayed on the LCD.
///////////////////////////////////////////////////////////////////////////////////
int pressure2;

// We need to store the activation status of the remote panel and track if it is on
// or off
//byte remoteActivationStatus = 0x00;
// We need to store the pad selection and track which pad(s) is currently selected
byte padSelectionStatus = 0x01;

///////////////////////////////////////////////////////////////////////////////////
/// @brief Track the on/off state of the launch button LED when the LED is
/// flashing.
///////////////////////////////////////////////////////////////////////////////////
boolean pulse = false;                    // Is the launch button LED on or off.

///////////////////////////////////////////////////////////////////////////////////
/// @brief Time since the last toggle of the LED on/off status.  Used to control
/// the flashing of the LED.
///////////////////////////////////////////////////////////////////////////////////
long timeSinceLastPulse = 0;

///////////////////////////////////////////////////////////////////////////////////
/// @brief Time since the last good Zigbee communication with the launch pad.
/// Used to monitor a timeout error.
///////////////////////////////////////////////////////////////////////////////////
long timeSinceLastPadSignal = 0;


///////////////////////////////////////////////////////////////////////////////////
/// @brief Time since the last good Zigbee communication with the Remote Panel.
/// Used to monitor a timeout error.
///////////////////////////////////////////////////////////////////////////////////
long timeSinceLastRemoteSignal = 0;


///////////////////////////////////////////////////////////////////////////////////
/// @brief Instance of the Zigbee Class.  Only one instance created.  It is used
/// to control/perform handshakes and other communications with the other Zigbee
/// devices in the Zigbee wireless network.
///////////////////////////////////////////////////////////////////////////////////
XBee xbee = XBee();

///////////////////////////////////////////////////////////////////////////////////
/// @brief Instance of the LaunchDataPacket Class.  
/// Create an instance of the LaunchDataPacket Class.  Only one instance created.
/// This class decyphers all data being passed between the Master Panel, the
/// Remote Panel, and the Launch Pad.  The single instance of the Zigbee class
/// is passed to the LaunchDataPacket so the Master Panel can begin
/// communicate with the Remote Panel and the Launch Pad.
///////////////////////////////////////////////////////////////////////////////////
LaunchDataPacket data = LaunchDataPacket(xbee);




///////////////////////////////////////////////////////////////////////////////////
/// Writes a character code to a specific location in the LCD screen to alert the
/// user that the safety is on or off.
///
/// @param onOff -boolean- TRUE = safety on, FALSE = safety off
///////////////////////////////////////////////////////////////////////////////////
void reportSafetyStatus(boolean onOff) {
  if (onOff) {
    lcd.cursorTo(1, 3);  //line=1, x=3
    lcd.printIn("S");
    return;
  }
  lcd.cursorTo(1, 3);  //line=1, x=3
  lcd.printIn(" ");
}

///////////////////////////////////////////////////////////////////////////////////
/// Writes a character code to a specific location in the LCD screen to alert the
/// user that the compressor is on or off.
///
/// @param onOff -boolean- TRUE = compressor on, FALSE = compressor off
///////////////////////////////////////////////////////////////////////////////////
void reportCompressorStatus(boolean onOff) {
  if (onOff) {
    lcd.cursorTo(1, 5);  //line=1, x=5
    lcd.printIn("C");
    return;
  }
  lcd.cursorTo(1, 5);  //line=1, x=5
  lcd.printIn(" ");
}

///////////////////////////////////////////////////////////////////////////////////
/// Writes a character code to a specific location in the LCD screen to alert the
/// user that the remote is enabled or disabled.
///
/// @param onOff -boolean- TRUE = remote enabled, FALSE = remote disabled
///////////////////////////////////////////////////////////////////////////////////
void reportRemoteEnabledStatus(boolean onOff) {
  if (onOff) {
    lcd.cursorTo(1, 7);  //line=1, x=7
    lcd.printIn("R");
    return;
  }
  lcd.cursorTo(1, 7);  //line=1, x=7
  lcd.printIn(" ");
}

///////////////////////////////////////////////////////////////////////////////////
/// Writes a character code to a specific location in the LCD screen to alert the
/// user which pad(s) has been selected.
///
/// @param padNum -byte- 0 = none, 1 = pad #1, 2 = pad #2, 3 = both pads
///////////////////////////////////////////////////////////////////////////////////
void reportPadSelect(byte padNum) {
  if (padNum == 0x01) {
    lcd.cursorTo(1, 9);  //line=1, x=9
    lcd.printIn("P1 ");
    return;
  }
  if (padNum == 0x02) {
    lcd.cursorTo(1,9);   //line=1, x=9
    lcd.printIn("P 2");
    return;
  }
  if (padNum == 0x03) {
    lcd.cursorTo(1,9);   //line=1, x=9
    lcd.printIn("P12");
    return;
  }
  lcd.cursorTo(1,9);   //line=1, x=9
  lcd.printIn("P  ");
}

///////////////////////////////////////////////////////////////////////////////////
/// Writes a character code to a specific location in the LCD screen to alert the
/// user of an error.
///
/// @param errorNum -byte- 0 = none, 1-9 = error codes 1 through 9
///////////////////////////////////////////////////////////////////////////////////
void reportError(byte errorNum) {
  if (errorNum == 0x00) {
    lcd.cursorTo(1, 13);  //line=1, x=13
    lcd.printIn("  ");
    return;
  }
  if (errorNum == 0x01) {
    lcd.cursorTo(1, 13);  //line=1, x=13
    lcd.printIn("E1");
    return;
  }
  if (errorNum == 0x02) {
    lcd.cursorTo(1, 13);  //line=1, x=13
    lcd.printIn("E2");
    return;
  }
  if (errorNum == 0x03) {
    lcd.cursorTo(1, 13);  //line=1, x=13
    lcd.printIn("E3");
    return;
  }
  if (errorNum == 0x04) {
    lcd.cursorTo(1, 13);  //line=1, x=13
    lcd.printIn("E4");
    return;
  }
  if (errorNum == 0x05) {
    lcd.cursorTo(1, 13);  //line=1, x=13
    lcd.printIn("E5");
    return;
  }
  if (errorNum == 0x06) {
    lcd.cursorTo(1, 13);  //line=1, x=13
    lcd.printIn("E6");
    return;
  }
  if (errorNum == 0x07) {
    lcd.cursorTo(1, 13);  //line=1, x=13
    lcd.printIn("E7");
    return;
  }
  if (errorNum == 0x08) {
    lcd.cursorTo(1, 13);  //line=1, x=13
    lcd.printIn("E8");
    return;
  }
  if (errorNum == 0x09) {
    lcd.cursorTo(1, 13);  //line=1, x=13
    lcd.printIn("E9");
    return;
  }
  lcd.cursorTo(1, 13);  //line=1, x=13
  lcd.printIn("E?");
}

///////////////////////////////////////////////////////////////////////////////////
/// Writes a character code to a specific location in the LCD screen to alert the
/// user that the local arm is on or off.
///
/// @param onOff -boolean- TRUE = Local Arm on, FALSE = Local Arm off
///////////////////////////////////////////////////////////////////////////////////
void reportLocalArmStatus(boolean onOff) {
  if (onOff) {
    lcd.cursorTo(2, 3);  //line=2, x=3
    lcd.printIn("LA");
    return;
  }
  lcd.cursorTo(2, 3);  //line=2, x=3
  lcd.printIn("  ");
}

///////////////////////////////////////////////////////////////////////////////////
/// Writes a character code to a specific location in the LCD screen to alert the
/// user that the remote arm is on or off.
///
/// @param onOff -boolean- TRUE = Remote Arm on, FALSE = Remote Arm off
///////////////////////////////////////////////////////////////////////////////////
void reportRemoteArmStatus(boolean onOff) {
  if (onOff) {
    lcd.cursorTo(2, 6);  //line=2, x=6
    lcd.printIn("RA");
    return;
  }
  lcd.cursorTo(2, 6);  //line=2, x=6
  lcd.printIn("  ");
}

///////////////////////////////////////////////////////////////////////////////////
/// Writes a character code to a specific location in the LCD screen to alert the 
/// user that the remote launch is on or off.
/// 
/// @param onOff -boolean- TRUE = Remote Launch on, FALSE = Remote Launch off
///////////////////////////////////////////////////////////////////////////////////
void reportRemoteLaunchStatus(boolean onOff) {
  if (onOff) {
    lcd.cursorTo(2, 9);  //line=2, x=9
    lcd.printIn("RL");
    return;
  }
  lcd.cursorTo(2, 9);  //line=2, x=9
  lcd.printIn("  ");
}

///////////////////////////////////////////////////////////////////////////////////
/// Writes a character code to a specific location in the LCD screen to alert the 
/// user that the local launch is on or off.
///
/// @param onOff -boolean- TRUE = Local Launch on, FALSE = Local Launch off
///////////////////////////////////////////////////////////////////////////////////
void reportLocalLaunchStatus(boolean onOff) {
  if (onOff) {
    lcd.cursorTo(2, 12);  //line=2, x=12
    lcd.printIn("LL");
    return;
  }
  lcd.cursorTo(1, 12);  //line=2, x=12
  lcd.printIn("  ");
}

///////////////////////////////////////////////////////////////////////////////////
/// Writes the two pressure values (in psi) as 3 digit integers specific locations 
/// in the LCD screeen to inform the user.
///
/// @param pressure1 -int- pressure #1 in psi
/// @param pressure2 -int- pressure #2 in psi
///////////////////////////////////////////////////////////////////////////////////
void reportPressure(int pressure1, int pressure2) {
  lcd.cursorTo(1, 0);  //line=1, x=0
  itoa(pressure1, number, 3);
  lcd.printIn(number);
  lcd.cursorTo(2, 0);  //line=2, x=0
  itoa(pressure2, number, 3);
  lcd.printIn(number);
}

///////////////////////////////////////////////////////////////////////////////////
/// This function is passed the ADC value.  It examines the ADC value to determine
/// which key was pressed.
///
/// @param input -unsigned int- value from analog pin ADC
///
/// @return int  -1 = no button pressed, 1 = Button #1, 2 = Button #2, 3 = Button #3
///////////////////////////////////////////////////////////////////////////////////
int get_key(unsigned int input)
{
  int k;
  for (k = 0; k < NUM_KEYS; k++) {
    if (input < adc_key_val[k]) {
      return k;
    }
  }
  if (k >= NUM_KEYS) k = -1;     // No valid key pressed
  return k;
}

///////////////////////////////////////////////////////////////////////////////////
/// Returns the TRUE/FALSE state of the safety key pin.
///
/// @return boolean - TRUE - safety key is removed, FALSE  - safety key is in.
///////////////////////////////////////////////////////////////////////////////////
boolean isSafetyOn(){
  return bitRead(_SafetyPort_, _SafetyBit_);
}

///////////////////////////////////////////////////////////////////////////////////
/// Returns the On/Off state of the arm switch.
///
/// @return boolean - TRUE - Arm switch is off, FALSE - Arm switch is on.
///////////////////////////////////////////////////////////////////////////////////
boolean isArmSwitchOff(){
  return bitRead(_ArmSwitchPort_, _ArmSwitchBit_);
}

///////////////////////////////////////////////////////////////////////////////////
/// Returns the On/Off state of the launch button.
///
/// @return boolean - TRUE - Launch button is off, FALSE - Launch button is on.
///////////////////////////////////////////////////////////////////////////////////
boolean isLaunchButtonOff(){
  return bitRead(_LaunchButtonPort_, _LaunchButtonBit_);
}

///////////////////////////////////////////////////////////////////////////////////
/// Supplies power to the Arm Switch and therefore supplies power to both the 
/// switch and the LED.  The LED dues not light up unless it has both power and 
/// the switch is in the on position.
///
/// @param state -boolean- TRUE = turn on, FALSE = turn off.
///////////////////////////////////////////////////////////////////////////////////
void setArmSwitchLedState(boolean state){
  if (state) {
    bitSet(_ArmLedPort_, _ArmLedBit_);
    return;
  }
  bitClear(_ArmLedPort_, _ArmLedBit_);
}

///////////////////////////////////////////////////////////////////////////////////
/// Turns the Launch button LED on or off.
/// 
/// @param state -boolean- TRUE = turn on, FALSE = turn off.
///////////////////////////////////////////////////////////////////////////////////
void setLaunchButtonLedState(boolean state){
  if (state) {
    bitSet(_LaunchLedPort_, _LaunchLedBit_);
    return;
  }
  bitClear(_LaunchLedPort_, _LaunchLedBit_);
}

///////////////////////////////////////////////////////////////////////////////////
/// Activates/Deactivates the Remote Panel.
///
/// @param state -boolean- TRUE = activate, FALSE = deactivate.
///////////////////////////////////////////////////////////////////////////////////
void setRemoteActivationStatus(boolean state){
  if (state) {
    data.remoteActivated();
    return;
  }
  data.remoteDeactivated();
}

///////////////////////////////////////////////////////////////////////////////////
/// Checks if there is data waiting in the Zigbee.  If data is present, it reads 
/// the data and stores it in class variables and updates the LCD display acording 
/// to the data content.
///////////////////////////////////////////////////////////////////////////////////
void readZigbee() {
  data.readDataFromXbee();
}

///////////////////////////////////////////////////////////////////////////////////
/// Checks the current state of the master controller swiches, updates the data
/// appropriate for the Remote Controller and sends a data packet to the Remote
/// Controller via Zigbee.  All variables are stored in the class instance.
///////////////////////////////////////////////////////////////////////////////////
void writeZigbeeRemote() {
  data.sendDataToRemote();
}

///////////////////////////////////////////////////////////////////////////////////
/// Checks the current state of the master controller swiches, updates the data
/// appropriate for the Pad Controller and sends a data packet to the Pad
/// Controller via Zigbee.  All variables are stored in the class instance.
///////////////////////////////////////////////////////////////////////////////////
void writeZigbeePad() {
  data.sendDataToPad();
}

///////////////////////////////////////////////////////////////////////////////////
/// Toggles the on/off state of the Launch LED bit.
///////////////////////////////////////////////////////////////////////////////////
void blinkLaunchLed() {
  if (pulse) {
    bitSet(_LaunchLedPort_, _LaunchLedBit_);
    return;
  }    
  bitClear(_LaunchLedPort_, _LaunchLedBit_);
}

///////////////////////////////////////////////////////////////////////////////////
/// Computes the pressure in psi from the ADC pressure reading and the ADC
/// calibration POT reading.  The computed pressure is rounded down to the nearest
/// integer in units of psi.
///
/// @param pressureRead - int - the ADC value from the pad.
/// @param pressureCal  - int - the ADC value from the calibation POT.
///
/// @return int - The computed pressure rounded down to the nearest integer.
///////////////////////////////////////////////////////////////////////////////////
int computePressure(int pressureRead, int pressureCal) {
  return (int) pressureRead * 30 / pressureCal;
}

///////////////////////////////////////////////////////////////////////////////////
/// Reads all switch states from the Master Panel, updates the data
/// (LaunchDataPacket instance), sends the data over Zigbee, and checks for any 
/// data waiting on the Zigbee and reads the data into the LaunchDataPacket
/// instance.
///////////////////////////////////////////////////////////////////////////////////
void readPanel() {
  // Read the calibation POTs.  Values will be used to compute the Pad pressure
  // readings in psi.
  pressureCal1 = analogRead(1);
  pressureCal2 = analogRead(2);

  // In order to blink an LED, we need to track when to turn the LED on and off.
  // pulseTime is the time since we last changed the state of the LEDs from on
  // to off or off to on.  pulse is the current state of the LEDs (on or off).
  long pulseTime = millis() - timeSinceLastPulse;
  if (pulseTime > _pulse_halfwidth_) {
    timeSinceLastPulse = millis();
    pulse = pulse ^ true;
  }

  // Check if one of the three momentary push buttons is being pressed.
  key = get_key(analogRead(0));
  if (key == 2) {  // #2 momentary push button controlls the compressor
    data.compressorOn();
  } else {
    data.compressorOff();
  }
  // if the button being pressed (key) is the same button that was pressed last
  // time we checked (oldkey), then do nothing.  If a different button is being
  // pressed, then we want to act on the button.
  if (key |= oldkey) {
    switch(key) {
      case 1: // Activate Remote Panel
        // toggle between TRUE and FALSE
        if (data.isRemoteActivated()) {
          data.remoteDeactivated();
        } else {
          data.remoteActivated();
        }
        break;
      case 3: // Select Launch Pad
        // count from 1 to 3 and loop back to 1.  3 = both pads
        padSelectionStatus = padSelectionStatus + 0x01;
        if (padSelectionStatus > 0x03) {
          padSelectionStatus = 0x01;
        }
        break;
    }
  }


  // Error if Zigbee communication timed out.
  if (data.didRemoteTimeout() || data.didPadTimeout()) { // Error
    data.remoteDeactivated();
    data.masterArmIsOff();
    data.masterLaunchClear();
    reportError(0x01);
  } else {
    reportError(0x00);

    // Calculate the two pressure values  
    pressure1 = computePressure(data.getPressure1(), pressureCal1);
    pressure2 = computePressure(data.getPressure2(), pressureCal2);
    // Display the two pressure values on the LCD display
    reportPressure(pressure1, pressure2);

    if (isSafetyOn()) { // if the safety is on, deactivate everything
      data.remoteDeactivated(); // deactivate remote panel
      data.masterArmIsOff();    // disarm the master panel
      data.masterLaunchClear(); // clear the launch command
      reportSafetyStatus(true);         // report 'safety on' on the LCD.
    } else {             // safety is not on
      reportSafetyStatus(false); // report 'safety off' on the LCD
      setArmSwitchLedState(true); // allow the system to by armed
      if (!isArmSwitchOff()) {  // if master panel arm switch is on
        data.masterArmIsOn(); // arm the master panel
        blinkLaunchLed();     // blink the Launch Pad LED
        if (data.isRemoteActivated()) { // if the remote panel is activated
          // when remote panel is activated, the master panel launch
          // button is deactivated.  Remote panel will control the
          // launch
        } else {  // the remote panel is deactivated
          if (!isLaunchButtonOff()) { // if the master panel launch button pressed
            data.masterLaunchSet(); // launch
          } else { // master panel launch button not pressed
            data.remoteLaunchClear(); // do not launch
          }
        }
      } else { // if master panel arm switch is off
        data.remoteArmIsOff();    // remote panel cannot arm the system
        data.remoteLaunchClear(); // remote panel cannot launch
        setLaunchButtonLedState(false); // master panel cannot launch
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////
/// Arduino 'setup' function.  The 'setup' function is excecuted only once each
/// time the Atmel microcontroller is powered up and it executes after the arduino
/// boot loader finishes executing. \n
/// It will initialize the digital I/O pins, the LCD screen, the serial port, and
/// the Zigbee.
///////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Setup the input and output pins
  pinMode(_SafetyBitMode_, INPUT);
  pinMode(_ArmSwitchBitMode_, INPUT);
  pinMode(_LaunchButtonBitMode_, INPUT);
  pinMode(_ArmLedBitMode_, OUTPUT);
  pinMode(_LaunchLedBitMode_, OUTPUT);

  // Turn on the pull-up resistor in the _ArmSwitchBit_ and _LaunchButtonBit_
  // Input Pins.
  bitSet(_ArmSwitchPort_, _ArmSwitchBit_);
  bitSet(_LaunchButtonPort_, _LaunchButtonBit_);

  // Initialize the pulse variable which keeps track if a flashing LED is on/off.
  pulse = false;

  // Initialize the lcd display.  
  lcd.init();

  // Initialize the serial port.
  Serial.begin(9600);

  // Initialize the Zigbee and define the Zigbee communication port as Serial
  xbee.setSerial(Serial);

  // Clear the LCD screen
  lcd.clear();

  // Print a Setup message to the LCD.  Used for debugging.
  lcd.printIn("Setup...");
}

///////////////////////////////////////////////////////////////////////////////////
/// Arduino 'loop' function.  The 'loop' function is excecuted over and over after
/// the 'setup' function finishes executing. \n
/// It will call the readPanel function in an infinite loop.
///////////////////////////////////////////////////////////////////////////////////
void loop() {
  readPanel();  
}

