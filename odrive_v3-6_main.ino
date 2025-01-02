#define CH1 8
#define CH2 9
#define CH3 10
#define CH4 11
#define CH5 12

#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj, T arg) {
  obj.print(arg);
  return obj;
}
template<> inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue) {
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the channel and return a boolean value
bool redSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

HardwareSerial& odrive_serial = Serial1;
ODriveArduino odrive(odrive_serial);

unsigned long previousMillis = 0; // Store the last time velocity was set
const long interval = 100; // Interval at which to set velocity (milliseconds)
float volt;
float gain = 0.5; // Assigning a value of 0.5
float velocity = 0.0;
float max_velocity = 4.0;

void setup() {
  odrive_serial.begin(115200);
  Serial.begin(19200);

  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);

  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ": Requesting state " << AXIS_STATE_CLOSED_LOOP_CONTROL << '\n';
    if (!odrive.run_state(axis, AXIS_STATE_CLOSED_LOOP_CONTROL, true /*don't wait*/)) return;
  }
}

void loop() {
  int ch1Value = readChannel(CH1, -100, 100, 0);
  int ch2Value = readChannel(CH2, -100, 100, 0);
  bool ch3Value = redSwitch(CH3, false);
  bool ch4Value = redSwitch(CH4, false);
  bool ch5Value = redSwitch(CH5, false);

  if (ch3Value) { // if there is a value
    Serial.print("on idle ");
    int requested_state = AXIS_STATE_IDLE;
    Serial << "Axis" << 0 << ": Requesting state " << AXIS_STATE_IDLE << '\n';
    if (!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
    Serial << "Axis" << 1 << ": Requesting state " << AXIS_STATE_IDLE << '\n';
    if (!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
    ch3Value = false; // Reset ch3Value after processing
  } else {
    Serial.println(" err ");
  }

  // Non-blocking while loop for ch3Value and ch1Value condition
  if (!ch3Value && (ch1Value > 20 && (ch1Value < 50))) { // Corrected syntax here
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis; // Save the last time velocity was set
      Serial.println("ch3 vel .5");
      odrive.SetVelocity(0, 1, 1);
      delay(10);

    }
    else if (!ch3Value && (ch1Value > 50)) {
      Serial.println("ch3 vel 2");
      odrive.SetVelocity(0, 2, 2);
      delay(10);
    }
    else {
      Serial.println("ch3 vel 2");
      odrive.SetVelocity(0, 0, 0);
      delay(10);


    }
  }

  // Read bus voltage
  if (ch4Value) {
    odrive_serial << "r vbus_voltage\n";
    Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
    delay(1000); // This delay can remain as it is for reading voltage
  }
}


///////////////////  VOID    COMMANDS //////////////////////////
//////////////////////////////////////////////////////////////
void leanForward() {                  //Function when leaning forward
  if (velocity < max_velocity) {      //Only increase velocity if it's not already max
    velocity = velocity + gain;       //Increase velocity
    odrive.SetVelocity(0, velocity);  //Send command to ODrive with the new velocity
  }
}
void leanBackward() {                 //Function when leaning backward
  if (velocity > -max_velocity) {     //Only reduce velocity if it's not already max
    velocity = velocity - gain;       //Reduce velocity
    odrive.SetVelocity(0, velocity);  //Send command to ODrive with the new velocity
  }
}

void standingUpright() {
  if (velocity > 0 && velocity < 1) {  //If going forward slowly
    leanBackward();                      //Call function leanBackward to stop
    delay(100);                          //100ms delay so as to not get too jerky movements
  }
  if (velocity < 0 && velocity > -1) {  //If reversing slowly
    leanForward();                        //Call function leanForward to stop
    delay(100);                           //100ms delay so as to not get too jerky movements
  }
}

void checkVoltage() {                       //Function to check voltage supplied to ODrive
  Serial.flush();                           //Waits for outgoing serial buffer to empty
  while (Serial.available()) {              //This while loop waits for the incoming buffer to empty,
    Serial.read();                          // otherwise problems reading voltage comes due to Arduino Uno's
  }                                         // instability of using SoftwareSerial at this high baud-rate
  odrive_serial.write("r vbus_voltage\n");  //Read voltage supplied to ODrive and write to serial
  volt = odrive.readFloat();                //Assign to float
}

void batteryLow() {          //Function to slow down when battery is low
  while (velocity > 3.0) {  //If going forwards quickly, slow down to 1000counts/s
    leanBackward();          //Slowing down
    delay(200);              //Delay 200ms each iteration, ramp down slowly
  }
  while (velocity < -3.0) {  //If reversing quickly, slow down to 1000counts/s
    leanForward();            //Slowing down
    delay(200);               //Delay 200ms each iteration, ramp down slowly
  }
  max_velocity = 1.5;  //Set new top speed to 1000 counts/s
}
