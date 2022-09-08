
Proteus Library

1. Arduino Board Library for Proteus
2. Genuino Proteus Library
3. GPS Library for Proteus
4. GSM Library for Proteus
5. Bluetooth Library for Proteus
6. XBee Proteus Library
7. Real-Time clock DS1307 Proteus Library
8. LCD Library for Proteus
9. Arduino Ultrasonic Sensor HC-SR04 Proteus Library
10. PIR Motion Sensor (HC-SR501) Library for Proteus
11. Gas Sensor Library for Proteus
12. Flame Sensor Library for Proteus
13. Proteus Vibration Sensor Library for Arduino projects
14. Motor Driver Library for Proteus
15. Infrared Sensor Library for Proteus simulation
16. Solar Panel Proteus Library
17. Magnetic Reed Switches Proteus Library
18. Proteus Rain Sensor Library for Arduino Projects
19. Proteus Optocoupler Library
20. NodeMCU Design Library for Proteus
21. Flex Sensor Library for Proteus
22. Heart Beat Sensor Library for Proteus

Table of Contents

1.	HC-SR04 Ultrasonic Sensor

2.	IR Infrared Obstacle Avoidance Sensor

3.	Solid Hygrometer Detection Module

4.	Soil Moisture Sensor

5.	Microphone Sensor

6.	Digital Barometric Pressure Sensor

7.	Photoresistor Sensor

8.	Digital Thermal Sensor-Temperature Sensor

9.	Rotary Encoder Module

10.	MQ-2 Gas Sensor

11.	SW-420 Motion Sensor

12.	Humidity and Rain Detection Sensor

13.	Passive Buzzer Module

14.	Speed Sensor Module

15.	IR Infrared Flame Detection Sensor

16.	5V 2-Channel Relay Module

17.	Breadboard Power Supply Module 3.3v

18.	HC-SR501 Pyroelectric Infrared Sensor

19.	Accelerometer Module

20.	DHT11 Temperature and Humidity Sensor

21.	RF 433MHZ Transmitter/Receiver

22.	Bluetooth Module

                                                    
# HC-SR04 Ultrasonic Sensor

Ultrasonic sensor can detect movement of targets and measure the distance to them in many automated factories and process plants.
 
Ultrasonic ranging module HC - SR04 provides 2cm - 400cm non-contact measurement function, the ranging accuracy can reach to 3mm. The modules includes ultrasonic transmitters, receiver and control circuit. The basic principle of work: (1) Using IO trigger for at least 10us high level signal, (2) The Module automatically sends eight 40 kHz and detect whether there is a pulse signal back. (3) IF the signal back, through high level , time of high output IO duration is the time from sending ultrasonic to returning. Test distance = (high level time×velocity of sound (340M/S) / 2,

## Wire connecting direct as following: 

•	5V Supply  
•	Trigger Pulse Input  
•	Echo Pulse Output  
•	0V Ground
	            	

Connection pip with Arduino          	

Pin	Arduino UNO
Trigger	D9
Echo	D12
VCC	5V
GND	GND

9
Schematic
 Connecting  Ultrasonic Sensor an the Arduino:
 
         


Pin Wiring		Arduino UNO
1st pin	VCC 5V
2nd pin	Pin 11
3rd pin	Pin 12
4th  pin	GND

Source code
Below you can find the code you need for this project. But first you need to install
the Ultrasonic Sensor library.

1. Download the Ultrasonic library here
2. Unzip the Ultrasonic library
3. Rename the extracted folder to Ultrasonic Sensore and remove the “-“. Otherwise your
Arduino IDE won’t recognize your library
4. Install the Ultrasonic in your Arduino IDE: go to Sketch  Include Library Add .
ZIP library and select the library you’ve just downloaded
5. Restart your Arduino IDE
6. Go to File ExamplesUltrasonic sensor library Ultrasonic tester
7. Upload the code
/*
 * created by Tec Santosh Kumar, http://santosh.code.blog 
 * Complete Guide for Ultrasonic Sensor HC-SR04

    *Ultrasonic sensor Pins:
        VCC: +5VDC
        Trig : Trigger (INPUT) - Pin11
        Echo: Echo (OUTPUT) - Pin 12
        GND: GND
 */
int trig = 11;    //Trig - green Jumper
int echo = 12;    //Echo - yellow Jumper
void setup() {
  //Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}
void loop()
{
 
 
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // convert the time into a distance
  cm = (duration/2) / 29.1;
  inches = (duration/2) / 74; 
  
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  
  delay(250);
}


                                            
                   IR Receiver Sensor
IR communication makes use of IR (Infrared) waves from the electromagnetic spectrum.
An IR LED is used to transmit data wirelessly in digital form (0 – LED OFF or 1 – LED ON).
An IR photodiode or IR phototransistor receives this data. The IR receiver (IR photodiode or IR phototransistor) gives different current values according to the intensity of light.
It is possible to modulate the data transmitted and there are special decoder IR receivers like TSOP1738 available that can receive the modulated data.
IR Receiver Modules for Remote Control Systems
An IR remote and receiver communicate with each other by transmitting and decoding a signal in the form of pulsed IR radiation.
                                
                                                      Sent and detected signal by IR transmitter and receiver
Infrared radiation (IR), or infrared light, is a type of electromanetic radiation with wavelengths ranting from 700 nm to 1mm. Because humans can only see light with wavelengths of roughly 400 (violet) to 700 (red) nanometers, IR radiation is invisible to the human eye.
 
                                        Electromagnetic spectrum with visible light highlighted 

Since IR transmission is a wireless protocol based on a type of light, it requires a clear line of sight between the transmitted (the remote) and the receiver. This means it can’t transmit through walls or ceilings, unlike WiFi or Bluetooth.
     Types of IR receiver
 IR receiver, sometimes called IR sensors or IR detection diodes, usually come in two different form factors. You can either buy the diodes separately or mounted on a smoll breakout board.
                                                   
                            IR receiver diode(left) and receiver mounted on a breakout board (right)
They work exactly the same, so it doesn’t matter which one you use. The only difference is that the breakout board often contains a small LED that blinks every time the receiver detecs a single which can be handy for debugging.
Where to buy?
Click the links below to compare the sensor at different stores and find the best price:
• Click here to see IR Sensor on Maker Advisor

             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
  IR Receiver Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic

Connecting IR Receiver Sensor an the Arduino
	

          
 

                 IR Receiver Connections

                       IR receiver	               Arduino UNO
                   OUT Pin	                        Pin 2
                    GND Pin	                        GND
                    VCC Pin	                         5v
            Installing the IR receiver Arduino library

We will be using the popular IR receiver library written by tec santosh kumar. This library is fairly easy to and supports many different type of remotes. You can find the source code of this library here on GitHub.
To install the library, go to Tools > Manager Libraries (Ctrl + Shift+ I on Windows) in the Arduino IDE. The Library manager will open and update the list of installed libraries.

     

                         Key	                          Code
   Power	0xFD00FF
VOL+	0xFD807F
FUNC/STOP	0xFD40BF
|<<	0xFD20DF
>||	0xFDA05F
>>|	0xFD609F
	0xFD10EF
	0xFD50AF
VOL-	0xFD906F
0	0xFD30CF
EQ	0xFDB04F
ST/REPT	0xFD708FG
1	0xFD8F7
2	0xFDD8877
3	0xFD48B7
4	0xFD28D7
5	0xFDA957
6	0xFD6897
7	0xFD18E7
8	0xFD9867
9	0xFD58A7

1.	/* IR remote and receiver Arduino example code. Print key values in the Serial Monitor. More info: https://www.makerguides.com */
2.	#include <IRremote.h> // include the IRremote library
3.	#define RECEIVER_PIN 2 // define the IR receiver pin
4.	IRrecv receiver(RECEIVER_PIN); // create a receiver object of the IRrecv class
5.	decode_results results; // create a results object of the decode_results class
6.	unsigned long key_value = 0; // variable to store the key value
7.	void setup() {
8.	Serial.begin(9600); // begin serial communication with a baud rate of 9600
9.	receiver.enableIRIn(); // enable the receiver
10.	receiver.blink13(true); // enable blinking of the built-in LED when an IR signal is received }
11.	void loop() {
12.	if (receiver.decode(&results)) { // decode the received signal and store it in results
13.	if (results.value == 0xFFFFFFFF) { // if the value is equal to 0xFFFFFFFF
14.	results.value = key_value; // set the value to the key value    }
15.	switch (results.value) { // compare the value to the following cases
16.	\case 0xFD00FF: // if the value is equal to 0xFD00FF
17.	Serial.println("POWER"); // print "POWER" in the Serial Monitor
18.	 break;
19.	case 0xFD807F:
20.	Serial.println("VOL+");
21.	 break;
22.	case 0xFD40BF:
23.	Serial.println("FUNC/STOP");
        break;
      case 0xFD20DF:
Serial.println("|<<");
        break;
      case 0xFDA05F:
Serial.println(">||");
        break ;
      case 0xFD609F:
Serial.println(">>|");
        break ;
      case 0xFD10EF:
Serial.println("DOWN");
        break ;
      case 0xFD906F:
Serial.println("VOL-");
        break ;
      case 0xFD50AF:
Serial.println("UP");
        break ;
      case 0xFD30CF:
Serial.println("0");
        break ;
      case 0xFDB04F:
Serial.println("EQ");
        break ;
      case 0xFD708F:
Serial.println("ST/REPT");
        break ;
      case 0xFD08F7:
Serial.println("1");
        break ;
      case 0xFD8877:
Serial.println("2");
        break ;
      case 0xFD48B7:
Serial.println("3");
        break ;
      case 0xFD28D7:
Serial.println("4");
        break ;
      case 0xFDA857:
Serial.println("5");
        break ;
      case 0xFD6897:
Serial.println("6");
        break ;
      case 0xFD18E7:
Serial.println("7");
        break ;
      case 0xFD9867:
Serial.println("8");
        break ;
      case 0xFD58A7:
Serial.println("9");
        break ;
    }
key_value = results.value; // store the value as key_value
receiver.resume(); // reset the receiver for the next code
  }
}

                                                                                      
                                                                   Analog Joystick

Analog joystick produces two voltages; one corresponding to position with respect to X-axis and another corresponding to the position with respect to Y-axis. The voltages produced depend on the position of the joystick
	Where to buy?
Click the links below to compare the sensor at different stores and find the best price:
• Click here to see DHT11 on Maker Advi

             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
  Analog Joystick Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	



Schematic

Connecting IR Receiver Sensor an the Arduino

.                                              
Code

#include "Mouse.h"

// set pin numbers for switch, joystick axes, and LED:
const int switchPin = 2;      // switch to turn on and off mouse control
const int mouseButton = 3;    // input pin for the mouse pushButton
const int xAxis = A0;         // joystick X axis
const int yAxis = A1;         // joystick Y axis
const int ledPin = 5;         // Mouse control LED

// parameters for reading the joystick:
int range = 12;               // output range of X or Y movement
int responseDelay = 5;        // response delay of the mouse, in ms
int threshold = range / 4;    // resting threshold
int center = range / 2;       // resting position value

bool mouseIsActive = false;    // whether or not to control the mouse
int lastSwitchState = LOW;        // previous switch state

void setup() {
  pinMode(switchPin, INPUT);       // the switch pin
  pinMode(ledPin, OUTPUT);         // the LED pin
  // take control of the mouse:
  Mouse.begin();
}

void loop() {
  // read the switch:
  int switchState = digitalRead(switchPin);
  // if it's changed and it's high, toggle the mouse state:
  if (switchState != lastSwitchState) {
    if (switchState == HIGH) {
      mouseIsActive = !mouseIsActive;
      // turn on LED to indicate mouse state:
      digitalWrite(ledPin, mouseIsActive);
    }
  }
  // save switch state for next comparison:
  lastSwitchState = switchState;

  // read and scale the two axes:
  int xReading = readAxis(A0);
  int yReading = readAxis(A1);

  // if the mouse control state is active, move the mouse:
  if (mouseIsActive) {
    Mouse.move(xReading, yReading, 0);
  }

  // read the mouse button and click or not click:
  // if the mouse button is pressed:
  if (digitalRead(mouseButton) == HIGH) {
    // if the mouse is not pressed, press it:
    if (!Mouse.isPressed(MOUSE_LEFT)) {
      Mouse.press(MOUSE_LEFT);
    }
  }
  // else the mouse button is not pressed:
  else {
    // if the mouse is pressed, release it:
    if (Mouse.isPressed(MOUSE_LEFT)) {
      Mouse.release(MOUSE_LEFT);
    }
  }

  delay(responseDelay);
}

/*
  reads an axis (0 or 1 for x or y) and scales the analog input range to a range
  from 0 to <range>
*/

int readAxis(int thisAxis) {
  // read the analog input:
  int reading = analogRead(thisAxis);

  // map the reading from the analog input range to the output range:
  reading = map(reading, 0, 1023, 0, range);

  // if the output reading is outside from the rest position threshold, use it:
  int distance = reading - center;

  if (abs(distance) < threshold) {
    distance = 0;
  }

  // return the distance for this axis:
  return distance;
}

               Sketch for Displying Data Received Via Bluetooth On Serial Monitor


                        
                                                                                      
                                             LM35 Temperature Sensor

 LM35 is a temperature sensore which can measure temperature in the range of -55oC to 150oC.
 It is a 3-terminal device that provides anlog voltage propotional to the temperature. Higher the temperature, higher is the output voltage.
The output analogvoltage can be converted to digital from using ADC so that a microcontroller camn process it.
	Where to buy?
Click the links below to compare the sensor at different stores and find the best price:
• Click here to see DHT11 on Maker Advisor

             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
 Temperature Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

   Arduino Sensor LM35 Temperature Sensor

    LM35 Connections
                       LM35	               Arduino UNO
                   OUT Pin	                        A1
                    GND Pin	                        GND
                    VCC Pin	                         5v

   Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:

                                            

                                     Sketch for Displying Data LM35 On Serial Monitor







                                      

                                      

	                          
                                                               HC-05 Bluetooth Module

HC-05 is a Bluetooth device used for wireless communication with Bluetooth enabled devices (like smartphone). It communication with microcontrollers using serial communication (USART)
As HC-05 Bluetooth module has 3.3 V level for RX/TX and microcontroller can detect 3.3 V level, So, there is no need to shift TX voltage level of  HC-05 module. But we need to shift the transmit voltage level from microcontroller to RX of HC-05 module.

	Where to buy?
Click the links below to compare the sensor at different stores and find the best price:
• Click here to see DHT11 on Maker Advisor

             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
 Bluetooth Sensor Module	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an HC-05 Bluetooth Module Sensor the Arduino:

                                          





	
                    Sketch for Displying Data Received Via Bluetooth On Serial Monitor







                                                     

      


          
                                               




                                           

                

                                                                                  
                                                                           4x4 Keyped

Keyped is used as an input device to read the key pressed by and to process it.
4x4 keypad consists of 4 rows and 4 columns. Switches are placed between the rows and columns.
A key press establishes a connection between the corresponding row and column, between which the switch is placed.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
   4x4 keyped Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an 4x4 keyped Sensor the Arduino:
                                
                   

Sketch for 4x4 Keypad
#include <Keypad.h>
const byte ROWS = 4; /* four rows */
const byte COLS = 4; /* four columns */
/* define the symbols on the buttons of the keypads */
char hexaKeys[ROWS][COLS] = {
  {'0','1','2','3'},
  {'4','5','6','7'},
  {'8','9','A','B'},
  {'C','D','E','F'}
};
byte rowPins[ROWS] = {10, 11, 12, 13}; /* connect to the row pinouts of the keypad */
byte colPins[COLS] = {6, 7, 8, 9}; /* connect to the column pinouts of the keypad */

/* initialize an instance of class NewKeypad */
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

void setup(){
Serial.begin(9600);
}

void loop(){
  char customKey = customKeypad.getKey();

  if (customKey){
Serial.println(customKey);
  }
}
 
Sketch
void setup() {
  Serial.begin(9600);	/* Define baud rate for serial communication */
}

void loop() {
  int adc_val;
  adc_val = analogRead(A1);	/* Read input from keypad */
  if (adc_val>850)
  {
    Serial.print("Key Pressed : ");
    Serial.println("0");
    delay(100);
  }
  else if ( adc_val>450  && adc_val<510)
  {
    Serial.print("Key Pressed : ");
    Serial.println("1");
    delay(100);
  }
  else if ( adc_val>300  && adc_val<350)
  {
    Serial.print("Key Pressed : ");
    Serial.println("2");
    delay(100);
  }
  else if ( adc_val>230  && adc_val<270)
  {
    Serial.print("Key Pressed : ");
    Serial.println("3");
    delay(100);
  }
  else if ( adc_val>160  && adc_val<180)
  {
    Serial.print("Key Pressed : ");
    Serial.println("4");
    delay(100);
  }
  else if ( adc_val>145  && adc_val<155)
  {
    Serial.print("Key Pressed : ");
    Serial.println("5");
    delay(100);
  }
  else if ( adc_val>125  && adc_val<135)
  {
    Serial.print("Key Pressed : ");
    Serial.println("6");
    delay(100);
  }
  else if ( adc_val>105  && adc_val<120)
  {
    Serial.print("Key Pressed : ");
    Serial.println("7");
    delay(100);
  }
  else if ( adc_val>92  && adc_val<99)
  {
    Serial.print("Key Pressed : ");
    Serial.println("8");
    delay(100);
  }
  else if ( adc_val>82  && adc_val<90)
  {
    Serial.print("Key Pressed : ");
    Serial.println("9");
    delay(100);
  }
  else if ( adc_val>77  && adc_val<81)
  {
    Serial.print("Key Pressed : ");
    Serial.println("A");
    delay(100);
  }
  else if ( adc_val>72  && adc_val<76)
  {
    Serial.print("Key Pressed : ");
    Serial.println("B");
    delay(100);
  }
  else if ( adc_val>63  && adc_val<68)
  {
    Serial.print("Key Pressed : ");
    Serial.println("C");
    delay(100);
  }
  else if ( adc_val>60  && adc_val<62)
  {
    Serial.print("Key Pressed : ");
    Serial.println("D");
    delay(100);
  }
  else if ( adc_val>57  && adc_val<59)
  {
    Serial.print("Key Pressed : ");
    Serial.println("E");
    delay(100);
  }
  else if( adc_val>52  && adc_val<56)
  {
    Serial.print("Key Pressed : ");
    Serial.println("F");
    delay(100);
  }
  else
  {
	  
  }
  delay(100);
}

                                                                                      
	                                            Accelerometer 

 Accelerometer is an electromechanical device that measures the force of acceleration due to gravity in g unit.
It can be used in applications requiring tilt sensing.
The ADXL335 measures acceleration along X, Y and Z axes and gives analog voltage output proportional to the acceleration along these 3 axes.
Microcontrollers can process these voltages by converting them to digital signals using ADC.
Schematic

	Where to buy?
Click the links below to compare the sensor at different stores and find the best price:
• Click here to see DHT11 on Maker Advisor

             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
  Accelerometer Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an the Accelerometer Sensor an  Arduino:

                                     
Sketch
#include <math.h>
const int x_out = A1; /* connect x_out of module to A1 of UNO board */
const int y_out = A2; /* connect y_out of module to A2 of UNO board */
const int z_out = A3; /* connect z_out of module to A3 of UNO board */

void setup() {
  Serial.begin(9600); 
}

void loop() {
  int x_adc_value, y_adc_value, z_adc_value; 
  double x_g_value, y_g_value, z_g_value;
  double roll, pitch, yaw;
  x_adc_value = analogRead(x_out); /* Digital value of voltage on x_out pin */ 
  y_adc_value = analogRead(y_out); /* Digital value of voltage on y_out pin */ 
  z_adc_value = analogRead(z_out); /* Digital value of voltage on z_out pin */ 
  Serial.print("x = ");
  Serial.print(x_adc_value);
  Serial.print("\t\t");
  Serial.print("y = ");
  Serial.print(y_adc_value);
  Serial.print("\t\t");
  Serial.print("z = ");
  Serial.print(z_adc_value);
  Serial.print("\t\t");
  //delay(100);
  
  x_g_value = ( ( ( (double)(x_adc_value * 5)/1024) - 1.65 ) / 0.330 ); /* Acceleration in x-direction in g units */ 
  y_g_value = ( ( ( (double)(y_adc_value * 5)/1024) - 1.65 ) / 0.330 ); /* Acceleration in y-direction in g units */ 
  z_g_value = ( ( ( (double)(z_adc_value * 5)/1024) - 1.80 ) / 0.330 ); /* Acceleration in z-direction in g units */ 

  roll = ( ( (atan2(y_g_value,z_g_value) * 180) / 3.14 ) + 180 ); /* Formula for roll */
  pitch = ( ( (atan2(z_g_value,x_g_value) * 180) / 3.14 ) + 180 ); /* Formula for pitch */
  //yaw = ( ( (atan2(x_g_value,y_g_value) * 180) / 3.14 ) + 180 ); /* Formula for yaw */
  /* Not possible to measure yaw using accelerometer. Gyroscope must be used if yaw is also required */

  Serial.print("Roll = ");
  Serial.print(roll);
  Serial.print("\t");
  Serial.print("Pitch = ");
  Serial.print(pitch);
  Serial.print("\n\n");
  delay(1000);
}




HMC5883L Magnetometer Module
 
Magnetometer HMC5883L measures the direction and magnitude of the Earth’s magnetic field and hence is used for low cost compassing and magnetometry.
It measures the Earth’s magnetic field value along the X, Y and Z axes from milli-gauss to 8 gauss.
It can be used as a compass to find direction or to find the direction of heading of a device.
It makes use of I2C protocol for communication with a microcontroller. Earth’s magnetic field values along the X, Y and Z axes can be found by reading values from addresses of certain registers using I2C communication.

             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
    Magnetometer Sensor	           



       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Interfacing Diagram
  
Interfacing HMC5883L Magnetometer Module With Arduino UNO
 
Sketch
/*

#include <Arduino.h>
#include <Wire.h>
#include <HMC5883L_Simple.h>

// Create a compass
HMC5883L_Simple Compass;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
    
  // Magnetic Declination is the correction applied according to your present location
  // in order to get True North from Magnetic North, it varies from place to place.
  // 
  // The declination for your area can be obtained from http://www.magnetic-declination.com/
  // Take the "Magnetic Declination" line that it gives you in the information, 
  //
  // Examples:
  //   Christchurch, 23° 35' EAST
  //   Wellington  , 22° 14' EAST
  //   Dunedin     , 25° 8'  EAST
  //   Auckland    , 19° 30' EAST
  //    
  Compass.SetDeclination(-0, 23, 'W');  
  
  // The device can operate in SINGLE (default) or CONTINUOUS mode
  //   SINGLE simply means that it takes a reading when you request one
  //   CONTINUOUS means that it is always taking readings
  // for most purposes, SINGLE is what you want.
  Compass.SetSamplingMode(COMPASS_SINGLE);
  
  // The scale can be adjusted to one of several levels, you can probably leave it at the default.
  // Essentially this controls how sensitive the device is.
  //   Options are 088, 130 (default), 190, 250, 400, 470, 560, 810
  // Specify the option as COMPASS_SCALE_xxx
  // Lower values are more sensitive, higher values are less sensitive.
  // The default is probably just fine, it works for me.  If it seems very noisy
  // (jumping around), incrase the scale to a higher one.
  Compass.SetScale(COMPASS_SCALE_130);
  
  // The compass has 3 axes, but two of them must be close to parallel to the earth's surface to read it, 
  // (we do not compensate for tilt, that's a complicated thing) - just like a real compass has a floating 
  // needle you can imagine the digital compass does too.
  //
  // To allow you to mount the compass in different ways you can specify the orientation:
  //   COMPASS_HORIZONTAL_X_NORTH (default), the compass is oriented horizontally, top-side up. when pointing North the X silkscreen arrow will point North
  //   COMPASS_HORIZONTAL_Y_NORTH, top-side up, Y is the needle,when pointing North the Y silkscreen arrow will point North
  //   COMPASS_VERTICAL_X_EAST,    vertically mounted (tall) looking at the top side, when facing North the X silkscreen arrow will point East
  //   COMPASS_VERTICAL_Y_WEST,    vertically mounted (wide) looking at the top side, when facing North the Y silkscreen arrow will point West  
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
  
}

// Our main program loop.
void loop()
{
   float heading = Compass.GetHeadingDegrees();
   
   Serial.print("Heading: \t");
   Serial.println( heading );   
   delay(1000);
}
	
 
PIR Sensor
 
PIR sensor is used for detecting infrared heat radiations. This makes them useful in applications involving detection of moving living objects that emit infrared heat radiations.
The output (in terms of voltage) of PIR sensor is high when it senses motion; whereas it is low when there is no motion (stationary object or no object).
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
    PIR Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Wiring Pin
Pin	Arduino UNO
5V 	5V
OUT	Arduni Digital pin
GND	GND

Schematic
 Connecting an PIR Sensor an the Arduino:

 
 
Code
/*Santosh Kumar*/
const int PIR_SENSOR_OUTPUT_PIN = 2;  /* PIR sensor O/P pin */
void setup() {
  pinMode(PIR_SENSOR_OUTPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), pir, FALLING);  /* Interrupt on rising edge on pin 2 */
  Serial.begin(9600); /* Define baud rate for serial communication */
  delay(20000); /* Power On Warm Up Delay */
}
void loop() {
}
void pir(){
  Serial.println("Object Detected");
}



   
DHT22/DHT11 Sensor
•	DHT11 sensor measures and provides humidity and temperature values serially over a single wire.
•	It can measure relative humidity in percentage (20 to 90% RH) and temperature in degree Celsius in the range of 0 to 50°C.
•	It has 4 pins; one of which is used for data communication in serial form.
•	Pulses of different TON and TOFF are decoded as logic 1 or logic 0 or start pulse or end of a frame.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
   DHT11 Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

       
                                                                     Pin Wiring to Arduino UNO

Pin Wiring	 Arduino UNO
1st pin	VCC 5V
2nd pin	Data OUT Digital pin 2
3rd pin 	Don’t Connect
4th pin	GND

Schematic
 Connecting an DHT11 Sensor an the Arduino:
 
 
 
Sketch For Reading Temperature And Humidity From DHT11

#include "DHT.h"
DHT dht;
void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)");
  dht.setup(2); // data pin 2
}
void loop()
{
  delay(dht.getMinimumSamplingPeriod());
  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();
  Serial.print(dht.getStatusString());
  Serial.print("\t");
  Serial.print(humidity, 1);
  Serial.print("\t\t");
  Serial.print(temperature, 1);
  Serial.print("\t\t");
  Serial.println(dht.toFahrenheit(temperature), 1);
}


 
DS1307 RTC Module

•	Real Time Clock (RTC) is used for monitoring time and maintaining a calendar.
•	In order to use an RTC, we need to first program it with the current date and time. Once this is done, the RTC registers can be read any time to know the time and date.
•	DS1307 is an RTC which works on I2C protocol. Data from various registers can be read by accessing their addresses for reading using I2C communication. 

             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
 Ds1307 RTC Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	


Schematic
 Connecting an Real Time Clock Sensor an the Arduino:
 
Interfacing DS1307 RTC Module With Arduino UNO
 
Sketch For Setting And Reading Time And Date In DS1307 using Arduino
/*
  D

#include <Wire.h>
#include <DS1307.h>
DS1307 rtc;
void setup()
{
  /*init Serial port*/
  Serial.begin(9600);
  while(!Serial); /*wait for serial port to connect - needed for Leonardo only*/
  /*init RTC*/
  Serial.println("Init RTC...");
  /*only set the date+time one time*/
  rtc.set(0, 0, 8, 24, 12, 2014); /*08:00:00 24.12.2014 //sec, min, hour, day, month, year*/
  /*stop/pause RTC*/
  // rtc.stop();

  /*start RTC*/
  rtc.start();
}
void loop()
{
  uint8_t sec, min, hour, day, month;
  uint16_t year;
  /*get time from RTC*/

  rtc.get(&sec, &min, &hour, &day, &month, &year);
  /*serial output*/

  Serial.print("\nTime: ");
  Serial.print(hour, DEC);
  Serial.print(":");
  Serial.print(min, DEC);
  Serial.print(":");
  Serial.print(sec, DEC);

  Serial.print("\nDate: ");
  Serial.print(day, DEC);
  Serial.print(".");
  Serial.print(month, DEC);
  Serial.print(".");
  Serial.print(year, DEC);

  /*wait a second*/
  delay(1000);
}

                                             
   Soil Moisture Sensor
 
Soil moisture is basically the content of water present in the soil. This can be measured using a soil moisture sensor which consists of two conducting probes that act as a probe. It can measure the moisture content in the soil based on the change in resistance between the two conducting plates.
The resistance between the two conducting plates varies in an inverse manner with the amount of moisture present in the soil.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
    Soil Moisture Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	
Pin Wiring

Pin	Arduino UNO
A0	A5
Do	Digitol
GND	GND
VCC	5V

Schematic
 Connecting an LM35 Soil Moisture Sensor an the Arduino:

            


Interfacing Soil Moisture Sensor With Arduino UNO
Measuring soil moisture in terms of percentage.

Here, the analog output of soil moisture sensor is processed using ADC. The moisture content in terms of percentage is displayed on the serial monitor.
The output of the soil moisture sensor changes in the range of ADC value from 0 to 1023.
This can be represented as moisture value in terms of percentage using formula given below.
 
Moisture in percentage = 100 – (Analog output * 100)
For zero moisture, we get maximum value of 10-bit ADC, i.e. 1023. This, in turn, gives 0% moisture.

 
Sketch
const int sensor_pin = A1;	/* Soil moisture sensor O/P pin */

void setup() {
  Serial.begin(9600);	/* Define baud rate for serial communication */
}

void loop() {
  float moisture_percentage;
  int sensor_analog;
  sensor_analog = analogRead(sensor_pin);
  moisture_percentage = ( 100 - ( (sensor_analog/1023.00) * 100 ) );
  Serial.print("Moisture Percentage = ");
  Serial.print(moisture_percentage);
  Serial.print("%\n\n");
  delay(1000);
}
       
               
Thermistor is a variable resistance element, whose resistance varies with change in temperature. The change in resistance value is a measure of the temperature.
Thermistors are classified as PTC (Positive Temperature Coefficient) or NTC (Negative Temperature Coefficient).
They can be used as current limiters, temperature sensors, overcurrent protectors etc.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
    Ultrasonic Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:
                 Interfacing Thermistor With Arduino UNO
 
Example
Measuring temperature using thermistor.
 
Here, an NTC type thermistor of 10kΩ is used. NTC of 10kΩ means that this thermistor has a resistance of 10kΩ at 25°C.
Voltage across the 10kΩ resistor is given to the ADC of UNO board.
The thermistor resistance is found out using simple voltage divider network formula.
 
Rth is the resistance of thermistor
   
Vout is the voltage measured by the ADC
  
The temperature can be found out from thermistor resistance using the Steinhart-Hart equation.
Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3) 
where A = 0.001129148, B = 0.000234125 and C = 8.76741*10^-8
and R is the thermistor resistance.

Sketch For Temperature Measurement Using Thermistor
#include <math.h>
const int thermistor_output = A1;

void setup() {
  Serial.begin(9600);	/* Define baud rate for serial communication */
}

void loop() {
  int thermistor_adc_val;
  double output_voltage, thermistor_resistance, therm_res_ln, temperature; 
  thermistor_adc_val = analogRead(thermistor_output);
  output_voltage = ( (thermistor_adc_val * 5.0) / 1023.0 );
  thermistor_resistance = ( ( 5 * ( 10.0 / output_voltage ) ) - 10 ); /* Resistance in kilo ohms */
  thermistor_resistance = thermistor_resistance * 1000 ; /* Resistance in ohms   */
  therm_res_ln = log(thermistor_resistance);
  /*  Steinhart-Hart Thermistor Equation: */
  /*  Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3)   */
  /*  where A = 0.001129148, B = 0.000234125 and C = 8.76741*10^-8  */
  temperature = ( 1 / ( 0.001129148 + ( 0.000234125 * therm_res_ln ) + ( 0.0000000876741 * therm_res_ln * therm_res_ln * therm_res_ln ) ) ); /* Temperature in Kelvin */
  temperature = temperature - 273.15; /* Temperature in degree Celsius */
  Serial.print("Temperature in degree Celsius = ");
  Serial.print(temperature);
  Serial.print("\t\t");
  Serial.print("Resistance in ohms = ");
  Serial.print(thermistor_resistance);
  Serial.print("\n\n");
  delay(1000);
}
 
                                                                  
                          Force Sensing Resistor Sensor

FSRs(Force Sensing Resistor sensor) are super robust pressure sensors that are used in all kinds of industries. You will find them in electronic drums, mobile phones, handheld gaming devices and many more portable electronics. These sensors are easy to use and great for sensing pressure
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
 Force Sensing Resistor Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an Force Sensing Resing Sensor an the Arduino:

     
 multichannel, 10-bit analog to digital converter. This means that it will map the input voltage between 0 and 5 V into integer values between 0 and 1023. So you should see a value between 0 and 1023 in the serial monitor, depending on how hard you squeeze the sensor.
/* Simple example code for Force Sensitive Resistor (FSR) with Arduino. More info: https://www.makerguides.com */
// Define FSR pin:
#define fsrpin A0
//Define variable to store sensor readings:
int fsrreading; //Variable to store FSR value
void setup() {
// Begin serial communication at a baud rate of 9600:
Serial.begin(9600);
}
void loop() {
// Read the FSR pin and store the output as fsrreading:
fsrreading = analogRead(fsrpin);
// Print the fsrreading in the serial monitor:
// Print the string "Analog reading = ".
Serial.print("Analog reading = ");
// Print the fsrreading:
Serial.print(fsrreading);
// We can set some threshholds to display how much pressure is roughly applied:
if (fsrreading < 10) {
Serial.println(" - No pressure");
} else if (fsrreading < 200) {
Serial.println(" - Light touch");
} else if (fsrreading < 500) {
Serial.println(" - Light squeeze");
} else if (fsrreading < 800) {
Serial.println(" - Medium squeeze");
} else {
Serial.println(" - Big squeeze");
}
delay(500); //Delay 500 ms.
}

	 

	                                    
                                  Knock Sensor
	
A Knock Sensor or a Vibration Sensor is a simple device that detects vibrations or shocks from knocking or tapping it. It is basically an electronic switch which normally open. When it detects any shock or vibrations, it closes (for that moment and returns back to its default open position).                                                                                                                                       Several Knock Sensors are available in the market and the cheaper ones are called KY-031 Knock Sensors. The following image shows the Knock Sensor Module used in this project.
                                                  
Circuit Diagram for Interfacing Knock Sensor with Arduino
The following image shows the circuit diagram of Interfacing a Knock Sensor with Arduino UNO.
The results are also displayed on the serial monitor.            
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    Knock Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Knock Sensor an the Arduino:

                                     
Code
The code of interfacing knock sensor with Arduino is given below.
const int knockPin = 8;
const int ledPin = 7;

int knockVal = HIGH;
boolean knockAlarm = false;
unsigned long prevKnockTime;


int knockAlarmTime = 100;


void setup ()
{
  Serial.begin(9600);  
  pinMode (ledPin, OUTPUT) ;
  pinMode (knockPin, INPUT) ;
}
void loop ()
{
  knockVal = digitalRead (knockPin) ;
  
  if (knockVal == LOW)
  {
  
    prevKnockTime = millis();
    
    if (!knockAlarm)
    {
      Serial.println("KNOCK, KNOCK");
      digitalWrite(ledPin,HIGH);
      knockAlarm = true;
      delay(1000);
    }
  }
  else
  {
    if( (millis()-prevKnockTime) > knockAlarmTime &&  knockAlarm)
    {
      digitalWrite(ledPin,LOW);
      Serial.println("No Knocks");
      knockAlarm = false;
    }
  }
}

                    
                          

                                       
                           SW-420 Motion Sensor

                          This is SW-420 Vibration module, which can work from 3.3V to the 5V. The sensor uses M393 computer to detect the vibration over a threshold point and provide digital data. Logic Low or Logic High, 0or 1.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    SW-420 Motion Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:

 

Code
/*//==============================================================================//
 * Vibration Sensor interfacing with Arduino
 * Date: - 25-11-2020
 * Author:- tec santosh kumar

 */ //=============================================================================//
#include <Arduino.h>
#include <stdio.h>
 
#define ON 1
#define OFF 0
 
/*
 * Pin Description
 */
int vibration_Sensor = A5;
int LED = 13;
 
/*
 * Programme flow Description
 */
int present_condition = 0;
int previous_condition = 0;
 
/*
 * Pin mode setup
 */
void setup() {
pinMode(vibration_Sensor, INPUT);
pinMode(LED, OUTPUT);
}
 
/*
 * Led blink
 */
void led_blink(void);
 
/*
 * main_loop
 */
 
void loop() {
previous_condition = present_condition;
present_condition = digitalRead(A5); // Reading digital data from the A5 Pin of the Arduino.
 
if (previous_condition != present_condition) {
led_blink();
 
} else {
digitalWrite(LED, OFF);
}
}
 
void led_blink(void) {
digitalWrite(LED, ON);
delay(250);
digitalWrite(LED, OFF);
delay(250);
digitalWrite(LED, ON);
delay(250);
digitalWrite(LED, OFF);
delay(250);
}                                      
                                          
                                                        Pulse Sensor Module

The pulse sensor module has a light which helps in measuring the pulse rate. When we place the finger on the pulse sensor. The light reflected will change based on the volume of blood inside the capillary blood vessels. During a heartbeat.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
    Pulse Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino: 
Code/Program
 
#include <math.h>
const int x_out = A1; /* connect x_out of module to A1 of UNO board */
const int y_out = A2; /* connect y_out of module to A2 of UNO board */
const int z_out = A3; /* connect z_out of module to A3 of UNO board */

void setup() {
  Serial.begin(9600); 
}

void loop() {
  int x_adc_value, y_adc_value, z_adc_value; 
  double x_g_value, y_g_value, z_g_value;
  double roll, pitch, yaw;
  x_adc_value = analogRead(x_out); /* Digital value of voltage on x_out pin */ 
  y_adc_value = analogRead(y_out); /* Digital value of voltage on y_out pin */ 
  z_adc_value = analogRead(z_out); /* Digital value of voltage on z_out pin */ 
  Serial.print("x = ");	
  Serial.print(x_adc_value);
  Serial.print("\t\t");
  Serial.print("y = ");
  Serial.print(y_adc_value);
  Serial.print("\t\t");
  Serial.print("z = ");
  Serial.print(z_adc_value);
  Serial.print("\t\t");
  //delay(100);
  
  x_g_value = ( ( ( (double)(x_adc_value * 5)/1024) - 1.65 ) / 0.330 ); /* Acceleration in x-direction in g units */ 
  y_g_value = ( ( ( (double)(y_adc_value * 5)/1024) - 1.65 ) / 0.330 ); /* Acceleration in y-direction in g units */ 
  z_g_value = ( ( ( (double)(z_adc_value * 5)/1024) - 1.80 ) / 0.330 ); /* Acceleration in z-direction in g units */ 
	
  roll = ( ( (atan2(y_g_value,z_g_value) * 180) / 3.14 ) + 180 );  /* Formula for roll */
  pitch = ( ( (atan2(z_g_value,x_g_value) * 180) / 3.14 ) + 180 );  /* Formula for pitch */
  //yaw = ( ( (atan2(x_g_value,y_g_value) * 180) / 3.14 ) + 180 );  /* Formula for yaw */
  /* Not possible to measure yaw using accelerometer. Gyroscope must be used if yaw is also required */
  Serial.print("Roll = ");
  Serial.print(roll);
  Serial.print("\t");
  Serial.print("Pitch = ");
  Serial.print(pitch);
  Serial.print("\n\n");
  delay(1000);                                
}

                             Hall Sensor Module
      
Based on the Hall Effect, a hall sensor is a one that varies its output voltage in response to a magnetic field. Hall sensors are used for proximity switching, positioning, speed detection, and current sensing applications.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
    Hall Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:


                                                                           
           Photoresistor Photoelectric Sensor Module

              LDR sensor module is used to detect the intensity of light. It is associated with both analog output pin and digital output pin labelled as AO and DO respectively on the board. When there is light, the resistance of LDR will become low according to the intensity of light.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
    Photorasister Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:


                                                                        
                        Photodiode Light Sensor Module

Photodiode module is most sensitive to the ambient light, generally used to detect the brightness of the ambient light intensity, photoresistor sensor module Universal in most cases, the difference between the two is that  photodiode module directional, can be perceived the fixed direction of the light source.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
    Light Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:

int photoDiode=2;                      
int GreenLed=13;                     
int senRead=0;                  
int SenseRate=905;                   
 void setup()    
 {  
  pinMode(photoDiode,OUTPUT);  
  pinMode(GreenLed,OUTPUT);
  pinMode(12,OUTPUT);
  digitalWrite(photoDiode,HIGH);       
  digitalWrite(GreenLed,LOW);      
  Serial.begin(9600);           
 }  
 void loop()  
 {  
  int val=analogRead(senRead);    
  Serial.println(val);            
  if(val <= SenseRate)               
  {
   digitalWrite(12,HIGH);  
   digitalWrite(GreenLed,LOW);       
   delay(20);  
  }  
  else if(val > SenseRate)            
  {  
   digitalWrite(12,LOW);
   digitalWrite(GreenLed,HIGH);       
   delay(20);  
  }  
 }  


                                                                
                            IR Sensor Module

The IR sensor module consists mainly of the IR Transmitter and Receiver, Opamp, Variable of Resistor (Trimmer pot), output LED in brief. IR LED Transmitter. 
IR LED emits light, in the range of Infrared frequency. IR light, is invisible to us as its Wavelength (700mm- 1mm) is much higher than the visible light range.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
  IR Sensor Module	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:

                                                                
                        Sound Sensor Module

The sound sensor is one type of module used to notice the sound. The application of this module mainly include switch, security, as well as monitoring. The accuracy of this sensor can be changed for the ease of usage.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
    Sound Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Pin wiring
Pin	Arduino UNO
A0	Analog pins
D0	Digital pins
VCC	5V
GND	GND

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:

                                      
                           Barometric Sensor

The barometric sensor, also commonly known as the barometric air pressure sensor ( BAP), is a type of engine management sensor commonly found on many vehicles, It is responsible for measuring the atmospheric pressure of the environment that the vehicle is driving in
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
   Barometric Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Wiring the sensor to the Arduino UNO is pretty straightforward:

          Pin	             Arduino UNO
        Vin 	              5V
     GND	        GND
     SCL	        A5
     SDA 	        A4
Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:

                                                        
                       Digital Thermal Sensor

Temperature sensor measure the amount of heat energy or even coldness that is generated by an object or system, allowing us to “sense” or detect any physical change to that temperature  producing either an analogus or digital output.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
 Digital  ThermalSensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:
#include <OneWire.h>
#include <DallasTemperature.h>
 
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2
 
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
 
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
 
void setup(void)
{
  // start serial port
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin();
}
 
 
void loop(void)
{
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print(" Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");

  Serial.print("Temperature is: ");
  Serial.print(sensors.getTempCByIndex(0)); // Why "byIndex"? 
    // You can have more than one IC on the same bus. 
    // 0 refers to the first IC on the wire
    delay(1000);
}


                                                          
                                          Rotary Encoder Module

Rotary encoders are used in a wide range of application that require monitoring or control, or both, robotics, photographic lenses, computer input devices such as optomechanical mice and trackballs, controlled tress rheometers, and rotating radar platforms.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
 Rotary encoder Module	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:

                                    
                           MQ-2 Gas Sensor

MQ-2 gas sensor is an electronic sensor used for sensing the concentration of gasses in the air such as LPG, propane, methane, hydrogen, alcohol, smoke and carbon monoxide.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
    MQ-2 Gas Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Pin wiring

Pin	Arduino UNO
A0	Analog
D0	Digital pins
VCC	5V
GND	GND

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:

	                    
            Humidity and Rain Detection Sensor

It can be used as a switch when raindrop falls through the raining board and also for measuring rainfall intensity.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    Humidity & Rain Detection Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Pin Wiring

Pin	Arduino UNO
A0	A5
D0	Digital pins
VCC	5V
GND	GND

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:

                                    
                            Speed Sensor Module

Here is a motor speed sensor module, the major goal is to check the rate of an electric motor. The module can be used in association with a microcontroller for motor speed detection, pulse count, position limit, etc. In principle, any rate meter simply measures the at which some event occurs.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
   Speed Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:

                                                        
          IR Infrared Flame Detection Sensor

The IR detector can detect low-frequency flickering IR radiation ranging from 1 to 15 HZ during combustion. It uses the IR flame flicker techniques, which enables the sensor to operate through a layer of oil, water vapour, dust, or ice.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
  IR Flame Detection Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:


                                                         
            RF433MHZTransmitter/Receiver

An RF transmitter receiver serial data and tranmits it wirelessly through RF through its antenna connected at pin4. The transmitted data is received by an RF receiver operating at the same frequency as that of the transmitter.
Specifications RF 433MHz Receiver
•	Frequency Range: 433.92 MHz
•	Modulation: ASK
•	Input Voltage: 5V
    Specifications RF 433MHz Transmitter
•	Frequency Range: 433.92MHz
•	Input Voltage: 3-12V
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
Transmitter/Receiver Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:

                                    
                           Gprs Gsm Module

A GSM module is used to communication between a microcontroller (or a microprocessor) and the GSM / GPSR Network. Here, GSM stands for Global System for Mobile Communication and GPRS stands for General Packet Radio Service.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    Gprs/Gsm Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:


                                                          
     LM393 Detection Touch Module Metal Detector

Metal detector is an electronic instrument that detects the presence of metal nearby. Metal detectors are useful for finding metal inclusions hidden within objects, or metal objects buried underground. They often consist of a handheld unit with a sensor probe which can be swept over the ground or other objects.
          Name	        Figure	Bying Price


   Arduino Uno	      
          
	
    LM393 Dtection Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:
                                                  
                                                                
                         Colour Sensor Module

Colour sensor module is based on TCS3200 RGB sensor chip and white LEDS. It can detect and measure a nearly limitless range of visible colours. Application include test strip reading, sorting by colour, ambient light sensing and calibration and colour matching. 
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
    Colour  Sensor 	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:
/* Arduino Color Detector 
 
 *   Color Sensor
 *    - frequency output scaling
 *      s0          s1         
 *       L           L         = power down no output
 *       L           H         = 2%
 *       H           L         = 20%
 *       H           H         = 100% 
 *   - EO = set low to enable 
 *   
 * Developed by tec Santosh Kumar */
#include "stationDefines.h"

/* LCD */
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27; 16 chars and 2 lines (use I2C scan to confirm address)

void setup()
{
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(outPin, INPUT); //out from sensor becomes input to arduino

    // Setting frequency scaling to 100%
  digitalWrite(s0,HIGH);
  digitalWrite(s1,HIGH);
  
  Serial.begin(9600);
  lcd.begin();
  // lcd.noBacklight();
  lcd.print("MJRoBot ColorDet");
  Serial.println("MJRoBot Color Detector");
  startTiming = millis();
}

void loop()
{
  getColor();
  if (DEBUG) printData(); 
  elapsedTime = millis()-startTiming; 
  if (elapsedTime > 1000) 
  {
    showDataLCD();
    startTiming = millis();
  }
}




                                                              
                             Alcohol Sensor

Sensor which can detect the presence of alcohol gasses at concentration from 0.05 mg/L to 10 mg/L. The sensitive material used for this sensor is sno2. Whose conductivity increases as the concentration of alcohol gasses.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
    Alcohol Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:

                              
                                         
                                                           
                   Finger Print Sensor Module

These module come with FLASH memory store the fingerprints and work with any microcontroller or system with TTL serial. These modules can be added to security systems, door locks, time attendance systems, and much more.
             Name	        Figure	Bying Price



        Arduino Uno	      
          
	
    
    
 Fingerprint Sensor	           
               
	


       Breadboard
	          	


10KΩ Resistor (or 4.7KΩ)	            	

    Jumper Wires	          	

Schematic
 Connecting an LM35 Temperature Sensor an the Arduino:
                                                                          
                            Tilt Sensor

 The tilt sensor is many times referred to as inclinometer, tilt switch or rolling ball
sensor. Using a tilt sensor is a simple way to detect orientation or inclination.
The tilt sensor module is the one in the following figure.
The tilt sensor allows to detect orientation or inclination. It detects if the sensor is
completely upright or if it is tilted.
This makes it very useful to be used, for example, in toys, robots and other
appliances whose working methodology depends on inclination.
How does it work?
The tilt sensor is cylindrical and contains a free conductive rolling ball inside with two
conductive elements (poles) beneath.
LIKE ARDUINO? GET 25 ARDUINO STEP-BY-STEP PROJECTS COURSE 58
Here's how it works:
• When the sensor is completely upright, the ball falls to the bottom of the
sensor and connects the poles, allowing the current to flow.
• When the sensor is tilted, the ball doesn't touch the poles, the circuit is open,
and the current doesn't flow.
This way, the tilt sensor acts like a switch that is turned on or off depending on its
inclination. So, it will give digital information to the Arduino, either a HIGH or a LOW
signal.
Where to buy?
Click the link below to compare the sensor at different stores and find the best price:
• Tilt Sensor

                          
                  MRFC522 RFID

RFID means radio-frequency identification. RFID uses electromagnetic fields to
transfer data over short distances. RFID is useful to identify people, to make
transactions, etc.
You can use an RFID system to open a door. For example, only the person with the
right information on his card can enter. An RFID system uses:
• tags attached to the object to be identified, in this example we have a keychain
and an electromagnetic card. Each tag has his own unique identification (UID).
• two-way radio transmitter-receiver, the reader, that send a signal to the tag
and read its response.
Specifications
• Input voltage: 3.3V
• Frequency: 13.56MHz
LIKE ARDUINO? GET 25 ARDUINO STEP-BY-STEP PROJECTS COURSE 71
Where to Buy?
Click the link below to compare the module at different stores and find the best price:
• MRFC522 RFID

     Pin wiring
Pin	Arduino UNO
SDA	D10
SCK	D13
MOSI	D11
MISO	D12
IRQ	Don’t Connect
RST	D9
3.3V	3.3V
GND	GND


                                              
                                     Relay Module

A relay is an electrically operated switch of mains voltage. It can be turned on or off,
letting the current go through or not. The relay module is shown in the figure below.
This module has two channels (those blue cubes).
Where to buy?
Click the link below to compare the module at different stores and find the best price:
• Relay module
Mains voltage connections
In relation to mains voltage, relays have 3 possible connections:
LIKE ARDUINO? GET 25 ARDUINO STEP-BY-STEP PROJECTS COURSE 78

