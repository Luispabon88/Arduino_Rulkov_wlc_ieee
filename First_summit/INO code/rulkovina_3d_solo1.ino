//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% NON-CHAOTIC RULKOV MODEL NEURON IMPLEMENTATION %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Author: Luis Pabon-Orozco
//Affiliation: Escuela Superior Politecnica del Litoral (ESPOL) Guayaquil - Ecuador
//mail: luispabon88@gmail.com
//Version 2.1
//October 2025
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//This code was developed to describe the behavior of the neuronal membrane and its 
//action potential, using the non-chaotic Rulkov model. It was compared with numerical
//simulations and served as proof-of-concept for the Winnerless Competition (WLC)
//dynamics.
//This code was implemented in a Three-Neuron Coupled Rulkov Network (3NRN) and this
// code is loaded in each neuron, and every neuron consider itself as "Neuron 0" or
// x^i(n), where i is an identificator as neuron number in network and n is the step on
// discrete-time simulation. Neuron 1 and 2 are external neurons, identificated with j
// [x^j(n)]. i=(1,2,3), j=(1,2,3) and i neq j.
// Here we use x^i(n) and x^i(n+1). First is previous value of membrane, second one is
// the actual value of membrane.
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#include <LiquidCrystal.h>
// LCD Pins: RS, E, D4, D5, D6, D7
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int potPin1 = A0; // Analogic alpha pin
const int potPin2 = A1; // Analogic sigma pin
const int potPin3 = A2; // Analogic g pin
const int Vout = 10; // x^(n+1) response - out to oscilloscope
const int ledPin1 = 7;  // Green LED
const int ledPin2 = 6;  // Yellow LED
const int ledPin3 = 9;  // Red LED
const int inputPin1 = A3; // IN from Neuron 1 to Neuron 0.
const int inputPin2 = A4; // IN from Neuron 2 to Neuron 0.
const int outputPin = A5; // OUT from Neuron 0 to others
const float threshold = 0.001; // Threshold edge to detect changes 
const int timingPin = 8; //to show time

float X_ext1 = 0.0; //init value from neuron 1
float X_ext2 = 0.0; //init value from neuron 2
float alfa = 0.0; // Save alpha value from pot
float sigma = 0.0; // Save sigma value from pot
float g = 0.00; // Save g value from pot
float betae = 0.113; //beta e value, ref. Tristan Et. Al
float sigmae = 1; //sigma e value, ref. Tristan Et. Al
float beta_n1 = 0.0; // init value for current from neuron 1
float beta_n2 = 0.0; // init value for current from neuron 2
float sigma_n1 = 0.0; // init value for current from neuron 1
float sigma_n2 = 0.0; // init value for current from neuron 2
float beta_n1_prev = 0.0; // pre init value for current from neuron 1
float beta_n2_prev = 0.0; // pre init value for current from neuron 2
float sigma_n1_prev = 0.0; // pre init value for current from neuron 1
float sigma_n2_prev = 0.0; // pre init value for current from neuron 1
float xi = 0.00; // present value of x^i_(n+1)
float yi = 0.00; // present value of y^i_(n+1)
float xi1 = 0.00; //previous value of x^i_(n+1), x^i_(n)
float yi1 = 0.00; //previos value of y^i_(n+1), y^i_(n)
float x0 = 0; // init value of x
float y0 = 0; // init value of y
float mu = 0.001; // fixed mu value

void setup() {
  //**********************************************
  //pinMode(timingPin, OUTPUT);// to show exc time
  //**********************************************
  Serial.begin(115200); // Init serial comm
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(Vout, OUTPUT);
  lcd.begin(16, 2);   // LCD init
}

void loop() {
  //to measure excecute time
  digitalWrite(timingPin, HIGH);
  unsigned long t0 = micros();
  //Read initial configuration values from potenciometers
  alfa = (0.1 * (analogRead(potPin1) / 10)) + 4; // Read alpha pot
  sigma = (1.0*(analogRead(potPin2) / 1023.0)) - 0.5; // Read sigma pot
  g = (0.200 * analogRead(potPin3) / 1023.0) - 0.100; // Read g pot
  //Read external membrane potentials x^j(n) from Neuron 1 and 2 (j=2,3)
  X_ext1 = analogRead(inputPin1) / 1023.0 * 5.0; // Neuron j=2 V.
  X_ext2 = analogRead(inputPin2) / 1023.0 * 5.0; // Neuron j=3 V.
  
//Non-Chaotic Rulkov Model implementation
  if (xi >= (alfa + yi + beta_n1_prev + beta_n2_prev)) {
    xi1 = -1.00;
    yi1 = yi - (mu * (xi + 1)) + (mu * sigma) + (mu * sigma_n1_prev) + (mu * sigma_n2_prev);
    beta_n1 = (g * betae * (X_ext1 - xi1));
    beta_n2 = (g * betae * (X_ext2 - xi1));
    sigma_n1 = (g * sigmae * (X_ext1 - xi1));
    sigma_n2 = (g * sigmae * (X_ext2 - xi1));
    xi = xi1; //Save x^1(n+1) to x^1(n)
    yi = yi1; //Save y^1(n+1) to y^1(n)
    beta_n1_prev = beta_n1; //Save actual beta_n_1 to be "previous beta_n_1"
    beta_n2_prev = beta_n2; //Save actual beta_n_2 to be "previous beta_n_2"
    sigma_n1_prev = sigma_n1; //Save actual sigma_n_1 to be "previous sigma_n_1"
    sigma_n2_prev = sigma_n2; //Save actual sigma_n_2 to be "previous sigma_n_2"
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    digitalWrite(ledPin3, HIGH); // Encendido Led Rojo
    delay(1);

  } else if ((xi > 0) & (xi < (alfa + yi + beta_n1_prev + beta_n2_prev))) {
    xi1 = alfa + yi + beta_n1_prev + beta_n2_prev;
    yi1 = yi - (mu * (xi + 1)) + (mu * sigma) + (mu * sigma_n1_prev) + (mu * sigma_n2_prev);
    beta_n1 = (g * betae * (X_ext1 - xi1));
    beta_n2 = (g * betae * (X_ext2 - xi1));
    sigma_n1 = (g * sigmae * (X_ext1 - xi1));
    sigma_n2 = (g * sigmae * (X_ext2 - xi1));
    xi = xi1; //Save x^1(n+1) to x^1(n)
    yi = yi1; //Save y^1(n+1) to y^1(n)
    beta_n1_prev = beta_n1; //Save actual beta_n_1 to be "previous beta_n_1"
    beta_n2_prev = beta_n2; //Save actual beta_n_2 to be "previous beta_n_2"
    sigma_n1_prev = sigma_n1; //Save actual sigma_n_1 to be "previous sigma_n_1"
    sigma_n2_prev = sigma_n2; //Save actual sigma_n_2 to be "previous sigma_n_2"
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, HIGH); // Yellow LED ON
    digitalWrite(ledPin3, LOW);
    delay(1);
  } else {
    xi1 = (alfa / (1 - xi)) + yi + beta_n1_prev + beta_n2_prev;
    yi1 = yi - (mu * (xi + 1)) + (mu * sigma) + (mu * sigma_n1_prev) + (mu * sigma_n2_prev);
    beta_n1 = (g * betae * (X_ext1 - xi1));
    beta_n2 = (g * betae * (X_ext2 - xi1));
    sigma_n1 = (g * sigmae * (X_ext1 - xi1));
    sigma_n2 = (g * sigmae * (X_ext2 - xi1));
    xi = xi1; //Save x^1(n+1) to x^1(n)
    yi = yi1; //Save y^1(n+1) to y^1(n)
    beta_n1_prev = beta_n1; //Save actual beta_n_1 to be "previous beta_n_1"
    beta_n2_prev = beta_n2; //Save actual beta_n_2 to be "previous beta_n_2"
    sigma_n1_prev = sigma_n1; //Save actual sigma_n_1 to be "previous sigma_n_1"
    sigma_n2_prev = sigma_n2; //Save actual sigma_n_2 to be "previous sigma_n_2"
    digitalWrite(ledPin1, HIGH); // Green LED ON
    digitalWrite(ledPin2, LOW);
    digitalWrite(ledPin3, LOW);
    delay(1);
  }
  //Map the value of xi to a range of 0 to 255 for the PWM output
  int pwmValue = map(xi * 100, -100, 100, 0, 255); 
  analogWrite(Vout, pwmValue); 
  // Convert pwmValue to an analog value and send it to the other Arduino
  analogWrite(outputPin, pwmValue / 255.0 * 1023.0);
 // Updates the alpha and sigma values ​​on the LCD
  lcd.setCursor(0, 0);
  lcd.print("A: ");
  lcd.print(alfa);
  lcd.setCursor(9, 0);
  lcd.print("g:");
  lcd.setCursor(9,1);
  lcd.print(g);
  lcd.setCursor(0, 1);
  lcd.print("S: ");
  lcd.print(sigma);
  Serial.println(xi); //Print in serial monitor
  delay(1); // Retardo antes de volver a entrar al bucle
  //*******************************************************
  //Excecute time read in serial *To verify values
  //*******************************************************
  //unsigned long t1 = micros();
  //digitalWrite(timingPin, LOW);
  //Serial.print("Loop time (us): ");
  //Serial.println(t1 - t0);
}
