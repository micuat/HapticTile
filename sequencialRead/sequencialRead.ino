int RXLED = 17;  // The RX LED has a defined Arduino pin
// The TX LED was not so lucky, we'll need to use pre-defined
// macros (TXLED1, TXLED0) to control that.
// (We could use the same macros for the RX LED too -- RXLED1,
//  and RXLED0.)

int readPins1[] = {2, 3, 4, 5, 3};
int readPins2[] = {10, 16, 14, 15, 16};
int readPins3[] = {6, 7, 8, 9, 7};
int readPins4[] = {21, 20, 19, 18, 20};

int vals[4][4];

void setup()
{
  pinMode(RXLED, OUTPUT);  // Set RX LED as an output
  // TX LED is set as an output behind the scenes

  Serial.begin(9600); //This pipes to the serial monitor

  for (int i = 0; i < 4; i++)
  {
    pinMode(readPins1[i], INPUT_PULLUP);
    pinMode(readPins2[i], INPUT_PULLUP);
    pinMode(readPins3[i], INPUT_PULLUP);
    pinMode(readPins4[i], INPUT_PULLUP);
  }

}

void loop()
{
  for (int i = 0; i < 4; i++)
  {
    int groundPin1 = readPins1[i];
    int groundPin2 = readPins2[i];
    int groundPin3 = readPins3[i];
    int groundPin4 = readPins4[i];
    pinMode(groundPin1, OUTPUT);
    pinMode(groundPin2, OUTPUT);
    pinMode(groundPin3, OUTPUT);
    pinMode(groundPin4, OUTPUT);
    digitalWrite(groundPin1, LOW);
    digitalWrite(groundPin2, LOW);
    digitalWrite(groundPin3, LOW);
    digitalWrite(groundPin4, LOW);
    delay(20);

    int readPin1 = readPins1[i + 1];
    int readPin2 = readPins2[i + 1];
    int readPin3 = readPins3[i + 1];
    int readPin4 = readPins4[i + 1];
    vals[0][i] = 1 - digitalRead(readPin1);
    vals[1][i] = 1 - digitalRead(readPin2);
    vals[2][i] = 1 - digitalRead(readPin3);
    vals[3][i] = 1 - digitalRead(readPin4);

    pinMode(groundPin1, INPUT_PULLUP);
    pinMode(groundPin2, INPUT_PULLUP);
    pinMode(groundPin3, INPUT_PULLUP);
    pinMode(groundPin4, INPUT_PULLUP);

    delay(20);
  }
  for(int i = 0; i < 4; i++)
  {
    Serial.print((char)('a' + i));
    Serial.print(' ');
    Serial.print(vals[i][0]);
    Serial.print(' ');
    Serial.print(vals[i][1]);
    Serial.print(' ');
    Serial.print(vals[i][2]);
    Serial.print(' ');
    Serial.println(vals[i][3]);
  }

  digitalWrite(RXLED, LOW);   // set the LED on
  TXLED0; //TX LED is not tied to a normally controlled pin
  delay(50);              // wait for a second
  digitalWrite(RXLED, HIGH);    // set the LED off
  TXLED1;
  delay(50);              // wait for a second
}
