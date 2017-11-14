
// defines pins numbers
const int trigPin[] = {7,9,11,13};
const int echoPin[] = {6,8,10,12};

// defines variables
long duration;
int distance;

void setup() {
  for(int i=0; i<4; i++){
    pinMode(trigPin[i], OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin[i], INPUT); // Sets the echoPin as an Input
  }
  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  Serial.print("B");
  for(int i=0; i<4; i++) {
    // Clears the trigPin
    digitalWrite(trigPin[i], LOW);
    delayMicroseconds(2);
    
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin[i], LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin[i], HIGH);
    
    // Calculating the distance
    distance= duration*0.034/2;
    
    
    Serial.print(distance);
    if(i != 3) Serial.print(",");
  
  }
  
  Serial.print("E");
}
