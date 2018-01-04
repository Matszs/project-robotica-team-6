
// These must be defined before including TinyEKF.h
#define Nsta 4     // 1 value
#define Mobs 4     // Four measurements: ultrasone

#include <TinyEKF.h>




// defines pins numbers
const int trigPin[] = {7,9,11,13};
const int echoPin[] = {6,8,10,12};

// defines variables
long duration;
double distance[4];



class Fuser : public TinyEKF {

    public:

        Fuser()
        {            
            // We approximate the process noise using a small constant
            this->setQ(0, 0, .0001);
            this->setQ(1, 1, .0001);
            this->setQ(2, 2, .0001);
            this->setQ(3, 3, .0001);

            // Same for measurement noise
            this->setR(0, 0, .0001);
            this->setR(1, 1, .0001);
            this->setR(2, 2, .0001);
            this->setR(3, 3, .0001);
        }

    protected:

        void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
        {
            // Process model is f(x) = x
            fx[0] = this->x[0];
            fx[1] = this->x[1];
            fx[2] = this->x[2];
            fx[3] = this->x[3];

            // So process model Jacobian is identity matrix
            F[0][0] = 1;
            F[1][1] = 1;
            F[2][2] = 1;
            F[3][3] = 1;

            // Measurement function simplifies the relationship between state and sensor readings for convenience.
            // A more realistic measurement function would distinguish between state value and measured value; e.g.:
            //   hx[0] = pow(this->x[0], 1.03);
            //   hx[1] = 1.005 * this->x[1];
            //   hx[2] = .9987 * this->x[1] + .001;
            hx[0] = this->x[0]; // Barometric pressure from previous state
            hx[1] = this->x[1]; // Baro temperature from previous state
            hx[2] = this->x[2]; // LM35 temperature from previous state
            hx[3] = this->x[3];

            // Jacobian of measurement function
            H[0][0] = 1;        // Barometric pressure from previous state
            H[1][1] = 1 ;       // Baro temperature from previous state
            H[2][2] = 1 ;       // LM35 temperature from previous state
            H[3][3] = 1;
        }
};

Fuser ekf;

void setup() {
  for(int i=0; i<4; i++){
    pinMode(trigPin[i], OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin[i], INPUT); // Sets the echoPin as an Input
  }
  Serial.begin(115200); // Starts the serial communication
}

void loop() {
    measure();
    ekf.step(distance);
    printMeasurements();
}

void printMeasurements(){
    Serial.print("B");
    for(int i=0; i<4; i++) {
      Serial.print((int)ekf.getX(i));
      if(i != 3) Serial.print(",");
    }
    Serial.print("E");
}

void measure(){
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
    distance[i]= duration*0.034/2;
  }
  
}

