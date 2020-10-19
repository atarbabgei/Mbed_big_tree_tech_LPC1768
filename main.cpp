#include "QEIx4.h"
#include "mbed.h"

#define timer_read_f(x) chrono::duration<float>((x).elapsed_time()).count()
#define timer_read_s(x)                                                        \
  chrono::duration_cast<chrono::seconds>((x).elapsed_time()).count();
#define timer_read_ms(x)                                                       \
  chrono::duration_cast<chrono::milliseconds>((x).elapsed_time()).count()
#define timer_read_us(x) (x).elapsed_time().count()

bool isTimerOn = false;

Timer t; 

unsigned long long previousMillis = 0; 
float previousPosition = 0;


// Receive with start- and end-markers combined with parsing
const unsigned char numChars = 8;
char receivedChars[numChars];
char tempChars[numChars]; // temporary array for use by strtok() function

// variables to hold the parsed data
char messageFromPC[numChars] = {0};
bool newData = false;
bool newCommand = false;
int integerFromPC = 0;

// Maximum number of element the application buffer can contain
#define MAXIMUM_BUFFER_SIZE 1

// Application buffer to receive the data
char buf[MAXIMUM_BUFFER_SIZE] = {0};

// Encoder Pinout
QEIx4 encoder(P0_1, P0_0, NC,
              (QEIx4::EMODE)(QEIx4::IRQ_NO_JAMMING |
                             QEIx4::SPEED)); // QEI with AB signals only

// Create a BufferedSerial object with a default baud rate.
static BufferedSerial serial_port(P0_2, P0_3);
FileHandle *mbed::mbed_override_console(int fd) { return &serial_port; }

// Pinout
static DigitalOut stepX(P2_2);
static DigitalOut dirX(P2_6);
static DigitalOut enX(P2_1);

static DigitalOut linearActuatorINA(P1_15);
static DigitalOut linearActuatorINB(P1_16);

static DigitalOut MS1(P1_17);
static DigitalOut MS2(P0_4);
static DigitalOut MS3(P1_10);
static DigitalIn limitSW(P0_23);

static DigitalOut MAG(P2_3);
static DigitalIn proxSW0(P2_0);
static DigitalIn proxSW1(P0_10);

// Get Serial and parse Start End Markers
void recvWithStartEndMarkers() {
  static bool recvInProgress = false;
  static unsigned char ndx = 0;
  char startMarker = '[';
  char endMarker = ']';
  char rc;

  while (serial_port.readable() > 0 && newData == false) {
    uint32_t num = serial_port.read(buf, sizeof(buf));
    rc = buf[0];

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
        newCommand = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

// Parsing Variable Type
void parseData() {
  // split the data into its parts
  char *strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ","); // get the first part - the string
  strcpy(messageFromPC, strtokIndx);   // copy it to messageFromPC

  strtokIndx =
      strtok(NULL, ","); // this continues where the previous call left off
  integerFromPC = atoi(strtokIndx); // convert this part to an integer
}

// pickup trolley function
void pickup() {
  // go forward until proxSW0 detected
  dirX.write(0);
  while (proxSW0 == 1) {
    stepX.write(1);
    ThisThread::sleep_for(1ms);
    stepX.write(0);
    ThisThread::sleep_for(1ms);
  }

  MAG.write(1);
  for (int steps =0; steps<100; steps++) {
    stepX.write(1);
    ThisThread::sleep_for(1ms);
    stepX.write(0);
    ThisThread::sleep_for(1ms);
  }
  // turn magnet ON, then wait
  

  ThisThread::sleep_for(500ms);
  // go backward until limitSW detected
  dirX.write(1);
  while (limitSW == 1) {
    stepX.write(1);
    ThisThread::sleep_for(1ms);
    stepX.write(0);
    ThisThread::sleep_for(1ms);
  }

  previousPosition = encoder.getPosition();
}

void mCommand(void) {
  // command [z] then CCW for "integerFromPC" steps
  if (messageFromPC[0] == 'z' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      printf("[%s]\r\n",messageFromPC);
      dirX.write(0);
      for (int x = 0; x < integerFromPC; x++) {
        //if (proxSW0 == 1) {
          stepX.write(1);
          ThisThread::sleep_for(1ms);
          stepX.write(0);
          ThisThread::sleep_for(1ms);
       // }
      }
    }
  }
  // command [x] then CW for "integerFromPC" steps
  else if (messageFromPC[0] == 'x' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      printf("[%s]\r\n",messageFromPC);
      dirX.write(1);
      for (int x = 0; x < integerFromPC; x++) {
        if (limitSW == 1) {
          stepX.write(1);
          ThisThread::sleep_for(1ms);
          stepX.write(0);
          ThisThread::sleep_for(1ms);
        }
      }
    }
  }
  // command [m,0]= magnet off, [m,1] = magnet on
  else if (messageFromPC[0] == 'm' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
        printf("[%s]\r\n",messageFromPC);
      if (integerFromPC == 0) {
        MAG.write(0);
      } else if (integerFromPC == 1) {
        MAG.write(1);
      }
    }
    // command [s] for picking up trolleys
  } else if (messageFromPC[0] == 's' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
        printf("[%s]\r\n",messageFromPC);
      pickup();
    }
  }

    // command [l,0]= linearact off, [l,1] = linearact down, [l,2] = linearact up
  else if (messageFromPC[0] == 'l' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
        printf("[%s]\r\n",messageFromPC);
      if (integerFromPC == 0) {
        linearActuatorINA.write(0);
        linearActuatorINB.write(0);
      } else if (integerFromPC == 1) {
        linearActuatorINA.write(0);
        linearActuatorINB.write(1);
      }
      else if (integerFromPC == 2) {
        linearActuatorINA.write(1);
        linearActuatorINB.write(0);
      }
    }
    }
  else if (messageFromPC[0] == 'r' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
    printf("[%s]\r\n",messageFromPC);
      MAG.write(0);
      t.reset();
      t.start();
      //printf("timer start!\n");
      isTimerOn = true;
    }

  }
  // get current timer
  else if (messageFromPC[0] == 't' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      printf("[t,%llu]\n", previousMillis);
    }
  }
  newCommand = false;
};

int main(void) {
  // Start Timer
  t.start();

  // Setup Encoder
  encoder.setSpeedFactor(1.0f);
  encoder.setPositionFactor((1.0 / 1600));

  serial_port.set_baud(57600);
  serial_port.set_format(8, BufferedSerial::None, 1);
  printf("Praktikum Fisdas\r\n");

  // enable stepper, set step_size
  enX.write(0);
  MS1.write(0);
  MS2.write(0);
  MS3.write(0);

  // stop linear actuator
  linearActuatorINA.write(0);
  linearActuatorINB.write(0);

  // Input Mode for Digital Inputs
  proxSW0.mode(PullNone);
  proxSW1.mode(PullNone);
  limitSW.mode(PullUp);

  while (1) {
    // receive serial
    unsigned long long currentMillis = timer_read_ms(t);
    float currentPosition = (encoder.getPosition() - previousPosition) * 14.3902;
    
    recvWithStartEndMarkers();

    // if valid data, then parse data
    if (newData == true) {
      strcpy(tempChars, receivedChars);
      parseData();
      newData = false;
    }

    // check any valid command received
    mCommand();
     
    if(isTimerOn == true){

        if (proxSW1.read() == 0){
        t.stop();
        isTimerOn = false;
        
        float currentSpeed = encoder.getSpeed(); 
            printf(" timer: %d, speed: %d, pos: %d \r\n", (int)currentMillis,
             (int)currentSpeed, (int)currentPosition);
             }

        if (currentMillis - previousMillis >= 100) { 
            previousMillis = currentMillis;
            
            float currentSpeed = encoder.getSpeed(); 
            printf(" timer: %d, speed: %d, pos: %d \r\n", (int)currentMillis,
             (int)currentSpeed, (int)currentPosition);
    }
    }
  }
}
