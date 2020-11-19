#include "QEIx4.h"
#include "mbed.h"
#include <cmath>

#define timer_read_f(x) chrono::duration<float>((x).elapsed_time()).count()
#define timer_read_s(x)                                                        \
  chrono::duration_cast<chrono::seconds>((x).elapsed_time()).count();
#define timer_read_ms(x)                                                       \
  chrono::duration_cast<chrono::milliseconds>((x).elapsed_time()).count()
#define timer_read_us(x) (x).elapsed_time().count()

// Ratio for
#define ENCODER_POSITION_FACTOR (1.0 / 1600 * 14.6386)
#define ENCODER_SPEED_FACTOR 0.09f

const int LIMITSW_POSITION = 100;

bool isTimerOn = false;
Timer t;
unsigned long long previousMillis = 0;
int currentPositionStepper;
float currentPosition;
float currentPositionStepper_inCM;
float currentPositionEncoder;
int previousPositionStepper;
float previousPositionEncoder = 0;
float previousSpeed = 0;
int data_counter = 50;

int data_timer[100], data_speed[100], data_position[100];

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
  for (int steps = 0; steps < 100; steps++) {
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

  previousPositionStepper = 0;
  previousPositionEncoder = encoder.getPosition();
}

void mCommand(void) {
  // command [z] then CCW for "integerFromPC" steps
  if (messageFromPC[0] == 'z' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      int _steps;
      dirX.write(0);
      for (int x = 0; x < integerFromPC; x++) {
        if (proxSW1 == 1) {
          _steps++;
          stepX.write(1);
          ThisThread::sleep_for(1ms);
          stepX.write(0);
          ThisThread::sleep_for(1ms);
        }
      }
      currentPositionStepper = previousPositionStepper + _steps;
      currentPositionStepper_inCM = round(currentPositionStepper* 0.01985);
      previousPositionStepper = currentPositionStepper;
      previousPositionEncoder = encoder.getPosition();
      printf("[%s,%d]\r\n", messageFromPC, (int)(100-currentPositionStepper_inCM));
    }
  }
  // command [x] then CW for "integerFromPC" steps
  else if (messageFromPC[0] == 'x' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      int _steps;
      dirX.write(1);
      for (int x = 0; x < integerFromPC; x++) {
        if (limitSW == 1) {
          _steps++;
          stepX.write(1);
          ThisThread::sleep_for(1ms);
          stepX.write(0);
          ThisThread::sleep_for(1ms);
        }
      }
      currentPositionStepper = previousPositionStepper - _steps;
      currentPositionStepper_inCM = round(currentPositionStepper* 0.01985);
      previousPositionStepper = currentPositionStepper;
      previousPositionEncoder = encoder.getPosition();
      printf("[%s,%d]\r\n", messageFromPC, (int)(100-currentPositionStepper_inCM));
    }
  }
  // command [m,0]= magnet off, [m,1] = magnet on
  else if (messageFromPC[0] == 'm' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      printf("[%s]\r\n", messageFromPC);
      if (integerFromPC == 0) {
        MAG.write(0);
      } else if (integerFromPC == 1) {
        MAG.write(1);
      }
    }
    // command [s] for picking up trolleys
  } else if (messageFromPC[0] == 's' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      printf("[%s]\r\n", messageFromPC);
      pickup();
    }
  }

  // command [l,0]= linearact off, [l,1] = linearact down, [l,2] = linearact up
  else if (messageFromPC[0] == 'l' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      printf("[%s]\r\n", messageFromPC);
      if (integerFromPC == 0) {
        linearActuatorINA.write(0);
        linearActuatorINB.write(0);
      } else if (integerFromPC == 1) {
        linearActuatorINA.write(0);
        linearActuatorINB.write(1);
      } else if (integerFromPC == 2) {
        linearActuatorINA.write(1);
        linearActuatorINB.write(0);
      }
    }
  } else if (messageFromPC[0] == 'r' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      printf("[%s]\r\n", messageFromPC);
      MAG.write(0);
      t.reset();
      t.start();
      // printf("timer start!\n");
      data_counter = 0;
      isTimerOn = true;
    }

  }
  // get current timer
  else if (messageFromPC[0] == 't' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      printf("[t,%llu]\n", previousMillis);
    }
  }
  // get current speed
  else if (messageFromPC[0] == 'v' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      printf("[v,%d]\n", (int)previousSpeed);
    }
  }

  // get current data
  else if (messageFromPC[0] == 'd' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      // first 3 data are usually random, so we rewrite to 0
      data_speed[0] = 0;
      data_speed[1] = 0;
      data_speed[2] = 0;

      // augmented first 100ms data, so it won't 0
      data_speed[2] = data_speed[3] / 2;

      // start from 2, so leave first 2 data.
      for (int i = 2; i <= data_counter; i++) {
        printf("[d,%d,%d,%d]", i - 1, data_timer[i], data_speed[i]);
      }
      printf("\r\n");
    }
  }

  // get current data position
  else if (messageFromPC[0] == 'e' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      // first 3 data are usually random, so we rewrite to 0
      data_position[0] = 0;
      data_position[1] = 0;
      data_position[2] = 0;

      // augmented first 100ms data, so it won't 0
      data_position[2] = data_position[3] / 2;

      // start from 2, so leave first 2 data.
      for (int i = 2; i <= data_counter; i++) {
        printf("[e,%d,%d,%d]", i - 1, data_timer[i], data_position[i]);
      }
      printf("\r\n");
    }
  }

  newCommand = false;
};

int main(void) {
  // Start Timer
  t.reset();
  t.start();

  // Setup Encoder
  encoder.setSpeedFactor(ENCODER_SPEED_FACTOR);
  encoder.setPositionFactor(ENCODER_POSITION_FACTOR);

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
    currentPositionEncoder = (encoder.getPosition() - previousPositionEncoder);
     
    recvWithStartEndMarkers();

    // if valid data, then parse data
    if (newData == true) {
      strcpy(tempChars, receivedChars);
      parseData();
      newData = false;
    }

    // check any valid command received
    mCommand();

    if (isTimerOn == true) {

      if (proxSW1.read() == 0) {
        t.stop();
        isTimerOn = false;

        float currentSpeed = encoder.getSpeed();
        previousSpeed = currentSpeed;
        data_counter = data_counter + 1;
        data_timer[data_counter] = (int)currentMillis;
        data_speed[data_counter] = (int)currentSpeed;
        data_position[data_counter] = (int)round(currentPositionEncoder);

        // get data when proxSW1 = 1;
        /*
                printf("count:%d, timer: %d, speed: %d, pos: %d \r\n",
           data_counter,  data_timer[data_counter], data_speed[data_counter],
           data_position[data_counter]);   */
        previousMillis = currentMillis;
      }

      if (currentMillis - previousMillis >= 100) {

        float currentSpeed = encoder.getSpeed();
        previousSpeed = currentSpeed;
        data_counter = data_counter + 1;
        data_timer[data_counter] = (int)currentMillis;
        data_speed[data_counter] = (int)currentSpeed;
        data_position[data_counter] = (int)round(currentPositionEncoder);

        // get data every 100ms;
        /*
                printf("count:%d, timer: %d, speed: %d, pos: %d \r\n",
           data_counter,  data_timer[data_counter], data_speed[data_counter],
           data_position[data_counter]);   */
        previousMillis = currentMillis;
      }
    }
  }
}
