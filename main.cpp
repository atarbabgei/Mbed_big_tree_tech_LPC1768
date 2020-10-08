#include "mbed.h"

unsigned long long timerValue;
volatile unsigned int temp,
    counter = 0; // This variable will increase or decrease depending on the
                 // rotation of encoder

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

// Create a BufferedSerial object with a default baud rate.
static BufferedSerial serial_port(P0_2, P0_3);
FileHandle *mbed::mbed_override_console(int fd) { return &serial_port;}

// Pinout
static DigitalOut stepX(P2_2);
static DigitalOut dirX(P2_6);
static DigitalOut enX(P2_1);

static DigitalOut MS1(P1_17);
static DigitalOut MS2(P0_4);
static DigitalOut MS3(P1_10);
static DigitalIn limitSW(P0_23);

static DigitalOut MAG(P2_3);
static DigitalIn proxSW0(P2_0);
static DigitalIn proxSW1(P0_10);
InterruptIn encoderPhaseA(P0_0);
InterruptIn encoderPhaseB(P0_1);

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
  // turn magnet ON, then wait
  MAG.write(1);
  ThisThread::sleep_for(500ms);
  // go backward until limitSW detected
  dirX.write(1);
  while (limitSW == 1) {
    stepX.write(1);
    ThisThread::sleep_for(1ms);
    stepX.write(0);
    ThisThread::sleep_for(1ms);
  }
}

void mCommand(void) {
  // command [z] then CCW for "integerFromPC" steps
  if (messageFromPC[0] == 'z' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      dirX.write(0);
      for (int x = 0; x < integerFromPC; x++) {
        if (proxSW0 == 1) {
          stepX.write(1);
          ThisThread::sleep_for(1ms);
          stepX.write(0);
          ThisThread::sleep_for(1ms);
        }
      }
    }
  }
  // command [x] then CW for "integerFromPC" steps
  else if (messageFromPC[0] == 'x' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
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
      if (integerFromPC == 0) {
        MAG.write(0);
      } else if (integerFromPC == 1) {
        MAG.write(1);
      }
    }
    // command [s] for picking up trolleys
  } else if (messageFromPC[0] == 's' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      pickup();
    }
  }
  // get current timer
  else if (messageFromPC[0] == 't' && messageFromPC[1] == '\0') {
    if (newCommand == true) {
      printf("[t,%llu]\n", timerValue);
    }
  }
  newCommand = false;
};

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (encoderPhaseB == 0) {
    counter++;
  } else {
    counter--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (encoderPhaseA == 0) {
    counter--;
  } else {
    counter++;
  }
}

int main(void) {
  
  serial_port.set_baud(57600);
  serial_port.set_format(8, BufferedSerial::None, 1);
  printf("hello world\r\n");
  // enable stepper, set step_size
  enX = 0;
  MS1 = 0;
  MS2 = 0;
  MS3 = 0;

  // Input Mode for Digital Inputs
  proxSW0.mode(PullNone);
  proxSW1.mode(PullNone);
  limitSW.mode(PullUp);
  encoderPhaseA.mode(PullUp);
  encoderPhaseB.mode(PullUp);

  encoderPhaseA.rise(&ai1);
  encoderPhaseB.rise(&ai0);

  while (1) {
    // receive serial
     
    recvWithStartEndMarkers();

    //if valid data, then parse data
    if (newData == true) {
      strcpy(tempChars, receivedChars);
      parseData();
      newData = false;
    }

    //check any valid command received
    mCommand();
     
    if (counter != temp) {
    printf("count = %d \r\n", counter);
    temp = counter;
    }
  }
}
