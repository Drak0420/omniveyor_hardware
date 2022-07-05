/* Requires Rosserial Arduino Library == 0.7.9 */
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#define numel(a) (sizeof(a) / sizeof(*a))

ros::NodeHandle nh;

/* std_msgs::UInt16MultiArray in_msg:
 * 
 * uint16_t state_flag_in
 * uint16_t masked_gpio_pin_level
 * 
 * State Flags: MSB 0b 0000 0000 0000 0000 LSB
 * LSB
 * bit 0..2: states 0...7 for the button & LED
 *            0: prelaunch ready
 *            1: manual long press to launch
 *            2: launch triggered
 *            3: active
 *            4: manual pause triggered
 *            5: paused / goal reached
 *            6: manual long press to shutdown / short press to resume
 *            7: manual shutdown triggered (go to prelaunch)
 * bit 3:    Charging state
 * bit 4:    Charging fault
 * bit 15:   Robot not ready
 * MSB
 */


std_msgs::UInt16MultiArray out_msg;
/* uint16_t state_flag_out
 * uint16_t masked_gpio_pin_level
 * 6*uint16_t A[0]...A[5] read-outs
 */

const uint8_t inputPins[7]={0,1,2,3,4,5,6};
//const uint16_t input_mask = 0b0000000001111111;

const uint8_t outputPins[3]={7,9,10};
//const uint16_t output_mask = 0b0000011010000000;

const uint8_t analogPins[6]={14,15,16,17,18,19};
const uint8_t out_msg_size = 2+numel(analogPins);

const uint8_t buttonPin = 8;
const uint8_t ledPin = 11;
const uint8_t chargingPin = 12;
const uint8_t chargingFaultPin = 13;

unsigned long buttonMillis = 0;
unsigned long buttonMillisDebounce = 0;
unsigned long ledMillis = 0;
unsigned long pubMillis = 0;
unsigned long chargingMillis = 0;

const uint8_t publishPeriod = 20;

bool buttonState = false;
bool buttonStateLast = false;
bool ledState = false;
bool chargingState = false;
bool chargingFault = false;
unsigned long chargingTime = 0;
const uint16_t intervalButton = 3000;
uint16_t programState = 1<<15;

void setCmd(const std_msgs::UInt16MultiArray& in_msg){
    if (in_msg.layout.dim[0].size == 2){
        programState = in_msg.data[0];
        uint8_t i;
        for (i=0; i<numel(outputPins); i++){
            bool pinStat = (in_msg.data[1]&(1<<outputPins[i]))>0;
            digitalWrite(outputPins[i], pinStat);
        }
    }
}

ros::Publisher statusPub("gpio/get", &out_msg);
ros::Subscriber<std_msgs::UInt16MultiArray> sub("gpio/set", &setCmd);

void setup() {
    uint8_t i;
    for (i = 0; i < numel(inputPins); i++){
        pinMode(inputPins[i], INPUT);
    }
    for (i = 0; i < numel(outputPins); i++){
        pinMode(outputPins[i], OUTPUT);
        digitalWrite(outputPins[i], LOW);
    }
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);
    pinMode(chargingPin, OUTPUT);
    pinMode(chargingFaultPin, INPUT_PULLUP);
    digitalWrite(ledPin, LOW);
    digitalWrite(chargingPin, LOW);
    
    nh.initNode();
    nh.subscribe(sub);

    out_msg.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension));
    out_msg.layout.dim[0].label = "gpioStates";
    out_msg.layout.dim[0].size = out_msg_size;
    out_msg.layout.dim[0].stride = 1;
    out_msg.layout.data_offset = 0;
    out_msg.data = (uint16_t *)malloc(out_msg_size*sizeof(uint16_t));
    out_msg.data_length = out_msg_size;
    out_msg.data[0] = programState;
    out_msg.data[1] = 0;
    for (i = 0; i < numel(inputPins); i++){
        bool pinLvl = digitalRead(inputPins[i]);
        out_msg.data[1] |= ((pinLvl&0x1) << inputPins[i]);
    }
    for (i = 0; i < numel(analogPins); i++){
        out_msg.data[2+i] = analogRead(analogPins[i]);
    }
    nh.advertise(statusPub);
} 

void loop(){
  const unsigned long curMillis = millis();
  pushbutton(curMillis);
  //relay();
  //currentsensor(curMillis);
  charging(curMillis);
  led(curMillis);
  publishROS(curMillis);
  nh.spinOnce();
}

void publishROS(const unsigned long curMillis){
  if (curMillis-pubMillis >= publishPeriod){
    uint8_t i;
    out_msg.data[0] = programState;
    out_msg.data[1] = 0;
    for (i = 0; i < numel(inputPins); i++){
        bool pinLvl = digitalRead(inputPins[i]);
        out_msg.data[1] |= ((pinLvl&0x1) << inputPins[i]);
    }
    for (i = 0; i < numel(analogPins); i++){
        out_msg.data[2+i] = analogRead(analogPins[i]);
    }
    statusPub.publish(&out_msg);
    pubMillis = curMillis;
  }
}

//For Push Button
void pushbutton(const unsigned long curMillis)
{
    int buttonRead = !digitalRead(buttonPin);
    if (buttonRead != buttonStateLast){
        buttonMillisDebounce = curMillis;
    }
    if (curMillis - buttonMillisDebounce > 10){
        if (buttonState != buttonRead)
        buttonState = buttonRead;
    }
    buttonStateLast = buttonRead;
    
    // MAIN STATE MACHINE CONTROLLED BY THE BUTTON
    /* USAGE:
     * Robot Ready (LED Slow flashing) --> Long Press Button (LED Lit UP) --> Running State (LED Solid On)
     * Robot Ready (LED Slow flashing) --> Short Press Button --> Robot Ready (LED Slow flashing)
     * Running State --> Press Button --> Paused State (LED Fast flashing)
     * Running State --> Software-Generated S3 Cmd --> Paused State (when goal is reached)
     * Paused State --> Short Press Button --> Resume Running State (LED Solid On)
     * Paused State --> Long Press Button (LED Goes OUT) --> Quit: back to Robot Ready
     */
    switch (programState & 0x8007){
        case 0:         // Pre-launch state: robot is ready
            if (buttonState){
                programState = 1;
                buttonMillis = curMillis;
            }
            break;
        case 1:         // intermediate state for determining long press that transits state 0 to state 2
            if (buttonState){
                if (curMillis - buttonMillis > intervalButton){
                    programState = 2;
                }
          }
            else{
               programState = 0;
            }
            break;
        case 2:         // Ready-to-launch state: robot will register a new nav goal when button is released
            if (!buttonState){
                programState = 3;
            }
            break;
        case 3:         // Active state: robot has a new nav target to achieve
            if (buttonState){
                programState = 4;
            }
            break;
        case 4:         // Pause state: robot is paused manually
            if (!buttonState){
                programState = 5;
            }
            break;
        case 5:         // Pause state: Goal reached / paused.
            if (buttonState){
                programState = 6;
                buttonMillis = curMillis;
            }
            break;
        case 6:         // Intermediate state for resume/shutdown branch
            if (buttonState){
                if (curMillis - buttonMillis > intervalButton){
                    programState = 7;
                }
            }
            else{
               programState = 3;
            }
            break;
        case 7:         // Ready-to-shutdown state: robot will deactivate (state 0) when button released.
            if (!buttonState){
                programState = 0;
            }
            break;
        default:        // robot is not ready.
            break;
    }
}

// led blinking control
void led(const unsigned long curMillis){
    switch (programState & 0x8007){
        case 0:
            if (curMillis - ledMillis > 1000){
                ledState = !ledState;
                ledMillis = curMillis;
                digitalWrite(ledPin, ledState);  
            }
            break;
        case 5:
            if (curMillis - ledMillis > 500){
                ledState = !ledState;
                ledMillis = curMillis;
                digitalWrite(ledPin, ledState);  
            }
            break;
        case 2:
        case 3:
        case 6:
            digitalWrite(ledPin,HIGH);
            break;
        default:
            digitalWrite(ledPin,LOW);
            break;
    }
}

void charging(const unsigned long curMillis){
    if (curMillis - chargingMillis >= 1000){
        if (!chargingState){
            if (programState & 0x0008){
                digitalWrite(chargingPin, HIGH);
                chargingState = true;
                chargingFault = false;
                chargingTime = 0;
            }
        }
        else{
            if (!programState & 0x0008){
                digitalWrite(chargingPin, LOW);
                chargingState = false;
                chargingFault = false;
            }
            else{
                digitalWrite(chargingPin, HIGH);
                chargingFault = !digitalRead(chargingFaultPin);
                programState |= ((chargingFault&0x1)<<4);
                chargingTime ++;
                if (chargingTime % 3600 == 0 || chargingFault)
                    digitalWrite(chargingPin, LOW);       // reset (toggle back on) every 1h.
            }
        }
        chargingMillis = curMillis;
    }
}
