
#include <SoftwareSerial.h>
#include <Servo.h>
#include <EEPROM.h>

#include "Board.h"
#include "PushButton_Simple.h"
#include "PushButton.h" //Use timer2
#include "pitches.h"
#include "Motor_Regulator.h"

//Create objs.
Board LABBOT_Board;
//Input:
PushButton_Simple startButton;

//Output:
Motor_Regulator motor1, motor2;
// Create a Software serial object.
SoftwareSerial mySerial(9, 4); // RX, TX
#define _useServo1
#define _useServo2
//#define _useServo3
//#define _useServo4
//#define _useServo5
//#define _useServo6

#if defined(_useServo1)
Servo servo1; // create servo object to control a servo
              //At most 6 servos can be attached to the board.
#endif
#if defined(_useServo2)
Servo servo2;
#endif
#if defined(_useServo3)
Servo servo3;
#endif
#if defined(_useServo4)
Servo servo4;
#endif
#if defined(_useServo5)
Servo servo5;
#endif
#if defined(_useServo6)
Servo servo6;
#endif

//RF_module
int rfReading;
int rfDataCode;

//Beep
int PROMPT_TONE1_TAB[] = {
    //This table consists of the freqs to be displayed
    NOTE_E6, // 1318HZ,treble  3
    NOTE_A6, // 1760HZ,treble 6
    NOTE_G6, // 1568HZ,treble 5
};
int PROMPT_TONE1_DURATION_TAB[] = {8, 8, 8};

int PROMPT_TONE2_TAB[] = {
    //This table consists of the freqs to be displayed
    NOTE_A6, // 1760HZ,treble 6
    NOTE_E6, // 1568HZ,treble 5
    NOTE_E6, // 1397HZ,treble 4
    NOTE_E6, // 1318HZ,treble 3
    NOTE_A6, // 1760HZ,treble 6
};
int PROMPT_TONE2_DURATION_TAB[] = {8, 8, 8, 8, 8};

int thisNote;
int noteDuration;
int pauseBetweenNotes; // noteDuration * 1.30;
unsigned long durationCount;
bool beep_on;
bool enterSettingModePrompt;
bool exitSettingModePrompt;

//#define MIN_POSITION_CEILING 1162 //the greatest value to which the minPosition can be assigned.
#define MIN_POSITION_CEILING 1056 //the greatest value to which the minPosition can be assigned.
//#define MIN_POSITION_FLOOR 544    //the least value to which the minPosition can be assigned.
#define MIN_POSITION_FLOOR 550 //the least value to which the minPosition can be assigned.

#define DEFAULT_MIN_POSITION 800 //default minmum servo position when no adjustment is applied.
#define DEFAULT_MAX_POSITION 2140
#define POSITION_RANGE 1340 //the range of the servo position.

struct servoAddIn_t //This structure contains additional information
                    //used for ctrlling multiple servos.Each entry of the array
                    //should append to a servo object.
{
    unsigned long regulationRate;
    int minPosition;
    int maxPosition;
    int currentAngle;
    int expectedAngle;
    bool zeroPositionCorrection;
    unsigned long correctionRate;
    bool zeroPositionCorrection_direction;
    int pinNbr;

} servoAddIn[6];

int servoIndex;
bool startServoCalibration;

int eeAddress, eeAddress1, eeAddress2;
byte checkingByte1, checkingByte2;

#define SETTING_MODE 1
#define RUNNING_MODE 2
int mode; // The program behaviour differently in different mode
bool mode_switching;

void setup()
{
    // put your setup code here, to run once:
    //The startup time is approximate to 1.6S.
    LABBOT_Board.initial_all_ioports();

    //Input:
    // Open serial communications and wait for port to open:
    Serial.begin(9600);
    // set the data rate for the SoftwareSerial port
    mySerial.begin(9600);
    /*----Common modules applied to all the subprograms.---*/
    startButton.create(START);

    //Output:
    //LED:
    pinMode(A1_BOARD, OUTPUT); //Attach LED1 to D1 pin
    digitalWrite(A1_BOARD, LOW);
    pinMode(A2_BOARD, OUTPUT); //Attach LED2 to D2 pin
    digitalWrite(A2_BOARD, LOW);

    //Motor
    motor1.attach(MOTOR1_INA, MOTOR1_INB); //Attach INA to the PWM output port
    motor2.attach(MOTOR2_INA, MOTOR2_INB); //Attach INB to the general ioport for direction ctrl
    motor1.set(CLKWISE, 0);
    motor2.set(CLKWISE, 0);

    //The first 2 bytes are used to check whether the EEPROM has been modified yet,
    //If the checking bytes don't match with the given values,which means the EEPROM hasn't
    //been modified,the servo associated variables should be assigned to the default values.
    //otherwise,read in the values from EEPROM.
    EEPROM.get(0, checkingByte1);
    EEPROM.get(1, checkingByte2);

    if (checkingByte1 != 0x55 || checkingByte2 != 0xaa)
    {

        //Assign the default value.
        for (int i = 0; i < 6; i++)
        {
            servoAddIn[i].minPosition = DEFAULT_MIN_POSITION;
            servoAddIn[i].maxPosition = DEFAULT_MAX_POSITION;
        }

        //Update EEPROM
        EEPROM.put(0, 0x55);
        EEPROM.put(1, 0xaa);

        eeAddress1 = 2;
        eeAddress2 = 4;
        for (int i = 0; i < 6; i++)
        {
            EEPROM.put(eeAddress1, servoAddIn[i].minPosition);
            EEPROM.put(eeAddress2, servoAddIn[i].maxPosition);
            eeAddress1 += 2 * sizeof(int); //Move to the next group of values.
            eeAddress2 += 2 * sizeof(int); //Move to the next group of values.
        }
    }
    else
    { //checkingByte1=0x55 && checkingByte2==0xaa
        //The EEPROM has already been written,read in the values from the EEPROM
        eeAddress1 = 2;
        eeAddress2 = 4;
        for (int i = 0; i < 6; i++)
        {
            Serial.println(EEPROM.get(eeAddress1, servoAddIn[i].minPosition));
            Serial.println(EEPROM.get(eeAddress2, servoAddIn[i].maxPosition));
            eeAddress1 += 2 * sizeof(int); //Move to the next group of values.
            eeAddress2 += 2 * sizeof(int); //Move to the next group of values.
        }
    }

    //Set all 6  servos to 90 degrees.
    for (int i = 0; i < 6; i++)
    {
        servoAddIn[i].expectedAngle = 90;
        servoAddIn[i].regulationRate = millis(); //Record the current time
    }
#if defined(_useServo1)
    servoAddIn[0].pinNbr = D1_BOARD;
    //Calling this fcn will set the threshold of the internal pulse width automatically.
    servo1.attach(servoAddIn[0].pinNbr, servoAddIn[0].minPosition, servoAddIn[0].maxPosition);
#endif
#if defined(_useServo2)
    servoAddIn[1].pinNbr = D2_BOARD;
    servo2.attach(servoAddIn[1].pinNbr, servoAddIn[1].minPosition, servoAddIn[1].maxPosition);
#endif
#if defined(_useServo3)
    servoAddIn[2].pinNbr = D3_BOARD;
    servo3.attach(servoAddIn[2].pinNbr, servoAddIn[2].minPosition, servoAddIn[2].maxPosition);
#endif
#if defined(_useServo4)
    servoAddIn[3].pinNbr = D4_BOARD;
    servo4.attach(servoAddIn[3].pinNbr, servoAddIn[3].minPosition, servoAddIn[3].maxPosition);
#endif
#if defined(_useServo5)
    servoAddIn[4].pinNbr = A1_BOARD;
    servo5.attach(servoAddIn[4].pinNbr, servoAddIn[4].minPosition, servoAddIn[4].maxPosition);
#endif
#if defined(_useServo6)
    servoAddIn[5].pinNbr = A2_BOARD;
    servo6.attach(servoAddIn[5].pinNbr, servoAddIn[5].minPosition, servoAddIn[5].maxPosition);
#endif

    mode = RUNNING_MODE;
    mode_switching = 1;
}
void loop()
{ // run over and over

    startButton.detection(); //Check if the button is pressed.
    /*------------------servo1 speed regulation--------------------------*/
#if defined(_useServo1)
    if (millis() - servoAddIn[0].regulationRate > 20) //Count for 20ms
    {
        servoAddIn[0].regulationRate = millis(); //Reset the timer

        if (servoAddIn[0].currentAngle < servoAddIn[0].expectedAngle)
        {
            servoAddIn[0].currentAngle++;
            servo1.write(servoAddIn[0].currentAngle);
        }
        else if (servoAddIn[0].currentAngle > servoAddIn[0].expectedAngle)
        {
            servoAddIn[0].currentAngle--;
            servo1.write(servoAddIn[0].currentAngle);
        }
    }
#endif
    /*------------------servo2 speed regulation--------------------------*/
#if defined(_useServo2)
    if (millis() - servoAddIn[1].regulationRate > 20) //Count for 20ms
    {
        servoAddIn[1].regulationRate = millis(); //Reset the timer

        if (servoAddIn[1].currentAngle < servoAddIn[1].expectedAngle)
        {
            servoAddIn[1].currentAngle++;
            servo2.write(servoAddIn[1].currentAngle);
        }
        else if (servoAddIn[1].currentAngle > servoAddIn[1].expectedAngle)
        {
            servoAddIn[1].currentAngle--;
            servo2.write(servoAddIn[1].currentAngle);
        }
    }
#endif
    /*------------------servo3 speed regulation--------------------------*/
#if defined(_useServo3)
    if (millis() - servoAddIn[2].regulationRate > 20) //Count for 20ms
    {
        servoAddIn[2].regulationRate = millis(); //Reset the timer

        if (servoAddIn[2].currentAngle < servoAddIn[2].expectedAngle)
        {
            servoAddIn[2].currentAngle++;
            servo3.write(servoAddIn[2].currentAngle);
        }
        else if (servoAddIn[2].currentAngle > servoAddIn[2].expectedAngle)
        {
            servoAddIn[2].currentAngle--;
            servo3.write(servoAddIn[2].currentAngle);
        }
    }
#endif
    /*------------------servo4 speed regulation--------------------------*/
#if defined(_useServo4)
    if (millis() - servoAddIn[3].regulationRate > 20) //Count for 20ms
    {
        servoAddIn[3].regulationRate = millis(); //Reset the timer

        if (servoAddIn[3].currentAngle < servoAddIn[3].expectedAngle)
        {
            servoAddIn[3].currentAngle++;
            servo4.write(servoAddIn[3].currentAngle);
        }
        else if (servoAddIn[3].currentAngle > servoAddIn[3].expectedAngle)
        {
            servoAddIn[3].currentAngle--;
            servo4.write(servoAddIn[3].currentAngle);
        }
    }
#endif
    /*------------------servo5 speed regulation--------------------------*/
#if defined(_useServo5)
    if (millis() - servoAddIn[4].regulationRate > 20) //Count for 20ms
    {
        servoAddIn[4].regulationRate = millis(); //Reset the timer

        if (servoAddIn[4].currentAngle < servoAddIn[4].expectedAngle)
        {
            servoAddIn[4].currentAngle++;
            servo5.write(servoAddIn[4].currentAngle);
        }
        else if (servoAddIn[4].currentAngle > servoAddIn[4].expectedAngle)
        {
            servoAddIn[4].currentAngle--;
            servo5.write(servoAddIn[4].currentAngle);
        }
    }
#endif
    /*------------------servo6 speed regulation--------------------------*/
#if defined(_useServo6)
    if (millis() - servoAddIn[5].regulationRate > 20) //Count for 20ms
    {
        servoAddIn[5].regulationRate = millis(); //Reset the timer

        if (servoAddIn[5].currentAngle < servoAddIn[5].expectedAngle)
        {
            servoAddIn[5].currentAngle++;
            servo6.write(servoAddIn[5].currentAngle);
        }
        else if (servoAddIn[5].currentAngle > servoAddIn[5].expectedAngle)
        {
            servoAddIn[5].currentAngle--;
            servo6.write(servoAddIn[5].currentAngle);
        }
    }
#endif

    if (beep_on)
    {
        if (millis() - durationCount > 200)
        {
            beep_on = 0;
            // stop the tone playing:
            noTone(BEEP);
        }
    }

    /*----------------------RUNNING_MODE---------------------------*/
    if (mode == RUNNING_MODE)
    {
        //Entrance:
        if (mode_switching)
        {
            mode_switching = 0;
            //Set the initial angle of the servo
            //Set servo1 to 90 degrees.
            servoAddIn[0].expectedAngle = 90;
            servoAddIn[0].regulationRate = millis(); //Record the current time
        }

        //If the startButton is pressed...
        if (startButton.isPress())
        {
            //do nothing
        }

        if (enterSettingModePrompt)
        {
            if (millis() - durationCount > pauseBetweenNotes)
            {
                durationCount = millis(); //reset the timer.
                if (thisNote < 2)         //thisNote < NOTES_NBR-1
                {
                    //the current note has asserted for long enough,
                    //point to the next note.
                    thisNote++;
                    // to calculate the note duration, take one second divided by the note type.
                    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
                    noteDuration = 1000 / PROMPT_TONE1_DURATION_TAB[thisNote];
                    pauseBetweenNotes = noteDuration * 1.30;
                    tone(BEEP, PROMPT_TONE1_TAB[thisNote], noteDuration);
                }
                else //The duration time of the last note has elapsed,
                     //shutdown the tone and exit.
                {
                    enterSettingModePrompt = 0;
                    // stop the tone playing:
                    noTone(BEEP);

                    //After prompt,switch to setting mode
                    mode = SETTING_MODE;
                    mode_switching = 1;
                }
            }
        }

        //Remote ctrl...
        if (mySerial.available())
        {
            //Serial.println(mySerial.read());
            //Read the data from the RF module.
            rfReading = mySerial.read();

            //If the reading has changed,refresh the dataCode.
            if (rfReading != rfDataCode)
            {
                rfDataCode = rfReading;

                Serial.println(rfDataCode);

                if (rfDataCode == 15) //endCode.
                {
                    //Servo stop
                    servoAddIn[0].expectedAngle = servoAddIn[0].currentAngle;
                    servoAddIn[0].regulationRate = millis(); //Record the current time
                    servoAddIn[1].expectedAngle = servoAddIn[1].currentAngle;
                    servoAddIn[1].regulationRate = millis(); //Record the current time                 
                    //Turn off the LEDs.
                    digitalWrite(A1_BOARD, LOW);
                    digitalWrite(A2_BOARD, LOW);
                }
                else
                {
                    //Prompt the user that the remote ctrller button
                    //has been pressed.
                    if (!beep_on)
                    {
                        beep_on = 1;
                        durationCount = millis(); //reset the timer.
                        tone(BEEP, NOTE_A6, 200);
                        switch (rfDataCode) //Execute the command.
                        {
                        case 5: //KEY1
                            break;
                        case 7: //KEY2
                            break;
                        case 8: //KEY3
                            break;
                        case 6: //KEY4
                            break;
                        case 1: //KEY5
                            //Servo points to 0 degrees.
                            servoAddIn[0].expectedAngle = 0;
                            servoAddIn[0].regulationRate = millis(); //Record the current time
                            //Turn on the LEDs.
                            digitalWrite(A1_BOARD, HIGH);
                            digitalWrite(A2_BOARD, HIGH);
                            break;
                        case 3: //KEY6
                            //Servo points to 180 degrees.
                            servoAddIn[0].expectedAngle = 180;
                            servoAddIn[0].regulationRate = millis(); //Record the current time
                            //Turn on the LEDs.
                            digitalWrite(A1_BOARD, HIGH);
                            digitalWrite(A2_BOARD, HIGH);
                            break;
                        case 4: //KEY7
                         //Servo points to 0 degrees.
                            servoAddIn[1].expectedAngle = 0;
                            servoAddIn[1].regulationRate = millis(); //Record the current time
                            //Turn on the LEDs.
                            digitalWrite(A1_BOARD, HIGH);
                            digitalWrite(A2_BOARD, HIGH);
                            break;
                        case 2: //KEY8
                        //Servo points to 180 degrees.
                            servoAddIn[1].expectedAngle = 180;
                            servoAddIn[1].regulationRate = millis(); //Record the current time
                            //Turn on the LEDs.
                            digitalWrite(A1_BOARD, HIGH);
                            digitalWrite(A2_BOARD, HIGH);
                            break;
                        case 10: //KEY_A
                            break;
                        case 9: //KEY_B
                            break;
                        case 11: //KEY_C
                            break;
                        case 12: //KEY_D
                            break;
                        case 69: //KEY1+KEY7
                            break;
                        case 37: //KEY1+KEY8
                            break;
                        case 71: //KEY2+KEY7
                            break;
                        case 39: //KEY2+KEY8
                            break;
                        case 72: //KEY3+KEY7
                            break;
                        case 40: //KEY3+KEY8
                            break;
                        case 70: //KEY4+KEY7
                            break;
                        case 38: //KEY4+KEY8
                            break;
                        case 21: //KEY1+KEY5
                            break;
                        case 53: //KEY1+KEY6
                            break;
                        case 23: //KEY2+KEY5
                            break;
                        case 55: //KEY2+KEY6
                            break;
                        case 24: //KEY3+KEY5
                            break;
                        case 56: //KEY3+KEY6
                            break;
                        case 22: //KEY4+KEY5
                            break;
                        case 54: //KEY4+KEY6
                            break;
                        case 89:  //KEY_B+KEY1
                        case 121: //KEY_B+KEY2
                        case 137: //KEY_B+KEY3
                        case 105: //KEY_B+KEY4
                        case 25:  //KEY_B+KEY5
                        case 57:  //KEY_B+KEY6
                            //Prompt the user that the program is switching to the setting mode.
                            if (!enterSettingModePrompt)
                            {
                                enterSettingModePrompt = 1;
                                //Point to the first note.
                                thisNote = 0;
                                // to calculate the note duration, take one second divided by the note type.
                                //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
                                noteDuration = 1000 / PROMPT_TONE1_DURATION_TAB[thisNote];
                                pauseBetweenNotes = noteDuration * 1.30;
                                durationCount = millis(); //reset the timer.
                                tone(BEEP, PROMPT_TONE1_TAB[thisNote], noteDuration);
                            }
                            switch (rfDataCode)
                            {
                            case 89: //KEY_B+KEY1
                                servoIndex = 0;
                                break;
                            case 121: //KEY_B+KEY2
                                servoIndex = 1;
                                break;
                            case 137: //KEY_B+KEY3
                                servoIndex = 2;
                                break;
                            case 105: //KEY_B+KEY4
                                servoIndex = 3;
                                break;
                            case 25: //KEY_B+KEY5
                                servoIndex = 4;
                                break;
                            case 57: //KEY_B+KEY6
                                servoIndex = 5;
                                break;
                            }
                            break;
                        }
                    }
                }
            }
        }
    }

    /*----------------------SETTING_MODE-----------------------*/
    else if (mode == SETTING_MODE)
    {
        //Entrance:
        if (mode_switching)
        {
            mode_switching = 0;
            //The servo has already been specified before entering the setting mode.
            //The angle of the servo remains the same.
            //Turn on the LEDs
            digitalWrite(A1_BOARD, HIGH);
            digitalWrite(A2_BOARD, HIGH);
        }

        if (startButton.isPress())
        { //do nothing
        }

        if (exitSettingModePrompt)
        {
            if (millis() - durationCount > pauseBetweenNotes)
            {
                durationCount = millis(); //reset the timer.

                if (thisNote < 4) //thisNote < NOTES_NBR-1
                {
                    //the current note has asserted for long enough,
                    //point to the next note.
                    thisNote++;

                    // to calculate the note duration, take one second divided by the note type.
                    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
                    noteDuration = 1000 / PROMPT_TONE2_DURATION_TAB[thisNote];
                    pauseBetweenNotes = noteDuration * 1.30;

                    tone(BEEP, PROMPT_TONE2_TAB[thisNote], noteDuration);
                }
                else //The duration time of the last note has elapsed,
                     //shutdown the tone and exit.
                {
                    exitSettingModePrompt = 0;
                    // stop the tone playing:
                    noTone(BEEP);

                    //Exit.
                    mode = RUNNING_MODE;
                    mode_switching = 1;
                    //Turn off the LEDs
                    digitalWrite(A1_BOARD, LOW);
                    digitalWrite(A2_BOARD, LOW);
                }
            }
        }

        //Only servo1 can be adjusted for the moment.
        //In order to adjust the other servos,the servo object should be created first.
        if (servoAddIn[servoIndex].zeroPositionCorrection)
        {
            if (millis() - servoAddIn[servoIndex].correctionRate > 10) //Count for 10ms
            {
                servoAddIn[servoIndex].correctionRate = millis(); //Reset the timer

                if (servoAddIn[servoIndex].zeroPositionCorrection_direction == COUNTER_CLKWISE)
                {
                    if (servoAddIn[servoIndex].minPosition > MIN_POSITION_FLOOR)
                    {
                        servoAddIn[servoIndex].minPosition--;
                        servoAddIn[servoIndex].maxPosition = servoAddIn[servoIndex].minPosition + POSITION_RANGE;
                        switch (servoIndex)
                        {
                        case 0:
#if defined(_useServo1)
                            servo1.attach(servoAddIn[0].pinNbr, servoAddIn[0].minPosition, servoAddIn[0].maxPosition);
                            servo1.write(servoAddIn[0].currentAngle);
#endif
                            break;
                        case 1:
#if defined(_useServo2)
                            servo2.attach(servoAddIn[1].pinNbr, servoAddIn[1].minPosition, servoAddIn[1].maxPosition);
                            servo2.write(servoAddIn[1].currentAngle);
#endif
                            break;
                        case 2:
#if defined(_useServo3)
                            servo3.attach(servoAddIn[2].pinNbr, servoAddIn[2].minPosition, servoAddIn[2].maxPosition);
                            servo3.write(servoAddIn[2].currentAngle);
#endif
                            break;
                        case 3:
#if defined(_useServo4)
                            servo4.attach(servoAddIn[3].pinNbr, servoAddIn[3].minPosition, servoAddIn[3].maxPosition);
                            servo4.write(servoAddIn[3].currentAngle);
#endif
                            break;
                        case 4:
#if defined(_useServo5)
                            servo5.attach(servoAddIn[4].pinNbr, servoAddIn[4].minPosition, servoAddIn[4].maxPosition);
                            servo5.write(servoAddIn[4].currentAngle);
#endif
                            break;
                        case 5:
#if defined(_useServo6)
                            servo6.attach(servoAddIn[5].pinNbr, servoAddIn[5].minPosition, servoAddIn[5].maxPosition);
                            servo6.write(servoAddIn[5].currentAngle);
#endif
                            break;
                        }
                    }
                }
                else
                { //CLKWISE
                    if (servoAddIn[servoIndex].minPosition < MIN_POSITION_CEILING)
                    {
                        servoAddIn[servoIndex].minPosition++;
                        servoAddIn[servoIndex].maxPosition = servoAddIn[servoIndex].minPosition + POSITION_RANGE;
                        switch (servoIndex)
                        {
                        case 0:
#if defined(_useServo1)
                            servo1.attach(servoAddIn[0].pinNbr, servoAddIn[0].minPosition, servoAddIn[0].maxPosition);
                            servo1.write(servoAddIn[0].currentAngle);
#endif
                            break;
                        case 1:
#if defined(_useServo2)
                            servo2.attach(servoAddIn[1].pinNbr, servoAddIn[1].minPosition, servoAddIn[1].maxPosition);
                            servo2.write(servoAddIn[1].currentAngle);
#endif
                            break;
                        case 2:
#if defined(_useServo3)
                            servo3.attach(servoAddIn[2].pinNbr, servoAddIn[2].minPosition, servoAddIn[2].maxPosition);
                            servo3.write(servoAddIn[2].currentAngle);
#endif
                            break;
                        case 3:
#if defined(_useServo4)
                            servo4.attach(servoAddIn[3].pinNbr, servoAddIn[3].minPosition, servoAddIn[3].maxPosition);
                            servo4.write(servoAddIn[3].currentAngle);
#endif
                            break;
                        case 4:
#if defined(_useServo5)
                            servo5.attach(servoAddIn[4].pinNbr, servoAddIn[4].minPosition, servoAddIn[4].maxPosition);
                            servo5.write(servoAddIn[4].currentAngle);
#endif
                            break;
                        case 5:
#if defined(_useServo6)
                            servo6.attach(servoAddIn[5].pinNbr, servoAddIn[5].minPosition, servoAddIn[5].maxPosition);
                            servo6.write(servoAddIn[5].currentAngle);
#endif
                            break;
                        }
                    }
                }
            }
        }

        //Remote ctrl...
        if (mySerial.available())
        {
            //Serial.println(mySerial.read());
            //Read the data from the RF module.
            rfReading = mySerial.read();

            //If the reading has changed,refresh the dataCode.
            if (rfReading != rfDataCode)
            {
                rfDataCode = rfReading;

                if (rfDataCode == 15) //endCode
                {
                    //If the servo is being calibrated,
                    //stop calibrating after the button is released.
                    if (servoAddIn[servoIndex].zeroPositionCorrection)
                        servoAddIn[servoIndex].zeroPositionCorrection = 0;
                }
                else
                {
                    //Prompt the user that the remote ctrller button
                    //has been pressed.
                    if (!beep_on)
                    {
                        beep_on = 1;
                        durationCount = millis(); //reset the timer.
                        tone(BEEP, NOTE_A6, 200);
                        switch (rfDataCode) //Execute the command.
                        {
                        case 5: //KEY1
                            //Servo points to 90 degrees.
                            servoAddIn[servoIndex].expectedAngle = 90;
                            servoAddIn[servoIndex].regulationRate = millis(); //Record the current time
                            break;
                        case 7: //KEY2
                            //Exit.
                            //Disable servo calibration fcn.
                            startServoCalibration = 0;
                            servoAddIn[servoIndex].zeroPositionCorrection = 0;

                            //Stop the servo
                            servoAddIn[servoIndex].expectedAngle = servoAddIn[servoIndex].currentAngle;
                            servoAddIn[servoIndex].regulationRate = millis(); //Record the current time

                            //Save the values into EEPROM
                            eeAddress1 = 2 + 2 * sizeof(int) * servoIndex; //Move to the next group of values.
                            eeAddress2 = 4 + 2 * sizeof(int) * servoIndex; //Move to the next group of values.
                            EEPROM.put(eeAddress1, servoAddIn[servoIndex].minPosition);
                            EEPROM.put(eeAddress2, servoAddIn[servoIndex].maxPosition);

                            //Prompt the user that the program is in the setting mode.
                            if (!exitSettingModePrompt)
                            {
                                exitSettingModePrompt = 1;
                                //Point to the first note.
                                thisNote = 0;
                                // to calculate the note duration, take one second divided by the note type.
                                //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
                                noteDuration = 1000 / PROMPT_TONE2_DURATION_TAB[thisNote];
                                pauseBetweenNotes = noteDuration * 1.30;
                                durationCount = millis(); //reset the timer.
                                tone(BEEP, PROMPT_TONE2_TAB[thisNote], noteDuration);
                            }
                            break;
                        case 8: //KEY3
                            //Servo points to 0 degrees.
                            servoAddIn[servoIndex].expectedAngle = 0;
                            servoAddIn[servoIndex].regulationRate = millis(); //Record the current time
                            break;
                        case 6: //KEY4
                            //Servo points to 180 degrees.
                            servoAddIn[servoIndex].expectedAngle = 180;
                            servoAddIn[servoIndex].regulationRate = millis(); //Record the current time
                            break;
                        case 1: //KEY5
                            break;
                        case 3: //KEY6
                            break;
                        case 4: //KEY7
                            if (startServoCalibration)
                            {
                                servoAddIn[servoIndex].zeroPositionCorrection = 1;
                                servoAddIn[servoIndex].zeroPositionCorrection_direction = COUNTER_CLKWISE;
                                servoAddIn[servoIndex].correctionRate = millis(); //Record the current time
                            }
                            break;
                        case 2: //KEY8
                            if (startServoCalibration)
                            {
                                servoAddIn[servoIndex].zeroPositionCorrection = 1;
                                servoAddIn[servoIndex].zeroPositionCorrection_direction = CLKWISE;
                                servoAddIn[servoIndex].correctionRate = millis(); //Record the current time
                            }
                            break;
                        case 10: //KEY_A
                            break;
                        case 9: //KEY_B
                            break;
                        case 11: //KEY_C
                                 //After entering setting mode,press KEY_C to activate servo angle calibration
                                 //fcn.
                            if (!startServoCalibration)
                                startServoCalibration = 1;
                            break;
                        case 12: //KEY_D
                            //Exit.
                            //Disable servo calibration fcn.
                            startServoCalibration = 0;
                            servoAddIn[servoIndex].zeroPositionCorrection = 0;

                            //Stop the servo
                            servoAddIn[servoIndex].expectedAngle = servoAddIn[0].currentAngle;
                            servoAddIn[servoIndex].regulationRate = millis(); //Record the current time

                            //Save the values into EEPROM
                            eeAddress1 = 2 + 2 * sizeof(int) * servoIndex; //Move to the next group of values.
                            eeAddress2 = 4 + 2 * sizeof(int) * servoIndex; //Move to the next group of values.
                            EEPROM.put(eeAddress1, servoAddIn[servoIndex].minPosition);
                            EEPROM.put(eeAddress2, servoAddIn[servoIndex].maxPosition);

                            //Prompt the user that the program is in the setting mode.
                            if (!exitSettingModePrompt)
                            {
                                exitSettingModePrompt = 1;
                                //Point to the first note.
                                thisNote = 0;
                                // to calculate the note duration, take one second divided by the note type.
                                //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
                                noteDuration = 1000 / PROMPT_TONE2_DURATION_TAB[thisNote];
                                pauseBetweenNotes = noteDuration * 1.30;
                                durationCount = millis(); //reset the timer.
                                tone(BEEP, PROMPT_TONE2_TAB[thisNote], noteDuration);
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
}
