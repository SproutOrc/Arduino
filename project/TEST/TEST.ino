#define MRA 8
#define MRB 9
#define MREN 6

#define MLA 10
#define MLB 11
#define MLEN 7

#define SPEED_INT_R 3
#define SPEED_INT_L 2

#define SPEED_DIR_R 5
#define SPEED_DIR_L 4

int rightSpeed = 0;
int leftSpeed = 0;
 
void portInit() {
    // left motor
    pinMode(MLA, OUTPUT);
    pinMode(MLB, OUTPUT);
    pinMode(MLEN, OUTPUT);

    // right motor
    pinMode(MRA, OUTPUT);
    pinMode(MRB, OUTPUT);
    pinMode(MREN, OUTPUT);

    attachInterrupt(1, setRightSpeed, RISING);
    attachInterrupt(0, setLeftSpeed, RISING);
}


void setRightSpeed() {
    if (digitalRead(SPEED_DIR_R) == 1) {
        rightSpeed += 1;
    } else {
        rightSpeed -= 1;
    }
}


void setLeftSpeed() {
    if (digitalRead(SPEED_DIR_L) == 1) {
        leftSpeed += 1;
    } else {
        leftSpeed -= 1;
    }
}
long a = 0;

void setup()
{
    portInit();
    Serial2.begin(57600);
    a = millis();
}


void loop()
{
    if (millis()> a + 10) {
        a = millis();
        Serial2.println(rightSpeed);
        rightSpeed = 0;
    }
}