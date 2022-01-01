#define LED_0 2
#define LED_1 3
#define LED_2 4
#define LED_3 5

#define SENSOR_0 A0
#define SENSOR_1 A1
#define SENSOR_2 A2
#define SENSOR_3 A3

#define MOTOR_L_SPEED 6
#define MOTOR_L_DIR1 7
#define MOTOR_L_DIR2 8

#define MOTOR_R_SPEED 9
#define MOTOR_R_DIR1 10
#define MOTOR_R_DIR2 11

#define SENSOR_NUM 4

#define BLACK 0
#define WHITE 1

/* variable */
uint16_t sensor_value[4] ={ 0, 0, 0, 0 };
uint16_t sensor_ref[4] = { 0, 0, 0, 0 };
bool line[4] = { 0, 0, 0, 0 };

/* init function */

/* led function */
void led_init();
void led_toggle();
void led_indicate(bool* line);

/* sensor function */
void sensor_init();
void sensor_read(uint16_t* sensor_value);
void sensor_calibration(uint16_t cal_num, uint16_t* sensor_ref);
void monitor_init();
void sensor_monitor(uint16_t* sensor_value);

/* motor function */
void motor_init();
void motor_forward(uint8_t left, uint8_t right);

/* line function */
void line_detect(uint16_t* sensor_value, uint16_t* sensor_ref, bool* line);

void setup() 
{
  led_init();
  sensor_init();
  monitor_init();
  sensor_calibration(1000, sensor_ref);
}

void loop() 
{
  sensor_read(sensor_value);
  //sensor_monitor(sensor_value);
  line_detect(sensor_value, sensor_ref, line);
  led_indicate(line);

  motor_forward(200, 200);
}

void led_init()
{
    pinMode(LED_0, OUTPUT);
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);

    digitalWrite(LED_0, HIGH);
    digitalWrite(LED_1, HIGH);
    digitalWrite(LED_2, HIGH);
    digitalWrite(LED_3, HIGH);
}

void led_toggle()
{
  digitalWrite(LED_0, HIGH);
  digitalWrite(LED_1, HIGH);
  digitalWrite(LED_2, HIGH);
  digitalWrite(LED_3, HIGH);
  delay(500);     
  
  digitalWrite(LED_0, LOW);
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
  digitalWrite(LED_3, LOW);
  delay(500);  
}

void led_indicate(bool* line)
{
    if(line[0]== BLACK)
        digitalWrite(LED_0, HIGH);
    else 
        digitalWrite(LED_0, LOW);

    if(line[1]== BLACK)
        digitalWrite(LED_1, HIGH);
    else 
        digitalWrite(LED_1, LOW);

    if(line[2]== BLACK)
        digitalWrite(LED_2, HIGH);
    else 
        digitalWrite(LED_2, LOW);

    if(line[3]== BLACK)
        digitalWrite(LED_3, HIGH);
    else 
        digitalWrite(LED_3, LOW);
}

void sensor_init()
{
    pinMode(SENSOR_0,INPUT);
    pinMode(SENSOR_1,INPUT);
    pinMode(SENSOR_2,INPUT);
    pinMode(SENSOR_3,INPUT);
}

void sensor_read(uint16_t* sensor_value)
{
    sensor_value[0] = analogRead(SENSOR_0);
    sensor_value[1] = analogRead(SENSOR_1);
    sensor_value[2] = analogRead(SENSOR_2);
    sensor_value[3] = analogRead(SENSOR_3);
}

void sensor_calibration(uint16_t cal_num, uint16_t* sensor_ref)
{
    Serial.println("sensor calibration start");

    uint16_t sensor_temp[4] = { 0, 0, 0, 0 };
    uint16_t sensor_max[4] = { 0, 0, 0, 0 };
    uint16_t sensor_min[4] = { 1023, 1023, 1023, 1023 };

    for(int j = 0; j < cal_num; j++)
    {
        sensor_read(sensor_temp);

        for (int i = 0; i < SENSOR_NUM; i++)
        {
            sensor_max[i] = (sensor_max[i] > sensor_temp[i]) ? sensor_max[i] : sensor_temp[i];
            sensor_min[i] = (sensor_min[i] < sensor_temp[i]) ? sensor_min[i] : sensor_temp[i];
        }

        Serial.println(j);
    }

    for(int k = 0; k < SENSOR_NUM; k++)
        sensor_ref[k] = ((sensor_max[k] + sensor_min[k]) / 2);

    Serial.println("sensor calibration end");

    Serial.print(sensor_ref[0]);
    Serial.print("\t");

    Serial.print(sensor_ref[1]);
    Serial.print("\t");

    Serial.print(sensor_ref[2]);
    Serial.print("\t");

    Serial.print(sensor_ref[3]);
    Serial.println("\t");
}

void motor_init()
{
    pinMode(MOTOR_L_SPEED, OUTPUT); 
    pinMode(MOTOR_R_SPEED, OUTPUT); 

    pinMode(MOTOR_L_DIR1, OUTPUT);
    pinMode(MOTOR_L_DIR2, OUTPUT);
    pinMode(MOTOR_R_DIR1, OUTPUT);
    pinMode(MOTOR_R_DIR2, OUTPUT);
}

void motor_forward(uint8_t left, uint8_t right)
{
    // direction
    digitalWrite(MOTOR_R_DIR1, LOW);
    digitalWrite(MOTOR_R_DIR2, HIGH);
    digitalWrite(MOTOR_L_DIR1, HIGH);
    digitalWrite(MOTOR_L_DIR2, LOW);

    // speed
    analogWrite(MOTOR_L_SPEED, left);
    analogWrite(MOTOR_R_SPEED, right);
}

void monitor_init()
{
    Serial.begin(9600); // sensor value monitor
}

void sensor_monitor(uint16_t* sensor_value)
{
    Serial.print(sensor_value[0]);
    Serial.print("\t");

    Serial.print(sensor_value[1]);
    Serial.print("\t");

    Serial.print(sensor_value[2]);
    Serial.print("\t");

    Serial.print(sensor_value[3]);
    Serial.println("\t");
}

void line_detect(uint16_t* sensor_value, uint16_t* sensor_ref, bool* line)
{
    for(int i = 0; i < SENSOR_NUM; i++)
    {
        if(sensor_value[i] < sensor_ref[i])
            line[i] = BLACK;
        else 
            line[i] = WHITE;
    }
}