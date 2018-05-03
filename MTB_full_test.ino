#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

/* OLED */
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_DC     37
#define OLED_CS     31
#define OLED_RESET  33
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);

#if (SSD1306_LCDHEIGHT != 64)
  #error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
/* END OLED */

/* THERMO */
#include <Adafruit_MAX31855.h>
#define MAX_CS    48

Adafruit_MAX31855 thermocouple(MAX_CS);

double temp_ambient;
double temp_motor;
/* END THERMO */

/* ADC */
#include <Adafruit_ADS1015.h>

Adafruit_ADS1015 adc0;
Adafruit_ADS1015 adc1(0x4A);

#define ADC_MULT 2.0F
const double NUM_READINGS = 5.0;

double voltage;
double voltage_r[10];
int voltage_c = 0;

double current;
double current_r[10];
int current_c = 0;

double lift;
double lift_r[10];
int lift_c = 0;

double lift_cal = 0.00;
/* END ADC */

/* BLE */
#define BLE Serial1
#define BUFSIZE 64

boolean USING_BLE = false;

uint8_t service_index = 1;

int char_voltage_index = 1;
int char_current_index = 2;
int char_lift_index = 3;
int char_ambient_temp_index = 4;
int char_motor_temp_index = 5;
int char_pulse_index = 6;
int char_pulse_control_index = 7;
/* END BLE */

/* ESC */
#include <Servo.h>

Servo esc;

#define ESC_MIN 1000
#define ESC_MAX 2000

double pulse = 1.0;
/* END ESC */

/* BUTTONS */
#define BUTTON_WAIT 1000 //timer will overflow after ~45 days - could possibly cause spike in pulse on button press

unsigned long buttons[4];
/* END BUTTONS */

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC);

  esc.attach(2, ESC_MIN, ESC_MAX);

  display.display();
  delay(2000);
  display.clearDisplay();

  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  buttons[0] = millis();
  buttons[1] = millis();
  buttons[2] = millis();
  buttons[3] = millis();

  adc0.begin();
  adc1.begin();
  adc0.setGain(GAIN_ONE);
  adc1.setGain(GAIN_ONE);

  Serial.begin(9600);
  if (USING_BLE) {
    BLE.begin(9600);
  }

  //setupBLE(); //allow this method to execute to provision the BLE module 
                    //(should only need to be done once per module)
}

void loop() {
  readADC();
  checkButtons();
  if (USING_BLE) {
    ble_tick();
  }
  doESC();
  readTemp();
  drawScreen();
  delay(50);
}

//handles reading ADC data from voltage, current, force sensors
//takes NUM_READINGS readings
//averages all NUM_READINGS readings on last of NUM_READINGS
void readADC() {
  double voltage_read = adc0.readADC_Differential_0_1();
  voltage_r[voltage_c] = ((voltage_read * ADC_MULT) / 1000.0)*4.0;
  if (voltage_c == (NUM_READINGS - 1)) {
    double total = 0.0;
    for (double d : voltage_r) {
      total += d;
    }
    voltage = (total / NUM_READINGS);
    voltage_c = 0;
  } else {
    voltage_c++;
  }
  
  double current_read = adc0.readADC_Differential_2_3();
  current_r[current_c] = (((current_read * ADC_MULT) / 1000.0) - 0.5092) / 0.1295;
  if (current_c == (NUM_READINGS - 1)) {
    double total = 0.0;
    for (double d : current_r) {
      total += d;
    }
    current = (total / NUM_READINGS);
    current_c = 0;
  } else {
    current_c++;
  }
  
  double lift_read = adc1.readADC_Differential_0_1();
  lift_r[lift_c] = (((((((lift_read) * ADC_MULT) * 1.25) / 1000.0) - 2.9939) / -0.0017) / 9.81) / 100.0;
  if (lift_c == (NUM_READINGS - 1)) {
    double total = 0.0;
    for (double d : lift_r) {
      total += d;
    }
    lift = (total / NUM_READINGS);
    lift_c = 0;
  } else {
    lift_c++;
  }
}

//checks for invalid pulse widths
//write the PWM signal to the pin
void doESC() {
  if (pulse < 1.0 || pulse > 2.0) {
    pulse = 1.0;
  }
  esc.writeMicroseconds(pulse * 1000.0);
}

//checks for button inputs
//BUTTON_WAIT delay between button presses to disallow pulse spikes or cliffs
void checkButtons() {
  //buttons will have slight delay in BLE mode. change USING_BLE to false for optimal local input
  if (digitalRead(4) == LOW) {
    if (millis() - buttons[0] >= BUTTON_WAIT) {
      if (pulse < 2.0) {
        pulse += 0.1;
      }
      buttons[0] = millis();
    }
  }
  if (digitalRead(5) == LOW) {
    if (millis() - buttons[1] >= BUTTON_WAIT) {
      if (pulse > 1.0) {
        pulse -= 0.1;
      }
      buttons[1] = millis();
    }
  }
  if (digitalRead(6) == LOW) {
    if (millis() - buttons[2] >= BUTTON_WAIT) {
      buttons[2] = millis();
      lift_cal = lift;
    }
  }
  if (digitalRead(7) == LOW) {
    if (millis() - buttons[3] >= BUTTON_WAIT) {
      buttons[3] = millis();
    }
  }
}

//read temperature data
void readTemp() {
  temp_ambient = thermocouple.readInternal();
  temp_motor = thermocouple.readCelsius();
}

//draw data onto the screen, nothing special here
void drawScreen() {
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(BLACK, WHITE);
  display.setCursor(0,0);

  if (USING_BLE) {
    display.print("  Pulse msec: ");
  } else {
    display.print("!!Pulse msec: ");
  }
  display.print(pulse);
  display.println("  ");
  
  display.setTextColor(WHITE);
  
  display.print("Voltage V:     ");
  display.print(voltage);
  display.println();
  
  display.print("Current A:     ");
  display.print(current);
  display.println();
  
  display.print("Power W:       ");
  display.print(voltage * current);
  display.println();
  
  display.print("Lift kg:       ");
  display.print(lift);
  display.println();
  
  display.print("Ambient Temp C: ");
  display.print(temp_ambient);
  display.println();
  
  display.print("Motor Temp C:   ");
  display.print(temp_motor);
  display.println();
  
  display.print("Eff kg/watt:  ");
  display.println((voltage * current) / lift);
  
  display.display();
}

//setup procedure for BLE UART Friend module
//errors sometimes occur, device will not work properly unless all setup steps complete correctly
//disable this method in setup() after the BLE device has been provisioned
boolean setupBLE() {
  //code must be altered to support double-digit service or characteristic indicies
  delay(1000);
  char response[32];
  
  char cmd_reset[] = "AT+FACTORYRESET\n";
  sendATCommand(cmd_reset, response);
  if (strcmp(response, "ERROR") == 0) {
    return false;
  }
  memset(response, 0, sizeof(response));

  char cmd_disable_echo[] = "ATE=0\n";
  sendATCommand(cmd_disable_echo, response);
  if (strcmp(response, "ERROR") == 0 || strcmp(response, "ATE=0OK") != 0) {
    return false;
  }
  memset(response, 0, sizeof(response));

  char cmd_gatt_reset[] = "AT+GATTCLEAR\n";
  sendATCommand(cmd_gatt_reset, response);
  if (strcmp(response, "ERROR") == 0) {
    return false;
  }
  memset(response, 0, sizeof(response));
  
  char cmd_add_service[] = "AT+GATTADDSERVICE=UUID128=d3-83-06-df-36-9a-4c-72-b3-93-2c-99-c0-3e-b8-1a\n";
  sendATCommand(cmd_add_service, response);
  if (strcmp(response, "ERROR") == 0) {
    return false;
  }
  service_index = (int)response[0] - 48;
  memset(response, 0, sizeof(response));

  char cmd_add_voltage_char[] = "AT+GATTADDCHAR=UUID=0x2A30,PROPERTIES=0x12,MIN_LEN=1,MAX_LEN=20,VALUE=0.00,DESCRIPTION=VOLTAGE\n";
  sendATCommand(cmd_add_voltage_char, response);
  if (strcmp(response, "ERROR") == 0) {
    return false;
  }
  char_voltage_index = (int)response[0] - 48;
  memset(response, 0, sizeof(response));

  char cmd_add_current_char[] = "AT+GATTADDCHAR=UUID=0x2A31,PROPERTIES=0x12,MIN_LEN=1,MAX_LEN=20,VALUE=0.00,DESCRIPTION=CURRENT\n";
  sendATCommand(cmd_add_current_char, response);
  if (strcmp(response, "ERROR") == 0) {
    return false;
  }
  char_current_index = (int)response[0] - 48;
  memset(response, 0, sizeof(response));

  char cmd_add_lift_char[] = "AT+GATTADDCHAR=UUID=0x2A32,PROPERTIES=0x12,MIN_LEN=1,MAX_LEN=20,VALUE=0.00,DESCRIPTION=LIFT\n";
  sendATCommand(cmd_add_lift_char, response);
  if (strcmp(response, "ERROR") == 0) {
    return false;
  }
  char_lift_index = (int)response[0] - 48;
  memset(response, 0, sizeof(response));

  char cmd_add_ambient_char[] = "AT+GATTADDCHAR=UUID=0x2A33,PROPERTIES=0x12,MIN_LEN=1,MAX_LEN=20,VALUE=0.00,DESCRIPTION=AMBIENTTEMP\n";
  sendATCommand(cmd_add_ambient_char, response);
  if (strcmp(response, "ERROR") == 0) {
    return false;
  }
  char_ambient_temp_index = (int)response[0] - 48;
  memset(response, 0, sizeof(response));

  char cmd_add_motor_char[] = "AT+GATTADDCHAR=UUID=0x2A34,PROPERTIES=0x12,MIN_LEN=1,MAX_LEN=20,VALUE=0.00,DESCRIPTION=MOTORTEMP\n";
  sendATCommand(cmd_add_motor_char, response);
  if (strcmp(response, "ERROR") == 0) {
    return false;
  }
  char_motor_temp_index = (int)response[0] - 48;
  memset(response, 0, sizeof(response));

  char cmd_add_pulse_char[] = "AT+GATTADDCHAR=UUID=0x2A35,PROPERTIES=0x12,MIN_LEN=1,MAX_LEN=20,VALUE=1.0,DESCRIPTION=PULSE\n";
  sendATCommand(cmd_add_pulse_char, response);
  if (strcmp(response, "ERROR") == 0) {
    return false;
  }
  char_pulse_index = (int)response[0] - 48;
  memset(response, 0, sizeof(response));
  
  char cmd_add_pulse_control_char[] = "AT+GATTADDCHAR=UUID=0x2A36,PROPERTIES=0x08,MIN_LEN=1,MAX_LEN=20,VALUE=-1,DESCRIPTION=PULSECON\n";
  sendATCommand(cmd_add_pulse_control_char, response);
  if (strcmp(response, "ERROR") == 0) {
    return false;
  }
  char_pulse_control_index = (int)response[0] - 48;
  memset(response, 0, sizeof(response));

  sendATCommand("ATZ\n", response);
  if (strcmp(response, "ERROR") == 0) {
    return false;
  }
  memset(response, 0, sizeof(response));

  sendATCommand("AT+GAPSTOPADV\n", response);
  if (strcmp(response, "ERROR") == 0) {
    return false;
  }
  memset(response, 0, sizeof(response));

  sendATCommand("AT+SETADVDATA=02-01-06-05-02-30-2a-31-2a-32-2a-33-2a-34-2a-35-2a\n", response);
  if (strcmp(response, "ERROR") == 0) {
    //return false;
  }
  memset(response, 0, sizeof(response));

  sendATCommand("AT+GAPSTARTADV\n", response);
  if (strcmp(response, "ERROR") == 0) {
    return false;
  }
  memset(response, 0, sizeof(response));
  return true;
}

//used only in BLE mode
//sends telemetry to BLE device for app to read. checks and acts upon and user-controlled characteristics
void ble_tick() {
  char response[32];
  char y[6];

  char cmd_disable_echo[] = "ATE=0\n";
  sendATCommand(cmd_disable_echo, response);
  BLE.flush();
  if (strcmp(response, "ERROR") == 0 || 
      (strcmp(response, "ATE=0OK") != 0 && strcmp(response, "OK") != 0)) {
    return false;
  }
  memset(response, 0, sizeof(response));
  memset(y, 0, sizeof(y));
  
  char cmd_set_voltage[20];
  String(voltage, 2).toCharArray(y, 6);
  sprintf(cmd_set_voltage, "AT+GATTCHAR=%d,%s%s", char_voltage_index, y, '\n');
  sendATCommand(cmd_set_voltage, response);
  BLE.flush();
  memset(response, 0, sizeof(response));
  memset(y, 0, sizeof(y));

  char cmd_set_current[20];
  String(current, 2).toCharArray(y, 6);
  sprintf(cmd_set_current, "AT+GATTCHAR=%d,%s%s", char_current_index, y, '\n');
  sendATCommand(cmd_set_current, response);
  BLE.flush();
  memset(response, 0, sizeof(response));
  memset(y, 0, sizeof(y));

  char cmd_set_lift[20];
  String(lift, 2).toCharArray(y, 6);
  sprintf(cmd_set_lift, "AT+GATTCHAR=%d,%s%s", char_lift_index, y, '\n');
  sendATCommand(cmd_set_lift, response);
  BLE.flush();
  memset(response, 0, sizeof(response));
  memset(y, 0, sizeof(y));

  char cmd_set_ambient[20];
  String(temp_ambient, 2).toCharArray(y, 6);
  sprintf(cmd_set_ambient, "AT+GATTCHAR=%d,%s%s", char_ambient_temp_index, y, '\n');
  sendATCommand(cmd_set_ambient, response);
  BLE.flush();
  memset(response, 0, sizeof(response));
  memset(y, 0, sizeof(y));

  char cmd_set_motor[20];
  String(temp_motor, 2).toCharArray(y, 6);
  sprintf(cmd_set_motor, "AT+GATTCHAR=%d,%s%s", char_motor_temp_index, y, '\n');
  sendATCommand(cmd_set_motor, response);
  BLE.flush();
  memset(response, 0, sizeof(response));
  memset(y, 0, sizeof(y));
  
  char cmd_get_control_pulse[20];
  sprintf(cmd_get_control_pulse, "AT+GATTCHAR=%i%s", char_pulse_control_index, '\n');
  sendATCommand(cmd_get_control_pulse, response);
  BLE.flush();
  if (strcmp(response, "-1OK") != 0) {
    if (strcmp(response, "ERROR") == 0) {
      return;
    }
    char pulse_c[3];
    pulse_c[0] = response[0];
    pulse_c[1] = response[1];
    pulse_c[2] = response[2];

    double _pulse = atof(pulse_c);

    pulse = _pulse;

    char cmd_set_control_pulse[20];
    sprintf(cmd_set_control_pulse, "AT+GATTCHAR=%d,%d%s", char_pulse_control_index, -1, '\n');
    sendATCommand(cmd_set_control_pulse, response);
    memset(response, 0, sizeof(response));
    BLE.flush();
  }
  memset(response, 0, sizeof(response));

  char cmd_set_pulse[20];
  String(pulse, 2).toCharArray(y, 6);
  sprintf(cmd_set_pulse, "AT+GATTCHAR=%d,%s%s", char_pulse_index, y, '\n');
  sendATCommand(cmd_set_pulse, response);
  memset(response, 0, sizeof(response));
  memset(y, 0, sizeof(y));
  BLE.flush();
}

//sends AT commands over Serial1 to BLE device
//has built-in timeout and OK/ERROR message parsing
//strips \r and \n from responses
void sendATCommand(char cmd[], char* resp) {
  Serial.print("Command: ");
  Serial.print(cmd);
  BLE.write(cmd);
  BLE.flush();

  delay(50);
  resp[0] = 0;
  unsigned long start = millis();
  while (true) {
    if (millis() - start > 5000) {
      break; //timeout
    }
    if (
        (resp[0] == 'O' && 
         resp[1] == 'K') //format of OK
         || 
        (resp[1] == 'O' && 
         resp[2] == 'K') //format of <index>OK
         || 
        (resp[2] == 'O' && 
         resp[3] == 'K') //format of <-1>OK
         || 
        (resp[3] == 'O' && 
         resp[4] == 'K') //format of <x.x>OK
         ){
          break; //known-state, break now
    } 
    if (resp[0] == 'E' &&
        resp[1] == 'R' &&
        resp[2] == 'R' &&
        resp[3] == 'O' &&
        resp[4] == 'R') {
          //format of ERROR
          break; //known-state, break now
    }
    while (BLE.available()) {
      int r = BLE.read();
      if ((char)r == '\r') {
        //Serial.println("R");
        continue;
      }
      if ((char)r == '\n') {
        //Serial.println("N");
        continue;
      }
      sprintf(resp, "%s%c", resp, (char)r); //add to response
    }
    delay(200);
  }
  Serial.print("Response: ");
  Serial.println(resp);
}
