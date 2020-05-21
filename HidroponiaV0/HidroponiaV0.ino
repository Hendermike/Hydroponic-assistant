class Relay {
  private:
    byte pin;
  public:
    Relay(byte pin) {
      this->pin = pin;
      init();
    }

    void init() {
      pinMode(pin, OUTPUT);
      off();
    }

    void on() {
      digitalWrite(pin, LOW); 
    }

    void off(){
      digitalWrite(pin, HIGH);
    }
};

class PeristalticPump {
  private:
    byte a_pin;
    byte b_pin;
    unsigned long interval;
    unsigned long currentPumpMillis; 
    unsigned long lastPumpMillis;
  public:      
    PeristalticPump(byte a_pin, byte b_pin, unsigned long interval) {
      this->a_pin = a_pin;
      this->b_pin = b_pin;
      this->interval = interval;
      init();
    }

    void init() {
      pinMode(a_pin, OUTPUT);
      pinMode(b_pin, OUTPUT);
      off();
    }

    void off() {
      analogWrite(a_pin, 0);
      analogWrite(b_pin, 0);
    }

    void pumpCycle() {
      int value = 0;
      int max_value = 200;   
      /*  ACCELERATION REGIME  */
      while (value < max_value) { //ACELERANDO  
        analogWrite(a_pin, value);
        
        lastPumpMillis = 0;
        currentPumpMillis = millis();
        while(lastPumpMillis < 5) {
          lastPumpMillis = millis() - currentPumpMillis;
        }
        
        value = value + 5;
      }
      /*  STATIONARY REGIME  */
      analogWrite(a_pin, max_value);
      
      lastPumpMillis = 0;
      currentPumpMillis = millis();
      while(lastPumpMillis < interval) {
         lastPumpMillis = millis() - currentPumpMillis;
      }

      /*  DECELERATION REGIME  */
      for (int x = 0; x <= max_value - 1; x++) {
        value--;
        analogWrite(a_pin, value);
        
        lastPumpMillis = 0;
        currentPumpMillis = millis();
        while(lastPumpMillis < 5) {
          lastPumpMillis = millis() - currentPumpMillis;
        }  
      }
      analogWrite(a_pin, 0);
    }
};

   
double EC_MEASURE;
unsigned long int EC_RAW_MEASURE;
int EC_MIN_REFERENCE = 57; // Valor promedio para solución de EC 1.413 ms/cm
int EC_MAX_REFERENCE = 462; // Valor promedio para solución de EC 12.880 ms/cm
class EC {
   /*  MEASUREMENT AND MODEL RELATED VARIABLES DEFINITION  */
  double EC_MODEL_SLOPE = (12.880 - 1.413)/double(EC_MAX_REFERENCE - EC_MIN_REFERENCE);
  double EC_MODEL_COEF = 1.413 - double(EC_MIN_REFERENCE)*EC_MODEL_SLOPE;
  unsigned long int EC_SAMPLES_SUM; 
  int EC_BUFFER[10],EC_TEMPORAL; 
  /*  PHYSICAL ADDRESS RELATED VARIABLES  */  
  const int EC_PIN = A1;
  public:
    void setup() {
      pinMode(EC_PIN, INPUT);
    }

    void loop() {
    }

    void sample(String MODE) {
      for(int i=0;i<10;i++){ 
        EC_BUFFER[i]=analogRead(EC_PIN);
        delay(10);
        }
 
      for(int i=0;i<9;i++){
        for(int j=i+1;j<10;j++){
          if(EC_BUFFER[i]>EC_BUFFER[j]){
            EC_TEMPORAL=EC_BUFFER[i];
            EC_BUFFER[i]=EC_BUFFER[j];
            EC_BUFFER[j]=EC_TEMPORAL;
            }
          }
       }
       
      EC_SAMPLES_SUM=0;
      for(int i=2;i<8;i++)
      EC_SAMPLES_SUM+=EC_BUFFER[i];

      if (MODE == "CALIBRATION") {
        EC_RAW_MEASURE = EC_SAMPLES_SUM/6;
      } else if (MODE == "MEASURE") {     
        EC_MEASURE = EC_MODEL_SLOPE*(EC_SAMPLES_SUM/6) + EC_MODEL_COEF;
      }
    
    }
    
};


float PH_MEASURE;
unsigned long int PH_RAW_MEASURE;
int PH_MIN_REF = 417; // Valor promedio para solución de pH 4
int PH_MAX_REF = 310; // Valor promedio para solución de pH 7
class pH {
  /*  MEASUREMENT AND MODEL RELATED VARIABLES DEFINITION  */
  float PH_MODEL_SLOPE = 3/float(PH_MAX_REF - PH_MIN_REF);
  float PH_MODEL_COEF = 7 - float(PH_MAX_REF)*PH_MODEL_SLOPE;
  int PH_BUFFER[10],PH_TEMPORAL;
  unsigned long int PH_SAMPLES_SUM;
  /*  PHYSICAL ADDRESS RELATED VARIABLES  */  
  const int PH_PIN = A2; 
  
  public:
    void setup() {
      pinMode(PH_PIN, INPUT);
    }

       void loop() {
      //PH_MEASURE = UPDATE(); 
      }

    float sample(String MODE) {
      for(int i=0;i<10;i++){ 
        PH_BUFFER[i]=analogRead(PH_PIN);
        delay(10);
        }
 
      for(int i=0;i<9;i++){
        for(int j=i+1;j<10;j++){
          if(PH_BUFFER[i]>PH_BUFFER[j]){
            PH_TEMPORAL=PH_BUFFER[i];
            PH_BUFFER[i]=PH_BUFFER[j];
            PH_BUFFER[j]=PH_TEMPORAL;
            }
          }
      }
      
      PH_SAMPLES_SUM=0;
      for(int i=2;i<8;i++)
      PH_SAMPLES_SUM+=PH_BUFFER[i];
 
      /* OPCIONES DE MEDICIÓN */
      float PH_MODEL_OUTPUT = PH_MODEL_SLOPE*(PH_SAMPLES_SUM/6) + PH_MODEL_COEF;
      /* OPCIONES DE CALIBRACIÓN */
       //float PH_MODEL_OUTPUT = (PH_SAMPLES_SUM/6);
      
      if (MODE == "CALIBRATION") {
        PH_RAW_MEASURE = PH_SAMPLES_SUM/6;
      } else if (MODE == "MEASURE") {     
        PH_MEASURE = PH_MODEL_OUTPUT;
      }
    }

};

long DURATION;
int DISTANCE;
float US_MEASURE = 0;
float US_MIN_REFERENCE;
float US_MAX_REFERENCE;
class Ultrasonido {
  // defines constant
  private:
  byte trig_pin;
  byte echo_pin;
  float us_ref;
  public:
    Ultrasonido(byte trig_pin, byte echo_pin, float us_ref) {
      this->trig_pin = trig_pin;
      this->echo_pin = echo_pin;
      this->us_ref = us_ref;
    }
    void setup() {
      pinMode(trig_pin, OUTPUT); // Sets the TRIG_PIN as an Output
      pinMode(echo_pin, INPUT);
    }

    void loop() {
    }

    float distancia() {
    // Clears the TRIG_PIN
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);

    DURATION = 0;
    //for (int i=0; i<5; i++) {
      // Sets the TRIG_PIN on HIGH state for 10 micro seconds
      digitalWrite(trig_pin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig_pin, LOW);
      // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
      DURATION = pulseIn(echo_pin, HIGH);
    //}
    //DURATION = DURATION/5; 
    
    // Calculating the DISTANCE
    float d = DURATION*0.034/2;
    return d;
    }

    void sample() {
      //float n = US_REF - distancia();
      US_MEASURE = distancia();  
    }
    
};

#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>

LiquidCrystal_PCF8574 lcd(0x27);

class LiquidCrystalI2C {
  public:
    void setup() {
      lcd.begin(16, 2);
      lcd.clear();
    }
    void refresh() {
      lcd.clear();
    }
    void measureDisplayUpdate() {
      lcd.setBacklight(255);
      lcd.home();
      lcd.clear();
      // Nivel
      lcd.setCursor(0, 0);
      lcd.print("Nivel:");
      lcd.print(US_MEASURE,1);
      // pH
      lcd.setCursor(0, 1);
      lcd.print("pH:");
      lcd.print(PH_MEASURE, 1);
      // EC
      lcd.print(" ");
      lcd.print("EC:");
      lcd.print(EC_MEASURE, 1);
    }

    void showTextOnRow(String text, int fila) {
      lcd.setBacklight(255);
      lcd.home();
      lcd.clear();
      lcd.setCursor(fila, 0);
      lcd.print(text);
    }
    
}; 
/***************************************/
/* SPECIFIC IMPLEMENTATION DEFINITIONS */
/***************************************/
/* RELAY PINS */
#define WATER_IN_PIN 22
#define WATER_OUT_PIN 23
#define LIGHT_PIN 24
#define PERISTALTIC_PIN 25
#define HEATER_PIN 26
/* PERISTALTIC PUMPS PINS */
#define P1_A_PIN 4
#define P1_B_PIN 5
#define P2_A_PIN 6
#define P2_B_PIN 7
#define PCALMAG_A_PIN 3
#define PCALMAG_B_PIN 2
#define PNUTRI_A_PIN 1
#define PNUTRI_B_PIN 0
/* ULTRASONIC PINS */
#define TRIG_PIN 9
#define ECHO_PIN 8
/* BLUETOOTH LINK PINS */
#define BT_STATE_PIN 28
#define BT_RX_PIN 11
#define BT_TX_PIN 10

/* RELAY INITIALIZATION */
Relay WATER_IN_CHANNEL(WATER_IN_PIN);
Relay WATER_OUT_CHANNEL(WATER_OUT_PIN);
Relay LIGHT_CHANNEL(LIGHT_PIN);
Relay PERISTALTIC_CHANNEL(PERISTALTIC_PIN);
Relay HEATER_CHANNEL(LIGHT_PIN);

/* TIME CONSTANTS AND REFERENCES */
const unsigned long PUMP_INTERVAL = 2000; //2 segundos
const unsigned long MEASURE_INTERVAL = 1000; //30 segundos
const unsigned long PH_CONTROL_INTERVAL = 1000; //300 segundos
const unsigned long EC_CONTROL_INTERVAL = 1000; //150 segundos
const unsigned long US_CONTROL_INTERVAL = 250; //0,25 segundos
float US_REF = 34.7;

#include <SoftwareSerial.h>   // Incluimos la librería  SoftwareSerial 

/* PERISTALTIC PUMPS INITIALIZATION */
PeristalticPump PUMP1(P1_A_PIN, P1_B_PIN, PUMP_INTERVAL); //PH
PeristalticPump PUMP2(P2_A_PIN, P2_B_PIN, PUMP_INTERVAL); //EC
PeristalticPump PUMP_CALMAG(PCALMAG_A_PIN, PCALMAG_B_PIN, PUMP_INTERVAL);
PeristalticPump PUMP_NUTRI(PNUTRI_A_PIN, PNUTRI_B_PIN, PUMP_INTERVAL);

/* SENSORS INITIALIZATION */
EC EC;
pH PH;
Ultrasonido US(TRIG_PIN, ECHO_PIN, US_REF);

/* SCREEN INITIALIZATION */
LiquidCrystalI2C SCREEN;

/* BT LINK INITIALIZATION */
SoftwareSerial BT(BT_RX_PIN,BT_TX_PIN);    // Definimos los pines RX y TX del Arduino conectados al Bluetooth
char msg[28] = "/X/Y/Z/W/hh:mm:ss/mm:ss:xxx";
int i = 0;
bool VALID_MSG = false;

unsigned long currentMillis, lastMillis;

String GROWTH_STAGE = "FLOWERING"; //FLORATION, VEGETATION
String GLOBAL_MODE = "STAND_BY"; //"STAND_BY", "MEASURE", "CONTROL", "BT"
String CONTROL_MODE = "OFF"; //"LEVEL", "PH", "EC", "OFF"
double US_MIN, US_MAX, PH_MIN, PH_MAX, EC_MIN, EC_MAX, EC_CALMAG_TH, EC_NUTRI_TH;
bool WATER_IN, WATER_OUT, PH_UP,PH_DOWN, EC_UP, EC_DOWN, EC_ADJUST; 

void setup() {
  // put your setup code here, to run once:
  SCREEN.setup();
  US.setup();
  PH.setup();
  EC.setup();
  PERISTALTIC_CHANNEL.init();
  WATER_IN_CHANNEL.init();
  WATER_OUT_CHANNEL.init();
  PUMP1.init();
  PUMP2.init();

  Serial.begin(9600);
  BT.begin(9600);
  pinMode(BT_STATE_PIN, INPUT);
}

void loop() {  

  if (GLOBAL_MODE == "STAND_BY") {
      GLOBAL_MODE = "CONFIG";
  }
  else if (GLOBAL_MODE == "CONTROL") {
    
    controlModeCheck();
    
    if (CONTROL_MODE != "OFF") {
      
      currentMillis = millis() - lastMillis;
      
      /*SCREEN.showTextOnRow(CONTROL_MODE,1);
      wait(1000);
      SCREEN.showTextOnRow("CONTROL MODE",0);
      wait(1000);*/
      
      if (CONTROL_MODE == "LEVEL" && currentMillis > US_CONTROL_INTERVAL) {
        /*  WATER IN OR OUT */
        if(WATER_IN) {
          SCREEN.showTextOnRow("WATER IN",0);
          wait(1000);
          WATER_IN_CHANNEL.on();
        }
        else if(WATER_OUT) {
          SCREEN.showTextOnRow("WATER OUT",0);
          wait(1000);
          WATER_OUT_CHANNEL.on();
        } else {
          WATER_IN_CHANNEL.off();
          WATER_IN_CHANNEL.off();
        }
        SCREEN.measureDisplayUpdate();
        //wait(US_CONTROL_INTERVAL);
        lastMillis = millis();
        
      }
      else if (CONTROL_MODE == "PH" && currentMillis > PH_CONTROL_INTERVAL) {
        if (PH_UP) {
          SCREEN.showTextOnRow("PH < PH MIN",0);
          wait(1000);
          SCREEN.showTextOnRow("Subiendo pH...",0);
          wait(1000);
          //TODO: IMPLEMENTAR Y CONFIGURAR BOMBA ADICIONAL
        }
        else if (PH_DOWN) {
          SCREEN.showTextOnRow("PH > PH MAX",0);
          wait(1000);
          SCREEN.showTextOnRow("PH DOWN",0);
          wait(1000);
          PUMP1.pumpCycle();
        }
        lastMillis = millis();
        /*  WAIT FOR THE SOLUTION TO SET  */  
        /*lastMillis = 0;
        currentMillis = millis();
        while(lastMillis < PH_CONTROL_INTERVAL) {
          lastMillis = millis() - currentMillis;
          globalSample();
          SCREEN.measureDisplayUpdate();
        }*/
      }
      else if (CONTROL_MODE == "EC" && currentMillis > EC_CONTROL_INTERVAL) {
        /*  EC CONTROL MODE */
        if (EC_UP) {
          /* EC UP CONTROL */
          SCREEN.showTextOnRow("EC < EC_MIN", 0);
          wait(1000);
          SCREEN.showTextOnRow("APPLYING",0);
          wait(1000);
          SCREEN.showTextOnRow("EC UP",0);
          wait(1000);
          /*  APPLY SOLUTION 2  */
          PUMP2.pumpCycle();
          /*lastMillis = 0;
          currentMillis = millis();
          while(lastMillis < EC_CONTROL_INTERVAL) {
            lastMillis = millis() - currentMillis;
            wait(1000);
            globalSample();
            SCREEN.measureDisplayUpdate();
          }*/
          lastMillis = millis();
        }
        else if (EC_DOWN) {
          // Esta rutina si tiene bloque de programa
          /*  WATER CLEANING  */
          //TODO: IMPLEMENTAR Y CONFIGURAR BOMBA ADICIONAL  
          SCREEN.showTextOnRow("EC > EC_MAX", 0);
          wait(1000);
          SCREEN.showTextOnRow("EC DOWN",0);
          wait(1000);
          SCREEN.showTextOnRow("WATER FLUSH",0);
          wait(1000);
          /*  EXTRACT 1/4 OF TOTAL VOLUME */
          SCREEN.showTextOnRow("WATER OUT",0);
          wait(1000); 
          SCREEN.showTextOnRow(String(0.75*(US_MAX + US_MIN)/2),0);
          wait(1000);
          WATER_OUT_CHANNEL.on(); 
          while(US_MEASURE > 0.75*(US_MAX + US_MIN)/2) {
            wait(US_CONTROL_INTERVAL);
            US.sample();
            SCREEN.measureDisplayUpdate();
          }
          WATER_OUT_CHANNEL.off();
          /*  RESTORE VOLUME */
          SCREEN.showTextOnRow("WATER IN",0);
          wait(1000);
          WATER_IN_CHANNEL.on(); 
          while(US_MEASURE < (US_MAX + US_MIN)/2) {
            wait(US_CONTROL_INTERVAL);
            US.sample();
            SCREEN.measureDisplayUpdate();
          }
          WATER_IN_CHANNEL.off();
        }
        else if(EC_ADJUST) {
          /*  ADJUST NEW WATER BATCH  */
          SCREEN.showTextOnRow("CONDITIONING", 0);
          wait(1000);
          SCREEN.showTextOnRow("WATER", 0);
          wait(1000);
          EC_CALMAG_TH = 20;
          EC_NUTRI_TH = 1400;         
          if(EC_MEASURE < EC_CALMAG_TH) {
            SCREEN.showTextOnRow("EC < CALMAG TH", 0);
            wait(1000);
            SCREEN.showTextOnRow("CALMAG IN", 0);
            wait(1000);
            /*PUMP_CALMAG.pumpCycle();*/
          }
          else if (EC_MEASURE >= EC_CALMAG_TH && EC_MEASURE < EC_NUTRI_TH) {
            SCREEN.showTextOnRow("EC > CALMAG TH", 0);
            wait(1000);
            SCREEN.showTextOnRow("EC < NUTRI TH", 0);
            wait(1000);
            SCREEN.showTextOnRow("NUTRI IN", 0);
            wait(1000);
            /*PUMP_NUTRI.pumpCycle();*/
          }
          /*currentMillis =  millis();
          lastMillis = 0;
          while(lastMillis < EC_CONTROL_INTERVAL) {
            lastMillis = millis() - currentMillis;
            globalSample();
            SCREEN.measureDisplayUpdate();
          }*/
          lastMillis = millis();
        }    
      }
      globalSample();
      controlModeCheck(); 
    }
    globalSample();
  }
  else if (GLOBAL_MODE == "MEASURE" && currentMillis > MEASURE_INTERVAL) {
    /*  UPDATE ALL SENSORS MEASUREMENT  */
    globalSample();
    /*  DISPLAY UPDATE */
    SCREEN.measureDisplayUpdate();  
    /*  WAIT FOR MEASURE INTERVAL  */
    //wait(MEASURE_INTERVAL);
    /*  PARAMETER CHECK */
    globalModeCheck();
    
    lastMillis = millis();
  }
  else if (GLOBAL_MODE == "CONFIG") {
    GROWTH_STAGE_CONFIG(GROWTH_STAGE);
    GLOBAL_MODE = "BT";
  }
  else if (GLOBAL_MODE == "BT") {
    /* BLUETOOTH EMPAREJADO */
    if(BT.available()) {
      msg[i] = BT.read();
      i = i + 1;
    } else {
      i = 0;
      msg_processing();
      delay(1000);
    }
    
  }
  if (GLOBAL_MODE != "BT") {
    SCREEN.showTextOnRow(GLOBAL_MODE,0);
    wait(1000);
    SCREEN.showTextOnRow("GLOBAL MODE",0);
    wait(1000);
  } else {
    SCREEN.showTextOnRow("BT LINK ON",0);
  }
  /* GLOBAL LOOP END */
}

void GROWTH_STAGE_CONFIG(String GROWTH_STAGE) {
  /*  US DESIRED RANGE SETTING  */
  US_MIN = 14;
  US_MAX = 16;
  /* FLORATION AND VEGETATION STAGE CONFIG */
  if (GROWTH_STAGE == "FLOWERING") {
    PH_MAX = 7;
    PH_MIN = 6;
    EC_MIN = 900; //EC_MIN -> EC_MIN
    EC_MAX = 1400; //EC_MAX -> EC_NUTRI 
  }
  else if (GROWTH_STAGE == "VEGETATION") {
    PH_MAX = 7;
    PH_MIN = 6;
    EC_MIN = 900;
    EC_MAX = 1400;    
  }
}

void globalSample() {
    PH.sample("MEASURE");
    EC.sample("MEASURE");
    US.sample();
}

void globalModeCheck() {
  /* MEASURES CHECK AND STATE UPDATE */
  measuresCheck();
  /*  GLOBAL MODE TRANSITION CHECK */
  if (digitalRead(BT_STATE_PIN) == 1) {
    GLOBAL_MODE = "BT";
  }
  else if (EC_UP || EC_DOWN || PH_UP || PH_DOWN || WATER_IN || WATER_OUT || EC_ADJUST) {
    GLOBAL_MODE = "CONTROL";
  }
  else {
    GLOBAL_MODE = "MEASURE";
  } 
}

void measuresCheck() {
  /*  US CHECK  */
  if (US_MEASURE < US_MIN) {
    WATER_IN = false;
    WATER_OUT = !WATER_IN;
  }
  else if (US_MEASURE > US_MAX) {
    WATER_IN = true;
    WATER_OUT = !WATER_IN;    
  }
  else {
    WATER_IN = false;
    WATER_OUT = false;
  }
  /*  PH CHECK  */
  if (PH_MEASURE < PH_MIN) {
    PH_UP = true;
    PH_DOWN = !PH_UP;
  }
  else if (PH_MEASURE > PH_MAX) {
    PH_UP = false;
    PH_DOWN = !PH_UP;
  }
  else {
    PH_UP = false;
    PH_DOWN = PH_UP;
  }
  /*  EC CHECK */
  if (EC_MEASURE < EC_CALMAG_TH) {
    EC_ADJUST = true;
    EC_DOWN = false;
    EC_UP = EC_DOWN;
  }
  else if (EC_MEASURE > EC_MAX) {
    EC_UP = false;
    EC_DOWN = !EC_UP;
  } else {
    EC_UP = false;
    EC_DOWN = EC_UP;
  }
}

void controlModeCheck() {
  /*  MEASURES CHECK  */
  measuresCheck();
  /*  PERISTALTIC PUMP CHECK  */
  if (PH_UP || PH_DOWN || EC_UP || EC_DOWN) {
    PERISTALTIC_CHANNEL.on();
  }
  else {
    PERISTALTIC_CHANNEL.off();
  }
  /* WATER LEVEL AS FIRST PRIORITY */
  if (WATER_IN || WATER_OUT) {
    CONTROL_MODE = "LEVEL";
  }
  else { // THEN EC...
    if (EC_UP || EC_DOWN || EC_ADJUST) {
      CONTROL_MODE = "EC";
    }
    else if (PH_UP || PH_DOWN) { // FINALLY PH
      CONTROL_MODE = "PH";
    }
    else {
      GLOBAL_MODE = "MEASURE";
    }
  } 
}

/* WAIT METHOD BLOCKS THE PROGRAM FOR THE GIVEN INTERVAL */
void wait(int interval) {
  unsigned long lastWaitMillis = 0;
  unsigned long currentWaitMillis = millis();
  while(lastWaitMillis < interval) {
    lastWaitMillis = millis() - currentMillis;
  }
}

/* BT RELATED FUNCTIONS */
void msg_processing() {
  for (int k = 0; k <= 6; k = k + 2) {
    VALID_MSG = (VALID_MSG && msg[k] == '/');
  }
  if (VALID_MSG) {
    Serial.println("MENSAJE VALIDO");
    if (msg[1] == 'C') {
      //CONFIG STATE
      if (msg[3] == 'C') {
        //CALIBRATION: msg[3] : sensorType, msg[5]: thresholdType.
        CALIBRATION_ROUTINE(msg[5], msg[7]);
      } else if (msg[3] == 'P') {
        //PARAMETER SETTING
        if (msg[5] == 'S') {
          //SOLUTION PARAMETER SETTING
          Serial.println("SOLUTION PARAMETER SETTING");
          }
        else if (msg[5] == 'C') {
          //CONTROL PARAMETER SETTING
          Serial.println("CONTROL PARAMETER SETTING");
          }
      }
    } else if (msg[1] == 'O') {
      //SET OPERATION MODE ON
      Serial.println("PASANDO A MODO NORMAL DE OPERACIÓN");
      wait(5000); //Intervalo para que la aplicación abandone
      globalModeCheck();
    }
  } else {
    Serial.println("ULTIMO MENSAJE INVALIDO");
    VALID_MSG = true;
  }
}

// IMPLEMENTAR MEDICIONES DE CALIBRACION
void CALIBRATION_ROUTINE(char sensorType, char thresholdType) {
  
  if (sensorType == 'P') {
    PH.sample("CALIBRATION");
    if (thresholdType == 'M') {
      Serial.println("REFERENCIA SUPERIOR DE PH FIJADA");
      PH_MAX_REF = PH_RAW_MEASURE;
    } else if (thresholdType == 'm') {
      Serial.println("REFERENCIA INFERIOR DE PH FIJADA");
      PH_MIN_REF = PH_RAW_MEASURE;
    }
  } else if (sensorType == 'E') {
    EC.sample("CALIBRATION");
    if (thresholdType == 'M') {
      Serial.println("REFERENCIA SUPERIOR DE EC FIJADA");
      EC_MAX_REFERENCE = EC_RAW_MEASURE;
    } else if (thresholdType == 'm') {
      Serial.println("REFERENCIA INFERIOR DE EC FIJADA");
      EC_MIN_REFERENCE = EC_RAW_MEASURE;
    }
  } else if (sensorType == 'L') {
    US.sample();
    if (thresholdType == 'M') {
      Serial.println("REFERENCIA SUPERIOR DE NIVEL FIJADA");
      US_MAX_REFERENCE = US_MEASURE;
    } else if (thresholdType == 'm') {
      Serial.println("REFERENCIA INFERIOR DE PH FIJADA");
      US_MIN_REFERENCE = US_MEASURE;
    }
  }

}
