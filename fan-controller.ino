#include <Adafruit_EMC2101.h>
#include <PID_v1.h>

Adafruit_EMC2101  emc2101;

// Define Variables we'll be connecting to
double targetTemp, temp, fanDC;

// Specify the PID tuning Parameters
// P_ON_M specifies that Proportional on Measurement be used
// P_ON_E (Proportional on Error) is the default behavior
double consKp=0.3, consKi=0.1, consKd=0.08;

int intDC = 0;

// Pid Controllers
PID fan1(&temp, &fanDC, &targetTemp, consKp, consKi, consKd, P_ON_M, DIRECT);
PID fan2(&temp, &fanDC, &targetTemp, consKp, consKi, consKd, P_ON_M, DIRECT);
PID fan3(&temp, &fanDC, &targetTemp, consKp, consKi, consKd, P_ON_M, DIRECT);
PID fan4(&temp, &fanDC, &targetTemp, consKp, consKi, consKd, P_ON_M, DIRECT);

// Multiplex selection function
#define PCAADDR 0x70
void pcaselect(uint8_t i) {
  if (i > 3) return; 
  Wire.beginTransmission(PCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

// Serial Stats output
void printStats(int temp, int targetTemp, int intDC,int rpm){
  Serial.print("Temp Internal: ");
  Serial.print(temp);
  Serial.print("C Target: ");
  Serial.print(targetTemp);
  Serial.print("C Difference: ");
  Serial.print(abs(targetTemp-temp));
  Serial.println("C");
  Serial.print("DC: ");
  Serial.print(intDC);
  Serial.print("% / RPM: ");
  Serial.print(rpm);
  Serial.println(" RPM");
  Serial.println("");
  Serial.println("");
}

void setup() {

  // Init serial console
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  // PID Controller Config fans 1-4
  fan1.SetOutputLimits(1,100);
  fan1.SetMode(AUTOMATIC);
  fan1.SetControllerDirection(REVERSE);
  fan1.SetSampleTime(200);

  fan2.SetOutputLimits(1,100);
  fan2.SetMode(AUTOMATIC);
  fan2.SetControllerDirection(REVERSE);
  fan2.SetSampleTime(200);

  fan3.SetOutputLimits(1,100);
  fan3.SetMode(AUTOMATIC);
  fan3.SetControllerDirection(REVERSE);
  fan3.SetSampleTime(200);

  fan4.SetOutputLimits(1,100);
  fan4.SetMode(AUTOMATIC);
  fan4.SetControllerDirection(REVERSE);
  fan4.SetSampleTime(200);

  Wire.begin();
  delay(500);

  // Select Fan1 Multiplex port
  pcaselect(0);

  // Try to initialize EMC2101
  if (!emc2101.begin()) {
    Serial.println("Failed to find EMC2101 chip");
    while (1) { delay(10); }
  }
 
  // Set base config
  emc2101.enableTachInput(true);
  emc2101.setPWMDivisor(0);

  // Setup Targets
  temp = emc2101.getInternalTemperature();
  targetTemp = 26;

  // Select Fan2 Multiplex port
  pcaselect(1);

  // Try to initialize EMC2101
  if (!emc2101.begin()) {
    Serial.println("Failed to find EMC2101 chip");
    while (1) { delay(10); }
  }
 
  // Set base config
  emc2101.enableTachInput(true);
  emc2101.setPWMDivisor(0);

  // Setup Targets
  temp = emc2101.getInternalTemperature();
  targetTemp = 26;

}

void loop() {
  // -------------------------------
  // Fan1
  // -------------------------------
  pcaselect(0);

  // Get current Temp for PID
  temp = emc2101.getInternalTemperature();

  // Calculate PID
  fan1.Compute();
  
  // Set EMC 2101 DC
  emc2101.setDutyCycle(fanDC);

  // Debug
  Serial.println("Fan 1");
  printStats(temp,targetTemp,fanDC,emc2101.getFanRPM());

  // -------------------------------
  // Fan2
  // -------------------------------
  pcaselect(1);

  // Get current Temp for PID
  temp = emc2101.getInternalTemperature();

  // Calculate PID
  fan2.Compute();
  
  // Set EMC 2101 DC
  emc2101.setDutyCycle(fanDC);

  // Debug
  Serial.println("Fan 2");
  printStats(temp,targetTemp,fanDC,emc2101.getFanRPM());

  delay(200);
}