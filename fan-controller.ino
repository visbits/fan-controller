#include <Adafruit_EMC2101.h>
#include <PID_v1.h>

Adafruit_EMC2101  emc2101;

// Define Variables we'll be connecting to
double temp, fanDC;

// Specify the PID tuning Parameters
// P_ON_M specifies that Proportional on Measurement be used
// P_ON_E (Proportional on Error) is the default behavior
double consKp=0.3, consKi=0.1, consKd=0.08;

// DC
int intDC = 0;

// Setup Targets
double targetTemp = 26;

// Pid Controllers
PID fan1(&temp, &fanDC, &targetTemp, consKp, consKi, consKd, P_ON_M, DIRECT);
PID fan2(&temp, &fanDC, &targetTemp, consKp, consKi, consKd, P_ON_M, DIRECT);
PID fan3(&temp, &fanDC, &targetTemp, consKp, consKi, consKd, P_ON_M, DIRECT);
PID fan4(&temp, &fanDC, &targetTemp, consKp, consKi, consKd, P_ON_M, DIRECT);

// Multiplex selection function
// Writes integer to 0x70 prior to sending following commands.
#define PCAADDR 0x70
void fanSelect(uint8_t i) {
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
}

void fanSetup(PID& fan, int fanId){
  // Configure PID controller
  fan.SetOutputLimits(1,100);
  fan.SetMode(AUTOMATIC);
  fan.SetControllerDirection(REVERSE);
  fan.SetSampleTime(200);
  
  // Select Multiplex Fan ID
  fanSelect(fanId);

  // Try to initialize EMC2101
  if (!emc2101.begin()) {
    Serial.println("Failed to find EMC2101 chip");
    while (1) { delay(10); }
  } else {
    // Set base config
    emc2101.enableTachInput(true);
    emc2101.setPWMDivisor(0);
  }
}

void setup() {

  // Init serial console
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  // Init serial and delay for init
  Wire.begin();
  delay(500);

  // PID Controller Config fans 1-4
  fanSetup(fan1,0);
  fanSetup(fan2,1);
  //fanSetup(fan3,2);
  //fanSetup(fan4,3);
}

void loop() {
  // -------------------------------
  // Fan1
  // -------------------------------
  fanSelect(0);

  // Get current Temp for PID
  temp = emc2101.getInternalTemperature();

  // Calculate PID
  fan1.Compute();
  
  // Set EMC 2101 DC
  emc2101.setDutyCycle(fanDC);

  // Debug
  Serial.println("Fan 1");
  printStats(temp,targetTemp,emc2101.getDutyCycle(), emc2101.getFanRPM());

  // -------------------------------
  // Fan2
  // -------------------------------
  fanSelect(1);

  // Get current Temp for PID
  temp = emc2101.getInternalTemperature();

  // Calculate PID
  fan2.Compute();
  
  // Set EMC 2101 DC
  emc2101.setDutyCycle(fanDC);

  // Debug
  Serial.println("Fan 2");
  printStats(temp, targetTemp, emc2101.getDutyCycle(), emc2101.getFanRPM());

  delay(200);
}