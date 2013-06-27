/* Weather Station - Sensor Transmitter
// This is the third version of the weather station, this version will consist of a 
// sensor transmitter and a base receiver.
// 
// Transmitter must gather sensor data, calculate any data necessary and transmit.
// Receiver must gather data and post to the internet and webserver, etc.
*/
/*
	The circuit: ATMega328
	LDR - A0 to Gnd
	DHT22 - D6
*/

#define DEBUG 1

//includes
#include <JeeLib.h>
#include <avr/sleep.h>
#include <math.h> 


//Constants
#define SEND_PERIOD 60000			// Minimum transmission period of sensor values
#define LDR_PORT 0   				// Defined if LDR is connected to a port's AIO pin
#define SMOOTH 2   					// LDR smoothing factor used for running averages
#define DHT22_PORT 6 				// DHT22 Port
#define FREQ RF12_868MHZ        	// Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
#define NODEID 16          		
#define GROUP 212  


//Humidity and LDR Sensor Port
DHTxx dht (DHT22_PORT);

//General Variables
struct {
	byte light;     // light sensor: 0..255
	int humidity;
	int temperature;
	int dewpoint;
	int cloudbase;
	byte vcc;		// 0 - 250,  1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250, etc. 20mV Steps
} payload;

volatile bool adcDone;
ISR(WDT_vect) { Sleepy::watchdogEvent(); }
ISR(ADC_vect) { adcDone = true; }
// End of config

static byte readVcc (byte count =4) 
{
    set_sleep_mode(SLEEP_MODE_ADC);
    ADMUX = bit(REFS0) | 14; // use VCC and internal bandgap
    bitSet(ADCSRA, ADIE);
    while (count-- > 0) {
      adcDone = false;
      while (!adcDone)
        sleep_mode();
    }
    bitClear(ADCSRA, ADIE);  
    // convert ADC readings to fit in one byte, i.e. 20 mV steps:
    //  1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250
	return (55U * 1023U) / (ADC + 1) - 50;
}

// Setup Routine
void setup()
{   
#if DEBUG
	Serial.begin(9600);
	Serial.println("\n[WXTX]");
#endif
	
	rf12_initialize(NODEID, FREQ, GROUP);                    
	rf12_sleep(RF12_SLEEP);
	
	pinMode(14+LDR_PORT, INPUT);
	digitalWrite(14+LDR_PORT, 1); // pull-up
}


// Main Loop
void loop()
{
		//Get Sensor Readings
		//Ambient Light Readings
		doLDRMeasure();
		
		//Humdity Sensor Readings
		int h, t;
		!dht.reading(t,h) ? t = h = 0 : t = t;

		//Vcc
		payload.vcc = readVcc();
		
		if (!(h == 0 && t == 0))
		{
			payload.humidity = payload.humidity + (h - payload.humidity)*0.50;
			payload.temperature = payload.temperature + (t - payload.temperature)*0.50;
		
			payload.dewpoint = calculateDewpoint (payload.humidity, payload.temperature);
			payload.cloudbase = calculateCloudbase (payload.temperature, payload.dewpoint);
				
			//send sensor readings		
			send_rf_data();  
		}
#if DEBUG
		printData();
#endif
	Sleepy::loseSomeTime(SEND_PERIOD);
}

//transmit payload
void send_rf_data()
{
  rf12_sleep(RF12_WAKEUP);
  // if ready to send + exit loop if it gets stuck as it seems too
  int i = 0; while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}
  rf12_sendStart(0, &payload, sizeof payload);//RF12_HDR_ACK
  // set the sync mode to 2 if the fuses are still the Arduino default
  // mode 3 (full powerdown) can only be used with 258 CK startup fuses
  rf12_sendWait(2);

  rf12_sleep(RF12_SLEEP);  
}

//debug print - set debug to 1
void printData()
{
#if DEBUG
	Serial.print(" l ");
	Serial.println(payload.light, DEC);
	Serial.print(" h ");
	Serial.println(payload.humidity, DEC);
	Serial.print(" t ");
	Serial.println(payload.temperature, DEC);
	Serial.print(" d ");
	Serial.println(payload.dewpoint, DEC);
	Serial.print(" c ");
	Serial.println(payload.cloudbase, DEC);
	Serial.print(" v ");
	Serial.println(payload.vcc, DEC);
#endif
}

//Light Reading
static void doLDRMeasure() {
	byte light = 255 - analogRead(LDR_PORT) / 4;
	
	payload.light = constrain(light, 0, 100);
}

//cloudBase calculation
float calculateCloudbase (float t, float d)
{ 
	t *= 0.1;
  return ( (t - d) * 400 );
}

//dewpoint Calculation
float calculateDewpoint(float h, float t)
{
	h *= 0.1;
	t *= 0.1;
	float k = (log10(h)-2)/0.4343 + (17.62*t)/(243.12+t); 
	
	return 243.12*k/(17.62-k); 
}