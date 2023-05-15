/*
  SmeLib.cpp - Library for Ahmed Reghini
  Created by Ahmed REGHINI, 2022
  
*/

//#include "WProgram.h" un-comment for use on older versions of Arduino IDE
#include "SmeLib.h"
#include <Arduino.h>
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* ******************************* Noise reduction Filter********************* */
#include "driver/adc.h"
#include "esp_adc_cal.h"

unsigned int AN_Pot1; 
unsigned int AN_Pot2;    
#define FILTER_LEN  10
 
uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0};
int AN_Pot1_i = 0;
int AN_Pot1_Raw = 0;
int AN_Pot1_Filtered = 0;

uint32_t AN_Pot2_Buffer[FILTER_LEN] = {0};
int AN_Pot2_i = 0;
int AN_Pot2_Raw = 0;
int AN_Pot2_Filtered = 0;

uint32_t readADC_Avg(int ADC_Raw)
{
  int i = 0;
  uint32_t Sum = 0;
  
  AN_Pot1_Buffer[AN_Pot1_i++] = ADC_Raw;
  if(AN_Pot1_i == FILTER_LEN)
  {
    AN_Pot1_i = 0;
  }
  for(i=0; i<FILTER_LEN; i++)
  {
    Sum += AN_Pot1_Buffer[i];
  }
  return (Sum/FILTER_LEN);
}

uint32_t readADC_Avg2(int ADC_Raw)
{
  int i = 0;
  uint32_t Sum = 0;
  
  AN_Pot2_Buffer[AN_Pot2_i++] = ADC_Raw;
  if(AN_Pot2_i == FILTER_LEN)
  {
    AN_Pot2_i = 0;
  }
  for(i=0; i<FILTER_LEN; i++)
  {
    Sum += AN_Pot2_Buffer[i];
  }
  return (Sum/FILTER_LEN);
}

/* **********************************get Stable values******************************** */
// costomer filter by ahmed reghini https://github.com/ahmedreghini/getStableValue.git
//#include <getStableValue.h>
// get stable value instance
//getStableValue emongsv;
// for test ADC_COUNTS = ADC_COUNTS - 1096;
//ADC_COUNTS = 3000;
//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors
//--------------------------------------------------------------------------------------
void EnergyMonitor::voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL)
{
  inPinV = _inPinV;
  VCAL = _VCAL;
  PHASECAL = _PHASECAL;
  offsetV = ADC_COUNTS>>1;
}

void EnergyMonitor::current(unsigned int _inPinI, double _ICAL)
{
  inPinI = _inPinI;
  ICAL = _ICAL;
  offsetI = ADC_COUNTS>>1;
}

//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors based on emontx pin map
//--------------------------------------------------------------------------------------
void EnergyMonitor::voltageTX(double _VCAL, double _PHASECAL)
{
  inPinV = 2;
  VCAL = _VCAL;
  PHASECAL = _PHASECAL;
  offsetV = ADC_COUNTS>>1;
}

void EnergyMonitor::currentTX(unsigned int _channel, double _ICAL)
{
  if (_channel == 1) inPinI = 3;
  if (_channel == 2) inPinI = 0;
  if (_channel == 3) inPinI = 1;
  ICAL = _ICAL;
  offsetI = ADC_COUNTS>>1;
}

//--------------------------------------------------------------------------------------
// emon_calc procedure
// Calculates realPower,apparentPower,powerFactor,Vrms,Irms,kWh increment
// From a sample window of the mains AC voltage and current.
// The Sample window length is defined by the number of half wavelengths or crossings we choose to measure.
//--------------------------------------------------------------------------------------
void EnergyMonitor::calcVI(unsigned int crossings, unsigned int timeout)
{
  //#if defined emonTxV3
  int SupplyVoltage = 3300;
  /* #else
  int SupplyVoltage = readVcc();
  #endif */
  AN_Pot1 = inPinV; //the pin that we will work with
  AN_Pot2 = inPinI;
  unsigned int crossCount = 0;                             //Used to measure number of times threshold is crossed.
  unsigned int numberOfSamples = 0;                        //This is now incremented

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

  while(1)                                   //the while loop...
  {
    startV = analogRead(inPinV); //gsv.GSV(inPinV, 100,10,1);    //using the voltage waveform
    if ((startV < (ADC_COUNTS*0.55)) && (startV > (ADC_COUNTS*0.45))) break;  //check its within range
    if ((millis()-start)>timeout) break;
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //-------------------------------------------------------------------------------------------------------------------------
  start = millis();

  while ((crossCount < crossings) && ((millis()-start)<timeout))
  {
    numberOfSamples++;                       //Count number of times looped.
    lastFilteredV = filteredV;               //Used for delay/phase compensation

    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
    AN_Pot1_Raw = analogRead(AN_Pot1);
    sampleV = readADC_Avg(AN_Pot1_Raw);
    //sampleV = analogRead(inPinV);  //emongsv.GSV(inPinV, 5,5,1,true);            //Read in raw voltage signal
    AN_Pot2_Raw = analogRead(AN_Pot2);
    sampleI = readADC_Avg2(AN_Pot2_Raw);
    //sampleI = analogRead(inPinI);//gsv.GSV(inPinI, 10,10,1);   analogRead(inPinI);            //Read in raw current signal

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    //int offsetMm = emongsv.GSV(inPinV,5,10,1,false);
    //int offsetMx = emongsv.GSV(inPinV, 5,10,1,true);
    //offsetV = 1940;//offsetMx - (offsetMm/2);
    offsetV = offsetV + ((sampleV-offsetV)/4096); //- offsetMm + (offsetMm / 2);
    filteredV = sampleV - offsetV;
    //Serial.println(String("filtred v :") + filteredV);
    offsetI = offsetI + ((sampleI-offsetI)/4096);
    filteredI = sampleI - offsetI;

    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------
    sqV= filteredV * filteredV;                 //1) square voltage values
    sumV += sqV;                                //2) sum

    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------
    sqI = filteredI * filteredI;                //1) square current values
    sumI += sqI;                                //2) sum

    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);

    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------
    instP = phaseShiftedV * filteredI;          //Instantaneous Power
    sumP +=instP;                               //Sum

    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------
    lastVCross = checkVCross;
    if (sampleV > startV) checkVCross = true;
                     else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;

    if (lastVCross != checkVCross) crossCount++;
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //-------------------------------------------------------------------------------------------------------------------------
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied.

  double V_RATIO = VCAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Vrms = sqrt(sumV / numberOfSamples);
  //Serial.println(String("Vrms sqrd sans ratio : ") + Vrms) ;
  //(Vrms <= 5)? Vrms = 0: Vrms=Vrms;
  Serial.println(String("Vrms sqrd sans ratio : ") + Vrms) ;
  //Vrms = V_RATIO * sqrt(sumV / numberOfSamples);

  
  double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Irms = sqrt(sumI / numberOfSamples);
  Serial.println(String("Irms sqrd sans ratio : ") + Irms) ;
  //Irms = I_RATIO * sqrt(sumI / numberOfSamples);

  //Calculation power values
  realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
  apparentPower = Vrms * Irms;
  powerFactor=realPower / apparentPower;

  //Reset accumulators
  sumV = 0;
  sumI = 0;
  sumP = 0;
//--------------------------------------------------------------------------------------
}

//--------------------------------------------------------------------------------------
double EnergyMonitor::calcIrms(unsigned int Number_of_Samples)
{

  //#if defined esp32V33
    int SupplyVoltage=3300;
  /* #else
    int SupplyVoltage = readVcc();
  #endif */


  for (unsigned int n = 0; n < Number_of_Samples; n++)
  {
    sampleI = analogRead(inPinI);//gsv.GSV(inPinI, 10,10,1);

    // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
    //  then subtract this - signal is now centered on 0 counts.
    offsetI = (offsetI + (sampleI-offsetI)/4096);
    filteredI = sampleI - offsetI;

    // Root-mean-square method current
    // 1) square current values
    sqI = filteredI * filteredI;
    // 2) sum
    sumI += sqI;
  }

  double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Irms = I_RATIO * sqrt(sumI / Number_of_Samples);

  //Reset accumulators
  sumI = 0;
  //--------------------------------------------------------------------------------------

  return Irms;
}

double EnergyMonitor::calcVrms(unsigned int crossings, unsigned int timeout)
{

  //#if defined esp32V33
    int SupplyVoltage=3300;
  /* #else
    int SupplyVoltage = readVcc();
  #endif */
  AN_Pot1 = inPinV;
  unsigned int crossCount = 0;                             //Used to measure number of times threshold is crossed.
  unsigned int numberOfSamples = 0;
  int start = millis();
  while(1)                                   //the while loop...
  {
    startV = analogRead(inPinV); //gsv.GSV(inPinV, 100,10,1);    //using the voltage waveform
    if ((startV < (ADC_COUNTS*0.55)) && (startV > (ADC_COUNTS*0.45))) break;  //check its within range
    if ((millis()-start)>timeout) break;
  }

  start = millis();
  while ((crossCount < crossings) && ((millis()-start)<timeout))
  {
    numberOfSamples++;                       //Count number of times looped.
    lastFilteredV = filteredV;               //Used for delay/phase compensation
    //sampleV = analogRead(inPinV);//gsv.GSV(inPinI, 10,10,1);
    
    AN_Pot1_Raw = analogRead(AN_Pot1);
    sampleV = readADC_Avg(AN_Pot1_Raw);
    // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
    //  then subtract this - signal is now centered on 0 counts.
    offsetI = (offsetV + (sampleV-offsetV)/4096);
    filteredV = sampleV - offsetV;

    // Root-mean-square method current
    // 1) square current values
    sqV = filteredV * filteredV;
    // 2) sum
    sumV += sqV;

    lastVCross = checkVCross;
    if (sampleV > startV) checkVCross = true;
                     else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;

    if (lastVCross != checkVCross) crossCount++;
  }

  double V_RATIO = VCAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Vrms = sqrt(sumV / numberOfSamples);
  //Vrms = V_RATIO * sqrt(sumV / numberOfSamples);

  //Reset accumulators
  sumV = 0;
  //--------------------------------------------------------------------------------------
  

  return Vrms;
}

double EnergyMonitor::medianFilterV(unsigned int crossings, unsigned int timeout){
  int BUFFER_SIZE = 5;
  double buffer[BUFFER_SIZE];
  double emonVrms;
  for(int i = BUFFER_SIZE; i > 0; i--){
  buffer[i] = 0;
  }
  for(int i = BUFFER_SIZE; i > 0; i--){
  emonVrms = calcVrms(crossings, timeout);
  buffer[BUFFER_SIZE-1] = emonVrms;
  }
  for(int j= BUFFER_SIZE-1;j>0;j--){
    if(buffer[j] < buffer[j-1]){
      int temp = buffer[j];
      buffer[j] = buffer[j-1];
      buffer[j-1]= temp;
    } else {break;}
  }
  //calculate the meian value of the samples in the buffer
  Vrms = buffer[BUFFER_SIZE / 2];
  if(BUFFER_SIZE % 2 == 0){
    Vrms = (Vrms + buffer[BUFFER_SIZE/2 - 1])/2;
  }
  return Vrms;
}

double EnergyMonitor::minV(unsigned int crossings, unsigned int timeout, double mult_cal, double mines_cal, unsigned int samplesmin, unsigned int readAVG ){
  #define FILTER_LEN = readAVG;
  int ff = samplesmin;//20;
  
	double arr [ff];
  double min = 0;
  

	for (int i =0; i < ff;i++) {
		arr[i] = calcVrms(crossings, timeout) ;
    Serial.println(String("calcVrms :") + arr[i]);
	}
  min = arr[0];
  for (int j =1; j < ff;j++) {
		if (min > arr[j]) {
            min = arr[j];
        } 

	}
  min = (min -mines_cal)* mult_cal;//2.22;
  (min < 0)?min =0:min=min;
	return min;

}

void EnergyMonitor::minVI(unsigned int crossings, unsigned int timeout, double mult_cal, double mines_cal, unsigned int samplesmin, unsigned int readAVG ){
  #define FILTER_LEN = readAVG;
  int ff = samplesmin;//20;
  
	double arrV [ff];
  double arrI [ff];
  double arrRP [ff];
  double arrAP [ff];
  double arrPF [ff];
  double minV = 0;
  double minI = 0;
  double minRP = 0;
  double minAP = 0;
  double minPF = 0;
  

	for (int i =0; i < ff;i++) {
    calcVI(crossings, timeout) ;
		arrV[i] = Vrms; 
    arrI[i] = Irms;
    arrRP[i] = realPower;
    arrAP[i] = apparentPower;
    arrPF[i] = powerFactor;
    //Serial.println(String("calcVI :") + serialprint());
	}
  minV = arrV[0];
  minI = arrI[0];
  minRP = arrRP[0];
  minAP = arrAP[0];
  minPF = arrPF[0];
  for (int j =1; j < ff;j++) {
		if (minV > arrV[j]) {minV = arrV[j]; Vrms = minV;} 
    if (minI > arrI[j]) {minI = arrI[j]; Irms = minI;}
    if (minRP > arrRP[j]) {minRP = arrRP[j]; realPower = minRP;}
    if (minAP > arrAP[j]) {minAP = arrAP[j]; apparentPower = minAP;}
    if (minPF > arrPF[j]) {minPF = arrPF[j]; powerFactor = minPF;}
	}
  /* min = (min -mines_cal)* mult_cal;//2.22;
  (min < 0)?min =0:min=min;
	return min; */

}

void EnergyMonitor::serialprint()
{
  Serial.print("realPower");
  Serial.print(' ');
  Serial.println(realPower);
  Serial.print(' ');
  Serial.print("apparentPower");
  Serial.print(' ');
  Serial.println(apparentPower);
  Serial.print(' ');
  Serial.print("Vrms");
  Serial.print(' ');
  Serial.println(Vrms);
  Serial.print(' ');
  Serial.print("Irms");
  Serial.print(' ');
  Serial.println(Irms);
  Serial.print(' ');
  Serial.print("powerFactor");
  Serial.print(' ');
  Serial.print(powerFactor);
  Serial.println(' ');
  delay(100);
}


long EnergyMonitor::readVcc() {
  long result;

  //not used on emonTx V3 - as Vcc is always 3.3V - eliminates bandgap error and need for calibration http://harizanov.com/2013/09/thoughts-on-avr-adc-accuracy/

  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90USB1286__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRB &= ~_BV(MUX5);   // Without this the function always returns -1 on the ATmega2560 http://openenergymonitor.org/emon/node/2253#comment-11432
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);

  #endif


  #if defined(__AVR__)
  delay(2);                                        // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                             // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = READVCC_CALIBRATION_CONST / result;  //1100mV*1024 ADC 
  return result;
  #elif defined(__arm__)
  return (3300);                                  //Arduino Due
  #else
  return (3300);                                  //Guess that other un-supported architectures will be running a 3.3V!
  #endif
}

