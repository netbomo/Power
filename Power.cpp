/**
 *******************************************************************************
 *******************************************************************************
 *
 *	License :
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * 
 *******************************************************************************
 *******************************************************************************
 *
 *
 *    @file   Anemometer.cpp
 *    @author gilou
 *    @date   24 mai 2017
 *    @brief  The Power class permit to measure AC or DC power with 1 voltage and 1 current

 */

#include "Arduino.h"

#include "Power.h"

/******************************************************************************
 * Class constructor and destructor
 */
Power::Power(unsigned char id) {
	m_id = id;
	m_eeprom_addr = 500 + m_id*50;		/**<  */

	load_param();

	set_v_pin();
	set_i_pin(id);

	s_average = 0;	/**< there is the result from the apparent power average */
	pf_average = 0;	/**< there is the result from the power factor average */

	v_offset = 512;		// preset voltage offset
	i_offset=512;		// preset current offset


	for(int i=0;i<50;++i){
		Vmeas=analogRead(m_v_pin); // channel ?? for the voltage compensated with the offset
		Imeas=analogRead(m_i_pin); // channel ?? for the current compensated with the offset

		// display measures
		//FSM::uart0.print(dtostrf(Vmeas,0,3,temp_char));FSM::uart0.print("	");FSM::uart0.print(dtostrf(Imeas,0,3,temp_char));FSM::uart0.print("\r\n");


		//-----------------------------------------------------------------------------
		// B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
		//     then subtract this - signal is now centred on 0 counts.
		//-----------------------------------------------------------------------------
		v_offset = v_offset + ((Vmeas-v_offset)/1024);
		i_offset = i_offset + ((Imeas-i_offset)/1024);
	}
}

Power::~Power() {
}


/******************************************************************************
 * Sensor's methods
 */
//This method clear data array.
void Power::clear(unsigned char measure_max){
	for(unsigned char i = 0; i < measure_max; ++i){
		p_data[i] = 0;
		s_data[i] = 0;
		pf_data[i] = 0;
		v_data[i] = 0;
		i_data[i] = 0;
	}
}



// This method read the sensor's value
void Power::read_value(unsigned char measure_number, unsigned char crossings, unsigned int timeout){
	if(is_enable()){
#ifdef DEBUG_POWER
		char tempString[10];
		Serial.println(print(tempString));		// useful to debug
#endif
		unsigned char crossCount = 0;                             //Used to measure number of times threshold is crossed.
		unsigned int numberOfSamples = 0;                        //This is now incremented

		unsigned int saved_num_samples[4]={0};				// used for display debug value
		long int saved_timeout = 0;				// used for display debug value
		char temp_char[12];				// used for display debug value


		sumV = 0;
		sumI = 0;
		sumP = 0;

		/// @todo increase offset calibration with an external 2.5V reference
		double SupplyVoltage = 5000; // VCC/2 is present on ADC5, not yet
		//long SupplyVoltage = readVcc();
#ifdef DEBUG_POWER
		Serial.print("Vcc = "); Serial.println(SupplyVoltage);  // for debug
#endif

		//-------------------------------------------------------------------------------------------------------------------------
		// 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
		//-------------------------------------------------------------------------------------------------------------------------
		bool st=false;                                  //an indicator to exit the while loop

		unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the	 loop if there is an error.

		while(st==false)                                   //the while loop...
		{
			//Serial.print('k');
			startV = analogRead(m_v_pin);                    //using the voltage waveform
			if ((startV < (ADC_COUNTS*0.55)) && (startV > (ADC_COUNTS*0.45))) {
				st=true;  //check its within range
#ifdef DEBUG_POWER
				Serial.print("exit while st startV\r\n");
#endif
			}
			if ((TCNT3-start)>timeout)  {
				st = true;
#ifdef DEBUG_POWER
				Serial.print("exit while st timeout\r\n");
#endif
			}
		}

		//-------------------------------------------------------------------------------------------------------------------------
		// 2) Main measurement loop
		//-------------------------------------------------------------------------------------------------------------------------

		start = millis();			// read millis value (started for anemometer measure)

		bool exit_by_crosscount = true;

		bool exit_by_timeout = true;

		while ((exit_by_crosscount) && (exit_by_timeout))
		{

			numberOfSamples++; //Count number of times looped.
			referenceV = v_filtered; //Used for delay/phase compensation

			Vmeas=analogRead(m_v_pin); // channel ?? for the voltage compensated with the offset
			Imeas=analogRead(m_i_pin); // channel ?? for the current compensated with the offset

			// display measures
#ifdef DEBUG_POWER
			Serial.print(dtostrf(Vmeas,0,3,temp_char));Serial.print("	");Serial.print(dtostrf(Imeas,0,3,temp_char));Serial.print("\r\n");
#endif

			//-----------------------------------------------------------------------------
			// B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
			//     then subtract this - signal is now centred on 0 counts.
			//-----------------------------------------------------------------------------
			//if(isAC){
				v_offset = v_offset + ((Vmeas-v_offset)/1024);

			//}
			v_filtered = Vmeas - v_offset;
			//if(isAC){
			i_offset = i_offset + ((Imeas-i_offset)/1024);
			//}
			i_filtered = Imeas - i_offset;

			sumV += v_filtered*v_filtered;			//voltage squared
			sumI += i_filtered*i_filtered;			//current squared

			Vshifted = referenceV + v_phase * (v_filtered - referenceV);  //calculates the voltage shift after the measurement

			//sumP += Vshifted*i_filtered; 	// stores the power data
			sumP += v_filtered*i_filtered; 	// debug

			//-----------------------------------------------------------------------------
			// G) Find the number of times the voltage has crossed the initial voltage
			//    - every 2 crosses we will have sampled 1 wavelength
			//    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
			//-----------------------------------------------------------------------------
			lastVCross = checkVCross;
			if (Vmeas > startV) checkVCross = true;
			else checkVCross = false;
			if (numberOfSamples==1) lastVCross = checkVCross;

			if (lastVCross != checkVCross) {
				saved_num_samples[crossCount] = numberOfSamples;
				crossCount++;
			}

			exit_by_crosscount = crossCount < crossings;

			saved_timeout = (millis()-start);

			exit_by_timeout = saved_timeout<timeout;

		}

		// check exit
#ifdef DEBUG_POWER
		   if(!exit_by_crosscount)	Serial.print("exit_by_crosscount\r\n");
		   if(!exit_by_timeout)		Serial.print("exit_by_timeout\r\n");
#endif
		//-------------------------------------------------------------------------------------------------------------------------
		// 3) Post loop calculations
		//-------------------------------------------------------------------------------------------------------------------------
		//Calculation of the root of the mean of the voltage and current squared (rms)
		//Calibration coefficients applied.

		double V_RATIO = v_factor *((SupplyVoltage/1000.0) / (ADC_COUNTS));

		double I_RATIO = i_factor *((SupplyVoltage/1000.0) / (ADC_COUNTS));

		v_data[measure_number]= sqrt(sumV/numberOfSamples) *V_RATIO; 			// stores the rms voltage data
		i_data[measure_number]= sqrt(sumI/numberOfSamples) * I_RATIO; 			// stores the rms current data
		p_data[measure_number]= (sumP/numberOfSamples)*V_RATIO* I_RATIO; 	// stores the power data
		s_data[measure_number]= v_data[measure_number]*i_data[measure_number];
		pf_data[measure_number] = p_data[measure_number]/s_data[measure_number];

	}

}

// This method calculate the average from the data array.
void Power::calc_average(unsigned char measure_max){
	if(is_enable()){
		//initialize variables
		v_average=0;
		i_average=0;
		p_average=0;
		s_average=0;
		pf_average=0;

		//char temp_char[10];
		//FSM::uart0.print("Detail average"); FSM::uart0.print("\r\n");

		//sums the data
		for(int counter =0; counter< measure_max;++counter)
		{
			//FSM::uart0.print(dtostrf(p_data[counter],0,3,temp_char)); FSM::uart0.print(" ");

			v_average+=v_data[counter]/measure_max;
			i_average+=i_data[counter]/measure_max;
			p_average+=p_data[counter]/measure_max;
			s_average+=s_data[counter]/measure_max;
			pf_average+=pf_data[counter]/measure_max;
		}
	}
}



char * Power::get_average(char *string){
	if(isAC){
		char temp_conv[12];
		dtostrf(v_average,0,m_prec,temp_conv);
		strcpy(string,temp_conv);
		strcat(string,"	");
		dtostrf(i_average,0,m_prec,temp_conv);
		strcat(string,temp_conv);
		strcat(string,"	");
		dtostrf(p_average,0,m_prec,temp_conv);
		strcat(string,temp_conv);
		strcat(string,"	");
		dtostrf(s_average,0,m_prec,temp_conv);
		strcat(string,temp_conv);
		strcat(string,"	");
		dtostrf(pf_average,0,m_prec,temp_conv);
		strcat(string,temp_conv);
		strcat(string,"	");
		return string;
	}
	else{
		char temp_conv[12];
		dtostrf(v_average,0,m_prec,temp_conv);
		strcpy(string,temp_conv);
		strcat(string,"	");
		dtostrf(i_average,0,m_prec,temp_conv);
		strcat(string,temp_conv);
		strcat(string,"	");
		return string;
	}
}


// The print method print in string "Anemo id"
char* Power::print(char *string){
	if(m_enable){
		char temp_char[6];
		 strcpy(string,"Power ");
		 strcat(string,itoa(m_id,temp_char,10));
	}

	return string;	// return
}

// This method print the sensor configuration for the sensor. It's a good idea to overload this function to do it more explicit for each sensor.
void Power::print_config(){
	Serial.println("Anemometer :");
	Serial.print("	*");Serial.print(m_id+5); Serial.print("1 enable :	");Serial.print(m_enable);Serial.println("			enable : 1, disable : 0");
	Serial.print("	*");Serial.print(m_id+5); Serial.print("2 AC power :	");Serial.print(isAC);Serial.println("			AC power : 1, DC power : 0");
	Serial.print("	*");Serial.print(m_id+5); Serial.print("3 voltage factor:	");Serial.print(v_factor);Serial.println("		can be a float value. ex: 42.42");
	Serial.print("	*");Serial.print(m_id+5); Serial.print("4 voltage offset:	");Serial.print(v_offset);Serial.println("		can be a float value. ex: 42.42, autoset for AC.");
	Serial.print("	*");Serial.print(m_id+5); Serial.print("5 voltage phase:	");Serial.print(v_phase);Serial.println("		can be a float value. ex: 1.7, only need for AC.");
	Serial.print("	*");Serial.print(m_id+5); Serial.print("6 current factor:	");Serial.print(i_factor);Serial.println("		can be a float value. ex: 42.42");
	Serial.print("	*");Serial.print(m_id+5); Serial.print("7 current offset:	");Serial.print(i_offset);Serial.println("		can be a float value. ex: 42.42, autoset for AC.");
	Serial.print("	*");Serial.print(m_id+5); Serial.print("9 Display accuracy:	");Serial.print(m_prec);Serial.println("		0,1,2 or 3 digit after the comma");
	//Serial.print("eeprom : "); Serial.println(m_eeprom_addr);

}

// The config method permit to update parameters
void Power::config(char *stringConfig){
	uint8_t item = stringConfig[2]-'0';	// convert item in char

	double arg_f = atof(stringConfig + 4);	// convert the second part, the value in double to cover all possibilities.
	unsigned char arg_uc = (unsigned char)arg_f;
	switch (item) {
		case 1:	// enable or disable anemo1
			if(arg_uc==0)m_enable = false;	// disable
			else m_enable = true;				// enable
			update_param();
		break;
		case 2:	// enable or disable anemo1
			if(arg_uc==0)isAC = false;	// disable
			else isAC = true;				// enable
			if(isAC==true)	eeprom_update_byte((unsigned char*)m_eeprom_addr+28,1);
			else eeprom_update_byte((unsigned char*)m_eeprom_addr+28,0);
			break;
		case 3:	// Set factor value
			v_factor = arg_f;
			update_param();
			break;
		case 4:	// Set offset value
			v_offset = arg_f;
			update_param();
			break;
		case 5:	// Set offset value
			v_phase = arg_f;
			update_param();
			break;
		case 6:	// Set factor value
			i_factor = arg_f;
			update_param();
			break;
		case 7:	// Set offset value
			i_offset = arg_f;
			update_param();
			break;
		case 9:	// Set display decimal accuracy
			m_prec = arg_uc;
			update_param();
			break;
		default:
			Serial.print("Bad request : ");Serial.println(item);
	}
}

// Use this method for debugging or calibration accuracy
void Power::print_data_array()const{
	char temp[20];
	Serial.print("\r\n");
	for(int i=0;i<10;i++)
	{
		Serial.print(i);Serial.print('	');Serial.print(dtostrf(v_data[i],0,3,temp));Serial.print("	");Serial.print(dtostrf(i_data[i],0,3,temp));Serial.print("\r\n");
	}
}

/******************************************************************************
 * sens_param management
 */
// Load saved parameters for sensors from the eeprom
void Power::load_param(){
	v_factor = eeprom_read_float((float*)m_eeprom_addr);
	v_offset = eeprom_read_float((float*)m_eeprom_addr+5);
	v_phase = eeprom_read_float((float*)m_eeprom_addr+10);
	i_factor = eeprom_read_float((float*)m_eeprom_addr+15);
	i_offset = eeprom_read_float((float*)m_eeprom_addr+20);

	if(eeprom_read_byte((unsigned char*)m_eeprom_addr+25)==0) m_enable = false;
	else m_enable = true;

	if(eeprom_read_byte((unsigned char*)m_eeprom_addr+28)==0) isAC = false;
	else isAC = true;
	m_prec = eeprom_read_byte((byte*)m_eeprom_addr+30);
}

// Update saved parameters for sensors in the eeprom
void Power::update_param(){
	eeprom_update_float((float*)m_eeprom_addr,v_factor);
	eeprom_update_float((float*)m_eeprom_addr+5,v_offset);
	eeprom_update_float((float*)m_eeprom_addr+10,v_phase);
	eeprom_update_float((float*)m_eeprom_addr+15,i_factor);
	eeprom_update_float((float*)m_eeprom_addr+20,i_offset);
	if(m_enable==true)	eeprom_update_byte((unsigned char*)m_eeprom_addr+25,1);
	else eeprom_update_byte((unsigned char*)m_eeprom_addr+25,0);
//	if(isAC==true)	eeprom_update_byte((unsigned char*)m_eeprom_addr+28,1);
//	else eeprom_update_byte((unsigned char*)m_eeprom_addr+28,0);
	eeprom_update_byte((unsigned char*)m_eeprom_addr+30,m_prec);
}

// Initialize the eeprom memory and the sens_param array
void Power::initialize_param(){
	eeprom_update_float((float*)m_eeprom_addr,1.0);
	eeprom_update_float((float*)m_eeprom_addr+5,0.0);
	eeprom_update_float((float*)m_eeprom_addr+10,1.0);
	eeprom_update_float((float*)m_eeprom_addr+15,1.0);
	eeprom_update_float((float*)m_eeprom_addr+20,0.0);
	eeprom_update_byte((unsigned char*)m_eeprom_addr+25,1);
	eeprom_update_byte((unsigned char*)m_eeprom_addr+28,1);
	eeprom_update_byte((unsigned char*)m_eeprom_addr+30,1);
	load_param();
}


