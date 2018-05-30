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
 *    @file   Anemometer.h
 *    @author gilou
 *    @date   24 mai 2017
 *    @brief  The Power class permit to measure AC or DC power with 1 voltage and 1 current
 *
 */
#ifndef POWER_H_
#define POWER_H_

//#define DEBUG_POWER

/**
 * \brief The Power class permit to measure AC or DC power with 1 voltage and 1 current
 *
 */
class Power {
public:
	/******************************************************************************
	 * Constructor and destructor
	 ******************************************************************************/
	/**
	 * Constructor
	 * @param id use a number as id, use to differenced multiple sensors and for eeprom address
	 */
	Power(unsigned char id);

	/**
	 * class destructor
	 */
	virtual ~Power();

	/******************************************************************************
	 * voltage configuration setters
	 */

	inline void set_v_pin() {((isAC = true)? (m_v_pin = 0) : (m_v_pin = 7)); update_param();}

	inline void set_v_factor(double factor) {v_factor=factor; update_param();}

	inline void set_v_offset(double offset) {v_offset=offset; update_param();}

	inline void set_v_phase(double phase) {v_phase=phase; update_param();}

	/******************************************************************************
	 * current configuration setters
	 */

	inline void set_i_pin(double pin) {m_i_pin=pin; update_param();}

	inline void set_i_factor(double factor) {i_factor=factor; update_param();}

	inline void set_i_offset(double offset) {i_offset=offset; update_param();}

	inline void set_enable(unsigned char enable) {m_enable=enable; update_param();}


	/******************************************************************************
	 * getters
	 ******************************************************************************/
	inline unsigned char is_enable()const{return m_enable;}

	/******************************************************************************
	 * Sensor's methods
	 ******************************************************************************/
	/**
	 * This method clear data array.
	 * @param measure_max data array max limit
	 */
	void clear(unsigned char measure_max);

	/**
	 * This method read the sensor's value
	 * @param measure_number the m_data array's index
	 */
	void read_value(unsigned char measure_number, unsigned char crossings, unsigned int timeout);

	/**
	* \brief This method calculate the average from the data array.
	*/
	void calc_average(unsigned char measure_max);



	char* get_average(char *string);		/// return the all averages

	/**
	 * The print method print in string "Power id"
	 * @return return string pointer
	 */
	char* print(char *string);

	/**
	 * \brief This method print the sensor configuration for the sensor. It's a good idea to overload this function to do it more explicit for each sensor.
	 */
	void print_config();

	/**
	 * The config method permit to update parameters
	 * @param stringConfig
	 */
	void config(char *stringConfig);

	/**
	 * \brief Use this method for debugging or calibration accuracy
	 */
	void print_data_array()const;

	/******************************************************************************
	 * sens_param management
	 ******************************************************************************/
	/**
	 * \brief Load saved parameters for sensors from the eeprom
	 * \return void
	 */
	void load_param();

	/**
	 * \brief Update saved parameters for sensors in the eeprom
	 * \return void
	 */
	void update_param();

	/**
	 * \brief Initialize the eeprom memory
	 * \return void
	 */
	void initialize_param();

	/**
	 * Static variables declaration
	 */
	static const int measure_max = 10; /// todo do it more flexible and global

private:
	/******************************************************************************
	 * private constant
	 */
	/******************************************************************************
	 * */
	const int ADC_COUNTS = 1024;

	/******************************************************************************
	 * Pin
	 */
	unsigned char m_v_pin;
	unsigned char m_i_pin;

	/******************************************************************************
	 * power data
	 */
	double p_data[measure_max]={0.0};	/**< this is the data array where where real power data is stored and used to calculate the average. */
	double s_data[measure_max]={0.0};	/**< this is the data array where where apparent power data is stored and used to calculate the average. */
	double pf_data[measure_max]={0.0};	/**< this is the data array where where power factor data is stored and used to calculate the average. */

	double p_average;					/**< there is the result from the real power average */

	/******************************************************************************
	 * voltage data
	 */
	double v_data[measure_max]={0.0};	/**< this is the data array where voltage data is stored and used to calculate the average. */
	double v_average;					/**< there is the result from the m_data average */
	double v_factor;					/** this is the voltage calibration factor. it initial value is 1, but if the sensor data need to be multiply, change the factor value. */
	double v_offset;					/** this is the voltage offset. It initial value is zero. Change it if you want add an offset to the sensor's data. */
	double v_phase;						/** this is the phase delay of the voltage offset. It initial value is zero. Change it if you want add an offset to the sensor's data. */

	/******************************************************************************
	 * current data
	 */
	double i_data[measure_max]={0.0};	/**< this is the where current data is stored and used to calculate the average. */
	double i_average;					/**< there is the result from the m_data average */
	double i_factor;					/** this is the current calibartion factor. it initial value is 1, but if the sensor data need to be multiply, change the factor value. */
	double i_offset;					/** this is the current offset. It initial value is zero. Change it if you want add an offset to the sensor's data. */


	/******************************************************************************
	 * Calculation variables
	 */

	double Vmeas;						/** Variable that holds measurement */
	double Imeas;						/** Variable that holds current measurement */
	double Vshifted;					/** Variable that holds the voltage value after phase compensation*/
	long double sumP, sumV, sumI;
	double referenceV;
	double v_filtered, i_filtered;
	long showSupply;


	/******************************************************************************
	 * Other variables
	 */

	int startV; 						//Instantaneous voltage at start of sample window.

	bool lastVCross, checkVCross;       /**< Used to measure number of times threshold is crossed. */

	unsigned char m_id;					/**< this the sensor's id in the Eeprom::sensor_counter */

	unsigned int m_eeprom_addr;			/**< This is the eeprom address calc from the m_id and the EEPROM_OFFSET */

	bool m_enable;						/**< If enable is off, the sensor is not process */

	bool isAC;							/**< If isAC is true, the measure is done for AC, return AC voltage, AC current and power factor, else return DC voltage and DC current */

	double m_voltage[measure_max]={0.0};/**< this is the data array where witch data are put and from where the average is done. */
	double s_average;					/**< there is the result from the apparent power average */
	double pf_average;					/**< there is the result from the power factor average */

	byte m_prec;						/**< Choose the decimal accuracy to display */
};

#endif /* POWER_H_ */
