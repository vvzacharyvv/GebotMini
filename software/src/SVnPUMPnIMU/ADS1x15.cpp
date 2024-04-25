#include "ADS1x15.h"
unsigned short Convert(unsigned short s) {
	char right, left;
	right = s& 0XFF;//低八位
	left = s >> 8;//高八位  右移8位
	s = right * 256 + left;
	return s;
}

uint16_t ADS1x15::_read(uint8_t mux, int gain, int data_rate, uint16_t mode) {
        uint16_t config = ADS1x15_CONFIG_OS_SINGLE;  // Go out of power-down mode for conversion.
        // Specify mux value.
        config |= (mux & 0x07) << ADS1x15_CONFIG_MUX_OFFSET;
        // Validate the passed in gain and then set it in the config.
        if (ADS1x15_CONFIG_GAIN.find(gain) == ADS1x15_CONFIG_GAIN.end()) {
            throw std::invalid_argument("Gain must be one of: 2/3, 1, 2, 4, 8, 16");
        }
        config |= ADS1x15_CONFIG_GAIN[gain];
        // Set the mode (continuous or single shot).
        config |= mode;
        // Get the default data rate if none is specified (default differs between
        // ADS1015 and ADS1115).
        if (data_rate == -1) {
            data_rate = _data_rate_default();
        }
        // Set the data rate (this is controlled by the subclass as it differs
        // between ADS1015 and ADS1115).
        config |= _data_rate_config(data_rate);
        config |= ADS1x15_CONFIG_COMP_QUE_DISABLE;  // Disable comparator mode.
        //sithch high and low
        config=Convert(config);
        wiringPiI2CWriteReg16(device, ADS1x15_POINTER_CONFIG, config);
        // wiringPiI2CWriteReg8(device, ADS1x15_POINTER_CONFIG, (config>>8)&0xFF);
        // wiringPiI2CWriteReg8(device, ADS1x15_POINTER_CONFIG+1, (config)&0xFF);
        // Wait for the ADC sample to finish based on the sample rate plus a
        // small offset to be sure (0.1 millisecond).
        usleep(1000000.0/data_rate+100);
        // Retrieve the result.
           uint16_t result = wiringPiI2CReadReg16(device, ADS1x15_POINTER_CONVERSION);
    //    uint8_t high=wiringPiI2CReadReg8(device,ADS1x15_POINTER_CONVERSION);
    //    uint8_t low=wiringPiI2CReadReg8(device,ADS1x15_POINTER_CONVERSION+1);
            uint8_t low=result>>8 &0xff;
            uint8_t high = result &0xff;
        //std::cout<<"channel "<<mux-0x04<<": low= "<<(int)low<<"high= "<<(int)high<<std::endl;
        //return result;
        return _conversion_value( (result >> 8) & 0xFF,result & 0xFF);
       // return _conversion_value(low, high);
    }
uint16_t ADS1x15::read_comparator(uint8_t mux, int gain, int data_rate, uint16_t mode, int high_threshold,
                        int low_threshold, bool active_low, bool traditional, bool latching,
                        int num_readings) {
        assert(num_readings == 1 || num_readings == 2 || num_readings == 4);
        high_threshold=Convert(high_threshold);
        low_threshold=Convert(low_threshold);
        // Set high and low threshold register values
        wiringPiI2CWriteReg16(device, ADS1x15_POINTER_HIGH_THRESHOLD, high_threshold);
        wiringPiI2CWriteReg16(device, ADS1x15_POINTER_LOW_THRESHOLD, low_threshold);

        // Build config register value
        uint16_t config = ADS1x15_CONFIG_OS_SINGLE;  // Go out of power-down mode for conversion

        // Specify mux value
        config |= (mux & 0x07) << ADS1x15_CONFIG_MUX_OFFSET;

        // Validate the passed in gain and then set it in the config
        // Add the gain mapping and validation here
        if (ADS1x15_CONFIG_GAIN.find(gain) == ADS1x15_CONFIG_GAIN.end()) {
            throw std::invalid_argument("Gain must be one of: 2/3, 1, 2, 4, 8, 16");
        }
        config |= ADS1x15_CONFIG_GAIN[gain];
        // Set the mode (continuous or single shot)
        config |= mode;

         if (data_rate == -1) {
            data_rate = _data_rate_default();
        }
        // Set the data rate (this is controlled by the subclass as it differs
        // between ADS1015 and ADS1115).
        config |= _data_rate_config(data_rate);
        // Get the default data rate if none is specified
        // Set the data rate (this is controlled by the subclass)
        // Add data rate configuration here

        // Enable window mode if required
        if (!traditional) {
            config |= ADS1x15_CONFIG_COMP_WINDOW;
        }

        // Enable active high mode if required
        if (!active_low) {
            config |= ADS1x15_CONFIG_COMP_ACTIVE_HIGH;
        }

        // Enable latching mode if required
        if (latching) {
            config |= ADS1x15_CONFIG_COMP_LATCHING;
        }

        // Set number of comparator hits before alerting
        config |= ADS1x15_CONFIG_COMP_QUE[num_readings];
        // Add comparator queue configuration here
        config=Convert(config);
        // Send the config value to start the ADC conversion
        wiringPiI2CWriteReg16(device, ADS1x15_POINTER_CONFIG, config);

        // Wait for the ADC sample to finish based on the sample rate plus a small offset
        usleep(1000000/data_rate + 100);  // Convert data rate to microseconds

        // Retrieve the result
        // Read the conversion register value and return it
        uint16_t result = wiringPiI2CReadReg16(device, ADS1x15_POINTER_CONVERSION);
        return _conversion_value((result >> 8) & 0xFF,result & 0xFF);
    }

uint16_t ADS1x15::read_adc(uint8_t channel, int gain , int data_rate) {
        // Implementation of read_adc method
        assert(0<=channel && channel<=3);
        return _read(channel+0x04, gain, data_rate, ADS1x15_CONFIG_MODE_SINGLE);
    }

uint16_t ADS1x15::read_adc_difference(uint8_t differential, int gain , int data_rate )
{   
        //  """Read the difference between two ADC channels and return the ADC value
        // as a signed integer result.  Differential must be one of:
        //   - 0 = Channel 0 minus channel 1
        //   - 1 = Channel 0 minus channel 3
        //   - 2 = Channel 1 minus channel 3
        //   - 3 = Channel 2 minus channel 3
        // """
    assert(0<=differential && differential<=3);
    return _read(differential, gain, data_rate, ADS1x15_CONFIG_MODE_SINGLE);
}
uint16_t ADS1x15::start_adc(uint8_t channel, int gain , int data_rate ){
        //  """Start continuous ADC conversions on the specified channel (0-3). Will
        // return an initial conversion result, then call the get_last_result()
        // function to read the most recent conversion result. Call stop_adc() to
        // stop conversions.
        // """
        assert(0<=channel && channel<=3);
        return _read(channel+0x04, gain, data_rate, ADS1x15_CONFIG_MODE_CONTINUOUS);
    }
uint16_t ADS1x15::start_adc_difference(uint8_t differential, int gain , int data_rate)
{   
        //  """Read the difference between two ADC channels and return the ADC value
        // as a signed integer result.  Differential must be one of:
        //   - 0 = Channel 0 minus channel 1
        //   - 1 = Channel 0 minus channel 3
        //   - 2 = Channel 1 minus channel 3
        //   - 3 = Channel 2 minus channel 3
        // """
    assert(0<=differential && differential<=3);
    return _read(differential, gain, data_rate, ADS1x15_CONFIG_MODE_SINGLE);
}
uint16_t ADS1x15:: start_adc_comparator(uint8_t channel, uint8_t high_threshold ,
                        uint8_t low_threshold, int gain , int data_rate , bool active_low, bool traditional, bool latching ,
                        int num_readings ) {
            assert(0<=channel && channel<=3);
        return read_comparator(channel+0x04, gain, data_rate, ADS1x15_CONFIG_MODE_CONTINUOUS,high_threshold, low_threshold, active_low,
                                     traditional, latching, num_readings);
    }

uint16_t ADS1x15::start_adc_difference_comparator(uint8_t differential, uint8_t high_threshold ,uint8_t low_threshold, int gain , int data_rate , bool active_low , bool traditional, bool latching ,
                        int num_readings ) {
            assert(0<=differential && differential<=3);
        return read_comparator(differential, gain, data_rate, ADS1x15_CONFIG_MODE_CONTINUOUS,high_threshold, low_threshold, active_low,
                                     traditional, latching, num_readings);
    }
void ADS1x15::stop_adc() {
        // Set the config register to its default value of 0x8583 to stop
        // continuous conversions
        uint16_t config = 0x8583;
        config=Convert(config);
        wiringPiI2CWriteReg16(device,ADS1x15_POINTER_CONFIG, config);
    }

uint16_t ADS1x15::get_last_result() {
        // Set the config register to its default value of 0x8583 to stop
        // continuous conversions
        uint16_t result = wiringPiI2CReadReg16(device, ADS1x15_POINTER_CONVERSION);
        return _conversion_value( (result >> 8) & 0xFF,result & 0xFF);
    }


