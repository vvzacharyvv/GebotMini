#pragma once
#include <iostream>
#include <unordered_map>
#include <thread> // For time.sleep() equivalent
#include <chrono> // For milliseconds
#include <stdexcept> // For throwing exceptions
#include <cassert>
#include <wiringPiI2C.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
// Define constants
constexpr uint8_t ADS1x15_DEFAULT_ADDRESS = 0x48;
constexpr uint8_t ADS1x15_POINTER_CONVERSION = 0x00;
constexpr uint8_t ADS1x15_POINTER_CONFIG = 0x01;
constexpr uint8_t ADS1x15_POINTER_LOW_THRESHOLD = 0x02;
constexpr uint8_t ADS1x15_POINTER_HIGH_THRESHOLD = 0x03;
constexpr uint16_t ADS1x15_CONFIG_OS_SINGLE = 0x8000;
constexpr uint8_t ADS1x15_CONFIG_MUX_OFFSET = 12;

// Define mappings of gain values to config register values
static std::unordered_map<double, uint16_t> ADS1x15_CONFIG_GAIN = {
    {2/3, 0x0000},
    {1,   0x0200},
    {2,   0x0400},
    {4,   0x0600},
    {8,   0x0800},
    {16,  0x0A00}
};
constexpr uint16_t ADS1x15_CONFIG_MODE_CONTINUOUS = 0x0000;
constexpr uint16_t ADS1x15_CONFIG_MODE_SINGLE = 0x0100;
// Define mappings of data/sample rate to config register values for ADS1015
static std::unordered_map<int, uint16_t> ADS1015_CONFIG_DR = {
    {128,   0x0000},
    {250,   0x0020},
    {490,   0x0040},
    {920,   0x0060},
    {1600,  0x0080},
    {2400,  0x00A0},
    {3300,  0x00C0}
};

// Define mappings of data/sample rate to config register values for ADS1115
static std::unordered_map<int, uint16_t> ADS1115_CONFIG_DR = {
    {8,    0x0000},
    {16,   0x0020},
    {32,   0x0040},
    {64,   0x0060},
    {128,  0x0080},
    {250,  0x00A0},
    {475,  0x00C0},
    {860,  0x00E0}
};
constexpr uint16_t ADS1x15_CONFIG_COMP_WINDOW      = 0x0010;
constexpr uint16_t ADS1x15_CONFIG_COMP_ACTIVE_HIGH = 0x0008;
constexpr uint16_t ADS1x15_CONFIG_COMP_LATCHING    = 0x0004;
static std::unordered_map<int, uint16_t> ADS1x15_CONFIG_COMP_QUE = {
    {1,0x0000},
    {2,0x0001},
    {4,0x0002},
};
constexpr uint16_t ADS1x15_CONFIG_COMP_QUE_DISABLE = 0x0003;

class ADS1x15 {
public:
    ADS1x15(int address = ADS1x15_DEFAULT_ADDRESS) {
      device = wiringPiI2CSetup(address);
        if (device == -1) {
            throw std::runtime_error("Failed to initialize I2C device.");
        }
    }

    ~ADS1x15() {
        
    }

    // Methods to be implemented by subclasses
    virtual int _data_rate_default() = 0;
    virtual uint16_t _data_rate_config(int data_rate) = 0;
    virtual uint16_t _conversion_value(uint8_t low, uint8_t high) = 0;

    uint16_t _read(uint8_t mux, int gain, int data_rate, uint16_t mode);
    uint16_t read_comparator(uint8_t mux, int gain, int data_rate, uint16_t mode, int high_threshold,
                        int low_threshold, bool active_low, bool traditional, bool latching,
                        int num_readings); 

    uint16_t read_adc(uint8_t channel, int gain = 1, int data_rate = -1);
    uint16_t read_adc_difference(uint8_t differential, int gain = 1, int data_rate = -1);
    uint16_t start_adc(uint8_t channel, int gain = 1, int data_rate = -1);
    uint16_t start_adc_difference(uint8_t differential, int gain = 1, int data_rate = -1);
    uint16_t start_adc_comparator(uint8_t channel, uint8_t high_threshold ,
                        uint8_t low_threshold, int gain = 1, int data_rate = -1, bool active_low = true, bool traditional = true, bool latching = false,
                        int num_readings = 1);
    uint16_t start_adc_difference_comparator(uint8_t differential, uint8_t high_threshold ,uint8_t low_threshold, int gain = 1, int data_rate = -1, bool active_low = true, bool traditional = true, bool latching = false,
                        int num_readings = 1);
    void stop_adc() ;
    uint16_t get_last_result();
private:
    uint8_t address;
    int device;
};

class ADS1115 : public ADS1x15 {
public:
    ADS1115(uint8_t address = ADS1x15_DEFAULT_ADDRESS) : ADS1x15(address) {}

    int _data_rate_default() override {
        // Implementation of _data_rate_default method
        return 128;
    }

    uint16_t _data_rate_config(int data_rate) override {
        // Implementation of _data_rate_config method
            if (ADS1115_CONFIG_DR.find(data_rate) == ADS1115_CONFIG_DR.end()) {
            throw std::invalid_argument("Data rate must be one of: 8, 16, 32, 64, 128, 250, 475, 860");}
            return ADS1115_CONFIG_DR[data_rate];
        
    }

    uint16_t _conversion_value(uint8_t low, uint8_t high) override {
        int value = ((high & 0xFF) << 8) | (low & 0xFF);
        if ((value & 0x8000) != 0)
            value -= 1 << 16;
        return value;
    }
};

class ADS1015 : public ADS1x15 {
public:
    ADS1015(uint8_t address = ADS1x15_DEFAULT_ADDRESS) : ADS1x15(address) {}

    int _data_rate_default() override {
        // Implementation of _data_rate_default method
         return 1600;
    }

    uint16_t _data_rate_config(int data_rate) override {
        // Implementation of _data_rate_config method
               if (ADS1015_CONFIG_DR.find(data_rate) == ADS1015_CONFIG_DR.end()) {
            throw std::invalid_argument("Data rate must be one of: 128, 250, 490, 920, 1600, 2400, 3300");}
            return ADS1015_CONFIG_DR[data_rate];
        
    }

    uint16_t _conversion_value(uint8_t low, uint8_t high) override {
        //  Convert to 12-bit signed value
        uint16_t value = ((high & 0xFF) << 4) | ((low & 0xFF) >> 4);
        bool bit=value&0x800;
        if (bit!= 0) 
            value -= (1 << 12);
        return value;
    }
};

