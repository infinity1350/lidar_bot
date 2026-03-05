#ifndef ARDUINO_COMMS_HPP
#define ARDUINO_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <cstdlib>

LibSerial::BaudRate convertBaudRate(int baudrate)
{
    switch(baudrate){
        case 1200:
            return LibSerial::BaudRate::BAUD_1200;
        case 2400:
            return LibSerial::BaudRate::BAUD_2400;
        case 4800:
            return LibSerial::BaudRate::BAUD_4800;
        case 9600:
            return LibSerial::BaudRate::BAUD_9600;
        case 19200:
            return LibSerial::BaudRate::BAUD_19200;
        case 38400:
            return LibSerial::BaudRate::BAUD_38400;
        case 57600:
            return LibSerial::BaudRate::BAUD_57600;
        case 115200:
            return LibSerial::BaudRate::BAUD_115200;
        case 230400:
            return LibSerial::BaudRate::BAUD_230400;

        default:
            std::cerr << "Unsupported baudrate: " << baudrate << std::endl;
            return LibSerial::BaudRate::BAUD_9600; // Default to 9600 if unsupported
    }

    
}

class ArduinoComms
{
    public:
        ArduinoComms() = default;

        void connect(const std::string &serial_device, int baudrate, int timeout_ms)
        {
            timeout_ms_ = timeout_ms;
            serial_port_.Open(serial_device);
            serial_port_.SetBaudRate(convertBaudRate(baudrate));
        }

        bool connected() const
        {
            return serial_port_.IsOpen();
        }

        void disconnect()
        {
            serial_port_.Close();
        }

        std::string send_msg(const std::string &msg, bool print_output = false)
        {
            serial_port_.FlushIOBuffers();
            serial_port_.Write(msg);

            std::string response = "";

            try
            {
                serial_port_.ReadLine(response,'\n', timeout_ms_);
            }
            catch (const LibSerial::ReadTimeout &)
            {
                std::cerr << "Read timeout occurred after " << timeout_ms_ << " ms." << std::endl;
                return "";
            }
            if (print_output)
            {
                std::cout << "Response: " << response << std::endl;
            }
             return response;

        }

        void send_empty_msg()
        {
            std::string response = send_msg("\r");
        }

        void read_encoders(int &encoder_value1, int &encoder_value2)
        {
            std::string response = send_msg("e\r");

            std::cout << "Raw encoder response: '" << response << "'" << std::endl;

            // Remove any whitespace/newlines
            response.erase(response.find_last_not_of(" \n\r\t") + 1);

            if (response.empty()) {
                std::cerr << "Empty response from Arduino" << std::endl;
                encoder_value1 = 0;
                encoder_value2 = 0;
                return;
            }
            
            try {

                std::size_t delimiter_pos = response.find(' ');
                std::string enc1 = response.substr(0, delimiter_pos);
                std::string enc2 = response.substr(delimiter_pos + 1);

                encoder_value1 = std::stoi(enc1);
                encoder_value2 = std::stoi(enc2);
            } 
            catch (const std::exception& e) {
                std::cerr << "Failed to convert encoder response: '" << response << "' Error: " << e.what() << std::endl;
                encoder_value1 = 0;
                encoder_value2 = 0;
            }
            
        }

        void set_motor_value(int motor_val1, int motor_val2)
        {
            std::stringstream ss;
            ss << "m "<< motor_val1 <<" "<< motor_val2 << "\r";
            send_msg(ss.str());
        }

        void set_pid_values(int k_p, int k_d, int k_o)
        {
            std::stringstream ss;
            ss << "u " << k_p << ":" << k_d << ":" << k_o << "\r";
            send_msg(ss.str());
        }

    private:
        LibSerial::SerialPort serial_port_;
        int timeout_ms_;
};
#endif //ARDUINO_COMMS_HPP