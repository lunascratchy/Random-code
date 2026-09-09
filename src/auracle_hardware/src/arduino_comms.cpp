#include "auracle_hardware/arduino_comms.hpp"
#include <iostream>
#include <string>
#include <sstream>

// This is a simplified implementation. 
// In reality, you'd use a library like 'libserial' or 'boost::asio'.

void ArduinoComms::sendVelocity(double left, double right) {
    // Format: "v 1.2 -0.5\n"
    std::stringstream ss;
    ss << "v " << left << " " << right << "\n";
    std::string cmd = ss.str();
    
    // write_to_serial(cmd); // Call your serial port write function
}

void ArduinoComms::readTelemetry(long &l_enc, long &r_enc, float* acc, float* gyro) {
    // read_from_serial() until '\n'
    std::string line = "e 1000 2000 i 0.1 0.2 9.8 0.01 0.02 0.03"; // Mock data
    
    if (line[0] == 'e') {
        // Parse: e <l> <r> i <ax> <ay> <az> <gx> <gy> <gz>
        sscanf(line.c_str(), "e %ld %ld i %f %f %f %f %f %f", 
               &l_enc, &r_enc, &acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
    }
}