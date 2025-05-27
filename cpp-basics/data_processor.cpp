#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <map>
#include <iomanip>

struct SensorReading {
    std::string timestamp;
    double distance;
    std::string sensor_id;
};

class DataProcessor {
private:
    std::vector<SensorReading> readings;
    
public:
    bool loadCSV(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Cannot open file " << filename << "\n";
            return false;
        }
        
        std::string line;
        // Skip header
        std::getline(file, line);
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            SensorReading reading;
            
            std::getline(ss, reading.timestamp, ',');
            ss >> reading.distance;
            ss.ignore(); // Skip comma
            std::getline(ss, reading.sensor_id);
            
            readings.push_back(reading);
        }
        
        file.close();
        return true;
    }
    
    void detectAnomalies() {
        std::cout << "\n=== Anomaly Detection ===\n";
        
        // Check for sudden jumps (possible sensor errors)
        for (size_t i = 1; i < readings.size(); ++i) {
            double diff = std::abs(readings[i].distance - readings[i-1].distance);
            
            // Flag if change > 2m in 10ms (unrealistic for most scenarios)
            if (diff > 2.0) {
                std::cout << "Anomaly at " << readings[i].timestamp 
                         << ": Jump of " << diff << "m detected\n";
            }
        }
        
        // Check for out-of-range values
        for (const auto& reading : readings) {
            if (reading.distance < 0.5 || reading.distance > 40.0) {
                std::cout << "Out of range at " << reading.timestamp 
                         << ": " << reading.distance << "m\n";
            }
        }
    }
    
    void generateReport() {
        if (readings.empty()) {
            std::cout << "No data to process\n";
            return;
        }
        
        // Calculate statistics
        double sum = 0, min_val = readings[0].distance, max_val = readings[0].distance;
        
        for (const auto& r : readings) {
            sum += r.distance;
            min_val = std::min(min_val, r.distance);
            max_val = std::max(max_val, r.distance);
        }
        
        double mean = sum / readings.size();
        
        // Calculate rate of change
        double total_change = 0;
        for (size_t i = 1; i < readings.size(); ++i) {
            total_change += std::abs(readings[i].distance - readings[i-1].distance);
        }
        double avg_change = total_change / (readings.size() - 1);
        
        // Output report
        std::cout << "\n=== Sensor Data Report ===\n";
        std::cout << "File: sensor_data.csv\n";
        std::cout << "Total readings: " << readings.size() << "\n";
        std::cout << "Time span: " << readings.front().timestamp 
                  << " to " << readings.back().timestamp << "\n";
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Distance range: " << min_val << "m - " << max_val << "m\n";
        std::cout << "Average distance: " << mean << "m\n";
        std::cout << "Average change rate: " << avg_change << "m per reading\n";
    }
    
    void exportProcessedData(const std::string& filename) {
        std::ofstream outFile(filename);
        outFile << "timestamp,distance_m,change_m,status\n";
        
        for (size_t i = 0; i < readings.size(); ++i) {
            double change = (i > 0) ? 
                readings[i].distance - readings[i-1].distance : 0.0;
            
            std::string status = "OK";
            if (std::abs(change) > 2.0) status = "ANOMALY";
            if (readings[i].distance < 0.5 || readings[i].distance > 40.0) {
                status = "OUT_OF_RANGE";
            }
            
            outFile << readings[i].timestamp << ","
                   << readings[i].distance << ","
                   << change << ","
                   << status << "\n";
        }
        
        outFile.close();
        std::cout << "\nProcessed data exported to " << filename << "\n";
    }
};

int main() {
    DataProcessor processor;
    
    std::cout << "Loading sensor data...\n";
    if (!processor.loadCSV("sensor_data.csv")) {
        return 1;
    }
    
    processor.generateReport();
    processor.detectAnomalies();
    processor.exportProcessedData("processed_sensor_data.csv");
    
    return 0;
}