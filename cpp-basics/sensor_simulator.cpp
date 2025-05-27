#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <chrono>
#include <thread>
#include <iomanip>
#include <ctime>

class SensorSimulator {
private:
    std::mt19937 gen;
    std::normal_distribution<> noise;
    double base_distance;
    double drift_rate;
    
public:
    SensorSimulator() : gen(std::random_device{}()), 
                       noise(0.0, 0.5),  // Mean 0, StdDev 0.5
                       base_distance(10.0),
                       drift_rate(0.1) {}
    
    double readSensor() {
        // Simulate object getting closer/farther
        base_distance += drift_rate * (std::rand() % 3 - 1);
        
        // Add sensor noise
        double reading = base_distance + noise(gen);
        
        // Clamp to realistic sensor range (0.5m to 50m)
        return std::max(0.5, std::min(50.0, reading));
    }
    
    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::time_t time = std::chrono::system_clock::to_time_t(now);
        std::tm* tm = std::localtime(&time);
        
        std::ostringstream oss;
        oss << std::put_time(tm, "%Y-%m-%d %H:%M:%S");
        oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
        
        return oss.str();
    }
};

int main() {
    SensorSimulator lidar;
    std::ofstream dataFile("sensor_data.csv");
    
    // Write CSV header
    dataFile << "timestamp,distance_m,sensor_id\n";
    
    int samples;
    std::cout << "How many sensor readings to simulate? ";
    std::cin >> samples;
    
    std::cout << "Simulating LIDAR sensor...\n";
    
    for (int i = 0; i < samples; ++i) {
        std::string timestamp = lidar.getCurrentTimestamp();
        double distance = lidar.readSensor();
        
        // Write to CSV
        dataFile << timestamp << "," 
                << std::fixed << std::setprecision(3) << distance 
                << ",LIDAR_01\n";
        
        // Display progress
        if (i % 10 == 0) {
            std::cout << "Reading " << i << ": " << distance << "m\n";
        }
        
        // Simulate 100Hz sensor rate
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    dataFile.close();
    std::cout << "\nData saved to sensor_data.csv\n";
    
    return 0;
}