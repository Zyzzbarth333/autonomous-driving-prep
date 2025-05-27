#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <numeric>
#include <cmath>

int main() {
    std::vector<double> data;
    double value;
    
    std::cout << "Enter sensor values (press Ctrl+D when done):\n";
    while (std::cin >> value) {
        data.push_back(value);
    }
    
    if (data.empty()) {
        std::cout << "No data entered!\n";
        return 1;
    }
    
    // Calculate statistics
    double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
    double min = *std::min_element(data.begin(), data.end());
    double max = *std::max_element(data.begin(), data.end());
    
    // Calculate median
    std::sort(data.begin(), data.end());
    double median;
    if (data.size() % 2 == 0) {
        median = (data[data.size()/2 - 1] + data[data.size()/2]) / 2.0;
    } else {
        median = data[data.size()/2];
    }
    
    // Calculate standard deviation
    double variance = 0.0;
    for (double val : data) {
        variance += std::pow(val - mean, 2);
    }
    variance /= data.size();
    double stddev = std::sqrt(variance);
    
    // Output results
    std::cout << "\nStatistics:\n";
    std::cout << "Count: " << data.size() << "\n";
    std::cout << "Mean: " << mean << "\n";
    std::cout << "Median: " << median << "\n";
    std::cout << "StdDev: " << stddev << "\n";
    std::cout << "Min: " << min << "\n";
    std::cout << "Max: " << max << "\n";
    
    // Save to file
    std::ofstream outFile("results.txt");
    outFile << "Sensor Data Statistics\n";
    outFile << "Count: " << data.size() << "\n";
    outFile << "Mean: " << mean << "\n";
    outFile << "Median: " << median << "\n";
    outFile << "StdDev: " << stddev << "\n";
    outFile << "Min: " << min << "\n";
    outFile << "Max: " << max << "\n";
    outFile.close();
    
    std::cout << "\nResults saved to results.txt\n";
    
    return 0;
}