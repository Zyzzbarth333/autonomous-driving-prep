#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <numeric>

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
    
    // Output to console
    std::cout << "\nStatistics:\n";
    std::cout << "Count: " << data.size() << "\n";
    std::cout << "Mean: " << mean << "\n";
    std::cout << "Min: " << min << "\n";
    std::cout << "Max: " << max << "\n";
    
    // Save to file
    std::ofstream outFile("results.txt");
    outFile << "Sensor Data Statistics\n";
    outFile << "Count: " << data.size() << "\n";
    outFile << "Mean: " << mean << "\n";
    outFile << "Min: " << min << "\n";
    outFile << "Max: " << max << "\n";
    outFile.close();
    
    std::cout << "\nResults saved to results.txt\n";
    
    return 0;
}