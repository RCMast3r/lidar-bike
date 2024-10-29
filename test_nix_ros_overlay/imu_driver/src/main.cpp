#include <driver.hpp>
#include <chrono>
#include <thread>

int main()
{
    I2CDriver driver("/dev/i2c-1");
    const std::chrono::milliseconds interval(5);

    while (true) {
        auto start = std::chrono::steady_clock::now();
        auto data = driver.sample_data();
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        // Sleep for the remaining time to maintain the fixed rate
        std::this_thread::sleep_for(interval - elapsed);
    }

    return 0;
}

