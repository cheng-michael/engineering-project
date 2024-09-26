# Drone Sensor Integration: Logging Temperature, Pressure, and Humidity Data

## Project Overview
This project aims to integrate a Bosch BME280 and Adafruit SHT45 sensor into a drone to measure temperature, pressure, and humidity during flight. The data is logged in real time to an SD card in CSV format and to the flight log, providing insights into environmental conditions throughout ground and flight testing.

## Goals
- Gather environmental data (temperature, pressure, humidity) during drone operations.
- Perform data logging both to the SD card and the flight controller's internal flight log.
- Conduct ground tests to verify sensor performance and flight tests to collect real-world data.
- Plot and analyze the collected data for variations during the flight path.

## Code Explanation
- **Arduino Code for Adafruit SHT45:** This script tests the Adafruit SHT45 sensor, reading temperature and humidity data with adjustable precision and heater settings. Data is logged to the serial monitor in real-time, giving insight into the environmental conditions around the drone.
- **Lua Script for Bosch BME280:** The Lua script communicates with the BME280 sensor on Matek or Pixhawk. It gathers raw temperature, pressure, and humidity data, converts the data using sensor calibration factors, and logs it to both a CSV file on the SD card and the flight controller’s flight log.
- **Logging and Analysis:** The project uses both onboard SD logging and dataflash logs for precise data capture during flight. After data is collected, the goal is to visualize and analyze the data.
