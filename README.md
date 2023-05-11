# DIY-Dyno
Dyno control and data acquisition based on ESP32 microprocessor and strain guage style load cell

## About 
The DIY Dyno project is an open source dyno controller and data acquisition system based on the ESP32 wifi enabled microcontroller.

The project consists of an ESP32 microcontroller and a PCB shield that interfaces to the necessary field sensors (load cell / tacho etc).

The ESP32 crunches the maths and then displays the data to the user via web browser as a graph that displays torque and power. 

Data can be saved to your local device and extracted for later comparison or printing.

 

## Software
The software code is developed in C++ and runs on an ESP32 microprocessor.  The ESP processes sensor data and uses the results to calculate torque and power, which is then displayed via web browser on any web enabled device. The system can be used to display results to various reference standards, for example SAE J1394 or ISO 1584.

The software also includes provision for data recording, allowing tests to be saved for later viewing and analysis.

The software supports both brake and inertia style dynos.

The code is open source and is available under the GNU GPL V3 license.

 

## Hardware
The hardware aspect of the project comprises of a PCB (commonly known as a 'shield') which connects to the ESP processor. The PCB includes environmental sensors to measure temperature, humidity and barometric pressure. The PCB also contains load cell bridges to connect to the load cells mounted to the dyno and tacho inputs for speed monitoring.

The circuit schematics are also open source which allows people to develop their own shields should they wish.

Both the software and hardware aspects of the project are available for free from the projects GitHub repository. The GitHub repo also contains all of the information needed to build your own volumetric flow bench in the form of a WIKI and Support forum.

The DIYDyno system can theoretically be used on any dyno and makes an ideal retrofit for older dynos as well as being the perfect choice for new builds.

 

## Project Goals
Validated software and hardware design.
Affordable & easy to source components.
Can be built & operated by a layman.
Generates results comparable with commercial dyno systems.
Open source software and hardware design.
DIY Shield kits available to purchase.
 

## Licenses
The project code is open source and released under the GNU GPLV3 license.

The project hardware is open source and released under the CERN–OHL–W license


## Project Status
The project is currently in Alpha stage working towards initial code release.
The goal with the initial release is to have basic functional code that has most of the V1 project functionality working to the point where the user can read torque and work done using a load cell and tacho input
At this stage the project will move into beta where code functionality will be validated in order to prepare a release candidate.

## Further Information 
[Project WIKI](https://github.com/DeeEmm/DIY-Dyno/wiki)
[Github repository](https://github.com/DeeEmm/DIY-Dyno)
[Support forums](https://github.com/DeeEmm/DIY-Dyno/discussions)
[Discord](https://discord.gg/9YpKxp5ctV)
[Join our project community](https://www.facebook.com/groups/diydyno/)
