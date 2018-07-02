# iot_eth_server
An Ethernet TCP server based on STM32F411 and KSZ8851SNL.

This project has been developed with a STM32F411E-DISCO board connected to an Ethernet1 Xplained Pro Extension Kit through SPI interface. The firmware has been developed on top of FreeRTOS+TCP. A dedicated 'NetworkInterface' module has been developed to port the TCP/IP stack on KSZ8851SNL through STM32F4's SPI peripheral.
The default local name is iot_server.local, provided through LLMNR service.


**Install**

Install ST LINK and OpenOCD on your machine.

Clone this repo in your projects directory:

    $ git clone https://github.com/marcorussi/iot_eth_server.git

Modify the Makefile according to your environment paths and preferences.


**Flash**

Connect the board, make and flash it:
 
    $ make
    $ make flash


**Client Script**

You can use the provided script for connecting to the board on local network. Install netcat and LLMNR daemon (https://github.com/tklauser/llmnrd.git) on your machine then:
 
    $ cd tools
    $ ./tcp_client 

Instert command 's' for searching your device and then connect with command 'c' by inserting the obtained IP address. Press 'Enter' for receiving last sensors data.


**TODO**

* replace dummy sensors data with real ones
* implement a proper rand function
* add mDNS service
* improve README file: add pinout and connections between boards


