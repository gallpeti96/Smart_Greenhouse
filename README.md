# Smart Greenhouse

This repository aims to collect relevant codes snippets, scripts written in Python and for Node-RED for a smart greenhouse environment.

## Basic architecture

The system contains several sensor nodes and a few intervener nodes. The nodes are based on the Raspberry PI Zero platform, and 
Python scripts are used for data reading and sending. The mesured data is transmitted using MQTT protocol over NB-IoT technology to 
a server node that collects, stores them, as well as it is able to intervene automatically according to the defined rules. 
The server node runs Node-RED that contains the rule definitions, and it is also responsible for the data visualization.

## Features

The main measured environmental parameters of the nodes:
* temperature (in and out)
* barometric pressure
* humidity
* gas
* light intensity
* soil moisture 
* wind speed (out)

Other measured parameters:
* energy consumption of nodes
* GNSS position of nodes

Controlable features:
* open/close window
* start/stop watering
* start/stop air circulation
* turn on/off lights
