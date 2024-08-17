# MVM0.1
The essence of this project is to develop a Machine Vision Module (MVM) that is 
capable of capturing images of the surrounding environment and process them frame 
by frame to obtain meaningful insights that can be used by mobile robots to improve 
their autonomous performance and perceptiveness of the surrounding. The key 
activities include integrating the OV7670 camera module with the STM32F407 by 
initializing the camera, as well as capturing and processing frames using different 
image processing algorithms. Although the focus of this project is not to stream the 
camera output for viewing, this feature will be implemented to allow monitoring of 
the images captured both before and after processing to evaluate the effectiveness of 
the system. 

For more detailed information, please refer to the slides attached:
https://github.com/a5ifshaik/MVM0.1/blob/main/MVM0.1_Slides.pdf

The development of this project would not have been made possible without past developers who have provided resources and guidance. You can find some of these resources here:

https://supuntharanga.blogspot.com/2014/04/stm32f4-discovery-board-ov7660-or.html

https://embeddedprogrammer.blogspot.com/2012/07/hacking-ov7670-camera-module-sccb-cheat.html

The overall hardware design of the CVM requires several components such as the 
OV7670, STM32F407 and pull up resistors which are connected to one another as 
shown below.

![image](https://github.com/user-attachments/assets/8874c9a0-2394-45ca-af80-6772b13edd87)

By default, the main task that is executed by the CVM after its initialisation is the 
streaming of unprocessed images via its USART peripheral. The selection of the 
algorithm to be run within the CVM is performed by sending a specific character via 
its USART RX pin, which subsequently sets a flag that determines which algorithm 
to run. Table 3.4 below summarises the different commands, the flags that is set and 
the task that is run as a result. 

![image](https://github.com/user-attachments/assets/c2492afb-b210-49d3-83c6-aecf6170fdbe)

To display the frames captured by the system, a readily available Python program written by Fabian Kung (https://github.com/fabiankung) was used.

The developed system is able to capture and process images with the following timing requirements:

![image](https://github.com/user-attachments/assets/ad42a4b3-7275-43bf-8241-0c7d1d24a260)             ![image](https://github.com/user-attachments/assets/a453bfab-8872-4dcf-9289-e9ae39da463c)

<ins>Sample Captures</ins>

![image](https://github.com/user-attachments/assets/ed1d1e3e-ebc1-46e4-8c96-f02a07d3b104)

<ins>Upcoming Releases</ins>

Improvements are currently being made for the next release, which will feature faster capture and processing times which can be achieved through the following methods:
- Exploring faster methods for image acquisition and storage, 
- Restructuring of firmware to include processing within image capture and storage (handled by DMA controller) timeframe
- Implementation of FreeRTOS for more efficient task management
- Optimization of processing algorithms

Once the performance parameters are optimized sufficiently, the following releases will focus on implementing a CNN into the MVM for object detection.

