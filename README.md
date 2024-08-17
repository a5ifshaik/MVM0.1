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

![image](https://github.com/user-attachments/assets/794f17fb-1f46-4011-b170-0784bb51dbcd)

By default, the main task that is executed by the CVM after its initialisation is the 
streaming of unprocessed images via its USART peripheral. The selection of the 
algorithm to be run within the CVM is performed by sending a specific character via 
its USART RX pin, which subsequently sets a flag that determines which algorithm 
to run. Table 3.4 below summarises the different commands, the flags that is set and 
the task that is run as a result. 

![image](https://github.com/user-attachments/assets/b7563fee-52ad-4c4e-85cf-adf6769dd062)

To display the frames captured by the system, a readily available Python program written by Fabian Kung (https://github.com/fabiankung) was used.

The developed system is able to capture and process images with the following timing requirements:

![image](https://github.com/user-attachments/assets/2fb0f865-2ac5-4673-bb36-5f0b2b51ce8c)             ![image](https://github.com/user-attachments/assets/95e1d5b9-1e31-4add-9aa2-a348b3f0e059)

<ins>Sample Captures</ins>

![image](https://github.com/user-attachments/assets/8ff83710-7801-4b8c-acad-66c668c7ee8a)

<ins>Upcoming Releases</ins>

Improvements are currently being made for the next release, which will feature faster capture and processing times which can be achieved through the following methods:
- Exploring faster methods for image acquisition and storage, 
- Restructuring of firmware to include processing within image capture and storage (handled by DMA controller) timeframe
- Implementation of FreeRTOS for more efficient task management
- Optimization of processing algorithms

Once the performance parameters are optimized sufficiently, the following releases will focus on implementing a CNN into the MVM for object detection.

