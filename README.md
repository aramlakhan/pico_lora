# Lora_Pico_Driver

## [Click here to quickly clone this repository template to Your Github](https://github.com/new?template_name=Lora_Pico_Driver&template_owner=cschorn01)

[![Top Langs](https://github-readme-stats.vercel.app/api/top-langs/?username=cschorn01&layout=compact&theme=dark)](https://github.com/cschorn01/Lora_Pico_Driver)

## Description

The **Lora_Pico_Driver** project is an open source project based on the [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/), 
[Lora Radio](https://www.semtech.com/products/wireless-rf/lora-connect/sx1280), and [freeRTOS](https://www.freertos.org/). It's goal is to give hobbyists and developers a strong starting point for their projects involving Lora. This project is the [PHY layer](https://lora-developers.semtech.com/documentation/tech-papers-and-guides/lora-and-lorawan) of the Lora modem, with [LoraWAN](https://lora-developers.semtech.com/documentation/tech-papers-and-guides/lora-and-lorawan) functionality coming soon.

## Functionality
An open driver for [Lora](https://www.semtech.com/products/wireless-rf/lora-connect/sx1280) on a [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/) would not easily allow for the expansion of an application.
Adding [freeRTOS](https://www.freertos.org/) to the [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/) allows an application developer to easily create new functionalities.
For example, it would be challenging to add a new sensor to a sensor array with [Lora](https://www.semtech.com/products/wireless-rf/lora-connect/sx1280) in a single 
superloop, but with freeRTOS we can create a new task, or individually addressable superloop, that interacts
with the [Lora](https://www.semtech.com/products/wireless-rf/lora-connect/sx1280) task independently of the original sensor array. In this project there are three tasks, 
and main():
1. `vSimpleLEDTask` is a few lines to show 
the structure of a task with the setup being above the infinite loop, and the blinking of the onboard pico
LED to show the action of the infinite loop.
2. `vUsbIOTask` which takes input from a serial monitor over usb using getchar(). A 0x00 or NULL is 
appended to the end of the unsigned 8 bit pointer array used as a dynamic ascii character array for a NULL terminated 'string'. 
The address of the pointer array is then sent to `vSx1280Task` for sending through the sx1280.
3. `vSx1280Task` will take the pointer array from `vUsbIOTask` through a *Task Notification* and reassign the pointer to a 
task local pointer array. This is done
because there are functions called which delays `vSx1280Task` and would allow `vUsbIOTask` to overwrite the *Task Notification* pointer array.
Once the data is task local `vSx1280Task` will perform a Tx operation, followed by an Rx operation. Both operations are done by using the 
`sx1280Setup`, `sx1280Tx`, and `sx1280Rx` functions, also in `main.c`.

## File Structure
The file structure of this project is dictated by the use of the [C/C++ SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf) offered by Raspberry Pi, and by [freeRTOS](https://www.freertos.org/). The files `CMakeLists.txt`, and `pico_sdk_import.cmake` come from the [C/C++ SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf), the file `include/FreeRTOSConfig.h`, and the directory `FreeRTOS-Kernel` come from the use of [freeRTOS](https://www.freertos.org/). If you have not, I encourage you to go through the [getting started with pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) documentation to set up your coding environment. The directory tree should resemble the list below:
- pico_projects
  - [pico_sdk](https://github.com/raspberrypi/pico-sdk)
  - Lora_Pico_driver  
    - [docs](https://github.com/cschorn01/Lora_Pico_Driver/tree/main/docs)  
    - [FreeRTOS-Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel)
      - [include](https://github.com/cschorn01/Lora_Pico_Driver/tree/main/FreeRTOS-Kernel/include)  
        - [FreeRTOSConfig.h](https://github.com/cschorn01/Lora_Pico_Driver/blob/main/FreeRTOS-Kernel/include/FreeRTOSConfig.h) 
    - [src](https://github.com/cschorn01/Lora_Pico_Driver/tree/main/src)  
      - [main.c](https://github.com/cschorn01/Lora_Pico_Driver/blob/main/src/main.c)  
    - [CMakeLists.txt](https://github.com/cschorn01/Lora_Pico_Driver/blob/main/CMakeLists.txt)  
    - [pico_sdk_import.cmake](https://github.com/cschorn01/Lora_Pico_Driver/blob/main/pico_sdk_import.cmake)
    - [FreeRTOS_Kernel_import.cmake](https://github.com/cschorn01/Lora_Pico_Driver/blob/main/FreeRTOS_Kernel_import.cmake)

## How To Use
This project is not meant to be used as a library, instead it's a [template](https://github.com/new?template_name=Lora_Pico_Driver&template_owner=cschorn01) to begin a given project involving a [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/), and [Lora Modem](https://www.semtech.com/products/wireless-rf/lora-connect/sx1280). In the `vSx1280Task` you will find an example of how to send and receive message packets over 2.4GHz Lora. This example even includes input over usb through a serial monitor which is included in an outbound message. I encourage you to add sensors, or displays and create your own long range wireless [Internet of Things](https://en.wikipedia.org/wiki/Internet_of_things) network.  

There are two ways of using this project:  
1. Creating functions to handle new hardware, and using them in `vSx1280Task`
2. Creating a task to handle new hardware with functions used in the newly written task
  
The second method is recomended, as it is more dynamic and easier to debug. Once downloaded however, it is your code with which you may do as you please.  
  
Given a new task has been created to handle new hardware there must be communication between tasks. Thats where [freeRTOS Task Notifications](https://www.freertos.org/RTOS-task-notifications.html) come in. They are fast and easy to use for sending data between task in a memory efficient manner. In your new task you should create an 8 bit pointer array to store data in an sx1280 message compatible format:  
  
>```
> uint8_t *dataBuffer = 0;
> ```  
  
Allocate the appropriate amount of memory, in 8 bit chunks, that you'll need to send your data:  
  
> ```
> dataBuffer = ( uint8_t * ) malloc( 256 * sizeof( uint8_t ) );
> ```  
  
Here 256 is being used because that is the maximum Lora packet size on the sx1280.  
Assign your data to the newly allocated data buffer:  
  
>```
>*( dataBuffer + 0 ) = 0x48; // 0x48 is ASCII Hexadecimal 'H' 
>*( dataBuffer + 1 ) = 0x49; // 0x49 is ASCII Hexadecimal 'I'
>```

Use the [`xTaskNotify()`](https://www.freertos.org/xTaskNotify.html) function to send a task notification from your new task to `vSx1280Task`:

> ```
> xTaskNotify(  
>             xSx1280TaskHandle,                  /* TaskHandle_t xTaskToNotify */  
>             ( uint32_t ) &*( dataBuffer ),      /* uint32_t ulValue (int)&buffer[0] */  
>             eSetValueWithoutOverwrite );        /* eNotifyAction eAction */  
>            )
> ```
  
In `vSx1280Task` the [`xTaskNotifyWait()`](https://www.freertos.org/xTaskNotifyWait.html) will accept *Task Notifications* from all tasks that are sending them. You must process the current *Task Notification* before allowing another task to run or the current *Task Notification* may be overwritten by an incoming *Task Notification* from another task.
  
  

