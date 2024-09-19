# STM32 Turret

## The Project
As a summer project, I set out to use an STM32 microcontroller (particularly the STM32l476rg Nucleo Board) to control a turret, which is driven by 2 stepper motors for gun aiming and 1 servo motor for a trigger-pulling mechanism. I did this with the purpose of familiarizing myself with the STM32 family of microcontrollers and of having a cool project to apply my knowledge of them. The turret was controlled remotely by a PS3 controller which interfaced with an ESP32 board. In addition to remotely controlling the turret's steppers and servos, there was an independent part of the project, in which I streamed video from a small camera (fixed to the gun-mounting arm), to a STM32 MPU, which I booted with a custom Linux distribution; however, this component of the project is not the concern of this repo. 

## This Repo
The focus of this repo is the C-programming and configuring of the STM32l476rg necessary to run the STM32 MCU component of the project. This repo contains the code and configuration files that I used to set up and program the board in STM32CubeIDE. Most of the configuration work consisted of configuring the a two timers to generate PWM signals at the correct frequencies for the motor drivers and for the servo motor, which also operated at PWM. I also had to configure a UART port to receive data from the ESP32 board that was receiving data via Bluetooth from the PS3 controller. In terms of programming, I had to receive data over UART from the ESP32 board, parse the data for commands (horizontal/vertical rotation & trigger pulling), and then update the frequency sent from the PWM pins according to the driver and motor specifications.

## Turret Photos
### Turret front-view with controller.
![image](https://github.com/user-attachments/assets/1d0982e7-cbb9-4a46-8cb6-d6c1163ade8b)

### Second turret front-view; trigger servo can be seen on flywheel laying unattached.
![image](https://github.com/user-attachments/assets/dc3d8c8a-047a-4e52-8144-dbaa9136c091)

### Inside of homemade project box containing microcontrollers, stepper motor drivers, and battery adapters.
![image](https://github.com/user-attachments/assets/204c5ae0-defb-4678-a837-e7ae471d3ad6)

