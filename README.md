# LIN bus demo using Peugeot 508 I seatbelt indicator (STM32G474RE)
A prequel to the [Toyota RAV4 climate control panel](https://github.com/ufnalski/lin_bus_rav4_climate_control_g474re) and the [Audi A3 8V wiper actuator](https://github.com/ufnalski/lin_bus_golf7_wiper_actuator_g474re). The used part can be bought for under $10.

![Peugeot 508 LIN bus seatbelt indicator in action](/Assets/Images/peugeot508_seatbelt_indicator_in_action.jpg)

![Peugeot 207 LIN bus seatbelt indicator in action](/Assets/Images/peugeot207_seatbelt_indicator_in_action.jpg)

# Missing files?
Don't worry :slightly_smiling_face: Just hit Alt-K to generate /Drivers/CMCIS/ and /Drivers/STM32G4xx_HAL_Driver/ based on the .ioc file. After a couple of seconds your project will be ready for building.

# Libraries
* [stm32-ssd1306](https://github.com/afiskon/stm32-ssd1306) (MIT license)

# Exemplary LIN transceivers
* [Dual LIN click](https://www.mikroe.com/dual-lin-click) (MIKROE)
* [LIN Click](https://www.mikroe.com/lin-click) (MIKROE)
* [MCP2003B Click](https://www.mikroe.com/mcp2003b-click) (MIKROE)

# Readings
* [Local Interconnect Network](https://en.wikipedia.org/wiki/Local_Interconnect_Network) (Wikipedia)
* [LIN Bus Explained - A Simple Intro [2023]](https://www.csselectronics.com/pages/lin-bus-protocol-intro-basics) (CSS Electronics)
* [Introduction to the LIN bus](https://kvaser.com/about-can/can-standards/linbus/) (KVASER)
* [Local Interconnect Network (LIN) Data Link Layer](https://developerhelp.microchip.com/xwiki/bin/view/applications/lin/data-link-layer/) (Microchip)
* [STM32 as the Lin Master || Configure & Send Data](https://controllerstech.com/stm32-uart-8-lin-protocol-part-1/) (ControllersTech)
* [Using Lin Transceivers to communicate between master and slave](https://controllerstech.com/stm32-uart-9-lin-protocol-part-2/) (ControllersTech)
* [Master - slave communication using LinBus](https://controllerstech.com/stm32-uart-10-lin-protocol-part-3/) (ControllersTech)
* [LIN Specification Package Revision 2.2A](https://www.cs-group.de/wp-content/uploads/2016/11/LIN_Specification_Package_2.2A.pdf)
* [Peugeot 508 I kontrolki pasów sterowanie BUS LIN.](https://www.youtube.com/watch?v=XsC6lvLjGlY) (eCarEdu.pl)
* [Awesome LIN Bus](https://github.com/iDoka/awesome-linbus)

# What's inside?
* [TLE7259-2GE [discontinued]](https://www.infineon.com/cms/en/product/transceivers/automotive-transceiver/automotive-lin-transceivers/tle7259-2ge/) (Infineon Technologies)
* [S9S08DZ32MLC 8-bit uC [discontinued]](https://kaimte.com/product/details/freescale/s9s08dz32mlc.html) (Freescale / NXP)

![Inside the indicator](/Assets/Images/peugeot508_seatbelt_indicator_inside.jpg)

# Call for action
Create your own [home laboratory/workshop/garage](http://ufnalski.edu.pl/control_engineering_for_hobbyists/2024_dzien_popularyzacji_matematyki/Dzien_Popularyzacji_Matematyki_2024.pdf)! Get inspired by [ControllersTech](https://www.youtube.com/@ControllersTech), [DroneBot Workshop](https://www.youtube.com/@Dronebotworkshop), [Andreas Spiess](https://www.youtube.com/@AndreasSpiess), [GreatScott!](https://www.youtube.com/@greatscottlab), [ElectroBOOM](https://www.youtube.com/@ElectroBOOM), [Phil's Lab](https://www.youtube.com/@PhilsLab), [atomic14](https://www.youtube.com/@atomic14), [That Project](https://www.youtube.com/@ThatProject), [Paul McWhorter](https://www.youtube.com/@paulmcwhorter), [Max Imagination](https://www.youtube.com/@MaxImagination), [Nikodem Bartnik](https://www.youtube.com/@nikodembartnik), and many other professional hobbyists sharing their awesome projects and tutorials! Shout-out/kudos to all of them!

> [!WARNING]
> Control engineering - do try this at home :sunglasses:

190+ challenges to start from: [Control Engineering for Hobbyists at the Warsaw University of Technology](http://ufnalski.edu.pl/control_engineering_for_hobbyists/Control_Engineering_for_Hobbyists_list_of_challenges.pdf).

Stay tuned!