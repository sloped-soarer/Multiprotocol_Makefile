The aim of this project was to build the Multiprotocol code base taken from https://github.com/pascallanger/DIY-Multiprotocol-TX-Module
outside of the Arduino environment with just a Makefile.
At the moment it will just build the OrangeRX Module code (AVR Xmega32d4) and requires some Human intervention to work.
I believe the Arduino environment uses Ctags and maybe a little java to resolve compilation issues due to missing  function prototypes and 
varaible references.
This code uses Cproto rather than Ctags to generate the above.

Feel free to make suggestions and adapt this code as neccessary as I openly admit it is not a complete project.
