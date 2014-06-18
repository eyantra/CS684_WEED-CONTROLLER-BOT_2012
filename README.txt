GROUP 4 :WEED CONTROLLER BOT Project
---------------------------------------

Team
-------
 Avantika Gupta    123050008
 Deepak Suyel      123050075
 Sandeep Garg 	   123050011
 Shruti Sharma     123050009

 Hardware Requirements
-------------------------
• Sharp IR Sensors
• Servo Motors
• Fire-bird
• Zig-bee Module
• Auxiliary Power Supply
• Gripper

Instructions
----------------
STEP:1 Entire code is present in Team4/interface.c, first open this file with AVR Studio IDE. "lcd.c" should also be present in same directory. Now, set project configurations according to frequency of bot. After this, build(compile) the project.

STEP:2 Now, interface.hex will be generated in the default folder.

STEP:3 Burn this interface.hex file using AVR Boot Loader on the BOT.

STEP:4 Restart the BOT.

STEP:5 Now, we will give command through common interface(i.e. Android Phone) according to the format specified below, that command will be received by BOT through ZigBee Module.

The format of common interface is:

BOT-Id$funCode$par1$par2$par3#

In case of our bot, the format for will be:
5$2$1#
Here, 	botId=5;
	funCode=2;	// Function for weeding is called.
	par1=1;		// This parameter tells about trough number on which function for weeding is called.
