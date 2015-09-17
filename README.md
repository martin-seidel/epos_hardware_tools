# epos_hardware_tools
Some tools to set and get parameters from maxon epos motor driver

Currently there two tools included:

## get_settings

prints a list of settings read from the epos. The upper part of the list match with the settings for the epos_hardware package.

__usage__:  
>rosrun epos_hardware_tools get_settings SERIAL_NUMBER

* SERIAL_NUMBER : epos serial_number in hex


## set_param

set a specific parameter in the epos

__usage__:  
>rosrun epos_hardware_tools set_param SERIAL_NUMBER INDEX SUBINDEX DATATYPE VALUE

* SERIAL_NUMBER :	 epos serial_number in hex  
* INDEX	:	address of object in the epos in hex  
* SUBINDEX :	sub-address of object in the epos in hex  
* DATATYPE :	datatype of VALUE; 'u' (unsigned) or nothing (signed) followed by the number of bits to write  
   * _Example_: u8 = uint8_t; 16 = int16_t  
* VALUE	:	the value you want to write  

__example__:  
set the velocity notation index (index 0x608b, subindex 0x00, int8_t) to 0 on an epos with SERIAL_NUMBER=serial  
>rosrun epos_hardware_tools set_param serial 608b 0 8 0
