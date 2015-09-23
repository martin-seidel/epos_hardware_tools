# epos_hardware_tools
Some tools to set and get parameters from maxon epos motor driver

Currently there two tools included:

## get_settings

prints a list of settings read from the epos. The upper part of the list match with the settings for the epos_hardware package.  
It's also possible to read just one specific value from an epos.  
Called with no arguments will print the list of parameter for all connected epos.

__usage__:  
>rosrun epos_hardware_tools get_settings  
>rosrun epos_hardware_tools get_settings SERIAL_NUMBER  
>rosrun epos_hardware_tools get_settings SERIAL_NUMBER INDEX SUBINDEX DATATYPE

* SERIAL_NUMBER : epos serial_number in hex or a "?" for all connected epos
* INDEX	:	address of object in the epos in hex  
* SUBINDEX :	sub-address of object in the epos in hex  
* DATATYPE :	datatype of VALUE; 'u' (unsigned) or nothing (signed) followed by the number of bits to write  
   * _Supported_: u8 = uint8_t; u16 = uint16_t; u32 = uint32_t; u64 = uint64_t; 8 = int8_t; 16 = int16_t; 32 = int32_t; 64 = int64_t

__example__:  
(1) read the velocity notation index (index 0x608b, subindex 0x00, int8_t) from all connected epos  
>rosrun epos_hardware_tools get_settings ? 608b 0 8  

(2) read the list of parameter from an epos with SERIAL_NUMBER=serial  
>rosrun epos_hardware_tools get_settings serial


## set_param

set a specific parameter on an epos

__usage__:  
>rosrun epos_hardware_tools set_param SERIAL_NUMBER INDEX SUBINDEX DATATYPE VALUE

* SERIAL_NUMBER :	 epos serial_number in hex  
* INDEX	:	address of object in the epos in hex  
* SUBINDEX :	sub-address of object in the epos in hex  
* DATATYPE :	datatype of VALUE; 'u' (unsigned) or nothing (signed) followed by the number of bits to write  
   * _Supported_: u8 = uint8_t; u16 = uint16_t; u32 = uint32_t; u64 = uint64_t; 8 = int8_t; 16 = int16_t; 32 = int32_t; 64 = int64_t
* VALUE	:	the value you want to write  

__example__:  
(1) set the velocity notation index (index 0x608b, subindex 0x00, int8_t) to 0 on an epos with SERIAL_NUMBER=serial  
>rosrun epos_hardware_tools set_param serial 608b 0 8 0
