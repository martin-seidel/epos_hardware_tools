#include <ros/ros.h>
#include "epos_hardware/utils.h"
#include "epos_library/Definitions.h"

NodeHandlePtr handle;
unsigned int lastEpos2ErrorCode;

void printEpos2Error()
{
   unsigned short maxStr=255; //max stringsize
   char errorText[maxStr];   //errorstring

   if ( VCS_GetErrorInfo(lastEpos2ErrorCode, errorText, maxStr) )
   {
      ROS_ERROR("%s (errorCode: %#x)", errorText, lastEpos2ErrorCode);
   } else {
      ROS_FATAL("Unable to resolve an errorText for the Epos2-ErrorCode %#x", lastEpos2ErrorCode);
   }
}

template <typename T>
bool setObject(uint16_t index, uint8_t sub_index, T value) {
   uint32_t bytes;

   if ( !VCS_SetObject(handle->device_handle->ptr, handle->node_id, index, sub_index, &value, sizeof(T), &bytes, &lastEpos2ErrorCode) ) {
      printEpos2Error();
      return false;
   } else {
      if ( !VCS_Store(handle->device_handle->ptr, handle->node_id, &lastEpos2ErrorCode) ) {
         printEpos2Error();
         return false;
      } else {
         return true;
      }
   }
}


int main(int argc, char** argv){

   uint64_t serial_number;
   uint16_t index;
   uint8_t sub_index;
   bool is_signed;
   int bits;
   int64_t value;
   bool no_error = true;

   if(argc == 6){
      if(!SerialNumberFromHex(argv[1], &serial_number)) {no_error = false;}

      uint64_t temp=0;
      if(!SerialNumberFromHex(argv[2], &temp)) {no_error = false;}
      index = temp;
      if(!SerialNumberFromHex(argv[3], &temp)) {no_error = false;}
      sub_index = temp;

      std::string type = argv[4];
      if (type.compare(0,1,"u") == 0) {
         is_signed = false;
         type = type.substr(1);
      } else {
         is_signed = true;
      }
      bits = atoi(type.c_str());

      value = atoi(argv[5]);
   } else {
      no_error = false;
   }

   if (no_error) {
      ROS_INFO("on EPOS2 (USB) with serial number %lx: try to set param at index %x-%x with type %sint%i_t to %li", serial_number, index, sub_index, (is_signed) ? "" : "u", bits, value);

      EposFactory epos_factory;

      if( handle = epos_factory.CreateNodeHandle("EPOS2", "MAXON SERIAL V2", "USB", serial_number, &lastEpos2ErrorCode) ) {

         bool success = false;

         if (is_signed) {
            switch (bits) {
               case 8: success = setObject(index, sub_index, (int8_t) value);
               break;
               case 16: success = setObject(index, sub_index, (int16_t) value);
               break;
               case 32: success = setObject(index, sub_index, (int32_t) value);
               break;
               case 64: success = setObject(index, sub_index, (int64_t) value);
               break;
               default: ROS_FATAL("datatype not valid.\nsupported datatypes: int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t");
            }
         } else {
            if (value < 0) {
               success = false;
               ROS_ERROR("an unsigned value coulnd't be smaller than zero.");
            } else {
               switch (bits) {
                  case 8: success = setObject(index, sub_index, (uint8_t) value);
                  break;
                  case 16: success = setObject(index, sub_index, (uint16_t) value);
                  break;
                  case 32: success = setObject(index, sub_index, (uint32_t) value);
                  break;
                  case 64: success = setObject(index, sub_index, (uint64_t) value);
                  break;
                  default: ROS_FATAL("datatype not valid.\nsupported datatypes: int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t");
               }
            }
         }

         if (success) {
            ROS_INFO("set parameter successfully");
         } else {
            ROS_ERROR("unable setting parameter");
         }

      } else {
         ROS_ERROR("Found no device.");
         printEpos2Error();
      }
   } else {
      ROS_FATAL("Wrong parameter(s)");
      std::cout << "Usage: rosrun epos_hardware_tools set_param SERIAL_NUMBER INDEX SUBINDEX DATATYPE VALUE\n\nExample: set the velocity notation index to 0 on an epos with SERIAL_NUMBER=serial\n\trosrun epos_hardware_tools set_param serial 608b 0 8 0\n\n\tSERIAL_NUMBER\tepos serial_number in hex\n\tINDEX\t\taddress of object in the epos in hex\n\tSUBINDEX\tsub-address of object in the epos in hex\n\tDATATYPE\tdatatype of VALUE; \'u\' (unsigned) or nothing (signed) followed by the number of bits to write\n\t\t\tExample: u8 = uint8_t; 16 = int16_t\n\tVALUE\t\tthe value you want to write\n";
   }

}
