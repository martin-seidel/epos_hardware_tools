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
T printObject(std::string name, uint16_t index, uint8_t sub_index, T type_dummy, double scale = 1) {
   T value = 0;
   uint32_t bytes;

   if ( !VCS_GetObject(handle->device_handle->ptr, handle->node_id, index, sub_index, &value, sizeof(T), &bytes, &lastEpos2ErrorCode) ) {
      printEpos2Error();
   } else {
      std::cout << "\t\t" << name << ": " << (double) value * scale << "\n";
   }

   return value;
}

void printEposHardwareSettings() {
   std::cout << "epos parameters supported by epos_hardware:\n";
   
   {int8_t temp = printObject("operation_mode_value", 0x6061, 0x00, int8_t());
   std::cout << "\toperation_mode: " << ((temp == OMD_PROFILE_POSITION_MODE) ? "\'profile_position\'" : ((temp == OMD_PROFILE_VELOCITY_MODE) ? "\'profile_velocity\'" : "mode is not supported by epos_hardware")) << "\n";}

   std::cout << "\tmotor:\n";
   if ( printObject("type", 0x6402, 0x00, uint16_t()) == MT_DC_MOTOR ) {
      std::cout << "\t\tdc_motor:\n";
   } else {
      std::cout << "\t\tec_motor:\n";
      printObject("\tnumber_of_pole_pairs", 0x6410, 0x03, uint8_t());
   }
   printObject("\tnominal_current", 0x6410, 0x01, uint16_t(), 0.001);
   printObject("\tmax_output_current", 0x6410, 0x02, uint16_t(), 0.001);
   printObject("\tthermal_time_constant", 0x6410, 0x05, uint16_t(), 0.1);

   std::cout << "\tsensor:\n";
   switch ( (printObject("type", 0x2210, 0x02, uint16_t()) & 0xF) ) {
      case ST_INC_ENCODER_3CHANNEL:
      case ST_INC_ENCODER_2CHANNEL:
         std::cout << "\t\tincremental_encoder:\n";
         printObject("\tresolution", 0x2210, 0x01, uint32_t());
         break;
      case ST_SSI_ABS_ENCODER_BINARY:
      case ST_SSI_ABS_ENCODER_GREY:{
         std::cout << "\t\tssi_absolute_encoder:\n";
         printObject("\tdata_rate", 0x2211, 0x01, uint16_t());
         uint16_t temp = printObject("\t(ssi_data_bits)", 0x2211, 0x02, uint16_t());
         std::cout << "\t\t\tnumber_of_multiturn_bits: " << ((temp & 0xff00) >> 8) << "\n";
         std::cout << "\t\t\tnumber_of_singleturn_bits: " << (temp & 0xff) << "\n";
         break; }
      case ST_HALL_SENSORS:
         std::cout << "\t\thall_sensor:\n";
         break;
      default:
         std::cout << "\t\tunknown sensor:\n";
         break;
   }
   printObject("\tinverted_polarity", 0x2210, 0x04, uint16_t());

   std::cout << "\tsafety:\n";
   printObject("max_following_error", 0x6065, 0x00, uint32_t());
   printObject("max_profile_velocity", 0x607f, 0x00, uint32_t());
   printObject("max_acceleration", 0x60c5, 0x00, uint32_t());

   std::cout << "\tposition_regulator:\n\t\tgain:\n";
   printObject("\tp", 0x60fb, 0x01, uint16_t());
   printObject("\ti", 0x60fb, 0x02, uint16_t());
   printObject("\td", 0x60fb, 0x03, uint16_t());
   std::cout << "\t\tfeed_forward:\n";
   printObject("\tvelocity", 0x60fb, 0x04, uint16_t());
   printObject("\tacceleration", 0x60fb, 0x05, uint16_t());

   std::cout << "\tvelocity_regulator:\n\t\tgain:\n";
   printObject("\tp", 0x60f9, 0x01, uint16_t());
   printObject("\ti", 0x60f9, 0x02, uint16_t());
   std::cout << "\t\tfeed_forward:\n";
   printObject("\tvelocity", 0x60f9, 0x04, uint16_t());
   printObject("\tacceleration", 0x60f9, 0x05, uint16_t());

   std::cout << "\tcurrent_regulator:\n\t\tgain:\n";
   printObject("\tp", 0x60f6, 0x01, uint16_t());
   printObject("\ti", 0x60f6, 0x02, uint16_t());

   std::cout << "\tposition_profile:\n";
   printObject("velocity", 0x6081, 0x00, uint32_t());
   printObject("acceleration", 0x6083, 0x00, uint32_t());
   printObject("deceleration", 0x6084, 0x00, uint32_t());
   std::cout << "\t\twindow:\n";
   printObject("\twindow", 0x6067, 0x00, uint16_t());
   printObject("\ttime", 0x6068, 0x00, uint16_t());

   std::cout << "\tvelocity_profile:\n";
   printObject("acceleration", 0x6083, 0x00, uint32_t());
   printObject("deceleration", 0x6084, 0x00, uint32_t());
   std::cout << "\t\twindow:\n";
   printObject("\twindow", 0x606d, 0x00, uint32_t());
   printObject("\ttime", 0x606e, 0x00, uint16_t());
}

void printAdditionalSettings() {
   std::cout << "\nepos parameters currently (ROS Wiki 2015-03-26) not supported by epos_hardware:\n";
   std::cout << "\tmotor:\n";
   printObject("\tmax_motor_speed", 0x6410, 0x04, uint32_t());

   std::cout << "\tgear:\n";
   printObject("ratio numerator", 0x2230, 0x01, uint32_t());
   printObject("ratio denominator", 0x2230, 0x02, uint16_t());
   printObject("maximal speed", 0x2230, 0x03, uint32_t());

   std::cout << "\tdimension and notation indices:\n";
   printObject("velocity notation index", 0x608b, 0x00, int8_t());

   {std::cout << "\tmiscellaneous configuration:\n";
   uint16_t temp = printObject("miscellaneous configuration", 0x2008, 0x00, uint16_t());
   std::cout << "\t\tpolarity bit: " << (((temp >> 8) & 1) ? "1 = inverse (CW)\n" : "0 = normal (CCW)\n");}

   std::cout << "\tposition_profile and velocity_profile:\n";
   printObject("quickstop_deceleration", 0x6085, 0x00, uint32_t());
   {int16_t temp = printObject("motion_profile_type", 0x6086, 0x00, int16_t());
   std::cout << "\t\t\t(" << ((temp == 0) ? "linear ramp" : ((temp == 1) ? "sin² ramp" : "unvalid value")) << ")\n";}
}


int main(int argc, char** argv){

   uint64_t serial_number;
   unsigned int lastEpos2ErrorCode;

   if(argc == 2){
      if(!SerialNumberFromHex(argv[1], &serial_number)) {
         ROS_ERROR("Expected a serial number");
         return 1;
      }
   } else {
      ROS_ERROR("Expected exactly one argument that is a serial number");
      return 1;
   }

   ROS_INFO("USB EPOS2 with serial number %lx:", serial_number);

   EposFactory epos_factory;

   if(handle = epos_factory.CreateNodeHandle("EPOS2", "MAXON SERIAL V2", "USB", serial_number, &lastEpos2ErrorCode)) {

      printEposHardwareSettings();
      printAdditionalSettings();

   } else {
      ROS_ERROR("Found no device.");
      printEpos2Error();
   }

}
