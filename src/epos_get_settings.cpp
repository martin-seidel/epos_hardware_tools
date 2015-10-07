#include <ros/ros.h>
#include "epos_hardware/utils.h"
#include "epos_library/Definitions.h"

EposFactory epos_factory;
std::vector<NodeHandlePtr> handles;
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
T printObject(std::string name, NodeHandlePtr handle, uint16_t index, uint8_t sub_index, T type_dummy, double scale = 1, bool print = true) {
   T value = 0;
   uint32_t bytes;

   if ( !VCS_GetObject(handle->device_handle->ptr, handle->node_id, index, sub_index, &value, sizeof(T), &bytes, &lastEpos2ErrorCode) ) {
      printEpos2Error();
   } else {
      if (print) {//testen scale value
         if (fabs(scale - 1) < 0.001) {
            if (sizeof(T) == 1) {
               if (std::numeric_limits<T>::is_signed) {
                  std::cout << "\t\t" << name << ": " << static_cast<int>(value) << "\n";
               } else {
                  std::cout << "\t\t" << name << ": " << static_cast<unsigned int>(value) << "\n";
               }
            } else {
               std::cout << "\t\t" << name << ": " << value << "\n";
            }
         } else {
            std::cout << "\t\t" << name << ": " << (double) value * scale << "\n";
         }
      }
   }

   return value;
}

template <typename T>
T getObject(NodeHandlePtr handle, uint16_t index, uint8_t sub_index, T type_dummy) {
   return printObject("", handle, index, sub_index, type_dummy, 1 ,false);
}

void printEposHardwareSettings(NodeHandlePtr handle) {
   std::cout << "epos parameters supported by epos_hardware:\n";

   {int8_t operation_mode = getObject(handle, 0x6061, 0x00, int8_t());
   std::cout << "\toperation_mode: " << ((operation_mode == OMD_PROFILE_POSITION_MODE) ? "\'profile_position\'" : ((operation_mode == OMD_PROFILE_VELOCITY_MODE) ? "\'profile_velocity\'" : "mode is not supported by epos_hardware")) << "\n";}

   std::cout << "\tmotor:\n";
   if ( printObject("type", handle, 0x6402, 0x00, uint16_t()) == MT_DC_MOTOR ) {
      std::cout << "\t\tdc_motor:\n";
   } else {
      std::cout << "\t\tec_motor:\n";
      printObject("\tnumber_of_pole_pairs", handle, 0x6410, 0x03, uint8_t());
   }
   printObject("\tnominal_current", handle, 0x6410, 0x01, uint16_t(), 0.001);
   printObject("\tmax_output_current", handle, 0x6410, 0x02, uint16_t(), 0.001);
   printObject("\tthermal_time_constant", handle, 0x6410, 0x05, uint16_t(), 0.1);

   std::cout << "\tsensor:\n";
   switch ( (printObject("type", handle, 0x2210, 0x02, uint16_t()) & 0xF) ) {
      case ST_INC_ENCODER_3CHANNEL:
      case ST_INC_ENCODER_2CHANNEL:
         std::cout << "\t\tincremental_encoder:\n";
         printObject("\tresolution", handle, 0x2210, 0x01, uint32_t());
         break;
      case ST_SSI_ABS_ENCODER_BINARY:
      case ST_SSI_ABS_ENCODER_GREY:{
         std::cout << "\t\tssi_absolute_encoder:\n";
         printObject("\tdata_rate", handle, 0x2211, 0x01, uint16_t());
         uint16_t ssi_data_bits = getObject(handle, 0x2211, 0x02, uint16_t());
         std::cout << "\t\t\tnumber_of_multiturn_bits: " << ((ssi_data_bits & 0xff00) >> 8) << "\n";
         std::cout << "\t\t\tnumber_of_singleturn_bits: " << (ssi_data_bits & 0xff) << "\n";
         break; }
      case ST_HALL_SENSORS:
         std::cout << "\t\thall_sensor:\n";
         break;
      default:
         std::cout << "\t\tunknown sensor:\n";
         break;
   }
   printObject("\tinverted_polarity", handle, 0x2210, 0x04, uint16_t());

   std::cout << "\tsafety:\n";
   printObject("max_following_error", handle, 0x6065, 0x00, uint32_t());
   printObject("max_profile_velocity", handle, 0x607f, 0x00, uint32_t());
   printObject("max_acceleration", handle, 0x60c5, 0x00, uint32_t());

   std::cout << "\tposition_regulator:\n\t\tgain:\n";
   printObject("\tp", handle, 0x60fb, 0x01, uint16_t());
   printObject("\ti", handle, 0x60fb, 0x02, uint16_t());
   printObject("\td", handle, 0x60fb, 0x03, uint16_t());
   std::cout << "\t\tfeed_forward:\n";
   printObject("\tvelocity", handle, 0x60fb, 0x04, uint16_t());
   printObject("\tacceleration", handle, 0x60fb, 0x05, uint16_t());

   std::cout << "\tvelocity_regulator:\n\t\tgain:\n";
   printObject("\tp", handle, 0x60f9, 0x01, uint16_t());
   printObject("\ti", handle, 0x60f9, 0x02, uint16_t());
   std::cout << "\t\tfeed_forward:\n";
   printObject("\tvelocity", handle, 0x60f9, 0x04, uint16_t());
   printObject("\tacceleration", handle, 0x60f9, 0x05, uint16_t());

   std::cout << "\tcurrent_regulator:\n\t\tgain:\n";
   printObject("\tp", handle, 0x60f6, 0x01, uint16_t());
   printObject("\ti", handle, 0x60f6, 0x02, uint16_t());

   std::cout << "\tposition_profile:\n";
   printObject("velocity", handle, 0x6081, 0x00, uint32_t());
   printObject("acceleration", handle, 0x6083, 0x00, uint32_t());
   printObject("deceleration", handle, 0x6084, 0x00, uint32_t());
   std::cout << "\t\twindow:\n";
   printObject("\twindow", handle, 0x6067, 0x00, uint16_t());
   printObject("\ttime", handle, 0x6068, 0x00, uint16_t());

   std::cout << "\tvelocity_profile:\n";
   printObject("acceleration", handle, 0x6083, 0x00, uint32_t());
   printObject("deceleration", handle, 0x6084, 0x00, uint32_t());
   std::cout << "\t\twindow:\n";
   printObject("\twindow", handle, 0x606d, 0x00, uint32_t());
   printObject("\ttime", handle, 0x606e, 0x00, uint16_t());
}

void printAdditionalSettings(NodeHandlePtr handle) {
   std::cout << "\nepos parameters currently (ROS Wiki 2015-03-26) not supported by epos_hardware:\n";
   std::cout << "\tmotor:\n";
   printObject("\tmax_motor_speed", handle, 0x6410, 0x04, uint32_t());

   std::cout << "\tgear:\n";
   printObject("ratio numerator", handle, 0x2230, 0x01, uint32_t());
   printObject("ratio denominator", handle, 0x2230, 0x02, uint16_t());
   printObject("maximal speed", handle, 0x2230, 0x03, uint32_t());

   std::cout << "\tdimension and notation indices:\n";
   printObject("velocity notation index", handle, 0x608b, 0x00, int8_t());

   {std::cout << "\tmiscellaneous configuration:\n";
   uint16_t miscellaneous = printObject("miscellaneous configuration", handle, 0x2008, 0x00, uint16_t());
   std::cout << "\t\tpolarity bit: " << (((miscellaneous >> 8) & 1) ? "1 = inverse (CW)\n" : "0 = normal (CCW)\n");}

   std::cout << "\tposition_profile and velocity_profile:\n";
   printObject("quickstop_deceleration", handle, 0x6085, 0x00, uint32_t());
   int16_t motion_profile_type = getObject(handle, 0x6086, 0x00, int16_t());
   std::cout << "\t\tmotion_profile_type: " << ((motion_profile_type == 0) ? "linear ramp" : ((motion_profile_type == 1) ? "sin² ramp" : "unvalid value")) << "\n";
}

bool printDevices() {
   unsigned short maxStr=64;

   for (NodeHandlePtr handle : handles) {
      bool no_error = true;
      char device[maxStr], interface[maxStr];
      unsigned short hardware_version, software_version, application_number, application_version;

      if ( !VCS_GetDeviceName(handle->device_handle->ptr, device, maxStr, &lastEpos2ErrorCode) ) {
         printEpos2Error();
         no_error = false;
      }
      if ( !VCS_GetInterfaceName(handle->device_handle->ptr, interface, maxStr, &lastEpos2ErrorCode) ) {
         printEpos2Error();
         no_error = false;
      }
      if ( !VCS_GetVersion(handle->device_handle->ptr, handle->node_id, &hardware_version, &software_version, &application_number, &application_version, &lastEpos2ErrorCode) ) {
         printEpos2Error();
         no_error = false;
      }

      if (no_error) {
         std::cout << "\n";
         ROS_INFO("%s %s with serial number %lx and node_id %u\n(HW-Version: %u, SW-Version: %u, App-Number: %u, App-Version: %u)", device, interface, getObject(handle, 0x2004, 0x00, uint64_t()), getObject(handle, 0x2000, 0x00, uint8_t()), hardware_version, software_version, application_number, application_version);

         printEposHardwareSettings(handle);
         printAdditionalSettings(handle);
      }
   }
}

bool getDevices() {
   std::vector<std::string> device_names;
   if ( GetDeviceNameList(&device_names, &lastEpos2ErrorCode) ) {
      for (std::string device_name : device_names) {
         std::vector<std::string> protocol_stack_names;
         if(GetProtocolStackNameList(device_name, &protocol_stack_names, &lastEpos2ErrorCode)) {
            for (std::string protocol_stack_name : protocol_stack_names) {
               std::vector<std::string> interface_names;
               if(GetInterfaceNameList(device_name, protocol_stack_name, &interface_names, &lastEpos2ErrorCode)) {
                  for (std::string interface_name : interface_names) {
                     std::vector<std::string> port_names;
                     if(GetPortNameList(device_name, protocol_stack_name, interface_name, &port_names, &lastEpos2ErrorCode)) {
                        for (std::string port_name : port_names) {
                           std::vector<EnumeratedNode> devices;
                           if(epos_factory.EnumerateNodes(device_name, protocol_stack_name, interface_name, port_name, &devices, &lastEpos2ErrorCode)) {
                              ROS_INFO("%li devices at %s (%s %s %s)", devices.size(), port_name.c_str(), device_name.c_str(), protocol_stack_name.c_str(), interface_name.c_str());
                              for (const EnumeratedNode& node : devices) {
                                 NodeHandlePtr temp_handle = epos_factory.CreateNodeHandle(device_name, protocol_stack_name, interface_name, node.serial_number, &lastEpos2ErrorCode);
                                 handles.push_back(temp_handle);
                              }
                              return true;
                           }
                        }
                     } else {
                        printEpos2Error();
                     }
                  }
               } else {
                  printEpos2Error();
               }
            }
         } else {
            printEpos2Error();
         }
      }
   } else {
      printEpos2Error();
   }

   return false;
}

bool printSingleObject(uint16_t index, uint8_t sub_index, bool is_signed, int bits) {
   bool no_error = true;
   for (NodeHandlePtr handle : handles) {
      bool success = false;

      std::ostringstream text;
      text << "parameter on node " << (int) getObject(handle, 0x2000, 0x00, uint8_t()) << " (serial: " << std::hex << getObject(handle, 0x2004, 0x00, uint64_t()) << ")";

      if (is_signed) {
         switch (bits) {
            case 8: success = printObject(text.str().c_str(), handle, index, sub_index, int8_t());
            break;
            case 16: success = printObject(text.str().c_str(), handle, index, sub_index, int16_t());
            break;
            case 32: success = printObject(text.str().c_str(), handle, index, sub_index, int32_t());
            break;
            case 64: success = printObject(text.str().c_str(), handle, index, sub_index, int64_t());
            break;
            default: ROS_FATAL("datatype not valid.\nsupported datatypes: int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t");
         }
      } else {
         switch (bits) {
            case 8: success = printObject(text.str().c_str(), handle, index, sub_index, uint8_t());
            break;
            case 16: success = printObject(text.str().c_str(), handle, index, sub_index, uint16_t());
            break;
            case 32: success = printObject(text.str().c_str(), handle, index, sub_index, uint32_t());
            break;
            case 64: success = printObject(text.str().c_str(), handle, index, sub_index, uint64_t());
            break;
            default: ROS_FATAL("datatype not valid.\nsupported datatypes: int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t");
         }
      }
      if (!success) {no_error = false;}
   }
   return no_error;
}


int main(int argc, char** argv){
   if(argc > 1){
      uint64_t serial_number;

      std::string param1 = argv[1];

      if( param1.compare("?") == 0 ) {
         if ( !getDevices() ) { return 1;}
         //std::cout << "\n";
      } else {
         if (SerialNumberFromHex(argv[1], &serial_number)) {
            NodeHandlePtr temp_handle;
            if(temp_handle = epos_factory.CreateNodeHandle("EPOS2", "MAXON SERIAL V2", "USB", serial_number, &lastEpos2ErrorCode)) {
               handles.push_back(temp_handle);
            } else {
               ROS_ERROR("Found no device.");
               printEpos2Error();
               return 1;
            }
         } else {
            ROS_ERROR("expected a serial number or a '?' as first parameter");
            return 1;
         }
      }

      if (argc == 5) {
         bool no_error = true;
         uint64_t temp=0;
         if(!SerialNumberFromHex(argv[2], &temp)) {no_error = false;}
         uint16_t index = temp;
         if(!SerialNumberFromHex(argv[3], &temp)) {no_error = false;}
         uint8_t sub_index = temp;

         bool is_signed;
         int bits;
         std::string type = argv[4];
         if (type.compare(0,1,"u") == 0) {
            is_signed = false;
            type = type.substr(1);
         } else {
            is_signed = true;
         }
         bits = atoi(type.c_str());

         if (no_error) {
            return ( printSingleObject(index, sub_index, is_signed, bits) ) ? 0 : 1;
         } else {
            return 1;
         }
      }

   } else {
      if ( !getDevices() ) { return 1;}
   }

   return ( printDevices() ) ? 0 : 1;
}
