// Generated by gencpp from file neo_msgs/RelayBoardV2.msg
// DO NOT EDIT!


#ifndef NEO_MSGS_MESSAGE_RELAYBOARDV2_H
#define NEO_MSGS_MESSAGE_RELAYBOARDV2_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace neo_msgs
{
template <class ContainerAllocator>
struct RelayBoardV2_
{
  typedef RelayBoardV2_<ContainerAllocator> Type;

  RelayBoardV2_()
    : relayboardv2_state()
    , communication_state(0)
    , charging_state(0)
    , temperature(0)
    , battery_voltage(0.0)
    , charging_current(0.0)
    , relay_states()
    , keypad()
    , shutdown(false)  {
      relayboardv2_state.assign(false);

      relay_states.assign(false);

      keypad.assign(false);
  }
  RelayBoardV2_(const ContainerAllocator& _alloc)
    : relayboardv2_state()
    , communication_state(0)
    , charging_state(0)
    , temperature(0)
    , battery_voltage(0.0)
    , charging_current(0.0)
    , relay_states()
    , keypad()
    , shutdown(false)  {
  (void)_alloc;
      relayboardv2_state.assign(false);

      relay_states.assign(false);

      keypad.assign(false);
  }



   typedef boost::array<uint8_t, 16>  _relayboardv2_state_type;
  _relayboardv2_state_type relayboardv2_state;

   typedef int16_t _communication_state_type;
  _communication_state_type communication_state;

   typedef int16_t _charging_state_type;
  _charging_state_type charging_state;

   typedef int16_t _temperature_type;
  _temperature_type temperature;

   typedef float _battery_voltage_type;
  _battery_voltage_type battery_voltage;

   typedef float _charging_current_type;
  _charging_current_type charging_current;

   typedef boost::array<uint8_t, 4>  _relay_states_type;
  _relay_states_type relay_states;

   typedef boost::array<uint8_t, 8>  _keypad_type;
  _keypad_type keypad;

   typedef uint8_t _shutdown_type;
  _shutdown_type shutdown;



  enum {
    CS_NOT_ESTABLISHED = 0,
    CS_CONFIGURATION_FAILED = 1,
    CS_OK = 2,
    CS_LOST = 3,
    CS_ERROR = 4,
    CHS_NOTCHARGING = 0,
    CHS_START_CHARGING = 1,
    CHS_CHARGING = 2,
    CHS_NO_CHARGER = 3,
    CHS_BRAKES_OPEN = 4,
    CHS_EMSTOP = 5,
    CHS_ABORT = 6,
    CHS_FINISHED = 7,
  };


  typedef boost::shared_ptr< ::neo_msgs::RelayBoardV2_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::neo_msgs::RelayBoardV2_<ContainerAllocator> const> ConstPtr;

}; // struct RelayBoardV2_

typedef ::neo_msgs::RelayBoardV2_<std::allocator<void> > RelayBoardV2;

typedef boost::shared_ptr< ::neo_msgs::RelayBoardV2 > RelayBoardV2Ptr;
typedef boost::shared_ptr< ::neo_msgs::RelayBoardV2 const> RelayBoardV2ConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::neo_msgs::RelayBoardV2_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::neo_msgs::RelayBoardV2_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace neo_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'neo_msgs': ['/home/yusseff/Neobotix/src/neo_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::neo_msgs::RelayBoardV2_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::neo_msgs::RelayBoardV2_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::neo_msgs::RelayBoardV2_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::neo_msgs::RelayBoardV2_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::neo_msgs::RelayBoardV2_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::neo_msgs::RelayBoardV2_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::neo_msgs::RelayBoardV2_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ccce5e129c9e20212af590d84bbc6931";
  }

  static const char* value(const ::neo_msgs::RelayBoardV2_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xccce5e129c9e2021ULL;
  static const uint64_t static_value2 = 0x2af590d84bbc6931ULL;
};

template<class ContainerAllocator>
struct DataType< ::neo_msgs::RelayBoardV2_<ContainerAllocator> >
{
  static const char* value()
  {
    return "neo_msgs/RelayBoardV2";
  }

  static const char* value(const ::neo_msgs::RelayBoardV2_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::neo_msgs::RelayBoardV2_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This message holds specific data available from Neobotix RelayBoardV2\n"
" \n"
"\n"
"# Possible Communication States\n"
"int16 CS_NOT_ESTABLISHED = 0 		# configuration process has not been started\n"
"int16 CS_CONFIGURATION_FAILED = 1	# configuration process failed\n"
"int16 CS_OK = 2 			# communication is OK\n"
"int16 CS_LOST = 3 			# comminication was lost\n"
"int16 CS_ERROR = 4			# comminication has errors\n"
"\n"
"# Possible Charging States\n"
"int16 CHS_NOTCHARGING = 0 		# no charging in progress; can be started with service /RelayBoardV2/StartCharging\n"
"int16 CHS_START_CHARGING = 1		# RelayBoardV2 is starting charging process\n"
"int16 CHS_CHARGING = 2 			# charging in progress; can be stopped with service /RelayBoardV2/StopCharging\n"
"int16 CHS_NO_CHARGER = 3 		# charging has been started but no charger was connected; this aborts charging\n"
"int16 CHS_BRAKES_OPEN = 4 		# brakes have been opened while charging; this aborts charging\n"
"int16 CHS_EMSTOP = 5 			# EMSTOP has been issued while charging; this aborts charging\n"
"int16 CHS_ABORT = 6 	   		# charging abort by host system\n"
"int16 CHS_FINISHED = 7 			# charging finished\n"
"\n"
"#\n"
"bool[16] relayboardv2_state		# Shows state for all possible errors\n"
"					# relayboardv2_state[0] if true no errors; if false check other fields\n"
"					# relayboardv2_state[1] if true charging relay error; check robot hardware\n"
"					# relayboardv2_state[2] if true release brakes button failed; check robot hardware\n"
"					# relayboardv2_state[3] if true motor error; check robot hardware\n"
"					# relayboardv2_state[4] if true safety relay error; check robot hardware\n"
"					# relayboardv2_state[5] if true Leistungsrelais error; check robot hardware\n"
"					# relayboardv2_state[6] if true EMStop system error; check robot hardware\n"
"					# relayboardv2_state[7] reserved\n"
"					# relayboardv2_state[8] reserved\n"
"					# relayboardv2_state[9] reserved\n"
"					# relayboardv2_state[10] reserved\n"
"					# relayboardv2_state[11] reserved\n"
"					# relayboardv2_state[12] reserved\n"
"					# relayboardv2_state[13] reserved\n"
"					# relayboardv2_state[14] reserved\n"
"					# relayboardv2_state[15] reserved\n"
"\n"
"\n"
"int16 communication_state		# values see above\n"
"\n"
"int16 charging_state			# values see above\n"
"\n"
"int16 temperature			# temperature on relayboardv2 [°C]\n"
"\n"
"float32 battery_voltage			# battery voltage [V]\n"
"\n"
"float32 charging_current		# charging current [A] if not charging = 0\n"
"\n"
"bool[4] relay_states			# state for each relay on RelayBoardV2; True = Active; False = Inactive\n"
"					# relay_states[0] is our charging relay\n"
"					# other are free to use\n"
"\n"
"bool[8] keypad				# state for each key on keypad; True = Pressed\n"
"					# keypad[0] info button\n"
"					# keypad[1] home button\n"
"					# keypad[2] start button\n"
"					# keypad[3] stop button\n"
"					# keypad[4] release brakes button\n"
"					# keypad[5] on demand digital input\n"
"					# keypad[6] on demand digital input\n"
"					# keypad[7] on demand digital input\n"
"\n"
"bool shutdown				# if true host system is asked to shut down\n"
"\n"
"\n"
;
  }

  static const char* value(const ::neo_msgs::RelayBoardV2_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::neo_msgs::RelayBoardV2_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.relayboardv2_state);
      stream.next(m.communication_state);
      stream.next(m.charging_state);
      stream.next(m.temperature);
      stream.next(m.battery_voltage);
      stream.next(m.charging_current);
      stream.next(m.relay_states);
      stream.next(m.keypad);
      stream.next(m.shutdown);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RelayBoardV2_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::neo_msgs::RelayBoardV2_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::neo_msgs::RelayBoardV2_<ContainerAllocator>& v)
  {
    s << indent << "relayboardv2_state[]" << std::endl;
    for (size_t i = 0; i < v.relayboardv2_state.size(); ++i)
    {
      s << indent << "  relayboardv2_state[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.relayboardv2_state[i]);
    }
    s << indent << "communication_state: ";
    Printer<int16_t>::stream(s, indent + "  ", v.communication_state);
    s << indent << "charging_state: ";
    Printer<int16_t>::stream(s, indent + "  ", v.charging_state);
    s << indent << "temperature: ";
    Printer<int16_t>::stream(s, indent + "  ", v.temperature);
    s << indent << "battery_voltage: ";
    Printer<float>::stream(s, indent + "  ", v.battery_voltage);
    s << indent << "charging_current: ";
    Printer<float>::stream(s, indent + "  ", v.charging_current);
    s << indent << "relay_states[]" << std::endl;
    for (size_t i = 0; i < v.relay_states.size(); ++i)
    {
      s << indent << "  relay_states[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.relay_states[i]);
    }
    s << indent << "keypad[]" << std::endl;
    for (size_t i = 0; i < v.keypad.size(); ++i)
    {
      s << indent << "  keypad[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.keypad[i]);
    }
    s << indent << "shutdown: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.shutdown);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NEO_MSGS_MESSAGE_RELAYBOARDV2_H
