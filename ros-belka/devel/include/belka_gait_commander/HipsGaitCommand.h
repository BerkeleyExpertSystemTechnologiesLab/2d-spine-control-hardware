// Generated by gencpp from file belka_gait_commander/HipsGaitCommand.msg
// DO NOT EDIT!


#ifndef BELKA_GAIT_COMMANDER_MESSAGE_HIPSGAITCOMMAND_H
#define BELKA_GAIT_COMMANDER_MESSAGE_HIPSGAITCOMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace belka_gait_commander
{
template <class ContainerAllocator>
struct HipsGaitCommand_
{
  typedef HipsGaitCommand_<ContainerAllocator> Type;

  HipsGaitCommand_()
    : hips_command()  {
    }
  HipsGaitCommand_(const ContainerAllocator& _alloc)
    : hips_command(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _hips_command_type;
  _hips_command_type hips_command;





  typedef boost::shared_ptr< ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator> const> ConstPtr;

}; // struct HipsGaitCommand_

typedef ::belka_gait_commander::HipsGaitCommand_<std::allocator<void> > HipsGaitCommand;

typedef boost::shared_ptr< ::belka_gait_commander::HipsGaitCommand > HipsGaitCommandPtr;
typedef boost::shared_ptr< ::belka_gait_commander::HipsGaitCommand const> HipsGaitCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace belka_gait_commander

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'belka_gait_commander': ['/home/drew/repositories/2d-spine-control-hardware/ros-belka/src/belka_gait_commander/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3e35c60667a75681d37226fc0b36d0b6";
  }

  static const char* value(const ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3e35c60667a75681ULL;
  static const uint64_t static_value2 = 0xd37226fc0b36d0b6ULL;
};

template<class ContainerAllocator>
struct DataType< ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "belka_gait_commander/HipsGaitCommand";
  }

  static const char* value(const ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string hips_command\n"
;
  }

  static const char* value(const ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.hips_command);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HipsGaitCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::belka_gait_commander::HipsGaitCommand_<ContainerAllocator>& v)
  {
    s << indent << "hips_command: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.hips_command);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BELKA_GAIT_COMMANDER_MESSAGE_HIPSGAITCOMMAND_H