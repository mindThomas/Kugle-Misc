#ifndef LSPC_MESSAGE_TYPES_HPP
#define LSPC_MESSAGE_TYPES_HPP

namespace lspc
{

	namespace MessageTypesFromPC
	{
		typedef enum MessageTypesFromPC: uint8_t
		{
			Test = 0x01,
			Control = 0x10,
			CalibrateIMU = 0xE0,
			VelocityReference_Inertial = 0x33,
			VelocityReference_Heading = 0x34
		} MessageTypesIn_t;
	}

	namespace MessageTypesToPC
	{
		typedef enum MessageTypesToPC: uint8_t
		{
			Test = 0x01,
			SysInfo = 0x10,
			Debug = 0xFF
		} MessageTypesOut_t;
	}

} // namespace lspc

#endif // LSPC_MESSAGE_TYPES_HPP

