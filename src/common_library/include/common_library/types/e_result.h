///////////////////////////////////////////////////////////
/// @file	e_result.h
/// @brief	Result type
/// @author	henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#ifndef E_RESULT_H
#define E_RESULT_H

#include <string>

namespace common_library {
namespace types {
///////////////////////////////////////////////////////////
/// @enum   eResult
/// @brief	Result type
/// @note
///////////////////////////////////////////////////////////
enum eResult
{
	/// success
	eSUCCESS = 0,
	/// error
	eERROR = 1
};

///////////////////////////////////////////////////////////
/// @brief		enum Is a string
/// @param[in]	_enum enum value
/// @return		char*
/// @note
///////////////////////////////////////////////////////////
static std::string ToString(eResult _enum)
{
	switch (_enum) {
	case eSUCCESS:
		return "eSUCCESS";
	case eERROR:
		return "eERROR";
	default:
		return "INVALID_ENUM_VALUE";
	}
}

///////////////////////////////////////////////////////////
/// @brief		Turn a string into an enum
/// @param[in]	_string String
/// @param[out]	_enum enum value
/// @retval		true
/// @retval		false
/// @note
///////////////////////////////////////////////////////////
static bool ToEnum(const std::string& _string, eResult& _enum)
{
	if (_string == "eSUCCESS") {
		_enum = eSUCCESS;
		return true;
	}
	else if (_string == "eERROR") {
		_enum = eERROR;
		return true;
	}
	return false;
}

} // namespace types
} // namespace common_library

#endif // E_RESULT_H

