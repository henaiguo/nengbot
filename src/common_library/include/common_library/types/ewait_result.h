///////////////////////////////////////////////////////////
/// @file	ewait_result.h
/// @brief	Wait result
/// @author	henaiguo
/// Copyright (C) 2013- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#ifndef EWAIT_RESULT_H
#define EWAIT_RESULT_H

#include <string>

namespace common_library {
namespace types {
///////////////////////////////////////////////////////////
/// @enum	eWaitResult
/// @brief	Wait result
/// @note
///////////////////////////////////////////////////////////
enum eWaitResult
{
	eWAIT_ERROR = 0,
	eWAIT_SUCCESS,
	eWAIT_TIMEOUT
};

///////////////////////////////////////////////////////////
/// @brief	enum Is a string
/// @param[in]	_enum enum value
/// @return	std::string
/// @note
///////////////////////////////////////////////////////////
static std::string ToString(eWaitResult _enum)
{
	switch (_enum) {
	case eWAIT_ERROR:
		return "eWAIT_ERROR";
	case eWAIT_SUCCESS:
		return "eWAIT_SUCCESS";
	case eWAIT_TIMEOUT:
		return "eWAIT_TIMEOUT";
	default:
		return "INVALID_ENUM_VALUE";
	}
}

///////////////////////////////////////////////////////////
/// @brief	Turn a string into an enum
/// @param[in]	_string String
/// @param[out]	_enum enum value
/// @retval	true
/// @retval	false
/// @note
///////////////////////////////////////////////////////////
static bool ToEnum(const std::string& _string, eWaitResult& _enum)
{
	if (_string == "eWAIT_ERROR") {
		_enum = eWAIT_ERROR;
		return true;
	}
	else if (_string == "eWAIT_SUCCESS") {
		_enum = eWAIT_SUCCESS;
		return true;
	}
	else if (_string == "eWAIT_TIMEOUT") {
		_enum = eWAIT_TIMEOUT;
		return true;
	}
	return false;
}
} // namespace types
} // namespace common_library

#endif // EWAIT_RESULT_H

