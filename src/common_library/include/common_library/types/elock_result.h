///////////////////////////////////////////////////////////
/// @file	elock_result.h
/// @brief	Lock result
/// @author	henaiguo
/// Copyright (C) 2013- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#ifndef ELOCK_RESULT_H
#define ELOCK_RESULT_H

#include <string>

namespace common_library {
namespace types {
///////////////////////////////////////////////////////////
/// @enum	eLockResult
/// @brief	Lock result
/// @note
///////////////////////////////////////////////////////////
enum eLockResult
{
	eLOCK_ERROR = 0,
	eLOCK_SUCCESS,
	eLOCK_ALREADY,
	eLOCK_TIMEOUT
};

///////////////////////////////////////////////////////////
/// @brief	enum Is a string
/// @param[in]	_enum enum value
/// @return	std::string
/// @note
///////////////////////////////////////////////////////////
static std::string ToString(eLockResult _enum)
{
	switch (_enum) {
	case eLOCK_ERROR:
		return "eLOCK_ERROR";
	case eLOCK_SUCCESS:
		return "eLOCK_SUCCESS";
	case eLOCK_ALREADY:
		return "eLOCK_ALREADY";
	case eLOCK_TIMEOUT:
		return "eLOCK_TIMEOUT";
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
static bool ToEnum(const std::string& _string, eLockResult& _enum)
{
	if (_string == "eLOCK_ERROR") {
		_enum = eLOCK_ERROR;
		return true;
	}
	else if (_string == "eLOCK_SUCCESS") {
		_enum = eLOCK_SUCCESS;
		return true;
	}
	else if (_string == "eLOCK_ALREADY") {
		_enum = eLOCK_ALREADY;
		return true;
	}
	else if (_string == "eLOCK_TIMEOUT") {
		_enum = eLOCK_TIMEOUT;
		return true;
	}
	return false;
}
} // namespace types
} // namespace common_library

#endif // ELOCK_RESULT_H

