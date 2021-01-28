///////////////////////////////////////////////////////////
/// @file	error.cpp
/// @brief	error
/// @author	henaiguo
/// Copyright (C) 2021- henaiguo All rights reserved.
///////////////////////////////////////////////////////////

#include "Error.h"

#include <cstdio>
#include <cstdarg>
#include <cstring>

namespace common_library {
///////////////////////////////////////////////////////////
/// @brief		Generate error object (error occurred)
/// @param[in]	_format, ... Error message
///////////////////////////////////////////////////////////
Error Error::CreateError(const char* _format, ...)
{
	static const int TEXT_MAX_LEN = 1024;
	char text[TEXT_MAX_LEN];
	::va_list ap;
	::va_start(ap, _format);
	::vsnprintf(text, TEXT_MAX_LEN - 1, _format, ap);
	::va_end(ap);

	std::string message(text);
	Error error(message);
	return error;
}

///////////////////////////////////////////////////////////
/// @brief		Generate an error object (no error)
/// @param[in]	None
///////////////////////////////////////////////////////////
Error Error::CreateNoError()
{
	Error error;
	return error;
}

///////////////////////////////////////////////////////////
/// @brief		constructor
/// @param[in]	None
/// @note		No error
///////////////////////////////////////////////////////////
Error::Error()
	: m_errorMessage("Success"), m_isError(false)
{
}

///////////////////////////////////////////////////////////
/// @brief		constructor
/// @param[in]	_message Error message
/// @note		Error occurred
///////////////////////////////////////////////////////////
Error::Error(const std::string& _message)
	: m_errorMessage(_message), m_isError(true)
{
}

///////////////////////////////////////////////////////////
/// @brief		Destructor
/// @return		None
///////////////////////////////////////////////////////////
Error::~Error()
{
}

///////////////////////////////////////////////////////////
/// @brief		Check if there is an error
/// @param[in]	None
/// @note		True on error
///////////////////////////////////////////////////////////
Error::operator bool() const
{
	return m_isError;
}

///////////////////////////////////////////////////////////
/// @brief		Get message
/// @param[in]	None
/// @return		message
///////////////////////////////////////////////////////////
const std::string &Error::Message() const
{
	return m_errorMessage;
}

} // namespace common_library
