///////////////////////////////////////////////////////////
/// @file	result.cpp
/// @brief	Result
/// @author	henaiguo
/// Copyright (C) 2021- henaiguo All rights reserved.
///////////////////////////////////////////////////////////

#include <common_library/types/result.h>

#include <cstdio>
#include <cstdarg>
#include <cstring>

namespace common_library {
namespace types {
///////////////////////////////////////////////////////////
/// @brief		Generate a result object (success)
///////////////////////////////////////////////////////////
Result Result::CreateSuccess()
{
	Result result(eSUCCESS);
	return result;
}

///////////////////////////////////////////////////////////
/// @brief		Generate a result object (error occurred)
/// @param[in]	_errorMessage Error message
///////////////////////////////////////////////////////////
Result Result::CreateError(const char *_errorMessage, ...)
{
	static const int TEXT_MAX_LEN = 1024;
	char text[TEXT_MAX_LEN];
	va_list ap;
	va_start(ap, _errorMessage);
	::vsnprintf(text, TEXT_MAX_LEN - 1, _errorMessage, ap);
	va_end(ap);
	std::string errorMessage(text);

	Result result(eERROR, errorMessage);
	return result;
}

///////////////////////////////////////////////////////////
/// @brief	Constructor
/// @note	No error
///////////////////////////////////////////////////////////
Result::Result()
	: m_eResult(eERROR),m_errorMessage(),m_hasErrorMessage(false)
{
	// None
}

///////////////////////////////////////////////////////////
/// @brief	Constructor
/// @param[in]	_eResult eResult type
/// @note
///////////////////////////////////////////////////////////
Result::Result(common_library::types::eResult _eResult)
	: m_eResult(_eResult),m_errorMessage(),m_hasErrorMessage(false)
{
	// None
}

///////////////////////////////////////////////////////////
/// @brief	Constructor
/// @param[in]	_eResult eResult type
/// @param[in]	_errorMessage Error message
/// @note
///////////////////////////////////////////////////////////
Result::Result(common_library::types::eResult _eResult, const std::string& _errorMessage)
	: m_eResult(_eResult)
{
	if (m_eResult == eERROR) {
		m_errorMessage = _errorMessage;
		m_hasErrorMessage = !m_errorMessage.empty() ? true : false;
	}
}

///////////////////////////////////////////////////////////
/// @brief	Copy constructor
/// @param[in]	_result Result
/// @note
///////////////////////////////////////////////////////////
Result::Result(const Result& _result)
{
	operator=(_result);
}

///////////////////////////////////////////////////////////
/// @brief	Destructor
/// @note
///////////////////////////////////////////////////////////
Result::~Result()
{
	// None
}

///////////////////////////////////////////////////////////
/// @brief	Set result
/// @param[in]	_eResult eResult type
/// @return None
/// @note
///////////////////////////////////////////////////////////
void Result::SetResult(common_library::types::eResult _eResult)
{
	m_eResult = _eResult;
	if (m_eResult == eSUCCESS) {
		m_errorMessage.clear();
		m_hasErrorMessage = false;
	}
}

///////////////////////////////////////////////////////////
/// @brief	Get result
/// @return common_library::types::eResult
/// @note
///////////////////////////////////////////////////////////
common_library::types::eResult Result::GetResult() const
{
	return m_eResult;
}

///////////////////////////////////////////////////////////
/// @brief	Set error message
/// @param[in]	_errorMessage Error message
/// @return None
/// @note Empty parameter means clear error message
///////////////////////////////////////////////////////////
void Result::SetErrorMessage(const std::string& _errorMessage)
{
	if (m_eResult == eSUCCESS) return;

	if (!_errorMessage.empty()) {
		m_errorMessage = _errorMessage;
		m_hasErrorMessage = true;
	}
	else {
		m_errorMessage.clear();
		m_hasErrorMessage = false;
	}
}

///////////////////////////////////////////////////////////
/// @brief	Get error message
/// @return	std::string
/// @note
///////////////////////////////////////////////////////////
const std::string &Result::GetErrorMessage() const
{
	return m_errorMessage;
}

///////////////////////////////////////////////////////////
/// @brief	Check if there is an error
/// @note	True on success and false on error
///////////////////////////////////////////////////////////
Result::operator bool() const
{
	return IsSuccess();
}	

///////////////////////////////////////////////////////////
/// @brief	Success or not
/// @return bool
/// @note	True on success
///////////////////////////////////////////////////////////
bool Result::IsSuccess() const
{
	return m_eResult == eSUCCESS;
}

///////////////////////////////////////////////////////////
/// @brief	Error or not
/// @return bool
/// @note	True on error
///////////////////////////////////////////////////////////
bool Result::IsError() const
{
	return m_eResult == eERROR;
}

///////////////////////////////////////////////////////////
/// @brief	Whether error message are set
/// @return bool
/// @note
///////////////////////////////////////////////////////////
bool Result::HasErrorMessage() const
{
	return m_hasErrorMessage;
}

///////////////////////////////////////////////////////////
/// @brief	Assignment operator
/// @param[in]	_result Original result
/// @note
///////////////////////////////////////////////////////////
Result& Result::operator=(const Result& _result)
{
	if (&_result == this) {
		return *this;
	}

	m_eResult = _result.m_eResult;
	m_errorMessage = _result.m_errorMessage;
	m_hasErrorMessage = _result.m_hasErrorMessage;
	return *this;
}

///////////////////////////////////////////////////////////
/// @brief	Assignment operator
/// @param[in]	_result Original result
/// @note
///////////////////////////////////////////////////////////
Result& Result::operator=(const common_library::types::eResult& _eResult)
{
	m_eResult = _eResult;
	if (!m_errorMessage.empty()) m_errorMessage.clear();
	m_hasErrorMessage = false;
}

///////////////////////////////////////////////////////////
/// @brief		Comparison operator (equal to)
/// @param[in]	_result Result type
/// @return		bool
/// @note
///////////////////////////////////////////////////////////
bool Result::operator==(const Result& _result) const
{
	if (&_result == this) return true;

	if ((m_eResult == _result.m_eResult) && 
		(m_errorMessage == _result.m_errorMessage) &&
		(m_hasErrorMessage == _result.m_hasErrorMessage)) {
		return true;
	}

	return false;
}

///////////////////////////////////////////////////////////
/// @brief		Comparison operator (not equal)
/// @param[in]	_result Result type
/// @return		bool
/// @note
///////////////////////////////////////////////////////////
bool Result::operator!=(const Result& _result) const
{
	return !operator==(_result);
}

///////////////////////////////////////////////////////////
/// @brief		Comparison operator (equal to)
/// @param[in]	_eResult eResult type
/// @return		bool
/// @note		Compare only eResult type
///////////////////////////////////////////////////////////
bool Result::operator==(common_library::types::eResult _eResult) const
{
	return m_eResult == _eResult;
}

///////////////////////////////////////////////////////////
/// @brief		Comparison operator (not equal)
/// @param[in]	_eResult eResult type
/// @return		bool
/// @note		Compare only eResult type
///////////////////////////////////////////////////////////
bool Result::operator!=(common_library::types::eResult _eResult) const
{
	return !operator==(_eResult);
}

} // namespace types
} // namespace common_library
