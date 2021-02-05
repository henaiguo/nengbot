///////////////////////////////////////////////////////////
/// @file   mutex.cpp
/// @brief  Mutex
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#include <common_library/thread/mutex.h>

#include <cstdio>
#include <cerrno>

namespace common_library {
namespace thread {
///////////////////////////////////////////////////////////
/// @brief  Default constructor
/// @return None
/// @note
///////////////////////////////////////////////////////////
Mutex::Mutex()
{
    ::pthread_mutex_init(&m_mutex, NULL);
	::pthread_cond_init(&m_condition, NULL);
}

///////////////////////////////////////////////////////////
/// @brief  Destructor
/// @return None
/// @note
///////////////////////////////////////////////////////////
Mutex::~Mutex()
{
    ::pthread_mutex_destroy(&m_mutex);
	::pthread_cond_destroy(&m_condition);
}

///////////////////////////////////////////////////////////	
/// @brief	Acquire lock
/// @return	common_library::types::eLockResult
/// @note
///////////////////////////////////////////////////////////
common_library::types::eLockResult Mutex::Lock()
{
	switch (::pthread_mutex_lock(&m_mutex)) {
	case 0:
		return common_library::types::eLOCK_SUCCESS;
	case EDEADLK:
		return common_library::types::eLOCK_ALREADY;
	default:
		return common_library::types::eLOCK_ERROR;
	}
}

///////////////////////////////////////////////////////////	
/// @brief	Acquire lock (with timeout)
/// @param[in]	_usec Timeout (microsecond)
/// @return	common_library::types::eLockResult
/// @note
///////////////////////////////////////////////////////////
common_library::types::eLockResult Mutex::Lock(unsigned long _usec)
{
    // TODO:
	switch (::pthread_mutex_timedlock(&m_mutex, 0)) {
	case 0:
		return common_library::types::eLOCK_SUCCESS;
	case EDEADLK:
		return common_library::types::eLOCK_ALREADY;
	case ETIMEDOUT:
		return common_library::types::eLOCK_TIMEOUT;
	default:
		return common_library::types::eLOCK_ERROR;
	}
}

///////////////////////////////////////////////////////////
/// @brief		Unlock
/// @return		None
/// @note
///////////////////////////////////////////////////////////
void Mutex::Unlock()
{
    ::pthread_mutex_unlock(&m_mutex);
}

///////////////////////////////////////////////////////////
/// @brief	Wait for condition (no timeout)
/// @return	common_library::types::eWaitResult
/// @note
///////////////////////////////////////////////////////////
common_library::types::eWaitResult Mutex::Wait()
{
	return ::pthread_cond_wait(&m_condition, &m_mutex) !=0 ? 
				common_library::types::eWAIT_ERROR :
				common_library::types::eWAIT_SUCCESS;
}

///////////////////////////////////////////////////////////
/// @brief	    Wait for condition (with timeout)
/// @param[in]	_usec Timeout (microsecond)
/// @return		common_library::types::eWaitResult
/// @note
///////////////////////////////////////////////////////////
common_library::types::eWaitResult Mutex::Wait(unsigned long _usec)
{
	// TODO:
	switch (::pthread_cond_timedwait(&m_condition, &m_mutex, 0))
	{
	case 0:
		return common_library::types::eWAIT_SUCCESS;
	case ETIMEDOUT:
		return common_library::types::eWAIT_TIMEOUT;
	default:
		return common_library::types::eWAIT_ERROR;
	}
}

///////////////////////////////////////////////////////////
/// @brief	Notify that the condition is met (wait release)
/// @return	None
/// @note
///////////////////////////////////////////////////////////
void Mutex::Signal()
{
	::pthread_cond_signal(&m_condition);
}

///////////////////////////////////////////////////////////
/// @brief	Notify that the condition is met (wait release)
/// @return	None
/// @note
///////////////////////////////////////////////////////////
void Mutex::Broadcast()
{
	::pthread_cond_broadcast(&m_condition);
}

} // namespace thread
} // namespace common_library