///////////////////////////////////////////////////////////
/// @file   mutex.cpp
/// @brief  Mutex
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#include <common_library/thread/mutex.h>

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
}

///////////////////////////////////////////////////////////
/// @brief  Destructor
/// @return None
/// @note
///////////////////////////////////////////////////////////
Mutex::~Mutex()
{
    ::pthread_mutex_destroy(&m_mutex);
}

///////////////////////////////////////////////////////////	
/// @brief	Acquire lock
/// @return	common_library::types::eLockResult
/// @note
///////////////////////////////////////////////////////////
common_library::types::eLockResult Mutex::Lock()
{
	int r = ::pthread_mutex_lock(&m_mutex);
	switch (r) {
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
	int r = ::pthread_mutex_timedlock(&m_mutex, 0);
	switch (r) {
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

} // namespace thread
} // namespace common_library