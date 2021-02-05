///////////////////////////////////////////////////////////
/// @file   rwlock.cpp
/// @brief  rwlock
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#include <common_library/thread/rwlock.h>

#include <cstdio>
#include <cerrno>

namespace common_library {
namespace thread {    
///////////////////////////////////////////////////////////
/// @brief  Default constructor
/// @return None
/// @note
///////////////////////////////////////////////////////////
RWLock::RWLock()
{
    ::pthread_rwlock_init(&m_lock, NULL);
}

///////////////////////////////////////////////////////////
/// @brief  Destructor
/// @return None
/// @note
///////////////////////////////////////////////////////////
RWLock::~RWLock()
{
    ::pthread_rwlock_destroy(&m_lock);
}

///////////////////////////////////////////////////////////	
/// @brief	Acquire read lock
/// @return	common_library::types::eLockResult
/// @note
///////////////////////////////////////////////////////////
common_library::types::eLockResult RWLock::ReadLock()
{
	switch (::pthread_rwlock_rdlock(&m_lock)) {
	case 0:
		return eLOCK_SUCCESS;
	case EDEADLK:
		return eLOCK_ALREADY;
	default:
		return eLOCK_ERROR;
	}
}

///////////////////////////////////////////////////////////	
/// @brief	Acquire read lock (with timeout)
/// @param[in]	_usec Timeout (microsecond)
/// @return	common_library::types::eLockResult
/// @note
///////////////////////////////////////////////////////////
common_library::types::eLockResult RWLock::ReadLock(unsigned long _usec)
{
    // TODO:
	switch (::pthread_rwlock_timedrdlock(&m_lock, 0)) {
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
/// @brief		Acquire write lock
/// @return		common_library::types::eLockResult
/// @note
///////////////////////////////////////////////////////////
common_library::types::eLockResult RWLock::WriteLock()
{
	switch (::pthread_rwlock_wrlock(&m_lock)) {
	case 0:
		return common_library::types::eLOCK_SUCCESS;
	case EDEADLK:
		return common_library::types::eLOCK_ALREADY;
	default:
		return common_library::types::eLOCK_ERROR;
	}
}

///////////////////////////////////////////////////////////
/// @brief		Acquire write lock
/// @param[in]	_usec Timeout time (microsecond)
/// @return		common_library::types::eLockResult
/// @note
///////////////////////////////////////////////////////////
common_library::types::eLockResult RWLock::WriteLock(unsigned long _usec)
{
	switch (::pthread_rwlock_timedwrlock(&m_lock, &ts)) {
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
void RWLock::Unlock()
{
    ::pthread_rwlock_unlock(&m_lock);
}

} // namespace thread
} // namespace common_library