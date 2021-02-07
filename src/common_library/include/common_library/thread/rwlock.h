///////////////////////////////////////////////////////////
/// @file	rwlock.h
/// @brief	rwlock
/// @author	henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////
#ifndef RWLOCK_H
#define RWLOCK_H

#include <common_library/types/elock_result.h>
#include <pthread.h>

namespace common_library {
namespace thread {
///////////////////////////////////////////////////////////
/// @class	RWLock
/// @brief	rwlock
/// @note
///////////////////////////////////////////////////////////
class RWLock
{
public:
	///////////////////////////////////////////////////////////
	/// @brief 	Default constructor
	/// @return	None
	/// @note
	///////////////////////////////////////////////////////////
	RWLock();

	///////////////////////////////////////////////////////////
	/// @brief	Destructor
	/// @return	None
	/// @note
	///////////////////////////////////////////////////////////
	virtual ~RWLock();

	///////////////////////////////////////////////////////////	
	/// @brief	Acquire read lock
	/// @return	common_library::types::eLockResult
	/// @note
	///////////////////////////////////////////////////////////
	common_library::types::eLockResult ReadLock();

	///////////////////////////////////////////////////////////	
	/// @brief	Acquire read lock (with timeout)
	/// @param[in]	_usec Timeout (microsecond)
	/// @return	common_library::types::eLockResult
	/// @note
	///////////////////////////////////////////////////////////
	common_library::types::eLockResult ReadLock(unsigned long _usec);

	///////////////////////////////////////////////////////////
	/// @brief	Acquire write lock
	/// @return	common_library::types::eLockResult
	/// @note
	///////////////////////////////////////////////////////////
	common_library::types::eLockResult WriteLock();

	///////////////////////////////////////////////////////////
	/// @brief	Acquire write lock
	/// @param[in]	_usec Timeout time (microsecond)
	/// @return	common_library::types::eLockResult
	/// @note
	///////////////////////////////////////////////////////////
	common_library::types::eLockResult WriteLock(unsigned long _usec);

	///////////////////////////////////////////////////////////
	/// @brief	Unlock
	/// @return	None
	/// @note
	///////////////////////////////////////////////////////////
	void Unlock();

private:
	/// POSIX mutex
	::pthread_rwlock_t m_lock;
};
} // namespace thread
} // namespace common_library

#endif // RWLOCK_H
