///////////////////////////////////////////////////////////
/// @file   Synchronizer.cpp
/// @brief  Synchronizer using POSIX mutex rwlock and condition
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#include <common_library/thread/Synchronizer.h>

#include <cerrno>

namespace common_library {
namespace thread {
///////////////////////////////////////////////////////////
/// @brief  Default constructor
/// @return None
/// @note
///////////////////////////////////////////////////////////
Synchronizer::Synchronizer()
{
    Synchronizer(eSYN_MUTEX);
}

///////////////////////////////////////////////////////////
/// @brief  Default constructor
/// @param[in]	_type eSynchronizeType
/// @return None
/// @note
///////////////////////////////////////////////////////////
Synchronizer::Synchronizer(eSynchronizeType _type)
    : m_type(_type), m_mutexCount(0UL), m_rwlockCount(0UL)
{
    switch (m_type)
    {
    case eSYN_MUTEX:
        ::pthread_mutex_init(&m_mutex, NULL);
        break;
    case eSYN_READWRITE:
        ::pthread_rwlock_init(&m_rwlock, NULL);
        break;
    default:
        break;
    }
}

///////////////////////////////////////////////////////////
/// @brief  Destructor
/// @return None
/// @note
///////////////////////////////////////////////////////////
Synchronizer::~Synchronizer()
{
    Leave();

    switch (m_type)
    {
    case eSYN_MUTEX:
        ::pthread_mutex_destroy(&m_mutex);
        break;
    case eSYN_READWRITE:
        ::pthread_rwlock_destroy(&m_rwlock);
        break;
    default:
        break;
    }
}

///////////////////////////////////////////////////////////
/// @brief		Enter guard section (no timeout)
/// @retval		true
/// @retval		false
/// @note
///////////////////////////////////////////////////////////
bool Synchronizer::Enter()
{
    if (m_type != eSYN_MUTEX) return false;
    int res = ::pthread_mutex_lock(&m_mutex);
    switch (res)
    {
    case 0:
        m_mutexCount++;
        return true;
    case ::EDEADLK:
        return true;   
    default:
        return false;
    }
}

///////////////////////////////////////////////////////////
/// @brief		Enter guard section (timeout)
/// @param[in]	_usec Timeout (microsecond)
/// @retval		true
/// @retval		false
/// @note
///////////////////////////////////////////////////////////
bool Synchronizer::Enter(unsigned long _usec)
{
    // TODO: time
    if (m_type != eSYN_MUTEX) return false;
    int res = ::pthread_mutex_timedlock(&m_mutex, 0);
    switch (res)
    {
    case 0:
        m_mutexCount++;
        return true;
    case ::EDEADLK:
        return true;   
    default:
        return false;
    }
}

///////////////////////////////////////////////////////////
/// @brief		Enter the read guard interval (no timeout)
/// @retval		true
/// @retval		false
/// @note
///////////////////////////////////////////////////////////
bool Synchronizer::ReadEnter()
{
    if (m_type != eSYN_READWRITE) return false;
	int res = ::pthread_rwlock_rdlock(&m_rwlock);
	switch (res) {
	case 0:
        m_rwlockCount++;
		return true;
	case ::EDEADLK:
		return true;
	default:
		return false;
	}
}

///////////////////////////////////////////////////////////
/// @brief		Enter write guard interval (no timeout)
/// @retval		true
/// @retval		false
/// @note
///////////////////////////////////////////////////////////
bool Synchronizer::WriteEnter()
{
    if (m_type != eSYN_READWRITE) return false;
	int res = ::pthread_rwlock_wrlock(&m_rwlock);
	switch (res) {
	case 0:
        m_rwlockCount++;
		return true;
	case ::EDEADLK:
		return true;
	default:
		return false;
	}
}

///////////////////////////////////////////////////////////
/// @brief		Enter the read guard section (with timeout)
/// @param[in]	_usec Timeout (microsecond)
/// @retval		true
/// @retval		false
/// @note
///////////////////////////////////////////////////////////
bool Synchronizer::ReadEnter(unsigned long _usec)
{
    // TODO: time
    if (m_type != eSYN_READWRITE) return false;
	int res = ::pthread_rwlock_timedrdlock(&m_lock, 0);
	switch (res) {
	case 0:
        m_rwlockCount++;
		return true;
	case ::EDEADLK:
		return true;
	default:
		return false;
	}
}

///////////////////////////////////////////////////////////
/// @brief		Enter the write guard section (with timeout)
/// @param[in]	_usec Timeout (microsecond)
/// @retval		true
/// @retval		false
/// @note
///////////////////////////////////////////////////////////
bool Synchronizer::WriteEnter(unsigned long _usec)
{
    // TODO: time
    if (m_type != eSYN_READWRITE) return false;
	int res = ::pthread_rwlock_timedwrlock(&m_lock, 0);
	switch (res) {
	case 0:
        m_rwlockCount++;
		return true;
	case ::EDEADLK:
		return true;
	default:
		return false;
	}
}

///////////////////////////////////////////////////////////
/// @brief		Exit the guard section
/// @return		None
/// @note
///////////////////////////////////////////////////////////
void Synchronizer::Leave()
{
    switch (m_type)
    {
    case eSYN_MUTEX:
        while (m_mutexCount > 0)
        {
            ::pthread_mutex_unlock(&m_mutex);
            m_mutexCount--;
        }
        break;
    case eSYN_READWRITE:
        while (m_rwlockCount > 0)
        {
            ::pthread_rwlock_unlock(&m_rwLock);
            m_rwlockCount--;
        }
        break;
    default:
        break;
    }    
}
} // namespace thread
} // namespace common_library