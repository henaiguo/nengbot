///////////////////////////////////////////////////////////
/// @file   Synchronizer.cpp
/// @brief  Synchronizer using POSIX mutex rwlock and condition
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#include <common_library/thread/synchronizer.h>

namespace common_library {
namespace thread {
///////////////////////////////////////////////////////////
/// @brief  Default constructor
/// @param[in]	_mutex Mutex object
/// @return None
/// @note
///////////////////////////////////////////////////////////
Synchronizer::Synchronizer(Mutex& _mutex, RWLock& _rwlock)
    : m_mutex(_mutex), m_mutexCount(0UL), m_rwlock(_rwlock), m_rwlockCount(0UL)
//    , m_type(eSYN_MUTEX)
{
    // None
}

///////////////////////////////////////////////////////////
/// @brief  Default constructor
/// @param[in]	_rwlock RWLock object
/// @return None
/// @note
///////////////////////////////////////////////////////////
//Synchronizer::Synchronizer(RWLock& _rwlock)
//    : m_rwlock(_rwlock), m_rwlockCount(0UL), m_mutex(NULL), m_mutexCount(0UL)
//    , m_type(eSYN_RWLOCK)
//{
    // None
//}

///////////////////////////////////////////////////////////
/// @brief  Destructor
/// @return None
/// @note
///////////////////////////////////////////////////////////
Synchronizer::~Synchronizer()
{
    Leave();
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
    switch (m_mutex.Lock()) {
	case common_library::types::eLOCK_SUCCESS:
        m_mutexCount++;
        return true;
    case common_library::types::eLOCK_ALREADY:
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
    if (m_type != eSYN_MUTEX) return false;
    switch (m_mutex.Lock(_usec))
    {
	case common_library::types::eLOCK_SUCCESS:
        m_mutexCount++;
        return true;
    case common_library::types::eLOCK_ALREADY:
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
    if (m_type != eSYN_RWLOCK) return false;
	switch (m_rwlock.ReadLock()) {
	case common_library::types::eLOCK_SUCCESS:
        m_rwlockCount++;
		return true;
    case common_library::types::eLOCK_ALREADY:
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
    if (m_type != eSYN_RWLOCK) return false;
	switch (m_rwlock.WriteLock()) {
	case common_library::types::eLOCK_SUCCESS:
        m_rwlockCount++;
		return true;
    case common_library::types::eLOCK_ALREADY:
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
    if (m_type != eSYN_RWLOCK) return false;
	switch (m_rwlock.ReadLock(_usec)) {
	case common_library::types::eLOCK_SUCCESS:
        m_rwlockCount++;
		return true;
    case common_library::types::eLOCK_ALREADY:
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
    if (m_type != eSYN_RWLOCK) return false;
	switch (m_rwlock.WriteLock(_usec)) {
	case common_library::types::eLOCK_SUCCESS:
        m_rwlockCount++;
		return true;
    case common_library::types::eLOCK_ALREADY:
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
            m_mutex.Unlock();
            m_mutexCount--;
        }
        break;
    case eSYN_RWLOCK:
        while (m_rwlockCount > 0)
        {
            m_rwlock.Unlock();
            m_rwlockCount--;
        }
        break;
    default:
        break;
    }    
}

///////////////////////////////////////////////////////////
/// @brief	    Wait for condition (no timeout)
/// @retval		true
/// @retval		false
/// @note
///////////////////////////////////////////////////////////
bool Synchronizer::Wait()
{
    if (m_type != eSYN_MUTEX) return false;    
	switch (m_mutex.Wait()) {
	case common_library::types::eWAIT_SUCCESS:
		return true;
    case common_library::types::eLOCK_ALREADY:
		return true;
	default:
		return false;
	}
}

///////////////////////////////////////////////////////////
/// @brief	    Wait for condition (with timeout)
/// @param[in]	_usec Timeout (microsecond)
/// @retval		true
/// @retval		false
/// @note
///////////////////////////////////////////////////////////
bool Synchronizer::Wait(unsigned long _usec)
{
    if (m_type != eSYN_MUTEX) return false;    
	switch (m_mutex.Wait(_usec)) {
	case common_library::types::eWAIT_SUCCESS:
		return true;
    case common_library::types::eLOCK_ALREADY:
		return true;
	default:
		return false;
	}
}

///////////////////////////////////////////////////////////
/// @brief	Notify that the condition is met (wait release)
/// @return	None
/// @note
///////////////////////////////////////////////////////////
void Synchronizer::Signal()
{
    if (m_type != eSYN_MUTEX) return;
    m_mutex.Signal();
}

///////////////////////////////////////////////////////////
/// @brief	Notify that the condition is met (wait release)
/// @return	None
/// @note
///////////////////////////////////////////////////////////
void Synchronizer::Broadcast()
{
    if (m_type != eSYN_MUTEX) return;
    m_mutex.Broadcast();
}

} // namespace thread
} // namespace common_library
