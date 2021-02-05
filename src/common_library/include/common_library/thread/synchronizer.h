///////////////////////////////////////////////////////////
/// @file   synchronizer.h
/// @brief  Synchronizer using POSIX mutex rwlock and condition
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////
#ifndef SYNCHRONIZER_H
#define SYNCHRONIZER_H

#include <common_library/thread/mutex.h>
#include <common_library/thread/rwlock.h>

#define SYN_ENTER_RETURN(mutex, synobj, ret) \
	common_library::thread::Synchronizer synobj(mutex); \
	if (!synobj.Enter()) { \
		return ret; \
	}

#define SYN_ENTER_RETURN_TIMEOUT(mutex, synobj, timeout, ret) \
	common_library::thread::Synchronizer synobj(mutex); \
	if (!synobj.Enter(timeout)) { \
		return ret; \
	}

#define SYN_ENTER(mutex, synobj) \
	common_library::thread::Synchronizer synobj(mutex); \
	if (!synobj.Enter()) { \
		return; \
	}

#define SYN_ENTER_TIMEOUT(mutex, synobj, timeout) \
	common_library::thread::Synchronizer synobj(mutex); \
	if (!synobj.Enter(timeout)) { \
		return; \
	}

#define SYN_READ_ENTER(rwlock, synobj) \
	common_library::thread::Synchronizer synobj(rwlock); \
	if (!synobj.ReadEnter()) { \
		return; \
	}

#define SYN_READ_ENTER_TIMEOUT(rwlock, synobj, timeout) \
	common_library::thread::Synchronizer synobj(rwlock); \
	if (!synobj.ReadEnter(timeout)) { \
		return; \
	}

#define SYN_READ_ENTER_RETURN(rwlock, synobj, ret) \
	common_library::thread::Synchronizer synobj(rwlock); \
	if (!synobj.ReadEnter()) { \
		return ret; \
	}

#define SYN_READ_ENTER_RETURN_TIMEOUT(rwlock, synobj, timeout, ret) \
	common_library::thread::Synchronizer synobj(rwlock); \
	if (!synobj.ReadEnter(timeout)) { \
		return ret; \
	}

#define SYN_WRITE_ENTER(rwlock, synobj) \
	common_library::thread::Synchronizer synobj(rwlock); \
	if (!synobj.WriteEnter()) { \
		return; \
	}

#define SYN_WRITE_ENTER_TIMEOUT(rwlock, synobj, timeout) \
	common_library::thread::Synchronizer synobj(rwlock); \
	if (!synobj.WriteEnter(timeout)) { \
		return; \
	}

#define SYN_WRITE_ENTER_RETURN(rwlock, synobj, ret) \
	common_library::thread::Synchronizer synobj(rwlock); \
	if (!synobj.WriteEnter()) { \
		return ret; \
	}

#define SYN_WRITE_ENTER_RETURN_TIMEOUT(rwlock, synobj, timeout, ret) \
	common_library::thread::Synchronizer synobj(rwlock); \
	if (!synobj.WriteEnter(timeout)) { \
		return ret; \
	}

#define SYN_EXIT(synobj) \
	synobj.Leave();

#define SYN_WAIT(mutex, synobj) \
	common_library::thread::Synchronizer synobj(mutex); \
	if (!synobj.Wait()) { \
		return; \
	}

#define SYN_WAIT_RETURN(mutex, synobj, ret) \
	common_library::thread::Synchronizer synobj(mutex); \
	if (!synobj.Wait()) { \
		return ret; \
	}

#define SYN_WAIT_TIMEOUT(mutex, synobj, timeout) \
	common_library::thread::Synchronizer synobj(mutex); \
	if (!synobj.Wait(timeout)) { \
		return; \
	}

#define SYN_WAIT_RETURN_TIMEOUT(mutex, synobj, timeout, ret) \
	common_library::thread::Synchronizer synobj(mutex); \
	if (!synobj.Wait(timeout)) { \
		return ret; \
	}

#define SYN_SIGNAL(synobj) \
	synobj.Signal();

#define SYN_BROADCAST(synobj) \
	synobj.Broadcast();

namespace common_library {
namespace thread {
///////////////////////////////////////////////////////////
/// @class	Synchronizer
/// @brief  Synchronizer using POSIX mutex rwlock and condition
/// @note
///////////////////////////////////////////////////////////
class Synchronizer
{
public:
    ///////////////////////////////////////////////////////////
    /// @brief  Default constructor
	/// @param[in]	_mutex Mutex object
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    Synchronizer(Mutex& _mutex, RWLock& _rwlock);

    ///////////////////////////////////////////////////////////
    /// @brief  Default constructor
	/// @param[in]	_rwlock RWLock object
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    //Synchronizer(RWLock& _rwlock);

    ///////////////////////////////////////////////////////////
    /// @brief  Destructor
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    virtual ~Synchronizer();

	///////////////////////////////////////////////////////////
	/// @brief		Enter SYN section (no timeout)
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
	bool Enter();

	///////////////////////////////////////////////////////////
	/// @brief		Enter SYN section (timeout)
	/// @param[in]	_usec Timeout (microsecond)
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////

	bool Enter(unsigned long _usec);
	///////////////////////////////////////////////////////////
	/// @brief		Enter the read SYN interval (no timeout)
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
	bool ReadEnter();

	///////////////////////////////////////////////////////////
	/// @brief		Enter write SYN interval (no timeout)
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
	bool WriteEnter();

	///////////////////////////////////////////////////////////
	/// @brief		Enter the read SYN section (with timeout)
	/// @param[in]	_usec Timeout (microsecond)
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
	bool ReadEnter(unsigned long _usec);

	///////////////////////////////////////////////////////////
	/// @brief		Enter the write SYN section (with timeout)
	/// @param[in]	_usec Timeout (microsecond)
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
	bool WriteEnter(unsigned long _usec);

	///////////////////////////////////////////////////////////
	/// @brief		Exit the SYN section
	/// @return		None
	/// @note
	///////////////////////////////////////////////////////////
	void Leave();

	///////////////////////////////////////////////////////////
	/// @brief	Wait for condition (no timeout)
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
	bool Wait();

	///////////////////////////////////////////////////////////
	/// @brief	    Wait for condition (with timeout)
	/// @param[in]	_usec Timeout (microsecond)
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
	bool Wait(unsigned long _usec);

	///////////////////////////////////////////////////////////
	/// @brief	Notify that the condition is met (wait release)
	/// @return	None
	/// @note
	///////////////////////////////////////////////////////////
	void Signal();

	///////////////////////////////////////////////////////////
	/// @brief	Notify that the condition is met (wait release)
	/// @return	None
	/// @note
	///////////////////////////////////////////////////////////
	void Broadcast();

private:
    ///////////////////////////////////////////////////////////
    /// @enum       eSynchronizeType
    /// @brief      Synchronize type
    /// @note
    ///////////////////////////////////////////////////////////
    enum eSynchronizeType
    {
        eSYN_MUTEX = 0,
        eSYN_RWLOCK
    };
        
    /// eSynchronizeType
    eSynchronizeType m_type;

    /// Mutex
    Mutex& m_mutex;

    /// RWlock
    RWLock& m_rwlock;

    /// Number of times the SYN sction has been entered(mutex)
    unsigned long m_mutexCount;

    /// Number of times the SYN sction has been entered(rwlock)
    unsigned long m_rwlockCount;

    ///////////////////////////////////////////////////////////
    /// @brief      Constructor
    /// @note       Constructor prohibited
    ///////////////////////////////////////////////////////////
    Synchronizer();

    ///////////////////////////////////////////////////////////
    /// @brief      Copy constructor
    /// @note       Copy prohibited
    ///////////////////////////////////////////////////////////
    Synchronizer(const Synchronizer& _src);

    ///////////////////////////////////////////////////////////
    /// @brief      Assignment operator
    /// @note       Substitution prohibited
    ///////////////////////////////////////////////////////////
    Synchronizer& operator=(const Synchronizer& _src);
};
} // namespace thread
} // namespace common_library

#endif // SYNCHRONIZER_H
