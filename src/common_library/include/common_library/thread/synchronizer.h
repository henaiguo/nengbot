///////////////////////////////////////////////////////////
/// @file   synchronizer.h
/// @brief  Synchronizer using POSIX mutex rwlock and condition
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////
#ifndef SYNCHRONIZER_H
#define SYNCHRONIZER_H

#include <pthread.h>

#define SYN_ENTER_RETURN(synobj, ret) \
	common_library::library::Synchronizer synobj(eSYN_MUTEX); \
	if (synobj.Enter() == false) { \
		return ret; \
	}

#define SYN_ENTER_RETURN_TIMEOUT(synobj, timeout, ret) \
	common_library::library::Synchronizer synobj(eSYN_MUTEX); \
	if (synobj.Enter(timeout) == false) { \
		return ret; \
	}

#define SYN_ENTER(synobj) \
	common_library::library::Synchronizer synobj(eSYN_MUTEX); \
	if (synobj.Enter() == false) { \
		return; \
	}

#define SYN_ENTER_TIMEOUT(synobj, timeout) \
	common_library::library::Synchronizer synobj(eSYN_MUTEX); \
	if (synobj.Enter(timeout) == false) { \
		return; \
	}

#define SYN_EXIT(synobj) \
	synobj.Leave();

#define READ_ENTER_RETURN(synobj, ret) \
	common_library::library::Synchronizer synobj(eSYN_READWRITE); \
	if (synobj.ReadEnter() == false) { \
		return ret; \
	}

#define READ_ENTER_RETURN_TIMEOUT(synobj, timeout, ret) \
	common_library::library::Synchronizer synobj(eSYN_READWRITE); \
	if (synobj.ReadEnter(timeout) == false) { \
		return ret; \
	}

#define READ_ENTER(synobj) \
	common_library::library::Synchronizer synobj(eSYN_READWRITE); \
	if (synobj.ReadEnter() == false) { \
		return; \
	}

#define READ_ENTER_TIMEOUT(synobj, timeout) \
	common_library::library::Synchronizer synobj(eSYN_READWRITE); \
	if (synobj.ReadEnter(timeout) == false) { \
		return; \
	}

#define READ_EXIT(synobj) \
	synobj.Leave();

#define WRITE_ENTER_RETURN(synobj, ret) \
	common_library::library::Synchronizer synobj(eSYN_READWRITE); \
	if (synobj.WriteEnter() == false) { \
		return ret; \
	}

#define WRITE_ENTER_RETURN_TIMEOUT(synobj, timeout, ret) \
	common_library::library::Synchronizer synobj(eSYN_READWRITE); \
	if (synobj.WriteEnter(timeout) == false) { \
		return ret; \
	}

#define WRITE_ENTER(synobj) \
	common_library::library::Synchronizer synobj(eSYN_READWRITE); \
	if (synobj.WriteEnter() == false) { \
		return; \
	}

#define WRITE_ENTER_TIMEOUT(synobj, timeout) \
	common_library::library::Synchronizer synobj(eSYN_READWRITE); \
	if (synobj.WriteEnter(timeout) == false) { \
		return; \
	}

#define WRITE_EXIT(synobj) \
	synobj.Leave();

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
	/// @enum   eSynchronizeType
	/// @brief	Synchronize type
	/// @note
	///////////////////////////////////////////////////////////
    enum eSynchronizeType
    {
        eSYN_MUTEX = 0,
        eSYN_READWRITE
    };

    ///////////////////////////////////////////////////////////
    /// @brief  Default constructor
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    Synchronizer();

    ///////////////////////////////////////////////////////////
    /// @brief  Default constructor
	/// @param[in]	_type eSynchronizeType
    /// @return None
    /// @note
    ///////////////////////////////////////////////////////////
    Synchronizer(eSynchronizeType _type);

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

private:
    /// Synchronize type
    eSynchronizeType m_type;

    /// POSIX mutex
    ::pthread_mutex_t m_mutex;

    /// POSIX rwlock
    ::pthread_rwlock_t m_rwlock;

    /// Number of times the SYN sction has been entered(mutex)
    unsigned long m_mutexCount;

    /// Number of times the SYN sction has been entered(rwlock)
    unsigned long m_rwlockCount;
};
} // namespace thread
} // namespace common_library

#endif // SYNCHRONIZER_H
