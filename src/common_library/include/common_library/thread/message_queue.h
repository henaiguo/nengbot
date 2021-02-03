///////////////////////////////////////////////////////////
/// @file	message_queue.h
/// @brief	Message queue
/// @author	henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#include <mqueue.h>
#include <string>
#include <vector>
#include "Error.h"
#include "ByteBuffer.h"

namespace common_library {
namespace thread {

class MessageQueue;

///////////////////////////////////////////////////////////
/// @class	INotifyMessage
/// @brief	Message incoming notification interface
/// 
/// - Message notification will only occur when a new message arrives on the empty queue
/// - Implement this interface to implement the process when a notification message is received
///
///////////////////////////////////////////////////////////
class INotifyMessage
{
public:
	///////////////////////////////////////////////////////////
	/// @brief		Destructor
	///////////////////////////////////////////////////////////
	virtual ~INotifyMessage() {};

	///////////////////////////////////////////////////////////
	/// @brief		Perform processing when message arrives
	/// 
	/// Called when a message arrives with the message queue empty
	/// Efficient because you don't have to poll for message arrival
	///
	/// Message arrives when message is in message queue
	/// Note that even if you do not call
	/// 
	/// @param[in]	mq Message queue to notify
	/// @note		
	///////////////////////////////////////////////////////////
	virtual void FirstMessageArrived(MessageQueue &mq) = 0;
};


///////////////////////////////////////////////////////////
/// @class MessageQueue
/// @brief	POSIX Message queue
///
/// Of data in the form of asynchronous messages between threads and processes
/// Provide a mechanism to interact
///
/// About message queue settings
/// The default setting of the message queue is predetermined by the system
///
/// $ sysctl -a 2> /dev/null | grep mqueue
/// fs.mqueue.msg_default = 10
/// fs.mqueue.msg_max = 10
/// fs.mqueue.msgsize_default = 8192
/// fs.mqueue.msgsize_max = 8192
/// fs.mqueue.queues_max = 256
///
/// $ ls /proc/sys/fs/mqueue
/// msg_default  msg_max  msgsize_default  msgsize_max  queues_max
///
/// Read
/// cat /proc/sys/fs/mqueue/msg_max You can refer to the value with
///
/// Write
/// sudo sysctl -w fs.mqueue.msg_max=1000
/// sudo sh -c 'echo 1000 > /proc/sys/fs/mqueue/msg_max Value can be set with (admin authority required)
///
/// Number of default messages that can be registered: /proc/sys/fs/mqueue/msg_default
/// Maximum number of messages that can be registered: /proc/sys/fs/mqueue/msg_max
/// Default message length: /proc/sys/fs/mqueue/msgsize_default
/// Maximum message length: /proc/sys/fs/mqueue/msgsize_max
/// Maximum number of message queues: /proc/sys/fs/mqueue/queues_max
/// These values ​​apply system-wide
/// ( (In the actual environment, msg_default and msgsize_default do not exist.)
/// 
/// To change the value at system startup (development environment only)
/// /etc/sysctl.d/NN-xxxxx.conf Create and register as fs.mqueue.xxxxx = value
/// 
///////////////////////////////////////////////////////////
class MessageQueue {
public:
	///////////////////////////////////////////////////////////
	/// @brief		Check if a message queue with the specified name exists
	/// @return		Error
	/// @note		name Must start with'/' eg) "/mq1"
	///////////////////////////////////////////////////////////
	static Error Exist(const std::string &name);

	///////////////////////////////////////////////////////////
	/// @brief		constructor
	///
	/// Use the created message queue as a reference
	///
	/// @param[in]	name name
	/// @note		name Must start with'/' eg) "/mq1"
	/// @note		Exist() determines if the message queue exists
	///////////////////////////////////////////////////////////
	MessageQueue(const std::string &name);

	///////////////////////////////////////////////////////////
	/// @brief		constructor
	///
	/// Creating and using a new message queue
	///
	/// @param[in]	name name
	/// @param[in]	maxMessageCount Maximum number of messages that can be registered in the message queue> 0
	/// @param[in]	maxMessageSize Maximum message length> 0
	/// @note		name Must start with'/' eg) "/mq1"
	/// @note		Create/delete message queue<br/>
	///////////////////////////////////////////////////////////
	MessageQueue(const std::string &name, long maxMessageCount, long maxMessageSize);

	///////////////////////////////////////////////////////////
	/// @brief		Destructor
	/// @note
	///////////////////////////////////////////////////////////
	virtual ~MessageQueue();

	///////////////////////////////////////////////////////////
	/// @brief		Get name
	/// @return		name
	/// @note
	///////////////////////////////////////////////////////////
	const std::string &Name() const;

	///////////////////////////////////////////////////////////
	/// @brief		Get the maximum number of messages that can be registered in the message queue
	/// @param[in]	None
	/// @return		Maximum number of messages
	/// @note		Get maxMessageCount specified in the constructor
	///////////////////////////////////////////////////////////
	long MaxMessageCount();

	///////////////////////////////////////////////////////////
	/// @brief		Get maximum size of message that can be registered in message queue
	/// @param[in]	None
	/// @return		Message length
	/// @note		Get maxMessageSize specified in the constructor
	///////////////////////////////////////////////////////////
	long MaxMessageSize();

	///////////////////////////////////////////////////////////
	/// @brief		Get the number of messages currently in the message queue
	/// @param[in]	None
	/// @return		Number of messages currently in the message queue
	/// @note		
	///////////////////////////////////////////////////////////
	long CurrentMessageCount();

	///////////////////////////////////////////////////////////
	/// @brief		Clear messages in the message queue
	/// @param[in]	None
	/// @return		None
	/// @note		
	///////////////////////////////////////////////////////////
	void Clear();

	///////////////////////////////////////////////////////////
	/// @brief		Send a message to the message queue
	/// @param[in]	message message
	/// @return		Error When it fails, the error content is set to Error
	/// @note		When the message queue is full, block until there is space
	/// @note		A message with a message size of 0 is also possible
	///////////////////////////////////////////////////////////
	Error Send(const ByteBuffer &message);

	///////////////////////////////////////////////////////////
	/// @brief		Send message to Message Queue with timeout
	/// @param[in]	message message
	/// @param[in]	millisec millisecond
	/// @return		Error When it fails, the error content is set to Error
	/// @note		millisec If is 0, block until sending is possible
	/// @note		If there is no space in the message queue, or if you cannot register after waiting for the specified time, an error occurs
	/// @note		A message with a message size of 0 is also possible
	///////////////////////////////////////////////////////////
	Error TimedSend(const ByteBuffer &message, unsigned long millisec);

	///////////////////////////////////////////////////////////
	/// @brief		Receive a message from the message queue
	/// @param[out]	outMessage message
	/// @return		Error When it fails, the error content is set to Error
	/// @note		When the message queue is empty, block until a newly added message is available
	/// @note		A message with a message size of 0 is also possible
	///////////////////////////////////////////////////////////
	Error Receive(ByteBuffer &outMessage);

	///////////////////////////////////////////////////////////
	/// @brief		Receive message from message queue with timeout
	/// @param[out]	message message
	/// @param[in]	millisec millisecond
	/// @return		Error When it fails, the error content is set to Error
	/// @note		When the message queue is empty, an error will occur if the message cannot be obtained after waiting for the specified time.
	/// @note		millisec If is 0, block until it can be obtained
	/// @note		A message with a message size of 0 is also possible
	///////////////////////////////////////////////////////////
	Error TimedReceive(ByteBuffer &outMessage, unsigned long millisec);

	///////////////////////////////////////////////////////////
	/// @brief		Receive all messages stored in the message queue
	/// @param[out]	outMessages Message list
	/// @return		Error When it fails, the error content is set to Error
	/// @note		If there are no messages, Error returns with success and the outMessages size is 0.
	/// @note		A message with a message size of 0 is also possible
	///////////////////////////////////////////////////////////
	Error Receive(std::vector<ByteBuffer> &outMessages);

	///////////////////////////////////////////////////////////
	/// @brief		INotifyMessage Set the interface
	/// @param[in]	notification Handler to be notified
	/// @note		Since the message queue is an empty queue (size 0)<br/>
	///             Note that we only notify you when new messages arrive
	/// @note		NULL Does not notify when set to
	///////////////////////////////////////////////////////////
	void SetNotifyMessage(INotifyMessage *notification);

	///////////////////////////////////////////////////////////
	/// @brief		INotifyMessage Get interface
	/// @param[in]	None
	/// @return		INotifyMessage 
	/// @note		
	///////////////////////////////////////////////////////////
	INotifyMessage *NotifyMessage();

private:
	///////////////////////////////////////////////////////////
	/// @brief		Copy constructor
	/// @note		Copy prohibited
	///////////////////////////////////////////////////////////
	MessageQueue(const MessageQueue &src);

	///////////////////////////////////////////////////////////
	/// @brief		Assignment operator
	/// @note		Substitution prohibited
	///////////////////////////////////////////////////////////
	MessageQueue& operator=(const MessageQueue &src);

	///////////////////////////////////////////////////////////
	/// @brief	Initialize
	/// @param[in]	maxMessageCount Maximum number of messages that can be registered in the message queue
	/// @param[in]	maxMessageSize Maximum message length
	///////////////////////////////////////////////////////////
	void Init(long maxMessageCount, long maxMessageSize);


	std::string     mName;         ///< name
	bool            mIsOwner;      ///< Ownership
	mqd_t           mMessageQueue; ///< POSIX Message queue
	mq_attr         mAttribute;    ///< Message queue attribute
	INotifyMessage *mNotification; ///< INotifyMessage
	sigevent        mSignalEvent;  ///< Signal event
};

}
}

#endif MESSAGE_QUEUE_H
