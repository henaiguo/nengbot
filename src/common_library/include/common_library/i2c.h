///////////////////////////////////////////////////////////
/// @file	i2c.h
/// @brief	i2c driver for beaglebone black
/// @author	henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////
#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdlib.h>
#include <string>

namespace CommonLibrary {
///////////////////////////////////////////////////////////
/// @class I2C
/// @brief i2c driver
/// @note
///////////////////////////////////////////////////////////
class I2C
{
public:
    /// Invalid i2c file description
    static const int INVALID_I2C_FD = -1;

    ///////////////////////////////////////////////////////////
    /// @brief		Default constructor
    /// @return		None
    /// @note
    ///////////////////////////////////////////////////////////
    I2C();

	///////////////////////////////////////////////////////////
	/// @brief		Destructor
	/// @return		None
	/// @note
	///////////////////////////////////////////////////////////
    virtual ~I2C();

	///////////////////////////////////////////////////////////
	/// @brief		Open i2c device
	/// @param[in]	_devName I2c device name
	/// @param[in]	_devAddr I2c device address
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
    bool Open(std::string _devName, uint8_t _devAddr);

	///////////////////////////////////////////////////////////
	/// @brief		Close i2c device
	/// @return		None
	/// @note
	///////////////////////////////////////////////////////////
    void Close();

	///////////////////////////////////////////////////////////
	/// @brief		Whether the i2c device is open
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
    bool IsOpen() const;

	///////////////////////////////////////////////////////////
	/// @brief		Get the FD of the i2c device
	/// @return		int
	/// @note
	///////////////////////////////////////////////////////////
    int GetFD() const;

	///////////////////////////////////////////////////////////
	/// @brief		Read a single byte from i2c device
	/// @param[in]	_regAddr Register address
	/// @param[out]	_data Read data
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
    bool ReadByte(uint8_t _regAddr, uint8_t* _data);

	///////////////////////////////////////////////////////////
	/// @brief		Read multiple bytes from i2c device
	/// @param[in]	_regAddr Register address
    /// @param[in]  _count Number of bytes to read
	/// @param[out]	_data Read data
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
    bool ReadBytes(uint8_t _regAddr, size_t _count, uint8_t* _data);

    ///////////////////////////////////////////////////////////
	/// @brief		Read single word from i2c device
	/// @param[in]	_regAddr Register address
	/// @param[out]	_data Read data
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
    bool ReadWord(uint8_t _regAddr, uint16_t* _data);

	///////////////////////////////////////////////////////////
	/// @brief		Read multiple words from i2c device
	/// @param[in]	_regAddr Register address
    /// @param[in]  _count Number of words to read
	/// @param[out]	_data Read data
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
    bool ReadWords(uint8_t _regAddr, size_t _count, uint16_t* _data);

	///////////////////////////////////////////////////////////
	/// @brief		Write a single byte to i2c device
	/// @param[in]	_regAddr Register address
	/// @param[in]	_data Byte to be writen
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
    bool WriteByte(uint8_t _regAddr, uint8_t _data);

	///////////////////////////////////////////////////////////
	/// @brief		Write multiple bytes to i2c device
	/// @param[in]	_regAddr Register address
	/// @param[in]	_data Bytes to be writen
    /// @param[in]  _count Number of bytes to be writen
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
    bool WriteBytes(uint8_t _regAddr, const uint8_t* _data, size_t _count);

	///////////////////////////////////////////////////////////
	/// @brief		Write a single word to i2c device
	/// @param[in]	_regAddr Register address
	/// @param[in]	_data word to be writen
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
    bool WriteWord(uint8_t _regAddr, uint16_t _data);

	///////////////////////////////////////////////////////////
	/// @brief		Write multiple words to i2c device
	/// @param[in]	_regAddr Register address
	/// @param[in]	_data Words to be writen
    /// @param[in]  _count Number of words to be writen
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
    bool WriteWords(uint8_t _regAddr, const uint16_t* _data, size_t _count);

	///////////////////////////////////////////////////////////
	/// @brief		Send a single byte to i2c device
	/// @param[in]	_data Byte to be send
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
    bool SendByte(uint8_t _data);

	///////////////////////////////////////////////////////////
	/// @brief		Send multiple bytes to i2c device
	/// @param[in]	_data Bytes to be send
    /// @param[in]  _count Number of bytes to be send
	/// @retval		true
	/// @retval		false
	/// @note
	///////////////////////////////////////////////////////////
    bool SendBytes(const uint8_t* _data, size_t _count);

private:
    /// I2C device fd
    int m_fd;

    /// I2C device address
    uint8_t m_devAddr;
};
} // namespace CommonLibrary

#endif
