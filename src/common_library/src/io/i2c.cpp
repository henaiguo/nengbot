///////////////////////////////////////////////////////////
/// @file   i2c.cpp
/// @brief  i2c driver for beaglebone black
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#include <common_library/io/i2c.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

using common_library::types::Result;

namespace common_library {
namespace io {
///////////////////////////////////////////////////////////
/// @brief  Default constructor
/// @return None
/// @note
///////////////////////////////////////////////////////////
I2C::I2C()
    : m_fd(INVALID_I2C_FD)
{
    // None
}

///////////////////////////////////////////////////////////
/// @brief  Destructor
/// @return None
/// @note
///////////////////////////////////////////////////////////
I2C::~I2C()
{
    Close();
}

///////////////////////////////////////////////////////////
/// @brief  Open i2c device
/// @param[in]  _devName I2c device name
/// @param[in]  _devAddr I2c device address
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result I2C::Open(std::string _devName, uint8_t _devAddr)
{
    int fd = ::open(_devName.c_str(), O_RDWR);
    if (fd == INVALID_I2C_FD) {
        return Result::CreateError("[I2C]Error in Open: Invalid device name(%s)", _devName.c_str());
    }

    if (::ioctl(fd, I2C_SLAVE, _devAddr) < 0) {
        return Result::CreateError("[I2C]Error in Open: Invalid i2c address(0x%02x)", _devAddr);
    }

    m_fd = fd;
    return Result::CreateSuccess();
}

///////////////////////////////////////////////////////////
/// @brief  Close i2c device
/// @return None
/// @note
///////////////////////////////////////////////////////////
void I2C::Close()
{
    if (!IsOpen()) return;

    ::close(m_fd);
}

///////////////////////////////////////////////////////////
/// @brief  Whether the i2c device is open
/// @retval true
/// @retval false
/// @note
///////////////////////////////////////////////////////////
bool I2C::IsOpen() const
{
    return !(m_fd == INVALID_I2C_FD);
}

///////////////////////////////////////////////////////////
/// @brief  Get the FD of the i2c device
/// @return int
/// @note
///////////////////////////////////////////////////////////
int I2C::GetFD() const
{
    return m_fd;
}

///////////////////////////////////////////////////////////
/// @brief  Read a single byte from i2c device
/// @param[in]  _regAddr Register address
/// @param[out] _data Read data
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result I2C::ReadByte(uint8_t _regAddr, uint8_t* _data)
{
    return ReadBytes(_regAddr, 1, _data);
}

///////////////////////////////////////////////////////////
/// @brief  Read multiple bytes from i2c device
/// @param[in]  _regAddr Register address
/// @param[in]  _count Number of bytes to read
/// @param[out] _data Read data
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result I2C::ReadBytes(uint8_t _regAddr, size_t _count, uint8_t* _data)
{
    if (!IsOpen()) {
        return Result::CreateError("[I2C]Error in ReadBytes: Device hasn't open yet");
    } 

    // Write register to device
    int ret = ::write(m_fd, &_regAddr, 1);
    if (ret != 1) {
        return Result::CreateError("[I2C]Error in ReadBytes: Failed to write to i2c device");
    }

    // Then read the response
    ret = ::read(m_fd, _data, _count);
    if (ret != _count) {
        return Result::CreateError("[I2C]Error in ReadBytes: Received %d bytes from device(expected %d)", ret, _count);
    }

    return Result::CreateSuccess();
}

///////////////////////////////////////////////////////////
/// @brief  Read single word from i2c device
/// @param[in]  _regAddr Register address
/// @param[out] _data Read data
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result I2C::ReadWord(uint8_t _regAddr, uint16_t* _data)
{
    return ReadWords(_regAddr, 1, _data);
}

///////////////////////////////////////////////////////////
/// @brief  Read multiple words from i2c device
/// @param[in]  _regAddr Register address
/// @param[in]  _count Number of words to read
/// @param[out] _data Read data
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result I2C::ReadWords(uint8_t _regAddr, size_t _count, uint16_t* _data)
{
    if (!IsOpen()) {
        return Result::CreateError("[I2C]Error in ReadWords: Device hasn't open yet");
    }
    
    // Write register to device
    int ret = ::write(m_fd, &_regAddr, 1);
    if (ret != 1) {
        return Result::CreateError("[I2C]Error in ReadWords: Failed to write to i2c device");
    }

    // Then read the response
    char buf[_count*2];
    ret = read(m_fd, buf, _count*2);
    if (ret != (signed)(_count*2)) {
        return Result::CreateError("[I2C]Error in ReadWords: Received %d bytes from device(expected %d)", ret, _count);
    }

    // Formate words from bytes and put into user's data array
    for (int i = 0; i < _count; i++) {
        _data[i] = (((uint16_t)buf[i*2])<<8 | buf[(i*2)+1]);
    }

    return Result::CreateSuccess();
}

///////////////////////////////////////////////////////////
/// @brief  Write a single byte to i2c device
/// @param[in]  _regAddr Register address
/// @param[in]  _data Byte to be writen
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result I2C::WriteByte(uint8_t _regAddr, uint8_t _data)
{
    if (!IsOpen()) {
        return Result::CreateError("[I2C]Error in WriteByte: Device hasn't open yet");
    }

    // Assemble array to send, starting with the register address
    uint8_t writeData[2];
    writeData[0] = _regAddr;
    writeData[1] = _data;

    // Write the bytes
    int ret = ::write(m_fd, writeData, 2);
    if (ret != 2) {
        return Result::CreateError("[I2C]Error in WriteByte: Wrote %d bytes to device(expected 2)", ret);
    }
    
    return Result::CreateSuccess();
}

///////////////////////////////////////////////////////////
/// @brief  Write multiple bytes to i2c device
/// @param[in]  _regAddr Register address
/// @param[in]  _data Bytes to be writen
/// @param[in]  _count Number of bytes to be writen
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result I2C::WriteBytes(uint8_t _regAddr, const uint8_t* _data, size_t _count)
{
    if (!IsOpen()) {
        return Result::CreateError("[I2C]Error in WriteBytes: Device hasn't open yet");
    }

    uint8_t writeData[_count+1];

    // assemble array to send, starting with the register address
    writeData[0] = _regAddr;
    for (int i = 0; i < _count; i++) {
        writeData[i+1] = _data[i];
    }

    // send the bytes
    int ret = ::write(m_fd, writeData, _count+1);
    if (ret != (signed)(_count+1)) {
        return Result::CreateError("[I2C]Error in WriteBytes: Write returned %d (expected %d)", ret, _count);
    }

    return Result::CreateSuccess();
}

///////////////////////////////////////////////////////////
/// @brief  Write a single word to i2c device
/// @param[in]  _regAddr Register address
/// @param[in]  _data word to be writen
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result I2C::WriteWord(uint8_t _regAddr, uint16_t _data)
{
    if (!IsOpen()) {
        return Result::CreateError("[I2C]Error in WriteWord: Device hasn't open yet");
    }
    
    uint8_t writeData[3];

    // assemble bytes to send from data casted as uint8_t*
    writeData[0] = _regAddr;
    writeData[1] = (uint8_t)(_data >> 8);
    writeData[2] = (uint8_t)(_data & 0xFF);

    int ret = ::write(m_fd, writeData, 3);
    if (ret != 3) {
        return Result::CreateError("[I2C]Error in WriteWord: Write returned %d (expected 3)", ret);
    }
    
    return Result::CreateSuccess();
}

///////////////////////////////////////////////////////////
/// @brief  Write multiple words to i2c device
/// @param[in]  _regAddr Register address
/// @param[in]  _data Words to be writen
/// @param[in]  _count Number of words to be writen
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result I2C::WriteWords(uint8_t _regAddr, const uint16_t* _data, size_t _count)
{
    if (!IsOpen()) {
        return Result::CreateError("[I2C]Error in WriteWords: Device hasn't open yet");
    }

    uint8_t writeData[(_count*2)+1];

    // assemble bytes to send
    writeData[0] = _regAddr;
    for (int i = 0; i < _count; i++) {
        writeData[(i*2)+1] = (uint8_t)(_data[i] >> 8);
        writeData[(i*2)+2] = (uint8_t)(_data[i] & 0xFF);
    }

    int ret = ::write(m_fd, writeData, (_count*2)+1);
    if (ret != (signed)(_count*2)+1) {
        return Result::CreateError("[I2C]Error in WriteWords: Write returned %d (expected %d)", ret, (_count*2)+1);
    }
    
    return Result::CreateSuccess();
}

///////////////////////////////////////////////////////////
/// @brief  Send a single byte to i2c device
/// @param[in]  _data Byte to be send
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result I2C::SendByte(uint8_t _data)
{
    return SendBytes(&_data, 1);
}

///////////////////////////////////////////////////////////
/// @brief  Send multiple bytes to i2c device
/// @param[in]  _data Bytes to be send
/// @param[in]  _count Number of bytes to be send
/// @return common_library::types::Result
/// @note
///////////////////////////////////////////////////////////
common_library::types::Result I2C::SendBytes(const uint8_t* _data, size_t _count)
{
    if (!IsOpen()) {
        return Result::CreateError("[I2C]Error in SendBytes: Device hasn't open yet");
    }

    // Send the bytes
    int ret = ::write(m_fd, _data, _count);

    // Write should have returned the correct # bytes written
    if (ret != (signed)_count){
        return Result::CreateError("[I2C]Error in SendBytes: Received %d bytes from device(expected %d)", ret, _count);
    }

    return Result::CreateSuccess();
}
} // namespace io
} // namespace common_library
