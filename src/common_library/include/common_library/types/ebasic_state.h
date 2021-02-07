///////////////////////////////////////////////////////////
/// @file   eBasicState.h
/// @brief  Baisc state type
/// @author henaiguo
/// Copyright (C) 2021- henaiguo. All rights reserved.
///////////////////////////////////////////////////////////

#ifndef EBAISC_STATE_H
#define EBAISC_STATE_H

#include <string>

namespace common_library {
namespace types {
///////////////////////////////////////////////////////////
/// @enum   eBasicState
/// @brief  Baisc state type
/// @note
///////////////////////////////////////////////////////////
enum eBasicState
{
    eSTATE_IDLE = 0,
    eSTATE_STAND,
    eSTATE_WALK
};

///////////////////////////////////////////////////////////
/// @brief  enum Is a string
/// @param[in]  _enum enum value
/// @return char*
/// @note
///////////////////////////////////////////////////////////
static std::string ToString(eBasicState _enum)
{
    switch (_enum) {
    case eSTATE_IDLE:
        return "eSTATE_IDLE";
    case eSTATE_STAND:
        return "eSTATE_STAND";
    case eSTATE_WALK:
        return "eSTATE_WALK";
    default:
        return "INVALID_ENUM_VALUE";
    }
}

///////////////////////////////////////////////////////////
/// @brief  Turn a string into an enum
/// @param[in]  _string String
/// @param[out] _enum enum value
/// @retval true
/// @retval false
/// @note
///////////////////////////////////////////////////////////
static bool ToEnum(const std::string& _string, eBasicState& _enum)
{
    if (_string == "eSTATE_IDLE") {
        _enum = eSTATE_IDLE;
        return true;
    }
    else if (_string == "eSTATE_STAND") {
        _enum = eSTATE_STAND;
        return true;
    }
    else if (_string == "eSTATE_WALK") {
        _enum = eSTATE_WALK;
        return true;
    }
    return false;
}

///////////////////////////////////////////////////////////
/// @brief  Set integer to enum
/// @param[in]  _value Integer value
/// @param[out] _enum enum value
/// @retval true
/// @retval false
/// @note
///////////////////////////////////////////////////////////
static bool ToEnum(int _value, eBasicState& _enum)
{
    switch (_value) {
    case eSTATE_IDLE:
        _enum = eSTATE_IDLE;
        return true;
    case eSTATE_STAND:
        _enum = eSTATE_STAND;
        return true;
    case eSTATE_WALK:
        _enum = eSTATE_WALK;
        return true;
    default:
        return false;
    }
}
} // namespace types
} // namespace common_library

#endif // EBAISC_STATE_H
