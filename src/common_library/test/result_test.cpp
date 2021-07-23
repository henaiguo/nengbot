///////////////////////////////////////////////////////////
/// @file	result_test.cpp
/// @brief	unit test for result
/// @author	henaiguo
/// Copyright (C) 2021- henaiguo All rights reserved.
///////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <common_library/types/result.h>

using common_library::types::result;

namespace common_library {
namespace types {

TEST(Result, CreateSucess)
{
    Result result = ::CreateSuccess();
    EXPECT_TRUE(result);
    EXPECT_EQ(result, eSuccess);
    EXPECT_EQ(result.GetResult(), eSuccess);
    EXPECT_TRUE(result.IsSuccess());
    EXPECT_FALSE(result.IsError());
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "");
}

TEST(Result, CreateError)
{
    Result result = ::CreateError("Error test");
    EXPECT_False(result);
    EXPECT_EQ(result, eError);
    EXPECT_EQ(result.GetResult(), eError);
    EXPECT_FALSE(result.IsSuccess());
    EXPECT_TRUE(result.IsError());
    EXPECT_TRUE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "Error test");
}

TEST(Result, ResultSuccess)
{
    Result result(eSuccess);
    EXPECT_TRUE(result);
    EXPECT_EQ(result, eSuccess);
    EXPECT_EQ(result.GetResult(), eSuccess);
    EXPECT_TRUE(result.IsSuccess());
    EXPECT_FALSE(result.IsError());
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "");
}

TEST(Result, ResultSuccessWithErrorMessage)
{
    Result result(eSuccess, "Error test");
    EXPECT_TRUE(result);
    EXPECT_EQ(result, eSuccess);
    EXPECT_EQ(result.GetResult(), eSuccess);
    EXPECT_TRUE(result.IsSuccess());
    EXPECT_FALSE(result.IsError());
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "");
}

TEST(Result, ResultError) { 
    Result result(eError);
    EXPECT_False(result);
    EXPECT_EQ(result, eError);
    EXPECT_EQ(result.GetResult(), eError);
    EXPECT_FALSE(result.IsSuccess());
    EXPECT_TRUE(result.IsError());
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "");

    reslt.SetErrorMessage("Error test");
    EXPECT_TRUE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "Error test");
}

TEST(Result, ResultErrorWithErrorMessage)
{
    Result result(eError, "Error test");
    EXPECT_False(result);
    EXPECT_EQ(result, eError);
    EXPECT_EQ(result.GetResult(), eError);
    EXPECT_FALSE(result.IsSuccess());
    EXPECT_TRUE(result.IsError());
    EXPECT_TRUE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "Error test");

    reslt.SetErrorMessage("Error new test");
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "Error new test");

    reslt.SetErrorMessage("");
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "");
}

TEST(Result, SuccessToError)
{
    Result result(eSuccess);
    result.SetResult(eError);
    EXPECT_False(result);
    EXPECT_EQ(result, eError);
    EXPECT_EQ(result.GetResult(), eError);
    EXPECT_TRUE(result.IsError());
    EXPECT_TRUE(result.IsError());
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "");
}

TEST(Result, ErrorToSuccess)
{
    Result result(eError, "Error Test");
    result.SetResult(eSuccess);
    EXPECT_TRUE(result);
    EXPECT_EQ(result, eSuccess);
    EXPECT_EQ(result.GetResult(), eSuccess);
    EXPECT_TRUE(result.IsSuccess());
    EXPECT_FALSE(result.IsError());
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "");
}

} // namespace types
} // namespace common_library