///////////////////////////////////////////////////////////
/// @file	result_test.cpp
/// @brief	unit test for result
/// @author	henaiguo
/// Copyright (C) 2021- henaiguo All rights reserved.
///////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <common_library/types/result.h>
#include <common_library/types/e_result.h>

using common_library::types::Result;

using namespace common_library::types;

TEST(Result, CreateSucess)
{
    Result result = Result::CreateSuccess();
    EXPECT_TRUE(result);
    EXPECT_EQ(result, eSUCCESS);
    EXPECT_EQ(result.GetResult(), eSUCCESS);
    EXPECT_TRUE(result.IsSuccess());
    EXPECT_FALSE(result.IsError());
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "");
}

TEST(Result, CreateError)
{
    Result result = Result::CreateError("Error test");
    EXPECT_FALSE(result);
    EXPECT_EQ(result, eERROR);
    EXPECT_EQ(result.GetResult(), eERROR);
    EXPECT_FALSE(result.IsSuccess());
    EXPECT_TRUE(result.IsError());
    EXPECT_TRUE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "Error test");
}

TEST(Result, ResultSuccess)
{
    Result result(eSUCCESS);
    EXPECT_TRUE(result);
    EXPECT_EQ(result, eSUCCESS);
    EXPECT_EQ(result.GetResult(), eSUCCESS);
    EXPECT_TRUE(result.IsSuccess());
    EXPECT_FALSE(result.IsError());
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "");
}

TEST(Result, ResultSuccessWithErrorMessage)
{
    Result result(eSUCCESS, "Error test");
    EXPECT_TRUE(result);
    EXPECT_EQ(result, eSUCCESS);
    EXPECT_EQ(result.GetResult(), eSUCCESS);
    EXPECT_TRUE(result.IsSuccess());
    EXPECT_FALSE(result.IsError());
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "");
}

TEST(Result, ResultError) { 
    Result result(eERROR);
    EXPECT_FALSE(result);
    EXPECT_EQ(result, eERROR);
    EXPECT_EQ(result.GetResult(), eERROR);
    EXPECT_FALSE(result.IsSuccess());
    EXPECT_TRUE(result.IsError());
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "");

    result.SetErrorMessage("Error test");
    EXPECT_TRUE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "Error test");
}

TEST(Result, ResultErrorWithErrorMessage)
{
    Result result(eERROR, "Error test");
    EXPECT_FALSE(result);
    EXPECT_EQ(result, eERROR);
    EXPECT_EQ(result.GetResult(), eERROR);
    EXPECT_FALSE(result.IsSuccess());
    EXPECT_TRUE(result.IsError());
    EXPECT_TRUE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "Error test");

    result.SetErrorMessage("Error new test");
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "Error new test");

    result.SetErrorMessage("");
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "");
}

TEST(Result, SuccessToError)
{
    Result result(eSUCCESS);
    result.SetResult(eERROR);
    EXPECT_FALSE(result);
    EXPECT_EQ(result, eERROR);
    EXPECT_EQ(result.GetResult(), eERROR);
    EXPECT_TRUE(result.IsError());
    EXPECT_TRUE(result.IsError());
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "");
}

TEST(Result, ErrorToSuccess)
{
    Result result(eERROR, "Error Test");
    result.SetResult(eSUCCESS);
    EXPECT_TRUE(result);
    EXPECT_EQ(result, eSUCCESS);
    EXPECT_EQ(result.GetResult(), eSUCCESS);
    EXPECT_TRUE(result.IsSuccess());
    EXPECT_FALSE(result.IsError());
    EXPECT_FALSE(result.HasErrorMessage());
    EXPECT_STREQ(result.GetErrorMessage().c_str(), "");
}