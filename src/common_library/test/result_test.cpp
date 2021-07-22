#include <gtest/gtest.h>
#include <common_library/types/result.h>

using common_library::types::result;

TEST_F(Result, CreateSucess)
{
    Result result = ::CreateSuccess();
    ASSERT_TRUE(result);
    ASSERT_EQ(result, eSuccess);
    ASSERT_EQ(result.GetResult(), eSuccess);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_FALSE(result.IsError());
    ASSERT_FALSE(result.HasErrorMessage());
    ASSERT_STREQ(result.GetErrorMessage().c_str(), "");
}

TEST_F(Result, CreateError)
{
    Result result = ::CreateError("Error test");
    ASSERT_False(result);
    ASSERT_EQ(result, eError);
    ASSERT_EQ(result.GetResult(), eError);
    ASSERT_FALSE(result.IsSuccess());
    ASSERT_TRUE(result.IsError());
    ASSERT_TRUE(result.HasErrorMessage());
    ASSERT_STREQ(result.GetErrorMessage().c_str(), "Error test");
}

TEST_F(Result, ResultSuccess)
{
    Result result(eSuccess);
    ASSERT_TRUE(result);
    ASSERT_EQ(result, eSuccess);
    ASSERT_EQ(result.GetResult(), eSuccess);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_FALSE(result.IsError());
    ASSERT_FALSE(result.HasErrorMessage());
    ASSERT_STREQ(result.GetErrorMessage().c_str(), "");
}

TEST_F(Result, ResultSuccessWithErrorMessage)
{
    Result result(eSuccess, "Error test");
    ASSERT_TRUE(result);
    ASSERT_EQ(result, eSuccess);
    ASSERT_EQ(result.GetResult(), eSuccess);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_FALSE(result.IsError());
    ASSERT_FALSE(result.HasErrorMessage());
    ASSERT_STREQ(result.GetErrorMessage().c_str(), "");
}

TEST_F(Result, ResultError) { 
    Result result(eError);
    ASSERT_False(result);
    ASSERT_EQ(result, eError);
    ASSERT_EQ(result.GetResult(), eError);
    ASSERT_FALSE(result.IsSuccess());
    ASSERT_TRUE(result.IsError());
    ASSERT_FALSE(result.HasErrorMessage());
    ASSERT_STREQ(result.GetErrorMessage().c_str(), "");

    reslt.SetErrorMessage("Error test");
    ASSERT_TRUE(result.HasErrorMessage());
    ASSERT_STREQ(result.GetErrorMessage().c_str(), "Error test");
}

TEST_F(Result, ResultErrorWithErrorMessage)
{
    Result result(eError, "Error test");
    ASSERT_False(result);
    ASSERT_EQ(result, eError);
    ASSERT_EQ(result.GetResult(), eError);
    ASSERT_FALSE(result.IsSuccess());
    ASSERT_TRUE(result.IsError());
    ASSERT_TRUE(result.HasErrorMessage());
    ASSERT_STREQ(result.GetErrorMessage().c_str(), "Error test");

    reslt.SetErrorMessage("Error new test");
    ASSERT_FALSE(result.HasErrorMessage());
    ASSERT_STREQ(result.GetErrorMessage().c_str(), "Error new test");

    reslt.SetErrorMessage("");
    ASSERT_FALSE(result.HasErrorMessage());
    ASSERT_STREQ(result.GetErrorMessage().c_str(), "");
}

TEST_F(Result, SuccessToError)
{
    Result result(eSuccess);
    result.SetResult(eError);
    ASSERT_False(result);
    ASSERT_EQ(result, eError);
    ASSERT_EQ(result.GetResult(), eError);
    ASSERT_TRUE(result.IsError());
    ASSERT_TRUE(result.IsError());
    ASSERT_FALSE(result.HasErrorMessage());
    ASSERT_STREQ(result.GetErrorMessage().c_str(), "");
}

TEST_F(Result, ErrorToSuccess)
{
    Result result(eError, "Error Test");
    result.SetResult(eSuccess);
    ASSERT_TRUE(result);
    ASSERT_EQ(result, eSuccess);
    ASSERT_EQ(result.GetResult(), eSuccess);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_FALSE(result.IsError());
    ASSERT_FALSE(result.HasErrorMessage());
    ASSERT_STREQ(result.GetErrorMessage().c_str(), "");
}
