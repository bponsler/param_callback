// system headers
#include <map>
#include <string>
#include <vector>

// boost headers
#include <boost/any.hpp>

// project header
#include <param_callback/ParamCallback.hpp>

// gtest includes
#include <gtest/gtest.h>


bool g_shouldNotCall;
bool g_paramSet;
bool g_paramDeleted;

std::string g_previousParamType;
boost::any g_previousValue;
std::string g_currentParamType;
boost::any g_currentValue;

// More complex types to test
typedef std::vector<std::string> StringVector;
typedef std::map<std::string, std::string> StringMap;


/**
 * \brief Test the value of a param
 *
 * Test the value of a param.
 *
 * \param[in] paramType is the type of the parameter
 * \param[in] paramValue is the value of the parameter
 * \param[in] value is the corresponding XmlRpcValue
 */
void testParam(
    std::string& paramType,
    boost::any& paramValue,
    XmlRpc::XmlRpcValue& value)
{
    if (paramType == "int")
    {
        EXPECT_EQ(
            boost::any_cast<int>(paramValue),
            static_cast<int>(value));
    }
    else if (paramType == "string")
    {
        EXPECT_EQ(
            boost::any_cast<std::string>(paramValue),
            static_cast<std::string>(value));
    }
    else if (paramType == "double")
    {
        EXPECT_EQ(
            boost::any_cast<double>(paramValue),
            static_cast<double>(value));
    }
    else if (paramType == "bool")
    {
        EXPECT_EQ(
            boost::any_cast<bool>(paramValue),
            static_cast<bool>(value));
    }
    else if (paramType == "StringVector")
    {
        StringVector list1 = boost::any_cast<StringVector>(paramValue);
        ASSERT_EQ(value.getType(), XmlRpc::XmlRpcValue::TypeArray);

        // All list items should be identical
        ASSERT_EQ(list1.size(), value.size());
        for (int i = 0; i < list1.size(); ++i) {
            EXPECT_EQ(list1[i], static_cast<std::string>(value[i]));
        }
    }
    else if (paramType == "StringMap")
    {
        StringMap map1 = boost::any_cast<StringMap>(paramValue);
        ASSERT_EQ(value.getType(), XmlRpc::XmlRpcValue::TypeStruct);

        // All map items should be identical
        ASSERT_EQ(map1.size(), value.size());
        XmlRpc::XmlRpcValue::iterator it;
        for (it = value.begin(); it != value.end(); it++)
        {
            const std::string& key = it->first;
            XmlRpc::XmlRpcValue& data = it->second;

            ASSERT_TRUE(map1.count(key) > 0);
            ASSERT_EQ(map1[key], static_cast<std::string>(data));
        }
    }
}

/**
 * \brief A ParamCallback function
 *
 * Callback to test parameter changes.
 *
 * \param[in] prev is the previous value of the parameter
 * \param[in] curr is the curretn value of the parameter
 */
void onCtorParam(
    XmlRpc::XmlRpcValue prev,
    XmlRpc::XmlRpcValue curr)
{
    if (g_shouldNotCall) {
        ADD_FAILURE() << "Unexpected setting of parameter" << std::endl;
    }
    else if (g_paramSet)
    {
        // Only the current value should be valid
        EXPECT_FALSE(prev.valid());
        EXPECT_TRUE(curr.valid());
        testParam(g_currentParamType, g_currentValue, curr);
    }
    else if (g_paramDeleted)
    {
        // Only the previous value should be valid
        EXPECT_TRUE(prev.valid());
        EXPECT_FALSE(curr.valid());
        testParam(g_previousParamType, g_previousValue, prev);
    }
    else
    {
        // Previous and current values should both be valid
        EXPECT_TRUE(prev.valid());
        EXPECT_TRUE(curr.valid());
        testParam(g_currentParamType, g_currentValue, curr);
        testParam(g_previousParamType, g_previousValue, prev);
    }
}

TEST(ParamCallbackTestSuite, testConstructor)
{
    ros::NodeHandle nh;

    std::string param = "some_param";
    nh.deleteParam(param);  // Just to be sure

    param_callback::ParamCallback pc(param, &onCtorParam);

    // The parameter is not set, and should not the callback
    g_shouldNotCall = true;
    g_paramSet = false;
    g_paramDeleted = false;
    pc.update();

    // Set the parameter value
    g_currentValue = 123;
    g_currentParamType = "int";
    nh.setParam(param, boost::any_cast<int>(g_currentValue));
    g_shouldNotCall = false;

    // Parameter was set for the first time
    g_shouldNotCall = false;
    g_paramSet = true;
    g_paramDeleted = false;
    pc.update();
    g_paramSet = false;

    // Update the parameter value
    g_previousValue = g_currentValue;
    g_previousParamType = g_currentParamType;
    g_currentValue = 321;
    g_currentParamType = "int";
    nh.setParam(param, boost::any_cast<int>(g_currentValue));

    // Parameter was set for the second time
    pc.update();

    // Delete the parameter
    nh.deleteParam(param);

    // Parameter was deleted
    g_paramDeleted = true;
    g_previousValue = g_currentValue;
    g_previousParamType = g_currentParamType;
    g_currentValue = -1;
    g_currentParamType = "";
    pc.update();

    // The parameter is not set, and should not the callback
    g_shouldNotCall = true;
    g_paramSet = false;
    g_paramDeleted = false;
    pc.update();
}

TEST(ParamCallbackTestSuite, testNhConstructor)
{
    ros::NodeHandle nh("some/namespace/to/test");

    std::string param = "a_very_different_parameter";
    nh.deleteParam(param);  // Just to be sure

    param_callback::ParamCallback pc(nh, param, &onCtorParam);

    // The parameter is not set, and should not the callback
    g_shouldNotCall = true;
    g_paramSet = false;
    g_paramDeleted = false;
    pc.update();

    // Set the parameter value
    g_currentValue = std::string("hello");
    g_currentParamType = "string";
    nh.setParam(param, boost::any_cast<std::string>(g_currentValue));
    g_shouldNotCall = false;

    // Parameter was set for the first time
    g_shouldNotCall = false;
    g_paramSet = true;
    g_paramDeleted = false;
    pc.update();
    g_paramSet = false;

    // Update the parameter value
    g_previousValue = g_currentValue;
    g_previousParamType = g_currentParamType;
    g_currentValue = std::string("goodbye");
    g_currentParamType = "string";
    nh.setParam(param, boost::any_cast<std::string>(g_currentValue));

    // Parameter was set for the second time
    pc.update();

    // Delete the parameter
    nh.deleteParam(param);

    // Parameter was deleted
    g_paramDeleted = true;
    g_previousValue = g_currentValue;
    g_previousParamType = g_currentParamType;
    g_currentValue = std::string("");
    g_currentParamType = "";
    pc.update();

    // The parameter is not set, and should not the callback
    g_shouldNotCall = true;
    g_paramSet = false;
    g_paramDeleted = false;
    pc.update();
}

TEST(ParamCallbackTestSuite, testConstructorInvalidParam)
{
    try
    {
        param_callback::ParamCallback pc("", &onCtorParam);
        ADD_FAILURE() << "Expected error when using empty param";
    }
    catch (...) { }  // Expected error

    try
    {
        ros::NodeHandle nh("/some/random/namespace");
        param_callback::ParamCallback pc(nh, "", &onCtorParam);
        ADD_FAILURE() << "Expected error when using empty param";
    }
    catch (...) { }  // Expected error
}

TEST(ParamCallbackTestSuite, testConstructorInvalidCallback)
{
    try
    {
        param_callback::ParamCallback pc(
            "param", param_callback::ChangeCallbackFn());
        ADD_FAILURE() << "Expected error when using invalid callback";
    }
    catch (...) { }  // Expected error

    try
    {
        ros::NodeHandle nh("/some/random/namespace");
        param_callback::ParamCallback pc(
            nh, "param", param_callback::ChangeCallbackFn());
        ADD_FAILURE() << "Expected error when using invalid callback";
    }
    catch (...) { }  // Expected error
}

TEST(ParamCallbackTestSuite, testChangingType)
{
    ros::NodeHandle nh;

    std::string param = "param_that_changes_types";
    nh.deleteParam(param);  // Just to be sure

    param_callback::ParamCallback pc(param, &onCtorParam);

    // The parameter is not set, and should not the callback
    g_shouldNotCall = true;
    g_paramSet = false;
    g_paramDeleted = false;
    pc.update();

    // Set the parameter value as an int to start
    g_currentValue = 123;
    g_currentParamType = "int";
    nh.setParam(param, boost::any_cast<int>(g_currentValue));
    g_shouldNotCall = false;

    // Parameter was set for the first time
    g_shouldNotCall = false;
    g_paramSet = true;
    g_paramDeleted = false;
    pc.update();
    g_paramSet = false;

    // Update the parameter value to a string
    g_previousValue = g_currentValue;
    g_previousParamType = g_currentParamType;
    g_currentValue = std::string("abcdefg");
    g_currentParamType = "string";
    nh.setParam(param, boost::any_cast<std::string>(g_currentValue));
    pc.update();

    // Update the parameter value to a double
    g_previousValue = g_currentValue;
    g_previousParamType = g_currentParamType;
    g_currentValue = 5.0;
    g_currentParamType = "double";
    nh.setParam(param, boost::any_cast<double>(g_currentValue));
    pc.update();

    // Update the parameter value to a bool
    g_previousValue = g_currentValue;
    g_previousParamType = g_currentParamType;
    g_currentValue = true;
    g_currentParamType = "bool";
    nh.setParam(param, boost::any_cast<bool>(g_currentValue));
    pc.update();

    // Update the parameter value to a list
    StringVector list;
    list.push_back("one");
    list.push_back("two");

    g_previousValue = g_currentValue;
    g_previousParamType = g_currentParamType;
    g_currentValue = list;
    g_currentParamType = "StringVector";
    nh.setParam(param, boost::any_cast<StringVector>(g_currentValue));
    pc.update();

    // Update the parameter value to a map of strings
    StringMap strMap;
    strMap["hello"] = "abcdefg";
    strMap["goodbye"] = "gfedcba";
    strMap["another"] = "another";
    strMap["fifteen"] = "hundred";

    g_previousValue = g_currentValue;
    g_previousParamType = g_currentParamType;
    g_currentValue = strMap;
    g_currentParamType = "StringMap";
    nh.setParam(param, boost::any_cast<StringMap>(g_currentValue));
    pc.update();

    // Delete the parameter
    nh.deleteParam(param);

    // Parameter was deleted
    g_paramDeleted = true;
    g_previousValue = g_currentValue;
    g_previousParamType = g_currentParamType;
    g_currentValue = -1;
    g_currentParamType = "";
    pc.update();

    // The parameter is not set, and should not the callback
    g_shouldNotCall = true;
    g_paramSet = false;
    g_paramDeleted = false;
    pc.update();
}

// Run all tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "param_callback_tests");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
