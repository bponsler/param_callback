# param_callback

The param_callback package provides the param_callback::ParamCallback class that is
capable of calling a specified callback function whenever a ROS parameter changes.

This callback function will be called under the following circumstances:

- the parameter's value is set for the first time
- the parameter's value is set to a different value
- the parameter is deleted

The signature for the callback function is as follows:

``` c++
void callback(XmlRpc::XmlRpcValue prev, XmlRpc::XmlRpcValue current);
```

Where "prev" is the parameter's previous value, and "current" is the parameter's
new/current value.

When the parameter is found for the first time the "prev" value will be invalid
(i.e., prev.valid() == false), and when the parameter is deleted the "current"
value will be invalid (i.e., current.valid() == false).


## Examples

Here are some examples of how to use the param_callback::ParamCallback class:

``` c++
#include <param_callback/ParamCallback.hpp>

void onChange(XmlRpc::XmlRpcValue prev, XmlRpc::XmlRpcValue current)
{
    if (!prev.valid()) {
        ROS_INFO("The param was found!");
    }
    else if (!current.valid()) {
        ROS_INFO("The param was lost!");
    }
    else {
        ROS_INFO("The param changed value!");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "non_class_example_param_callback");

    ros::NodeHandle nh;

    param_callback::ParamCallback pc(
        nh,
        "my_param",
        &onChange);

    ros::Rate rate(1);
    while (ros::ok())
    {
        pc.update();  // Monitor for changes to the parameter

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
```


This example demonstrates how to use the param_callback::ParamCallback class
with non-class functions.

The next example demonstrates how to use class functions:

``` c++
#include <param_callback/ParamCallback.hpp>

class Test
{
public:
    void onChange(XmlRpc::XmlRpcValue prev, XmlRpc::XmlRpcValue current)
    {
        if (!prev.valid()) {
            ROS_INFO("The param was found!");
        }
        else if (!current.valid()) {
            ROS_INFO("The param was lost!");
        }
        else {
            ROS_INFO("The param changed value!");
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "class_example_param_callback");

    ros::NodeHandle nh;

    Test t;
    param_callback::ParamCallback pc(
        nh,
        "my_param",
        boost::bind(&Test::onChange, t, _1, _2));

    ros::Rate rate(1);
    while (ros::ok())
    {
        pc.update();  // Monitor for changes to the parameter

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
```