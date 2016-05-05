#ifndef PARAM_CALLBACK_PARAM_CALLBACK_HPP
#define PARAM_CALLBACK_PARAM_CALLBACK_HPP

// standard headers
#include <string>

// boost headers
#include <boost/function.hpp>

// xml rpc headers
#include <XmlRpcValue.h>

// ros headers
#include <ros/ros.h>


namespace param_callback
{


/** The function signature for on change notifications. */
typedef boost::function<void (
    XmlRpc::XmlRpcValue,
    XmlRpc::XmlRpcValue)> ChangeCallbackFn;


/**
 * \brief The ParamCallback class
 *
 * This class takes the name of a ROS parameter to monitor for changes
 * and calls a callback function whenever the parameter:
 *
 *     - is found
 *     - is changed
 *     - is deleted
 *
 * The callback function is passed the previous and new/current value
 * of the parameter to allow logic to be executed for this change event.
 */
class ParamCallback
{
public:
    /**
     * \brief The constructor
     *
     * Create a ParamCallback object which monitors a given parameter
     * for changes. This function takes a callback function that will
     * be called whenever the given ROS parameter changes values. This
     * function uses the default ros::NodeHandle when accessing the parameter.
     *
     * \param[in] param is the name of the parameter to monitor
     * \param[in] onChangeCallback is the function that gets called when
     *            the parameter changes values
     * \throws std::runtime_error if param is an empty string
     * \throws std::runtime_error if onChangeCallback is not a valid function
     */
    ParamCallback(
        const std::string& param,
        ChangeCallbackFn onChangeCallback);

    /**
     * \brief The constructor
     *
     * Create a ParamCallback object which monitors a given parameter
     * for changes. This function takes a callback function that will
     * be called whenever the given ROS parameter changes values.
     *
     * \param[in] nh is the ros::NodeHandle to use
     * \param[in] param is the name of the parameter to monitor
     * \param[in] onChangeCallback is the function that gets called when
     *            the parameter changes values
     * \throws std::runtime_error if param is an empty string
     * \throws std::runtime_error if onChangeCallback is not a valid function
     */
    ParamCallback(
        ros::NodeHandle& nh,
        const std::string& param,
        ChangeCallbackFn onChangeCallback);

    /**
     * \brief Monitor the parameter for changes
     *
     * Monitor the parameter for changes and notify the callback
     * function if the parameter's value has changes since the last update.
     */
    void update();

private:
    /**
     * \brief Validate the given parameters.
     *
     * Validate the given parameters.
     *
     * \throws std::runtime_error if param is an empty string
     * \throws std::runtime_error if onChangeCallback is not a valid function
     */
    void validate();

    /** The standard ROS node handle. */
    ros::NodeHandle m_nh;

    /** The parameter to monitor. */
    std::string m_param;

    /** The function called when the parameter changes value. */
    ChangeCallbackFn m_onChangeCallback;

    /** True if the parameter was found during the last cycle. */
    bool m_hasParam;

    /** The value of the parameter last cycle. */
    XmlRpc::XmlRpcValue m_prevParam;

};


}  // end of the param_callback namespace


#endif // PARAM_CALLBACK_PARAM_CALLBACK_HPP
