#include <param_callback/ParamCallback.hpp>


namespace param_callback
{


ParamCallback::ParamCallback(
    const std::string& param,
    ChangeCallbackFn onChangeCallback)
    :
    m_param(param),
    m_onChangeCallback(onChangeCallback),
    m_hasParam(false)
{
    validate();
}

ParamCallback::ParamCallback(
    ros::NodeHandle& nh,
    const std::string& param,
    ChangeCallbackFn onChangeCallback)
    :
    m_nh(nh),
    m_param(param),
    m_onChangeCallback(onChangeCallback),
    m_hasParam(false)
{
    validate();
}

void
ParamCallback::validate()
{
    // Make sure we have a valid parameter name to monitor
    if (m_param.size() == 0)
    {
        throw std::runtime_error(
            "ParamCallback requires a non-empty parameter!");
    }

    // It does not make sense to use this class without a callback
    // function -- so do not allow it
    if (!m_onChangeCallback)
    {
        throw std::runtime_error(
            "ParamCallaback requires a valid function to call when "
            "the parameter changes!");
    }
}

void
ParamCallback::update()
{
    // Determine if the parameter exists
    bool hasParam = m_nh.hasParam(m_param);

    // Check if the parameter was found or lost, or did not change state
    if (hasParam != m_hasParam)
    {
        //// Parameter was found or lost
        XmlRpc::XmlRpcValue newValue;
        if (hasParam) {
            m_nh.getParam(m_param, newValue);  // The parameter was found
        }

        // Call the on change function, and store the new parameter value
        m_onChangeCallback(m_prevParam, newValue);
        m_prevParam = newValue;

        m_hasParam = hasParam;
    }
    else if (hasParam)  // Wait for the parameter to exist...
    {
        //// Check if the value changed
        XmlRpc::XmlRpcValue newValue;
        m_nh.getParam(m_param, newValue);

        if (m_prevParam != newValue)
        {
            //// The value has changed -- notify
            m_onChangeCallback(m_prevParam, newValue);
            m_prevParam = newValue;
        }
    }
}


}  // end of the param_callback namespace
