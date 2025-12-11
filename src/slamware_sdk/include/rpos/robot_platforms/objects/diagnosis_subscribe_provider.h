#pragma once

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/noncopyable.hpp>
  
#include <rpos/robot_platforms/slamware_sdp_platform_config.h>
#include <rpos/message/message.h>

namespace rpos { namespace robot_platforms { namespace objects {

    template<typename SubType>
    class DiagnosisSubscribeProviderImpl;

    template<typename SubType>
    class RPOS_SLAMWARE_API DiagnosisSubscribeProvider : private boost::noncopyable
    {
    public:  
        DiagnosisSubscribeProvider(boost::shared_ptr<DiagnosisSubscribeProviderImpl<SubType>> impl);
        ~DiagnosisSubscribeProvider();

        bool readMessages(std::vector<rpos::message::Message<SubType>>& messages);  
    private: 
        boost::shared_ptr<DiagnosisSubscribeProviderImpl<SubType>> m_pImpl;
    };

} } }