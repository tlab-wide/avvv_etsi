#include <visually/net_status.hpp>

namespace net_status
{

net_status::NetworkField intToNetworkField(int intput)
{
    switch (intput) {
        case 0: return net_status::delay;
        case 1: return net_status::jitter;
        case 2: return net_status::rssi;
        case 3: return net_status::packetLoss;
        default:
            return net_status::none;
    }
}

NetStatusRepr::NetStatusRepr(const NetParamRanges& ranges)
    : net_param_ranges_(ranges)
{

}

NetStatusRepr::~NetStatusRepr()
{
    
}

} // namespace net_status