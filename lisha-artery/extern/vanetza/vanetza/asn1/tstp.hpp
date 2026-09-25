#ifndef TSTP_HPP_WEWZK69S
#define TSTP_HPP_WEWZK69S

#include <vanetza/asn1/asn1c_conversion.hpp>
#include <vanetza/asn1/asn1c_wrapper.hpp>
#include <vanetza/asn1/its/TSTP.h>

namespace vanetza
{
namespace asn1
{

class Tstp : public asn1c_per_wrapper<TSTP_t>
{
public:
    Tstp() : asn1c_per_wrapper(asn_DEF_TSTP) {}
};

} // namespace asn1
} // namespace vanetza

#endif /* CPM_HPP_WEWZK69S */

