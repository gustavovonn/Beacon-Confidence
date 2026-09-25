#ifndef Pcm_HPP_WEWZK69S1
#define Pcm_HPP_WEWZK69S1

#include <vanetza/asn1/asn1c_conversion.hpp>
#include <vanetza/asn1/asn1c_wrapper.hpp>
#include <vanetza/asn1/its/PCM.h>

namespace vanetza
{
namespace asn1
{

class Pcm : public asn1c_per_wrapper<PCM_t>
{
public:
    Pcm() : asn1c_per_wrapper(asn_DEF_PCM) {}
};

} // namespace asn1
} // namespace vanetza

#endif /* Pcm_HPP_WEWZK69S1 */

