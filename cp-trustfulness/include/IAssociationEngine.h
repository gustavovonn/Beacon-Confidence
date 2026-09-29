#ifndef I_ASSOCIATION_ENGINE_H_
#define I_ASSOCIATION_ENGINE_H_

#include "CpmTypes.h"
#include <string>
#include <vector>

namespace artery {

struct AssociationContext {
    const std::vector<ProjectedTrack>& tracks;
    const std::vector<ObjectClaim>& objects;
    const AssociationParams& params;
};

/// Resolves anonymous object claims to CAM identities. Objects left unmatched stay unresolved.
class IAssociationEngine {
public:
    virtual ~IAssociationEngine() = default;
    virtual std::vector<AssociationMatch> associate(const AssociationContext& ctx) = 0;
    virtual std::string getName() const = 0;
};

}  // namespace artery

#endif  // I_ASSOCIATION_ENGINE_H_
