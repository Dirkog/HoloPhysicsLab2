#include "BoundaryConditions.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace hlp {

// ==================== SDF ====================

double SDF::eval_node(const SDFNode* node, const dvec3& point) const {
    if (!node) return std::numeric_limits<double>::max();
    
    switch (node->type) {
        case SDFNode::SPHERE: {
            return glm::length(point - node->center) - node->radius;
        }
        case SDFNode::BOX: {
            dvec3 diff = point - node->center;
            dvec3 q = dvec3(std::abs(diff.x), std::abs(diff.y), std::abs(diff.z)) - node->half_extents;
            return glm::length(dvec3(std::max(q.x, 0.0), std::max(q.y, 0.0), std::max(q.z, 0.0)))
                 + std::min({std::max(q.x, std::max(q.y, q.z)), 0.0});
        }
        case SDFNode::PLANE: {
            return glm::dot(point, node->normal) - node->offset;
        }
        case SDFNode::CYLINDER: {
            dvec2 d(std::abs(glm::length(dvec2(point.x - node->center.x, point.z - node->center.z)) - node->radius),
                    std::abs(point.y - node->center.y) - node->height * 0.5);
            return std::min(std::max(d.x, d.y), 0.0) + glm::length(dvec2(std::max(d.x, 0.0), std::max(d.y, 0.0)));
        }
        case SDFNode::TORUS: {
            dvec2 q(glm::length(dvec2(point.x - node->center.x, point.z - node->center.z)) - node->major,
                    point.y - node->center.y);
            return glm::length(q) - node->minor;
        }
        case SDFNode::UNION: {
            double a = eval_node(node->left.get(), point);
            double b = eval_node(node->right.get(), point);
            return std::min(a, b);
        }
        case SDFNode::INTERSECT: {
            double a = eval_node(node->left.get(), point);
            double b = eval_node(node->right.get(), point);
            return std::max(a, b);
        }
        case SDFNode::SUBTRACT: {
            double a = eval_node(node->left.get(), point);
            double b = eval_node(node->right.get(), point);
            return std::max(a, -b);
        }
    }
    return 0;
}

double SDF::distance(const dvec3& point) const {
    return eval_node(root_.get(), point);
}

dvec3 SDF::gradient(const dvec3& point) const {
    // Central differences
    double eps = 1e-6;
    dvec3 grad;
    grad.x = (distance(point + dvec3(eps, 0, 0)) - distance(point - dvec3(eps, 0, 0))) / (2 * eps);
    grad.y = (distance(point + dvec3(0, eps, 0)) - distance(point - dvec3(0, eps, 0))) / (2 * eps);
    grad.z = (distance(point + dvec3(0, 0, eps)) - distance(point - dvec3(0, 0, eps))) / (2 * eps);
    
    double len = glm::length(grad);
    if (len > 1e-10) grad /= len;
    return grad;
}

void SDF::add_sphere(const dvec3& center, double radius) {
    auto node = std::make_unique<SDFNode>();
    node->type = SDFNode::SPHERE;
    node->center = center;
    node->radius = radius;
    
    if (!root_) { root_ = std::move(node); return; }
    auto union_node = std::make_unique<SDFNode>();
    union_node->type = SDFNode::UNION;
    union_node->left = std::move(root_);
    union_node->right = std::move(node);
    root_ = std::move(union_node);
}

void SDF::add_box(const dvec3& center, const dvec3& half_extents) {
    auto node = std::make_unique<SDFNode>();
    node->type = SDFNode::BOX;
    node->center = center;
    node->half_extents = half_extents;
    if (!root_) { root_ = std::move(node); return; }
    auto u = std::make_unique<SDFNode>();
    u->type = SDFNode::UNION; u->left = std::move(root_); u->right = std::move(node);
    root_ = std::move(u);
}

void SDF::add_plane(const dvec3& normal, double offset) {
    auto node = std::make_unique<SDFNode>();
    node->type = SDFNode::PLANE; node->normal = normal; node->offset = offset;
    if (!root_) { root_ = std::move(node); return; }
    auto u = std::make_unique<SDFNode>();
    u->type = SDFNode::UNION; u->left = std::move(root_); u->right = std::move(node);
    root_ = std::move(u);
}

void SDF::add_cylinder(const dvec3& center, double radius, double height) {
    auto node = std::make_unique<SDFNode>();
    node->type = SDFNode::CYLINDER; node->center = center; node->radius = radius; node->height = height;
    if (!root_) { root_ = std::move(node); return; }
    auto u = std::make_unique<SDFNode>();
    u->type = SDFNode::UNION; u->left = std::move(root_); u->right = std::move(node);
    root_ = std::move(u);
}

void SDF::add_torus(const dvec3& center, double major, double minor) {
    auto node = std::make_unique<SDFNode>();
    node->type = SDFNode::TORUS; node->center = center; node->major = major; node->minor = minor;
    if (!root_) { root_ = std::move(node); return; }
    auto u = std::make_unique<SDFNode>();
    u->type = SDFNode::UNION; u->left = std::move(root_); u->right = std::move(node);
    root_ = std::move(u);
}

void SDF::clear() { root_.reset(); }

// ==================== BoundarySystem ====================

BoundarySystem::BoundarySystem() = default;
BoundarySystem::~BoundarySystem() = default;

void BoundarySystem::add_plane_boundary(const PlaneBoundary& boundary) {
    planes_.push_back(boundary);
}

void BoundarySystem::add_box_boundary(const BoxBoundary& boundary) {
    boxes_.push_back(boundary);
}

void BoundarySystem::add_sphere_boundary(const SphereBoundary& boundary) {
    spheres_.push_back(boundary);
}

void BoundarySystem::clear_boundaries() {
    planes_.clear();
    boxes_.clear();
    spheres_.clear();
    sdf_.reset();
}

void BoundarySystem::set_sdf(std::unique_ptr<SDF> sdf) {
    sdf_ = std::move(sdf);
}

void BoundarySystem::set_periodic_box(const dvec3& box_size) {
    periodic_enabled_ = true;
    periodic_box_size_ = box_size;
}

int BoundarySystem::active_boundary_count() const {
    return (int)(planes_.size() + boxes_.size() + spheres_.size() + (sdf_ ? 1 : 0));
}

void BoundarySystem::apply(std::vector<Body>& bodies, double dt) {
    for (auto& body : bodies) {
        if (body.is_static()) continue;
        apply_to_body(body, dt);
        
        // Periodic wrapping
        if (periodic_enabled_) apply_periodic_to_body(body);
    }
}

void BoundarySystem::apply_to_body(Body& body, double dt) {
    // SDF-based boundary
    if (sdf_) {
        double d = sdf_->distance(body.state.position);
        if (d < 0) {
            dvec3 n = sdf_->gradient(body.state.position);
            body.state.position += n * (-d);
            double vn = glm::dot(body.state.velocity, n);
            if (vn < 0) body.state.velocity -= (1 + 0.5) * vn * n;
        }
    }
    
    // Primitive boundaries
    for (const auto& b : planes_) apply_plane(body, b, dt);
    for (const auto& b : boxes_) apply_box(body, b, dt);
    for (const auto& b : spheres_) apply_sphere(body, b, dt);
}

void BoundarySystem::apply_plane(Body& body, const PlaneBoundary& b, double dt) {
    double d = glm::dot(body.state.position, b.normal) - b.offset;
    double penetration = -d + body.props.radius;
    
    if (penetration <= 0) return;
    
    switch (b.type) {
        case BoundaryType::REFLECTIVE: {
            // Position correction
            body.state.position += b.normal * penetration;
            // Velocity reflection with restitution
            double vn = glm::dot(body.state.velocity, b.normal);
            if (vn < 0) {
                body.state.velocity -= (1.0 + b.restitution) * vn * b.normal;
                // Friction
                dvec3 vt = body.state.velocity - glm::dot(body.state.velocity, b.normal) * b.normal;
                body.state.velocity -= vt * std::min(b.friction, 1.0);
            }
            break;
        }
        case BoundaryType::ABSORBING: {
            body.flags &= ~BODY_VISIBLE; // Mark for removal
            body.state.position += b.normal * (body.props.radius * 2 + 100); // Teleport away
            break;
        }
        case BoundaryType::THERMAL: {
            apply_plane(body, b, dt); // First reflect
            // Thermal coupling
            body.props.temperature += (b.temperature - body.props.temperature) * 0.1;
            break;
        }
        default: break;
    }
}

void BoundarySystem::apply_box(Body& body, const BoxBoundary& b, double dt) {
    for (int axis = 0; axis < 3; axis++) {
        dvec3 normal(0);
        normal[axis] = 1;
        
        double d_min = body.state.position[axis] - b.min[axis] - body.props.radius;
        double d_max = b.max[axis] - body.state.position[axis] - body.props.radius;
        
        if (d_min < 0) {
            PlaneBoundary pb;
            pb.normal = normal;
            pb.type = b.type;
            pb.restitution = b.restitution;
            pb.friction = b.friction;
            pb.offset = b.min[axis];
            apply_plane(body, pb, dt);
        }
        
        if (d_max < 0) {
            PlaneBoundary pb;
            pb.normal = -normal;
            pb.type = b.type;
            pb.restitution = b.restitution;
            pb.friction = b.friction;
            pb.offset = -b.max[axis];
            apply_plane(body, pb, dt);
        }
    }
}

void BoundarySystem::apply_sphere(Body& body, const SphereBoundary& b, double dt) {
    dvec3 delta = body.state.position - b.center;
    double dist = glm::length(delta);
    double penetration = b.radius - dist + body.props.radius;
    
    if (penetration <= 0 || dist < 1e-10) return;
    
    dvec3 normal = delta / dist;
    body.state.position += normal * penetration;
    
    double vn = glm::dot(body.state.velocity, normal);
    if (vn < 0) {
        body.state.velocity -= (1.0 + b.restitution) * vn * normal;
    }
}

void BoundarySystem::apply_periodic_to_body(Body& body) {
    body.state.position = apply_periodic(body.state.position);
}

dvec3 BoundarySystem::apply_periodic(const dvec3& pos) const {
    dvec3 result = pos;
    for (int i = 0; i < 3; i++) {
        while (result[i] < -periodic_box_size_[i] * 0.5) result[i] += periodic_box_size_[i];
        while (result[i] > periodic_box_size_[i] * 0.5) result[i] -= periodic_box_size_[i];
    }
    return result;
}

} // namespace hlp
