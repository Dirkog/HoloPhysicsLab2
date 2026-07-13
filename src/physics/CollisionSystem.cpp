#include "CollisionSystem.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace hlp {

// ==================== Spatial Hash ====================

SpatialHash::SpatialHash(double cell_size) : cell_size_(cell_size) {}

uint64_t SpatialHash::hash_coord(int x, int y, int z) const {
    // Simple hash combining three coordinates
    return (uint64_t)(x * 73856093) ^ (uint64_t)(y * 19349663) ^ (uint64_t)(z * 83492791);
}

void SpatialHash::get_cells_for_body(const Body& b, std::vector<uint64_t>& cells) const {
    cells.clear();
    double r = b.props.radius;
    dvec3 p = b.state.position;
    
    int min_x = (int)std::floor((p.x - r) / cell_size_);
    int max_x = (int)std::floor((p.x + r) / cell_size_);
    int min_y = (int)std::floor((p.y - r) / cell_size_);
    int max_y = (int)std::floor((p.y + r) / cell_size_);
    int min_z = (int)std::floor((p.z - r) / cell_size_);
    int max_z = (int)std::floor((p.z + r) / cell_size_);
    
    for (int x = min_x; x <= max_x; x++)
        for (int y = min_y; y <= max_y; y++)
            for (int z = min_z; z <= max_z; z++)
                cells.push_back(hash_coord(x, y, z));
}

void SpatialHash::build(const std::vector<Body>& bodies) {
    grid_.clear();
    std::vector<uint64_t> cells;
    
    for (const auto& b : bodies) {
        if (b.is_static()) continue;
        get_cells_for_body(b, cells);
        
        // Remove duplicates
        std::sort(cells.begin(), cells.end());
        cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
        
        for (auto cell_hash : cells) {
            grid_[cell_hash].push_back(b.id);
        }
    }
}

void SpatialHash::get_potential_pairs(std::vector<CollisionPair>& pairs) const {
    pairs.clear();
    std::vector<std::pair<uint64_t, uint64_t>> seen;
    
    for (const auto& [hash, body_ids] : grid_) {
        for (size_t i = 0; i < body_ids.size(); i++) {
            for (size_t j = i + 1; j < body_ids.size(); j++) {
                uint64_t a = body_ids[i], b = body_ids[j];
                if (a > b) std::swap(a, b);
                seen.push_back({a, b});
            }
        }
    }
    
    // Remove duplicate pairs
    std::sort(seen.begin(), seen.end());
    seen.erase(std::unique(seen.begin(), seen.end()), seen.end());
    
    for (auto& [a, b] : seen) {
        pairs.push_back({a, b, 0});
    }
}

void SpatialHash::clear() { grid_.clear(); }

// ==================== BVH ====================

dvec3 BVH::compute_centroid(const std::vector<Body>& bodies,
                             const std::vector<int>& indices) {
    dvec3 c(0);
    for (int idx : indices) {
        if (idx >= 0 && idx < (int)bodies.size())
            c += bodies[idx].state.position;
    }
    if (!indices.empty()) c /= (double)indices.size();
    return c;
}

void BVH::compute_bounds(const std::vector<Body>& bodies,
                          const std::vector<int>& indices,
                          dvec3& min_b, dvec3& max_b) {
    min_b = dvec3(std::numeric_limits<double>::max());
    max_b = dvec3(std::numeric_limits<double>::lowest());
    
    for (int idx : indices) {
        if (idx < 0 || idx >= (int)bodies.size()) continue;
        const auto& p = bodies[idx].state.position;
        double r = bodies[idx].props.radius;
        min_b = glm::min(min_b, p - r);
        max_b = glm::max(max_b, p + r);
    }
}

void BVH::build(const std::vector<Body>& bodies) {
    nodes_.clear();
    if (bodies.empty()) { root_ = -1; return; }
    
    std::vector<int> indices(bodies.size());
    for (size_t i = 0; i < bodies.size(); i++) indices[i] = i;
    
    root_ = build_recursive(indices, 0);
}

int BVH::build_recursive(std::vector<int>& indices, int depth) {
    if (indices.empty()) return -1;
    
    BVHNode node;
    node.axis = depth % 3;
    
    if (indices.size() == 1) {
        node.body_id = indices[0];
        node.is_leaf = true;
        node.left_child = -1;
        node.right_child = -1;
        nodes_.push_back(node);
        return (int)nodes_.size() - 1;
    }
    
    // Sort along axis
    std::sort(indices.begin(), indices.end(),
        [&, axis = node.axis](int a, int b) {
            const auto& bodies = *reinterpret_cast<const std::vector<Body>*>(
                (const char*)this - offsetof(BVH, nodes_) + sizeof(BVH));
            return false; // Simplified: use a member variable instead
        });
    
    // For simplicity, use midpoint split
    int mid = (int)indices.size() / 2;
    std::vector<int> left_indices(indices.begin(), indices.begin() + mid);
    std::vector<int> right_indices(indices.begin() + mid, indices.end());
    
    node.left_child = build_recursive(left_indices, depth + 1);
    node.right_child = build_recursive(right_indices, depth + 1);
    node.is_leaf = false;
    
    nodes_.push_back(node);
    return (int)nodes_.size() - 1;
}

void BVH::clear() { nodes_.clear(); root_ = -1; }

// ==================== GJK ====================

dvec3 GJK::support_sphere(const Body& body, const dvec3& dir) const {
    dvec3 dir_n = glm::length(dir) > 1e-10 ? glm::normalize(dir) : dvec3(0, 1, 0);
    return body.state.position + dir_n * body.props.radius;
}

dvec3 GJK::support_box(const Body& body, const dvec3& dir) const {
    dvec3 dir_n = glm::length(dir) > 1e-10 ? glm::normalize(dir) : dvec3(0, 1, 0);
    dvec3 result = body.state.position;
    // Box with implicit half-extents = radius
    result.x += (dir_n.x > 0 ? 1 : -1) * body.props.radius;
    result.y += (dir_n.y > 0 ? 1 : -1) * body.props.radius;
    result.z += (dir_n.z > 0 ? 1 : -1) * body.props.radius;
    return result;
}

dvec3 GJK::support_cylinder(const Body& body, const dvec3& dir) const {
    dvec3 dir_n = glm::length(dir) > 1e-10 ? glm::normalize(dir) : dvec3(0, 1, 0);
    double r = body.props.radius;
    double hh = r * 0.5; // half-height proportional to radius
    dvec3 result = body.state.position;
    result.x += dir_n.x * r;
    result.z += dir_n.z * r;
    result.y += (dir_n.y > 0 ? 1 : -1) * hh;
    return result;
}

dvec3 GJK::support_torus(const Body& body, const dvec3& dir) const {
    dvec3 dir_n = glm::length(dir) > 1e-10 ? glm::normalize(dir) : dvec3(0, 1, 0);
    double major = body.props.radius * 0.7;
    double minor = body.props.radius * 0.3;
    
    // Find point on torus that maximizes dot with dir
    // Approximate: use a ring of radius major
    dvec3 dir_xy = glm::normalize(dvec3(dir_n.x, 0, dir_n.z));
    if (glm::length(dir_xy) < 1e-10) {
        return body.state.position + dvec3(0, dir_n.y > 0 ? minor : -minor, 0);
    }
    
    dvec3 ring_point = dvec3(dir_xy.x * major, 0, dir_xy.z * major);
    dvec3 to_ring = ring_point;
    dvec3 from_ring_to_dir = dir_n - glm::normalize(to_ring) * glm::dot(dir_n, glm::normalize(to_ring));
    
    dvec3 result = body.state.position + ring_point;
    if (glm::length(from_ring_to_dir) > 1e-10) {
        result += glm::normalize(from_ring_to_dir) * minor;
    }
    return result;
}

dvec3 GJK::support(const Body& body, const dvec3& dir) const {
    switch (body.visual.shape) {
        case ShapeType::Box:      return support_box(body, dir);
        case ShapeType::Cylinder: return support_cylinder(body, dir);
        case ShapeType::Torus:    return support_torus(body, dir);
        default:                  return support_sphere(body, dir);
    }
}

bool GJK::solve_simplex(std::vector<dvec3>& simplex, dvec3& dir) {
    if (simplex.size() == 2) {
        // Line case
        dvec3 a = simplex[0], b = simplex[1];
        dvec3 ab = b - a;
        dvec3 ao = -a;
        
        if (glm::dot(ab, ao) > 0) {
            dir = glm::cross(glm::cross(ab, ao), ab);
            if (glm::length(dir) < 1e-10) dir = glm::cross(ab, dvec3(1, 0, 0));
            if (glm::length(dir) < 1e-10) dir = glm::cross(ab, dvec3(0, 1, 0));
            dir = glm::normalize(dir);
        } else {
            simplex = {a};
            dir = glm::normalize(ao);
        }
        return false;
    }
    
    if (simplex.size() == 3) {
        // Triangle case
        dvec3 a = simplex[0], b = simplex[1], c = simplex[2];
        dvec3 ab = b - a, ac = c - a, ao = -a;
        dvec3 abc = glm::cross(ab, ac);
        
        if (glm::dot(glm::cross(abc, ac), ao) > 0) {
            if (glm::dot(ac, ao) > 0) {
                simplex = {a, c};
                dir = glm::cross(glm::cross(ac, ao), ac);
                if (glm::length(dir) < 1e-10) dir = glm::cross(ac, dvec3(1, 0, 0));
                dir = glm::normalize(dir);
            } else if (glm::dot(ab, ao) > 0) {
                simplex = {a, b};
                dir = glm::cross(glm::cross(ab, ao), ab);
                if (glm::length(dir) < 1e-10) dir = glm::cross(ab, dvec3(1, 0, 0));
                dir = glm::normalize(dir);
            } else {
                simplex = {a};
                dir = glm::normalize(ao);
            }
        } else if (glm::dot(glm::cross(ab, abc), ao) > 0) {
            if (glm::dot(ab, ao) > 0) {
                simplex = {a, b};
                dir = glm::cross(glm::cross(ab, ao), ab);
                if (glm::length(dir) < 1e-10) dir = glm::cross(ab, dvec3(1, 0, 0));
                dir = glm::normalize(dir);
            } else {
                simplex = {a};
                dir = glm::normalize(ao);
            }
        } else {
            return true; // Origin is inside triangle
        }
        return false;
    }
    
    if (simplex.size() >= 4) {
        // Tetrahedron case - check if origin is inside
        dvec3 a = simplex[0], b = simplex[1], c = simplex[2], d = simplex[3];
        
        auto same_side = [](const dvec3& p1, const dvec3& p2, 
                           const dvec3& a, const dvec3& b, const dvec3& c) {
            dvec3 n = glm::cross(b - a, c - a);
            double d1 = glm::dot(p1 - a, n);
            double d2 = glm::dot(p2 - a, n);
            return (d1 > 0) == (d2 > 0);
        };
        
        dvec3 origin(0);
        if (same_side(origin, a, b, c, d) &&
            same_side(origin, a, c, d, b) &&
            same_side(origin, a, b, d, c) &&
            same_side(origin, b, c, d, a)) {
            return true; // Origin is inside tetrahedron
        }
        
        // Fall back to triangle check
        simplex = {simplex[0], simplex[1], simplex[2]};
        return solve_simplex(simplex, dir);
    }
    
    return false;
}

bool GJK::intersect(const Body& a, const Body& b) {
    // Quick sphere-sphere check first
    dvec3 delta = b.state.position - a.state.position;
    double dist = glm::length(delta);
    double min_dist = a.props.radius + b.props.radius;
    if (dist > min_dist + 0.5) return false; // Early exit for distant bodies
    
    dvec3 dir(1, 0, 0);
    std::vector<dvec3> simplex;
    simplex.push_back(support(a, dir) - support(b, -dir));
    dir = -simplex[0];
    
    for (int iter = 0; iter < 32; iter++) {
        dvec3 p = support(a, dir) - support(b, -dir);
        if (glm::dot(p, dir) < 0) return false; // No intersection
        
        simplex.push_back(p);
        if (solve_simplex(simplex, dir)) return true;
    }
    
    return false;
}

bool GJK::get_contact(const Body& a, const Body& b,
                      dvec3& contact_point, dvec3& normal,
                      double& penetration) {
    // For sphere-sphere, use simplified formula
    if (a.visual.shape == ShapeType::Sphere && b.visual.shape == ShapeType::Sphere) {
        dvec3 delta = b.state.position - a.state.position;
        double dist = glm::length(delta);
        double min_dist = a.props.radius + b.props.radius;
        
        if (dist >= min_dist || dist < 1e-10) return false;
        
        penetration = min_dist - dist;
        normal = delta / dist;
        contact_point = a.state.position + normal * (a.props.radius - penetration * 0.5);
        return true;
    }
    
    // GJK + EPA for general shapes
    dvec3 dir(1, 0, 0);
    std::vector<dvec3> simplex;
    simplex.push_back(support(a, dir) - support(b, -dir));
    dir = -simplex[0];
    
    for (int iter = 0; iter < 32; iter++) {
        dvec3 p = support(a, dir) - support(b, -dir);
        if (glm::dot(p, dir) < 0) return false;
        simplex.push_back(p);
        if (solve_simplex(simplex, dir) && iter > 0) {
            return epa(simplex, contact_point, normal, penetration);
        }
    }
    
    return false;
}

bool GJK::epa(const std::vector<dvec3>& simplex,
              dvec3& contact_point, dvec3& normal, double& penetration) {
    // EPA - expanding polytope algorithm
    // Build initial polytope from simplex
    std::vector<dvec3> polytope = simplex;
    std::vector<std::array<int, 3>> faces;
    
    if (polytope.size() == 4) {
        faces = {{0,1,2}, {0,2,3}, {0,3,1}, {1,3,2}};
    } else {
        return false;
    }
    
    for (int iter = 0; iter < 64; iter++) {
        // Find closest face to origin
        double min_dist = std::numeric_limits<double>::max();
        int closest_face = -1;
        dvec3 closest_normal;
        
        for (size_t i = 0; i < faces.size(); i++) {
            auto& f = faces[i];
            dvec3 a = polytope[f[0]], b = polytope[f[1]], c = polytope[f[2]];
            dvec3 n = glm::normalize(glm::cross(b - a, c - a));
            double d = std::abs(glm::dot(n, a));
            
            if (d < min_dist) {
                min_dist = d;
                closest_face = i;
                closest_normal = n;
            }
        }
        
        if (closest_face < 0) break;
        
        // Check winding
        dvec3 center(0);
        for (auto& p : polytope) center += p;
        center /= (double)polytope.size();
        if (glm::dot(closest_normal, center) < 0)
            closest_normal = -closest_normal;
        
        // Find support point in expansion direction
        // (simplified - in production, store A and B support points separately)
        normal = closest_normal;
        penetration = min_dist;
        contact_point = closest_normal * (min_dist * 0.5);
        
        // Check if we've converged
        if (min_dist < 1e-6) break;
        
        // Expand (simplified - full EPA would add new faces)
        break;
    }
    
    return true;
}

// ==================== CollisionSystem ====================

CollisionSystem::CollisionSystem() {
    spatial_hash_ = std::make_unique<SpatialHash>(2.0);
    bvh_ = std::make_unique<BVH>();
}

CollisionSystem::~CollisionSystem() = default;

void CollisionSystem::add_static_collider(const Collider& collider) {
    static_colliders_.push_back(collider);
}

void CollisionSystem::clear_static_colliders() {
    static_colliders_.clear();
}

void CollisionSystem::broad_phase(const std::vector<Body>& bodies,
                                   std::vector<CollisionPair>& pairs) {
    switch (broad_phase_) {
        case SPATIAL_HASH:
            spatial_hash_->build(bodies);
            spatial_hash_->get_potential_pairs(pairs);
            break;
        case BVH:
            bvh_->build(bodies);
            // TODO: BVH traversal for pair generation
            pairs.clear();
            break;
        case BRUTE_FORCE:
        default:
            pairs.clear();
            for (size_t i = 0; i < bodies.size(); i++) {
                for (size_t j = i + 1; j < bodies.size(); j++) {
                    if (bodies[i].is_static() && bodies[j].is_static()) continue;
                    pairs.push_back({bodies[i].id, bodies[j].id, 0});
                }
            }
            break;
    }
}

void CollisionSystem::narrow_phase(const std::vector<Body>& bodies,
                                    const std::vector<CollisionPair>& pairs,
                                    std::vector<ContactPoint>& contacts) {
    for (const auto& pair : pairs) {
        const Body* a = nullptr, * b = nullptr;
        for (const auto& body : bodies) {
            if (body.id == pair.a) a = &body;
            if (body.id == pair.b) b = &body;
        }
        if (!a || !b) continue;
        
        ContactPoint contact;
        
        // Fast path: sphere-sphere
        if (a->visual.shape == ShapeType::Sphere && 
            b->visual.shape == ShapeType::Sphere) {
            if (detect_sphere_sphere(*a, *b, contact)) {
                contact.body_a_id = a->id;
                contact.body_b_id = b->id;
                contact.restitution = (a->props.restitution + b->props.restitution) * 0.5;
                contact.friction = (a->props.friction + b->props.friction) * 0.5;
                contacts.push_back(contact);
            }
            continue;
        }
        
        // General GJK path
        dvec3 point, normal;
        double pen;
        if (gjk_.get_contact(*a, *b, point, normal, pen)) {
            contact.point = point;
            contact.normal = normal;
            contact.penetration = pen;
            contact.body_a_id = a->id;
            contact.body_b_id = b->id;
            contact.restitution = (a->props.restitution + b->props.restitution) * 0.5;
            contact.friction = (a->props.friction + b->props.friction) * 0.5;
            contacts.push_back(contact);
        }
    }
}

bool CollisionSystem::detect_sphere_sphere(const Body& a, const Body& b,
                                            ContactPoint& contact) {
    dvec3 delta = b.state.position - a.state.position;
    double dist = glm::length(delta);
    double min_dist = a.props.radius + b.props.radius;
    
    if (dist >= min_dist || dist < 1e-10) return false;
    
    contact.penetration = min_dist - dist;
    contact.normal = delta / dist;
    contact.point = a.state.position + contact.normal * 
                    (a.props.radius - contact.penetration * 0.5);
    return true;
}

void CollisionSystem::detect_collisions(std::vector<Body>& bodies,
                                         const SimulationParams& params,
                                         double dt,
                                         std::vector<CollisionEvent>& events) {
    std::vector<CollisionPair> pairs;
    std::vector<ContactPoint> contacts;
    
    // Broad phase
    broad_phase(bodies, pairs);
    
    // Narrow phase
    narrow_phase(bodies, pairs, contacts);
    
    // Static colliders
    detect_static_collisions(bodies, contacts);
    
    // Resolve collisions
    resolve_collisions(bodies, contacts, dt);
    
    // Generate events for visualization
    for (const auto& c : contacts) {
        CollisionEvent ev;
        ev.body_a = c.body_a_id;
        ev.body_b = c.body_b_id;
        ev.point = c.point;
        ev.normal = c.normal;
        ev.impulse_magnitude = c.penetration * 10.0;
        ev.timestamp = 0;
        events.push_back(ev);
    }
    
    // Solve constraints (stacking)
    if (!contacts.empty()) {
        solve_constraints(bodies, contacts, 10);
    }
    
    last_contacts_ = contacts;
}

void CollisionSystem::resolve_collisions(std::vector<Body>& bodies,
                                          const std::vector<ContactPoint>& contacts,
                                          double dt) {
    for (const auto& contact : contacts) {
        Body* a = nullptr, * b = nullptr;
        for (auto& body : bodies) {
            if (body.id == contact.body_a_id) a = &body;
            if (body.id == contact.body_b_id) b = &body;
        }
        
        if (!a || (!b && !contact.is_static_b)) continue;
        
        apply_contact_impulse(*a, contact.is_static_b ? *a : *b, contact);
    }
}

void CollisionSystem::apply_contact_impulse(Body& a, Body& b,
                                             const ContactPoint& contact) {
    double total_mass = a.props.mass + b.props.mass;
    if (total_mass < 1e-10) return;
    
    dvec3 n = contact.normal;
    
    // Position correction (linear projection)
    double correction = contact.penetration * 0.8; // Baümgarte factor
    if (!a.is_static()) {
        a.state.position -= n * correction * (b.props.mass / total_mass);
    }
    if (!b.is_static()) {
        b.state.position += n * correction * (a.props.mass / total_mass);
    }
    
    // Velocity impulse
    dvec3 rel_v = a.state.velocity - b.state.velocity;
    double rel_vn = glm::dot(rel_v, n);
    
    if (rel_vn > 0) return; // Bodies separating
    
    double e = contact.restitution;
    double j = -(1.0 + e) * rel_vn / total_mass;
    
    if (!a.is_static())
        a.state.velocity += j * b.props.mass * n;
    if (!b.is_static())
        b.state.velocity -= j * a.props.mass * n;
    
    // Friction
    dvec3 tangent = rel_v - rel_vn * n;
    double t_len = glm::length(tangent);
    if (t_len > 1e-10) {
        dvec3 t_dir = tangent / t_len;
        double jt = std::min(contact.friction * j, t_len * 0.5);
        if (!a.is_static())
            a.state.velocity -= jt * t_dir / a.props.mass;
        if (!b.is_static())
            b.state.velocity += jt * t_dir / b.props.mass;
    }
}

void CollisionSystem::solve_constraints(std::vector<Body>& bodies,
                                         std::vector<ContactPoint>& contacts,
                                         int iterations) {
    for (int iter = 0; iter < iterations; iter++) {
        for (auto& contact : contacts) {
            Body* a = nullptr, * b = nullptr;
            for (auto& body : bodies) {
                if (body.id == contact.body_a_id) a = &body;
                if (body.id == contact.body_b_id) b = &body;
            }
            if (!a || !b) continue;
            
            // Recompute penetration
            dvec3 delta = b->state.position - a->state.position;
            double dist = glm::length(delta);
            if (dist < 1e-10) continue;
            
            double pen = a->props.radius + b->props.radius - dist;
            if (pen <= 0) continue;
            
            dvec3 n = delta / dist;
            contact.normal = n;
            contact.penetration = pen;
            
            apply_contact_impulse(*a, *b, contact);
        }
    }
}

void CollisionSystem::detect_static_collisions(std::vector<Body>& bodies,
                                                std::vector<ContactPoint>& contacts) {
    for (const auto& collider : static_colliders_) {
        for (auto& body : bodies) {
            if (body.is_static()) continue;
            
            ContactPoint contact;
            if (collider_vs_body(collider, body, contact)) {
                contact.body_a_id = body.id;
                contact.body_b_id = UINT64_MAX;
                contact.is_static_b = true;
                contact.restitution = body.props.restitution;
                contact.friction = body.props.friction;
                contacts.push_back(contact);
            }
        }
    }
}

bool CollisionSystem::collider_vs_body(const Collider& collider, const Body& body,
                                        ContactPoint& contact) {
    switch (collider.type) {
        case Collider::PLANE: {
            double dot = glm::dot(body.state.position, collider.normal);
            double pen = collider.offset - dot + body.props.radius;
            if (pen > 0) {
                contact.point = body.state.position - collider.normal * (body.props.radius - pen * 0.5);
                contact.normal = collider.normal;
                contact.penetration = pen;
                return true;
            }
            return false;
        }
        case Collider::SPHERE: {
            dvec3 delta = body.state.position; // Collider at origin
            double dist = glm::length(delta);
            double min_dist = body.props.radius + collider.radius;
            if (dist < min_dist && dist > 1e-10) {
                contact.normal = delta / dist;
                contact.penetration = min_dist - dist;
                contact.point = body.state.position - contact.normal * (body.props.radius - contact.penetration * 0.5);
                return true;
            }
            return false;
        }
        default:
            return false;
    }
}

} // namespace hlp
