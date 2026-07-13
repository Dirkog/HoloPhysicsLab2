#include "Types.h"
#include <nlohmann/json.hpp>

namespace hlp {

void to_json(nlohmann::json& j, const dvec3& v) {
    j = nlohmann::json::array({v.x, v.y, v.z});
}

void from_json(const nlohmann::json& j, dvec3& v) {
    v.x = j[0].get<double>();
    v.y = j[1].get<double>();
    v.z = j[2].get<double>();
}

void to_json(nlohmann::json& j, const Body& b) {
    j = nlohmann::json{
        {"id", b.id},
        {"position", b.state.position},
        {"velocity", b.state.velocity},
        {"angular_velocity", b.state.angular_velocity},
        {"orientation", {b.state.orientation.w, b.state.orientation.x, 
                         b.state.orientation.y, b.state.orientation.z}},
        {"properties", {
            {"mass", b.props.mass},
            {"radius", b.props.radius},
            {"restitution", b.props.restitution},
            {"friction", b.props.friction},
            {"charge", b.props.charge},
            {"magnetic_moment", b.props.magnetic_moment},
            {"temperature", b.props.temperature}
        }},
        {"visual", {
            {"color", {b.visual.color.r, b.visual.color.g, b.visual.color.b}},
            {"shape", static_cast<int>(b.visual.shape)},
            {"label", b.visual.label}
        }},
        {"flags", b.flags}
    };
}

void from_json(const nlohmann::json& j, Body& b) {
    b.id = j.at("id").get<uint64_t>();
    b.state.position = j.at("position").get<dvec3>();
    b.state.velocity = j.at("velocity").get<dvec3>();
    b.state.angular_velocity = j.at("angular_velocity").get<dvec3>();
    
    if (j.contains("properties")) {
        const auto& p = j["properties"];
        b.props.mass = p.value("mass", 1.0);
        b.props.radius = p.value("radius", 0.5);
        b.props.restitution = p.value("restitution", 0.5);
        b.props.friction = p.value("friction", 0.3);
        b.props.charge = p.value("charge", 0.0);
    }
    
    if (j.contains("visual")) {
        const auto& v = j["visual"];
        auto c = v.value("color", std::vector<float>{0,1,0.8f});
        b.visual.color = glm::vec3(c[0], c[1], c[2]);
        b.visual.shape = static_cast<ShapeType>(v.value("shape", 0));
        b.visual.label = v.value("label", "");
    }
    
    b.flags = j.value("flags", BODY_VISIBLE | BODY_GRAVITY);
}

void to_json(nlohmann::json& j, const SimulationParams& p) {
    j = nlohmann::json{
        {"gravity", p.gravity},
        {"damping", p.damping},
        {"restitution", p.restitution},
        {"friction", p.friction},
        {"time_scale", p.time_scale},
        {"wind", p.wind},
        {"air_density", p.air_density},
        {"buoyancy", p.buoyancy},
        {"magnetic_field", p.magnetic_field},
        {"electric_field", p.electric_field},
        {"central_mass", p.central_mass},
        {"coulomb_k", p.coulomb_k},
        {"cohesion", p.cohesion},
        {"spring_k", p.spring_k},
        {"vortex_strength", p.vortex_strength},
        {"coriolis", p.coriolis},
        {"turbulence", p.turbulence},
        {"enables", {
            {"wind", p.enable_wind},
            {"magnet", p.enable_magnet},
            {"coulomb", p.enable_coulomb},
            {"cohesion", p.enable_cohesion},
            {"vortex", p.enable_vortex},
            {"coriolis", p.enable_coriolis},
            {"central_g", p.enable_central_g},
            {"turbulence", p.enable_turbulence},
            {"buoyancy", p.enable_buoyancy},
            {"springs", p.enable_springs},
            {"drag", p.enable_drag},
            {"trails", p.show_trails}
        }}
    };
}

void from_json(const nlohmann::json& j, SimulationParams& p) {
    p.gravity = j.value("gravity", 9.81);
    p.damping = j.value("damping", 0.001);
    p.restitution = j.value("restitution", 0.5);
    p.friction = j.value("friction", 0.3);
    p.time_scale = j.value("time_scale", 1.0);
    if (j.contains("wind")) p.wind = j["wind"].get<dvec3>();
    p.air_density = j.value("air_density", 0.1);
    p.magnetic_field = j.value("magnetic_field", 0.0);
    p.central_mass = j.value("central_mass", 0.0);
    p.coulomb_k = j.value("coulomb_k", 0.0);
    p.cohesion = j.value("cohesion", 0.0);
    p.spring_k = j.value("spring_k", 0.0);
    p.vortex_strength = j.value("vortex_strength", 0.0);
    p.coriolis = j.value("coriolis", 0.0);
    p.turbulence = j.value("turbulence", 0.0);
    
    if (j.contains("enables")) {
        const auto& e = j["enables"];
        p.enable_wind = e.value("wind", false);
        p.enable_magnet = e.value("magnet", false);
        p.enable_coulomb = e.value("coulomb", false);
        p.enable_vortex = e.value("vortex", false);
        p.enable_coriolis = e.value("coriolis", false);
        p.enable_central_g = e.value("central_g", false);
        p.enable_turbulence = e.value("turbulence", false);
        p.enable_buoyancy = e.value("buoyancy", false);
        p.show_trails = e.value("trails", true);
    }
}

} // namespace hlp
