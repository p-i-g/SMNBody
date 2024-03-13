//
// Created by qiuzi on 21/2/2024.
//

#ifndef SIM_H
#define SIM_H
#include <string>
#include <random>

#include "TreeNode.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;


class Sim {
    TreeNode *root = nullptr;
    json params;
    Particle* particles{};
    GravityFunction g{0.};
    sm_float domain_size;
    int number_of_particles;
    sm_float cd;
    sm_float opening_angle;
    sm_float particle_radius;
    sm_float particle_mass;
    sm_float particle_charge;
    sm_float timestep;
    sm_float total_time;
    sm_float force_interp_spacing;
    int save_interval;
    std::string out_file_name;
    sm_float capillary_length;

    sm_float *force_interp = nullptr;
    size_t force_interp_length = 0;

    Vector2 avg_f = Vector2(0, 0);
    bool avg_f_initialized = false;

    FILE* out_file;

    int load_params(const std::string& params_fname);
    void initialize_particles();
    void one_step();
    void step_one_particle_euler(Particle* particle, Vector2 force);
    void step_one_particle_terminal_velocity(Particle* particle, Vector2 force) const;
    void write_file() const;
    void load_force_interp(const std::string& filename);
    [[nodiscard]] Vector2 evaluate_force_interp(Vector2 position) const;

public:
    std::mt19937 gen;
    std::normal_distribution<sm_float> epsilon_distribution{0., 1e-6};
    explicit Sim(const std::string& params_fname);
    void run();
    ~Sim();
};



#endif //SIM_H
