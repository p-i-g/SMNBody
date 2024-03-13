//
// Created by qiuzi on 21/2/2024.
//

#include "Sim.h"
#include <fstream>
#include <random>
#include <cmath>
#include <chrono>
#include <iostream>


Sim::Sim(const std::string& params_fname) : gen{std::random_device{}()} {
    if (load_params(params_fname)) {
        std::cerr << "Failed to load parameters file: " << params_fname << std::endl;
        return;
    }
    std::cout << "Successfullly loaded parameters: " << params_fname <<  std::endl;

    number_of_particles = params["number_of_particles"];
    opening_angle = params["opening_angle"];
    particle_radius = params["particle_radius"];
    domain_size = params["domain_size"];
    particle_charge = params["particle_charge"];
    particle_mass = params["particle_mass"];
    cd = params["cd"];
    timestep = params["timestep"];
    total_time = params["total_time"];
    save_interval = params["save_interval"];
    out_file_name = params["output_file_name"];
    force_interp_spacing = params["force_interp_spacing"];
    capillary_length = params["capillary_length"];

    g = GravityFunction(capillary_length);

    epsilon_distribution = std::normal_distribution<sm_float>(0, particle_radius / 10);

    initialize_particles();
    load_force_interp(params["force_input_file"]);
    out_file = fopen(out_file_name.c_str(), "wb");
    if (!out_file) {
        std::cerr << "Failed to open output file: " << out_file_name << std::endl;
    }
}

int Sim::load_params(const std::string& params_fname) {
    std::ifstream params_file(params_fname.c_str());
    if (params_file.fail())
        return 1;

    params_file >> params;
    params_file.close();
    return 0;
}

void Sim::load_force_interp(const std::string &filename) {
    FILE* file = fopen(filename.c_str(), "rb");
    if (!file) {
        std::cerr << "Failed to open force interpolation file." << std::endl;
        return;
    }

    // get file size
    fseek(file, 0, SEEK_END);
    size_t const file_size = ftell(file);
    rewind(file);

    force_interp = new sm_float[file_size / sizeof(sm_float)];
    force_interp_length = file_size / sizeof(sm_float);

    // read the file into the buffer
    if (size_t const bytesRead = fread(force_interp, 1, file_size, file); bytesRead != file_size) {
        std::cerr << "Failed to read force interpolation file." << std::endl;
        delete[] force_interp;
    }
}

Vector2 Sim::evaluate_force_interp(Vector2 const position) const {
    sm_float const r = position.norm() / force_interp_spacing;

    // check for out of bounds
    if (r >= static_cast<sm_float>(force_interp_length - 1)) return position * force_interp[force_interp_length - 1] / position.norm();

    // evaluate interp
    sm_float const floor = std::floor(r);
    int const idx = static_cast<int>(floor);
    sm_float const alpha = r - floor;
    return position * (force_interp[idx] + alpha * (force_interp[idx + 1] - force_interp[idx])) / position.norm();
}



void Sim::initialize_particles() {
    particles = static_cast<Particle *>(malloc(number_of_particles * sizeof(Particle)));

    std::uniform_real_distribution<> theta_distribution(0, 2 * std::numbers::pi);
    std::vector i{0, domain_size / 2};
    std::vector<sm_float> w{0, 1};
    std::piecewise_linear_distribution r_distribution(i.begin(), i.end(), w.begin());

    for (int it = 0; it < number_of_particles; it++) {
        const sm_float r = r_distribution(gen);
        const sm_float theta = theta_distribution(gen);
        particles[it] = Particle(Vector2(r * cos(theta), r * sin(theta)), particle_mass, particle_charge, this);
    }
}

void Sim::one_step() {
    root = new TreeNode(nullptr, domain_size, Vector2(0, 0), this);
    // add particles to the tree
    for (int i = 0; i < number_of_particles; i++) {
        root->insert_particle(&particles[i], 0);
    }
    for (int i = 0; i < number_of_particles; i++) {
        // auto inner_t0 = std::chrono::high_resolution_clock::now();
        Vector2 f = root->calculate_force(&particles[i], opening_angle, g, particle_radius);
        f = particles[i].check_lock(f);
        particles[i].clear_lock();
        // if (f.x > 10 || f.y > 10 || f.x < -10 || f.y < -10) {
        //     std::cout << f.x << ", " << f.y << std::endl;
        // }
        // auto inner_t1 = std::chrono::high_resolution_clock::now();
        // std::cout << "Force time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(inner_t1 - inner_t0).count() << std::endl;
        // auto inner_t2 = std::chrono::high_resolution_clock::now();
        // if ((particles[i].velocity * cd).norm() > 1e-10) {
        //     std::cout << "Drag: " << (particles[i].velocity * cd).norm() << std::endl;
        // }
        f = f + evaluate_force_interp(particles[i].position);
        if (avg_f_initialized) {
            avg_f = (avg_f * 0.99999 + evaluate_force_interp(particles[i].position) * 0.00001);
        } else {
            avg_f = evaluate_force_interp(particles[i].position);
            avg_f_initialized = true;
        }
        step_one_particle_terminal_velocity(&particles[i], f);
        particles[i].cache_angles.clear();
        // if (particles[i].current_node != nullptr) {
        //     particles[i].current_node->resolve_collisions(&particles[i], particle_radius);
        // }

        // std::cout << particles[i].current_node->center.x << ", " << particles[i].current_node->center.y << std::endl;
        // std::cout << "Timestep time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(inner_t2 - inner_t1).count() << std::endl;
    }

    for (int i = 0; i < number_of_particles; i++) {
        if (particles[i].current_node != nullptr) {
            particles[i].current_node->resolve_collisions(&particles[i], particle_radius);
        }
    }

    delete root;
}

void Sim::step_one_particle_euler(Particle *particle, Vector2 force) {
    // std::cout << particle->position.x << ", " << particle->position.y << std::endl;
    particle->position = particle->position + particle->velocity * timestep;
    // std::cout << particle->position.x << ", " << particle->position.y << std::endl;
    particle->velocity = particle->velocity + force / particle_mass * timestep;
}

void Sim::step_one_particle_terminal_velocity(Particle *particle, Vector2 const force) const {
    particle->position = particle->position + particle->velocity * timestep;
    particle->velocity = force / cd;
}


void Sim::run() {
    int const total_steps = static_cast<int>(total_time / timestep);
    for (int i = 0; i < total_steps; i++) {
        std::cout << "Step: " << i << std::endl;
        one_step();
        // std::cout << i << std::endl;

        if (i % save_interval == 0) {
            write_file();
        }
    }
}

void Sim::write_file() const {
    std::cout << avg_f.x << ", " << avg_f.y << std::endl;
    // std::cout << "Before: " << hehe_before.x << ", " << hehe_before.y << std::endl;
    // std::cout << "After: " << hehe_after.x << ", " << hehe_after.y << std::endl;
    int i;
    for (i = 0; i < number_of_particles; i++) {
        fwrite(&particles[i].position.x, sizeof(sm_float), 1, out_file);
        fwrite(&particles[i].position.y, sizeof(sm_float), 1, out_file);
    }
    fflush(out_file);
    std::cout << i << std::endl;
}

Sim::~Sim() {
    free(particles);
    delete[] force_interp;
    fclose(out_file);
}





