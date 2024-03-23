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
    substeps = params["substeps"];
    timestep = params["timestep"];
    total_time = params["total_time"];
    save_interval = params["save_interval"];
    out_file_name = params["output_file_name"];
    force_interp_spacing = params["force_interp_spacing"];
    capillary_length = params["capillary_length"];

    timestep = timestep / substeps;

    g = GravityFunction(capillary_length);

    epsilon_distribution = std::normal_distribution<sm_float>(0, particle_radius / 10);

    initialize_particles();
    load_force_interp(params["force_input_file"]);
    out_file = fopen(out_file_name.c_str(), "wb");
    if (!out_file) {
        std::cerr << "Failed to open output file: " << out_file_name << std::endl;
    }

    counters = new int[100000];
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
    force_interp_length = file_size / sizeof(sm_float);

    double* temp = new double[force_interp_length];

    force_interp = new sm_float[force_interp_length];

    // read the file into the buffer
    if (size_t const bytesRead = fread(temp, sizeof(double), force_interp_length, file); bytesRead != force_interp_length) {
        std::cerr << "Failed to read force interpolation file." << std::endl;
        delete[] force_interp;
        delete[] temp;
        fclose(file);
        return;
    }

    for (size_t i = 0; i < force_interp_length; i++) {
        force_interp[i] = static_cast<sm_float>(temp[i]);
    }

    delete[] temp;
    fclose(file);
    std::cout << "val: " << force_interp[0] << std::endl;
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

    std::uniform_real_distribution<> theta_distribution(0, std::numbers::pi * 2);
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
    Vector2 avg_moses{0, 0};
    Vector2 avg_velocity{0, 0};
    Vector2 avg_position{0, 0};
    root = new TreeNode(nullptr, domain_size, Vector2(0, 0), this);
    // add particles to the tree
    for (int i = 0; i < number_of_particles; i++) {
        root->insert_particle(&particles[i], 0);
    }
    for (int i = 0; i < number_of_particles; i++) {
        Vector2 gravity = root->calculate_force(&particles[i], opening_angle, g, particle_radius);
        // Vector2 gravity(0, 0);
        // for (int j = 0; j < number_of_particles; j++) {
        //     if ((particles[i].position - particles[j].position).norm() > particle_radius * 2.1 && (particles[i].position - particles[j].position).norm() < particle_radius * 100)
        //         gravity = gravity + g.call(particles[i].position, particles[j].position) * particle_charge;
        //     // std::cout << gravity.x << std::endl;
        // }
        // f = particles[i].check_lock(f);
        // if (f.x > 10 || f.y > 10 || f.x < -10 || f.y < -10) {
        //     std::cout << f.x << ", " << f.y << std::endl;
        // }
        // auto inner_t2 = std::chrono::high_resolution_clock::now();
        // if ((particles[i].velocity * cd).norm() > 1e-10) {
        //     std::cout << "Drag: " << (particles[i].velocity * cd).norm() << std::endl;
        // }
        Vector2 f = particles[i].check_lock(gravity + evaluate_force_interp(particles[i].position));
        avg_moses = avg_moses + f;
        step_one_particle_terminal_velocity(&particles[i], f);
        avg_position = avg_position - particles[i].position;
        particles[i].cache_angles.clear();
        // for (int j = 0; j < substeps; j++) {
        //     Vector2 f = particles[i].check_lock(gravity + evaluate_force_interp(particles[i].position));
        //     avg_f.x = avg_f.x * 0.9999 + f.x * 0.0001;
        //     avg_f.y = avg_f.y * 0.9999 + f.y * 0.0001;
        //     step_one_particle_terminal_velocity(&particles[i], f);
        //     particles[i].cache_angles.clear();
        //     if (particles[i].current_node != nullptr) {
        //         // auto inner_t0 = std::chrono::high_resolution_clock::now();
        //         particles[i].clear_lock();
        //         particles[i].current_node->resolve_collisions(&particles[i], particle_radius);
        //         // auto inner_t1 = std::chrono::high_resolution_clock::now();
        //         // std::cout << "Force time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(inner_t1 - inner_t0).count() << std::endl;
        //     }
        // }
        // f = f + evaluate_force_interp(particles[i].position);
        // step_one_particle_terminal_velocity(&particles[i], f);
        // particles[i].cache_angles.clear();
        // if (particles[i].current_node != nullptr) {
        //     particles[i].current_node->resolve_collisions(&particles[i], particle_radius);
        // }

        // std::cout << particles[i].current_node->center.x << ", " << particles[i].current_node->center.y << std::endl;
        // std::cout << "Timestep time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(inner_t2 - inner_t1).count() << std::endl;
        // for (int j = 0; j < 6; j++) {
        //     // for (int k = static_cast<int>(particles[i].lock_start[j] * 100000 / 2 / std::numbers::pi);
        //     //     k < static_cast<int>(particles[i].lock_end[j] * 100000 / 2 / std::numbers::pi) && k < 100000; k++) {
        //     //     counters[k]++;
        //     // }
        //     // if (particles[i].lock_end[j] < particles[i].lock_start[j]) {
        //     //     for (int k = static_cast<int>(particles[i].lock_start[j] * 100000 / 2 / std::numbers::pi);
        //     //          k < 100000; k++) {
        //     //         counters[k]++;
        //     //     }
        //     //     for (int k = 0; k < static_cast<int>(particles[i].lock_end[j] * 100000 / 2 / std::numbers::pi) && k < 100000; k++) {
        //     //         counters[k]++;
        //     //     }
        //     // }
        // }
        particles[i].clear_lock();
    }
    for (int i = 0; i < number_of_particles; i++) {
        if (particles[i].current_node != nullptr) {
            particles[i].clear_lock();
            particles[i].current_node->resolve_collisions(&particles[i], particle_radius, counters);
            avg_position = avg_position + particles[i].position;
        }
    }
    // FILE *f = fopen("outputs/counter.bin", "wb");
    // fwrite(counters, sizeof(int), 100000, f);
    // fclose(f);
    std::cout << "Average force: " << avg_moses.x << ", " << avg_moses.y << std::endl;
    // std::cout << "Average change in position: " << avg_position.x << ", " << avg_position.y << std::endl;
    // std::cout << "Average diff: " << avg_velocity.x << ", " << avg_velocity.y << std::endl;

    delete root;
}

void Sim::step_one_particle_euler(Particle *particle, Vector2 const force) const {
    // std::cout << particle->position.x << ", " << particle->position.y << std::endl;
    particle->position = particle->position + particle->velocity * timestep;
    // std::cout << particle->position.x << ", " << particle->position.y << std::endl;
    particle->velocity = particle->velocity + force / particle_mass * timestep;
}

void Sim::step_one_particle_terminal_velocity(Particle *particle, Vector2 const force) const {
    particle->velocity = force / cd;
    particle->position = particle->position + particle->velocity * timestep;
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
        auto x = static_cast<double>(particles[i].position.x);
        auto y = static_cast<double>(particles[i].position.y);
        fwrite(&x, sizeof(sm_float), 1, out_file);
        fwrite(&y, sizeof(sm_float), 1, out_file);
    }
    fflush(out_file);
    std::cout << i << std::endl;
}

Sim::~Sim() {
    free(particles);
    delete[] force_interp;
    fclose(out_file);
}





