#include "global_racetrajectory_optimization/global_racetrajectory_optimization.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

namespace global_racetrajectory_optimization::utils {

std::map<std::string, std::string> parseConfigFile(const std::string& filename) {
    std::map<std::string, std::string> config;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open config file: " + filename);
    }
    
    std::string line;
    std::string current_section;
    
    while (std::getline(file, line)) {
        // Remove leading/trailing whitespace
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);
        
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#' || line[0] == ';') {
            continue;
        }
        
        // Check for section headers [SECTION]
        if (line[0] == '[' && line.back() == ']') {
            current_section = line.substr(1, line.length() - 2);
            continue;
        }
        
        // Parse key-value pairs
        size_t eq_pos = line.find('=');
        if (eq_pos != std::string::npos) {
            std::string key = line.substr(0, eq_pos);
            std::string value = line.substr(eq_pos + 1);
            
            // Remove whitespace around key and value
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            
            // Remove quotes if present
            if ((value.front() == '"' && value.back() == '"') ||
                (value.front() == '\'' && value.back() == '\'')) {
                value = value.substr(1, value.length() - 2);
            }
            
            std::string full_key = current_section.empty() ? key : current_section + "." + key;
            config[full_key] = value;
        }
    }
    
    return config;
}

bool parseVehicleParams(const std::map<std::string, std::string>& config, VehicleParameters& params) {
    try {
        // Parse individual parameters with fallback to defaults
        auto get_double = [&config](const std::string& key, double default_val) -> double {
            auto it = config.find(key);
            return (it != config.end()) ? std::stod(it->second) : default_val;
        };
        
        // Try different possible key formats
        params.v_max = get_double("GENERAL_OPTIONS.veh_params.v_max", params.v_max);
        params.v_max = get_double("veh_params.v_max", params.v_max);
        params.v_max = get_double("v_max", params.v_max);
        
        params.length = get_double("GENERAL_OPTIONS.veh_params.length", params.length);
        params.length = get_double("veh_params.length", params.length);
        params.length = get_double("length", params.length);
        
        params.width = get_double("GENERAL_OPTIONS.veh_params.width", params.width);
        params.width = get_double("veh_params.width", params.width);
        params.width = get_double("width", params.width);
        
        params.mass = get_double("GENERAL_OPTIONS.veh_params.mass", params.mass);
        params.mass = get_double("veh_params.mass", params.mass);
        params.mass = get_double("mass", params.mass);
        
        params.dragcoeff = get_double("GENERAL_OPTIONS.veh_params.dragcoeff", params.dragcoeff);
        params.dragcoeff = get_double("veh_params.dragcoeff", params.dragcoeff);
        params.dragcoeff = get_double("dragcoeff", params.dragcoeff);
        
        params.curvlim = get_double("GENERAL_OPTIONS.veh_params.curvlim", params.curvlim);
        params.curvlim = get_double("veh_params.curvlim", params.curvlim);
        params.curvlim = get_double("curvlim", params.curvlim);
        
        params.g = get_double("GENERAL_OPTIONS.veh_params.g", params.g);
        params.g = get_double("veh_params.g", params.g);
        params.g = get_double("g", params.g);
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error parsing vehicle parameters: " << e.what() << std::endl;
        return false;
    }
}

bool parseOptimizationOptions(const std::map<std::string, std::string>& config, OptimizationOptions& opts) {
    try {
        auto get_double = [&config](const std::string& key, double default_val) -> double {
            auto it = config.find(key);
            return (it != config.end()) ? std::stod(it->second) : default_val;
        };
        
        auto get_int = [&config](const std::string& key, int default_val) -> int {
            auto it = config.find(key);
            return (it != config.end()) ? std::stoi(it->second) : default_val;
        };
        
        auto get_bool = [&config](const std::string& key, bool default_val) -> bool {
            auto it = config.find(key);
            if (it == config.end()) return default_val;
            std::string val = it->second;
            std::transform(val.begin(), val.end(), val.begin(), ::tolower);
            return val == "true" || val == "1" || val == "yes";
        };
        
        // Try different sections for optimization options
        opts.width_opt = get_double("OPTIMIZATION_OPTIONS.optim_opts_mincurv.width_opt", opts.width_opt);
        opts.width_opt = get_double("optim_opts_mincurv.width_opt", opts.width_opt);
        opts.width_opt = get_double("width_opt", opts.width_opt);
        
        opts.iqp_iters_min = get_int("OPTIMIZATION_OPTIONS.optim_opts_mincurv.iqp_iters_min", opts.iqp_iters_min);
        opts.iqp_iters_min = get_int("optim_opts_mincurv.iqp_iters_min", opts.iqp_iters_min);
        opts.iqp_iters_min = get_int("iqp_iters_min", opts.iqp_iters_min);
        
        opts.iqp_curverror_allowed = get_double("OPTIMIZATION_OPTIONS.optim_opts_mincurv.iqp_curverror_allowed", opts.iqp_curverror_allowed);
        opts.iqp_curverror_allowed = get_double("optim_opts_mincurv.iqp_curverror_allowed", opts.iqp_curverror_allowed);
        opts.iqp_curverror_allowed = get_double("iqp_curverror_allowed", opts.iqp_curverror_allowed);
        
        // Mintime specific options
        opts.penalty_delta = get_double("OPTIMIZATION_OPTIONS.optim_opts_mintime.penalty_delta", opts.penalty_delta);
        opts.penalty_F = get_double("OPTIMIZATION_OPTIONS.optim_opts_mintime.penalty_F", opts.penalty_F);
        opts.mue = get_double("OPTIMIZATION_OPTIONS.optim_opts_mintime.mue", opts.mue);
        opts.n_gauss = get_int("OPTIMIZATION_OPTIONS.optim_opts_mintime.n_gauss", opts.n_gauss);
        opts.dn = get_double("OPTIMIZATION_OPTIONS.optim_opts_mintime.dn", opts.dn);
        opts.limit_energy = get_bool("OPTIMIZATION_OPTIONS.optim_opts_mintime.limit_energy", opts.limit_energy);
        opts.energy_limit = get_double("OPTIMIZATION_OPTIONS.optim_opts_mintime.energy_limit", opts.energy_limit);
        opts.safe_traj = get_bool("OPTIMIZATION_OPTIONS.optim_opts_mintime.safe_traj", opts.safe_traj);
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error parsing optimization options: " << e.what() << std::endl;
        return false;
    }
}

} // namespace global_racetrajectory_optimization::utils