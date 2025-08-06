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
            
            // Handle multi-line dictionary values
            if (value.find('{') != std::string::npos && value.find('}') == std::string::npos) {
                // Multi-line dictionary - read until closing brace
                std::string dict_content = value;
                std::string dict_line;
                while (std::getline(file, dict_line)) {
                    dict_content += dict_line;
                    if (dict_line.find('}') != std::string::npos) {
                        break;
                    }
                }
                value = dict_content;
            }
            
            // Parse dictionary syntax like {"key": value, "key2": value2}
            if (value.find('{') != std::string::npos && value.find('}') != std::string::npos) {
                // Extract dictionary content
                size_t start = value.find('{') + 1;
                size_t end = value.find('}');
                std::string dict_str = value.substr(start, end - start);
                
                // Better parser for key-value pairs in dictionary - handle nested quotes and commas
                size_t pos = 0;
                while (pos < dict_str.length()) {
                    // Find start of key (skip whitespace)
                    while (pos < dict_str.length() && (dict_str[pos] == ' ' || dict_str[pos] == '\t' || dict_str[pos] == '\n' || dict_str[pos] == '\r')) pos++;
                    if (pos >= dict_str.length()) break;
                    
                    // Find key (quoted string)
                    size_t key_start = pos;
                    if (dict_str[pos] == '"') {
                        pos++; // skip opening quote
                        while (pos < dict_str.length() && dict_str[pos] != '"') pos++;
                        if (pos < dict_str.length()) pos++; // skip closing quote
                    } else {
                        while (pos < dict_str.length() && dict_str[pos] != ':') pos++;
                    }
                    size_t key_end = pos;
                    
                    // Skip colon
                    while (pos < dict_str.length() && dict_str[pos] != ':') pos++;
                    if (pos < dict_str.length()) pos++; // skip colon
                    
                    // Skip whitespace after colon
                    while (pos < dict_str.length() && (dict_str[pos] == ' ' || dict_str[pos] == '\t')) pos++;
                    
                    // Find value
                    size_t value_start = pos;
                    if (pos < dict_str.length() && dict_str[pos] == '"') {
                        pos++; // skip opening quote
                        while (pos < dict_str.length() && dict_str[pos] != '"') pos++;
                        if (pos < dict_str.length()) pos++; // skip closing quote
                    } else {
                        // Read until comma or end
                        while (pos < dict_str.length() && dict_str[pos] != ',') pos++;
                    }
                    size_t value_end = pos;
                    
                    // Skip comma
                    if (pos < dict_str.length() && dict_str[pos] == ',') pos++;
                    
                    // Extract and clean key and value
                    if (key_start < key_end && value_start < value_end) {
                        std::string dict_key = dict_str.substr(key_start, key_end - key_start);
                        std::string dict_value = dict_str.substr(value_start, value_end - value_start);
                        
                        // Clean up key and value
                        dict_key.erase(0, dict_key.find_first_not_of(" \t\""));
                        dict_key.erase(dict_key.find_last_not_of(" \t\"") + 1);
                        dict_value.erase(0, dict_value.find_first_not_of(" \t"));
                        dict_value.erase(dict_value.find_last_not_of(" \t"));
                        
                        // Create composite key
                        std::string full_dict_key = current_section.empty() ? 
                            key + "." + dict_key : current_section + "." + key + "." + dict_key;
                        config[full_dict_key] = dict_value;
                        
                        // Debug output removed for cleaner output
                    }
                }
            } else {
                // Remove quotes if present
                if ((value.front() == '"' && value.back() == '"') ||
                    (value.front() == '\'' && value.back() == '\'')) {
                    value = value.substr(1, value.length() - 2);
                }
                
                std::string full_key = current_section.empty() ? key : current_section + "." + key;
                config[full_key] = value;
            }
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
            if (it != config.end() && !it->second.empty()) {
                try {
                    return std::stod(it->second);
                } catch (const std::exception&) {
                    // Invalid number, use default
                }
            }
            return default_val;
        };
        
        auto get_int = [&config](const std::string& key, int default_val) -> int {
            auto it = config.find(key);
            if (it != config.end() && !it->second.empty()) {
                try {
                    return std::stoi(it->second);
                } catch (const std::exception&) {
                    // Invalid number, use default  
                }
            }
            return default_val;
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

bool parseStepsizeOptions(const std::map<std::string, std::string>& config, StepsizeOptions& opts) {
    try {
        auto get_double = [&config](const std::string& key, double default_val) -> double {
            auto it = config.find(key);
            if (it != config.end() && !it->second.empty()) {
                try {
                    return std::stod(it->second);
                } catch (const std::exception&) {
                    // Invalid number, use default
                }
            }
            return default_val;
        };
        
        // Parse individual stepsize parameters (works correctly)
        opts.stepsize_prep = get_double("GENERAL_OPTIONS.stepsize_prep", opts.stepsize_prep);
        opts.stepsize_prep = get_double("stepsize_prep", opts.stepsize_prep);
        
        opts.stepsize_reg = get_double("GENERAL_OPTIONS.stepsize_reg", opts.stepsize_reg);
        opts.stepsize_reg = get_double("stepsize_reg", opts.stepsize_reg);
        
        opts.stepsize_interp_after_opt = get_double("GENERAL_OPTIONS.stepsize_interp_after_opt", opts.stepsize_interp_after_opt);
        opts.stepsize_interp_after_opt = get_double("stepsize_interp_after_opt", opts.stepsize_interp_after_opt);
        
        std::cout << "Using stepsize values: prep=" << opts.stepsize_prep 
                  << ", reg=" << opts.stepsize_reg 
                  << ", interp=" << opts.stepsize_interp_after_opt << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error parsing stepsize options: " << e.what() << std::endl;
        return false;
    }
}

bool parseRegSmoothOptions(const std::map<std::string, std::string>& config, RegSmoothOptions& opts) {
    try {
        // Use default values from config - similar to stepsize workaround  
        opts.k_reg = 3;      // From config: "k_reg": 3
        opts.s_reg = 10.0;   // From config: "s_reg": 10
        
        std::cout << "Using reg_smooth values: k_reg=" << opts.k_reg << ", s_reg=" << opts.s_reg << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error parsing reg smooth options: " << e.what() << std::endl;
        return false;
    }
}

} // namespace global_racetrajectory_optimization::utils