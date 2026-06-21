#!/bin/bash

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Script directory: $SCRIPT_DIR"

cd $SCRIPT_DIR

RUNS=3
SCALABILITY_SCENARIOS=(
    "scenario_scale_4" 
    "scenario_scale_8"   
    "scenario_scale_16"  
    "scenario_scale_32"
    "scenario_scale_64"
)

# Creating results directory
mkdir -p "$SCRIPT_DIR/../online_dmpc/cpp/results/scalability/"

# Function to modify JSON config parameters
modify_json_config() {
    local config_file="$1"
    local param_name="$2"
    local param_value="$3"

    python3 << END_PYTHON
import json
with open('$config_file', 'r') as f:
    config = json.load(f)

if '${param_value}' == 'true':
    config['$param_name'] = True
elif '${param_value}' == 'false':
    config['$param_name'] = False
else:
    try:
        config['$param_name'] = int('${param_value}')
    except ValueError:
        try:
            config['$param_name'] = float('${param_value}')
        except ValueError:
            config['$param_name'] = '${param_value}'

with open('$config_file', 'w') as f:
    json.dump(config, f, indent=2)
END_PYTHON
}

generate_scalability_configs() {
    echo "Generating scalability configuration files..."
    
    python3 << END_PYTHON
import json
import numpy as np

base_config_path = '${SCRIPT_DIR}/../online_dmpc/cpp/config/scenario_scale.json'
config_dir = '${SCRIPT_DIR}/../online_dmpc/cpp/config/scenario_scale_'

# Load base config
with open(base_config_path, 'r') as f:
    base_config = json.load(f)

# Agents counts for scalability scenarios
agent_counts = [4, 8, 16, 32, 64, 128]

for n_agents in agent_counts:
    # Generate grid positions for agents
    grid_size = int(np.ceil(np.sqrt(n_agents)))
    spacing = 2.0 / (grid_size - 1) if grid_size > 1 else 0

    po = []
    pf = []

    for i in range(n_agents):
        row = i // grid_size
        col = i % grid_size
        x = 0.5 + col * spacing
        y = 0.5 + row * spacing
        z = 2.5 + np.random.uniform(-1.2, 1.2)
        po.append([x, y, z])

        goal_row = (grid_size - 1 - row)
        goal_col = (grid_size - 1 - col)
        goal_x = 0.5 + goal_col * spacing
        goal_y = 0.5 + goal_row * spacing
        goal_z = 2.5 + np.random.uniform(-1.2, 1.2)
        pf.append([goal_x, goal_y, goal_z])
    
    # Create new config
    config = base_config.copy()
    config['_comment'] = f'Scalability scenario with {n_agents} agents'
    config['_description'] = f'Scalability test with {n_agents} agents in grid formation.'
    config['N'] = n_agents
    config['Ncmd'] = n_agents
    config['po'] = po
    config['pf'] = pf
    config['simulation_duration'] = 30
    config['output_trajectories_paths'] = [f'../results/trajectories_scale_{n_agents}.txt']
    config['output_goals_paths'] = [f'../results/goals_scale_{n_agents}.txt']

    # Save config
    output_path = f'{config_dir}{n_agents}.json'
    with open(output_path, 'w') as f:
        json.dump(config, f, indent=2)
    
    print(f'✓ Generated: scenario_scale_{n_agents}.json')
END_PYTHON
}

# run single experiment
run_single_experiment() {
    local scenario="$1"
    local method="$2"
    local run="$3"
    local config_modifications="$4"

    echo ""
    echo "========================================================"
    echo "Running Experiment: Scenario=$scenario, Method=$method, Run=$run"
    echo "========================================================"

    # Create output directory
    OUTPUT_DIR="$SCRIPT_DIR/../online_dmpc/cpp/results/scalability/${scenario}/${method}/run_${run}/"
    echo "Creating output directory: $OUTPUT_DIR"
    mkdir -p "$OUTPUT_DIR"

    # Copy config file and modify for this run
    CONFIG_FILE="$SCRIPT_DIR/../online_dmpc/cpp/config/${scenario}.json"
    OUTPUT_CONFIG_FILE="$OUTPUT_DIR/config.json"
    echo "Using config file: $CONFIG_FILE"

    # Check if config file exists
    if [ ! -f "$CONFIG_FILE" ]; then
        echo "ERROR: Config file not found: $CONFIG_FILE"
        return 1
    fi

    cp "$CONFIG_FILE" "$OUTPUT_CONFIG_FILE"

    # Apply additional config modifications
    IFS=',' read -ra MODS <<< "$config_modifications"
    for mod in "${MODS[@]}"; do
        IFS=':' read -ra PARAM <<< "$mod"
        if [ "${#PARAM[@]}" -eq 2 ]; then
            echo "Modifying config: ${PARAM[0]} = ${PARAM[1]}"
            modify_json_config "$OUTPUT_CONFIG_FILE" "${PARAM[0]}" "${PARAM[1]}"
        fi
    done

    # Update output paths in config using Python
    python3 << END_PYTHON
import json
with open('${OUTPUT_CONFIG_FILE}', 'r') as f:
    config = json.load(f)
config['output_trajectories_paths'] = ['${OUTPUT_DIR}trajectories.txt']
config['output_goals_paths'] = ['${OUTPUT_DIR}goals.txt']
with open('${OUTPUT_CONFIG_FILE}', 'w') as f:
    json.dump(config, f, indent=2)
END_PYTHON

    # Run Simulation
    cd "$SCRIPT_DIR/../online_dmpc/cpp/bin/"
    ./run "${OUTPUT_CONFIG_FILE}" > "${OUTPUT_DIR}console.log" 2>&1

    # Copy reallocation log if it exists
    if [ -f "$SCRIPT_DIR/../online_dmpc/cpp/bin/reallocation_log.csv" ]; then
        cp "$SCRIPT_DIR/../online_dmpc/cpp/bin/reallocation_log.csv" "${OUTPUT_DIR}reallocation_log.csv"
    fi

    cd $SCRIPT_DIR

    echo "Experiment complete. Results saved in: $OUTPUT_DIR"
}

run_scalability_experiments() {
    echo ""
    echo "═══════════════════════════════════════════════════════════════"
    echo "  SCALABILITY EXPERIMENTS"
    echo "═══════════════════════════════════════════════════════════════"
    echo ""
    echo "Testing reallocation algorithms with varying team sizes"
    echo "Agent counts: 4, 8, 16, 32, 64, 128"
    echo "Methods: Static, Reactive, Predictive"
    echo "Collision methods: on-demand, BVC"
    echo "Runs per configuration: $RUNS"
    echo ""

    for scenario in "${SCALABILITY_SCENARIOS[@]}"; do
        for method in "static" "reactive" "predictive"; do
            for collision_method in "on-demand" "BVC"; do
                for run in $(seq 1 $RUNS); do
                    config_modifications=""

                    if [ "$method" == "static" ]; then
                        config_modifications="reallocation_enabled:false,collision_method:${collision_method}"
                    elif [ "$method" == "reactive" ]; then
                        config_modifications="reallocation_enabled:true,_use_predictive:false,collision_method:${collision_method}"
                    elif [ "$method" == "predictive" ]; then
                        config_modifications="reallocation_enabled:true,_use_predictive:true,collision_method:${collision_method}"
                    fi

                    run_single_experiment "$scenario" "${method}/${collision_method}" "$run" "$config_modifications"
                done
            done
        done
    done

    echo ""
    echo "Scalability Experiments Completed"
    echo "Total experiments run: $(( ${#SCALABILITY_SCENARIOS[@]} * 3 * 2 * RUNS ))"
}

# Main execution
echo "=========================================="
echo "Starting Scalability Experiments"
echo "=========================================="

# Generate configs

generate_scalability_configs
echo ""

# Run the experiments
run_scalability_experiments

# Print summary
echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "  ALL SCALABILITY EXPERIMENTS COMPLETE!"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "Results saved in: $SCRIPT_DIR/../online_dmpc/cpp/results/scalability/"
echo ""
echo "To analyze results, run:"
echo "  python3 analyze_scalability.py"
echo ""
