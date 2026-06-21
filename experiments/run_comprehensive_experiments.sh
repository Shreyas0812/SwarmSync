#!/bin/bash

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Script directory: $SCRIPT_DIR"

cd $SCRIPT_DIR

RUNS=3
SCENARIOS=(
    "scenario_1"  # Cross pattern - clear benefit case
    "scenario_2"  # Dense cross - moderate conflict
    "scenario_3"  # Circle formation - high conflict
    "scenario_4"  # Cross with BVC - conservative collision
    "scenario_5"  # Cross with on-demand - aggressive collision
    "scenario_6"  # Dense collision method comparison
    "scenario_7"  # Translating goals - dynamic targets
    "scenario_8"  # Circular goals - complex motion
    "scenario_9"  # Combined motion - ultimate challenge
)

# Creating results directory
mkdir -p "$SCRIPT_DIR/../online_dmpc/cpp/results/experiments/"


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

# Function to run 1 experiment
run_single_experiment() {
    local scenario="$1"
    local method="$2"
    local run="$3"
    local config_modifications="$4"

    echo ""
    echo "=========================================="
    echo "Running Experiment: Scenario=$scenario, Method=$method, Run=$run"
    echo "=========================================="

    # Create output directory
    OUTPUT_DIR="$SCRIPT_DIR/../online_dmpc/cpp/results/experiments/${scenario}/${method}/run_${run}/"
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

run_baseline_experiments() {
    echo "Running Baseline Experiments"

    BASELINE_SCENARIOS=("scenario_1" "scenario_2" "scenario_3")

    for scenario in "${BASELINE_SCENARIOS[@]}"; do
        echo ">>> Testing Scenario: $scenario"
        
        for run in $(seq 1 $RUNS); do
            run_single_experiment "$scenario" "static" "$run" "reallocation_enabled:false"
        done

        for run in $(seq 1 $RUNS); do
            run_single_experiment "$scenario" "reactive" "$run" "reallocation_enabled:true,_use_predictive:false,reallocation_period:2.0"
        done

        for run in $(seq 1 $RUNS); do
            run_single_experiment "$scenario" "predictive" "$run" "reallocation_enabled:true,_use_predictive:true,reallocation_period:2.0,prediction_horizon:1.0"
        done
    done

    echo "Baseline Experiments Completed"
    echo " Total experiments run: $(( ${#BASELINE_SCENARIOS[@]} * 3 * RUNS ))"
}

run_collision_experiments() {
    echo "Running Collision Method Comparison Experiments"

    COLLISION_SCENARIOS=("scenario_4" "scenario_5" "scenario_6")
    COLLISION_TYPES=("BVC" "ONDemand")

    for scenario in "${COLLISION_SCENARIOS[@]}"; do
        for collision_type in "${COLLISION_TYPES[@]}"; do
            echo ">>> Testing Scenario: $scenario with Collision Type: $collision_type"
            
            # static method
            for run in $(seq 1 $RUNS); do
                run_single_experiment "$scenario" "static" "$run" "reallocation_enabled:false,collision_method:${collision_type}"
            done

            for run in $(seq 1 $RUNS); do
                run_single_experiment "$scenario" "reactive" "$run" "reallocation_enabled:true,_use_predictive:false,reallocation_period:2.0,collision_method:${collision_type}"
            done

            for run in $(seq 1 $RUNS); do
                run_single_experiment "$scenario" "predictive" "$run" "reallocation_enabled:true,_use_predictive:true,reallocation_period:2.0,prediction_horizon:1.0,collision_method:${collision_type}"
            done

        done
    done

    echo "Collision Method Comparison Experiments Completed"
    echo " Total experiments run: $(( ${#COLLISION_METHODS[@]} * RUNS ))"
}

run_dynamic_goal_experiments() {
    echo "Running Dynamic Goal Experiments"

    DYNAMIC_GOAL_SCENARIOS=("scenario_7" "scenario_8" "scenario_9")

    for scenario in "${DYNAMIC_GOAL_SCENARIOS[@]}"; do
        echo ">>> Testing Scenario: $scenario"
        
        for run in $(seq 1 $RUNS); do
            run_single_experiment "$scenario" "static" "$run" "reallocation_enabled:false"
        done

        for run in $(seq 1 $RUNS); do
            run_single_experiment "$scenario" "reactive" "$run" "reallocation_enabled:true,_use_predictive:false,reallocation_period:2.0"
        done

        for run in $(seq 1 $RUNS); do
            run_single_experiment "$scenario" "predictive" "$run" "reallocation_enabled:true,_use_predictive:true,reallocation_period:2.0,prediction_horizon:1.0"
        done
    done

    echo "Dynamic Goal Experiments Completed"
    echo " Total experiments run: $(( ${#DYNAMIC_GOAL_SCENARIOS[@]} * 3 * RUNS ))"
}

echo "=========================================="
echo "Starting Experiments"
echo "=========================================="

EXPERIMENT_TYPE="${1:-full}"

case "$EXPERIMENT_TYPE" in
    baseline)
        run_baseline_experiments
        ;;
    collision)
        run_collision_experiments
        ;;
    dynamic_goals)
        run_dynamic_goal_experiments
        ;;
    full)
        echo "Running: All Experiments"
        run_baseline_experiments
        run_collision_experiments
        run_dynamic_goal_experiments
        ;;
    *)
        echo "ERROR: Invalid experiment type: $EXPERIMENT_TYPE"
        echo "Usage: $0 {baseline|collision|dynamic_goals|full}"
        exit 1
        ;;
esac

# Print summary
echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "  ALL EXPERIMENTS COMPLETE!"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "Results saved in: $SCRIPT_DIR/../online_dmpc/cpp/results/experiments/"
echo ""