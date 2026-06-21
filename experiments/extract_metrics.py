#!/usr/bin/env python3
import os
import re
import pandas as pd

def parse_console_log(log_path):
    """Parse the console log to extract metrics."""
    metrics = {
        'num_reallocations': 0,
        'all_goals_reached': False,
        'collisions': False,
        'avg_distance_to_goal': 0,
        'final_assignment_cost': 0,
        'avg_solving_frequency': 0,
        'cost_over_time': []
    }

    if not os.path.exists(log_path):
        print(f"Log file {log_path} not found.")
        return metrics

    with open(log_path, 'r') as f:
        log_content = f.read()

    # Extract number of reallocations
    realloc_matches = re.findall(r'=== Reallocation #(\d+)', log_content)
    metrics['num_reallocations'] = len(realloc_matches)

    # Check if all goals were reached
    metrics['all_goals_reached'] = 'All the vehicles reached their goals!' in log_content

    # Check for collisions
    metrics['collisions'] = 'No collisions found!' not in log_content

    # Extract total assignment cost
    cost_matches = re.findall(r'Total assignment cost: ([\d.e+-]+)', log_content)
    if cost_matches:
        metrics['final_assignment_cost'] = float(cost_matches[-1])
        costs = [float(cost) for cost in cost_matches]
        metrics['cost_over_time'] = costs
        metrics['avg_distance_to_goal'] = sum(costs) / len(costs)
    
    # Extract solving frequencies
    freq_matches = re.findall(r'Solving frequency = ([\d.]+) Hz', log_content)
    if freq_matches:
        frequencies = [float(freq) for freq in freq_matches]
        metrics['avg_solving_frequency'] = sum(frequencies) / len(frequencies)
    
    return metrics

def parse_reallocation_log(log_path):
    """Parse the reallocation log to extract metrics."""
    metrics = {
        'total_reallocation_distance': 0,
        'avg_reallocation_distance': 0,
        'max_reallocation_distance': 0,
        'num_agents_reallocated': 0,
        'reallocations_per_agent': {}
    }

    if not os.path.exists(log_path):
        print(f"Reallocation log file {log_path} not found.")
        return metrics
    
    df = pd.read_csv(log_path)

    if df.empty:
        return metrics
    
    # Total and average reallocation distance
    metrics['total_reallocation_distance'] = df['distance'].sum()
    metrics['avg_reallocation_distance'] = df['distance'].mean()
    metrics['max_reallocation_distance'] = df['distance'].max()

    # Number of unique agents reallocated
    metrics['num_agents_reallocated'] = df['agent_id'].nunique()

    # Number of reallocations per agent
    realloc_counts = df['agent_id'].value_counts().to_dict()
    metrics['reallocations_per_agent'] = realloc_counts

    return metrics

def extract_all_metrics():
    """Extract metrics from all experiment result directories."""
    results = []

    base_dir = './online_dmpc/cpp/results/experiments'
    
    # All scenarios from run_comprehensive_experiments.sh
    scenarios = [
        'scenario_1', 'scenario_2', 'scenario_3',  # baseline
        'scenario_4', 'scenario_5', 'scenario_6',  # collision
        'scenario_7', 'scenario_8', 'scenario_9'   # dynamic goals
    ]
    
    # Methods used in the experiment script
    methods = ['static', 'reactive', 'predictive']
    
    # Number of runs per experiment (from RUNS=5 in bash script)
    num_runs = 5

    for scenario in scenarios:
        for method in methods:
            for run in range(1, num_runs + 1):
                run_dir = f'{base_dir}/{scenario}/{method}/run_{run}'

                if not os.path.exists(run_dir):
                    print(f"{run_dir} not found")
                    continue

                console_log = f'{run_dir}/console.log'
                console_metrics = parse_console_log(console_log) 
                
                reallocation_log = f'{run_dir}/reallocation_log.csv'
                realloc_metrics = parse_reallocation_log(reallocation_log)

                result = {
                    'scenario': scenario,
                    'method': method,
                    'run': run,
                    **console_metrics,
                    **realloc_metrics
                }

                results.append(result)

                print(f"Extracted: {scenario} - {method} - run {run}")

    results_df = pd.DataFrame(results)
    output_path = f'{base_dir}/aggregated_metrics.csv'
    results_df.to_csv(output_path, index=False)
    print(f"Aggregated metrics saved to {output_path}")

    return results_df


if __name__ == "__main__":
    df = extract_all_metrics()
    print("\n=== Aggregated Metrics ===")
    print(df.head())