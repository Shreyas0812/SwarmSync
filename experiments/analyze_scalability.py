#!/usr/bin/env python3
"""
Scalability Analysis Script for DMPC Experiments
Analyzes performance metrics across different team sizes (4, 8, 16, 32, 64, 128 agents)
"""

import os
import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from pathlib import Path

# Set style
plt.style.use('seaborn-v0_8-darkgrid')
sns.set_palette("husl")


def parse_console_log(log_path):
    """Parse the console log to extract metrics."""
    metrics = {
        'num_reallocations': 0,
        'all_goals_reached': False,
        'collisions': False,
        'avg_distance_to_goal': 0,
        'final_assignment_cost': 0,
        'avg_solving_frequency': 0,
        'total_computation_time': 0,
        'cost_over_time': []
    }

    if not os.path.exists(log_path):
        print(f"Warning: Log file {log_path} not found.")
        return metrics

    with open(log_path, 'r') as f:
        log_content = f.read()

    # Extract number of reallocations
    realloc_matches = re.findall(r'=== Reallocation #(\d+)', log_content)
    metrics['num_reallocations'] = len(realloc_matches)

    # Check if all goals were reached
    metrics['all_goals_reached'] = 'All the vehicles reached their goals!' in log_content

    # Parse individual vehicle goal distances
    vehicle_goal_distances = re.findall(r'Vehicle \d+ did not reached its goal by ([\d.]+) m', log_content)
    if vehicle_goal_distances:
        distances = [float(d) for d in vehicle_goal_distances]
        metrics['avg_distance_to_goal'] = sum(distances) / len(distances)
    
    # Check for collisions
    metrics['collisions'] = 'No collisions found!' not in log_content

    # Extract total assignment cost
    cost_matches = re.findall(r'Total assignment cost: ([\d.e+-]+)', log_content)
    if cost_matches:
        metrics['final_assignment_cost'] = float(cost_matches[-1])
        costs = [float(cost) for cost in cost_matches]
        metrics['cost_over_time'] = costs
    
    # Extract solving frequencies
    freq_matches = re.findall(r'Solving frequency = ([\d.]+) Hz', log_content)
    if freq_matches:
        frequencies = [float(freq) for freq in freq_matches]
        metrics['avg_solving_frequency'] = sum(frequencies) / len(frequencies)
        # Estimate total computation time
        metrics['total_computation_time'] = len(frequencies) / metrics['avg_solving_frequency'] if metrics['avg_solving_frequency'] > 0 else 0
    
    return metrics


def parse_reallocation_log(log_path, method='reactive'):
    """Parse the reallocation log to extract metrics."""
    metrics = {
        'total_reallocation_distance': 0,
        'avg_reallocation_distance': 0,
        'max_reallocation_distance': 0,
        'num_agents_reallocated': 0,
        'reallocations_per_agent': 0
    }

    # Static method should not have reallocations
    if method == 'static':
        return metrics
    
    if not os.path.exists(log_path):
        return metrics
    
    try:
        df = pd.read_csv(log_path)
        
        if df.empty:
            return metrics
        
        # Total and average reallocation distance
        metrics['total_reallocation_distance'] = df['distance'].sum()
        metrics['avg_reallocation_distance'] = df['distance'].mean()
        metrics['max_reallocation_distance'] = df['distance'].max()

        # Number of unique agents reallocated
        metrics['num_agents_reallocated'] = df['agent_id'].nunique()

        # Average reallocations per agent
        realloc_counts = df['agent_id'].value_counts()
        metrics['reallocations_per_agent'] = realloc_counts.mean()
    except Exception as e:
        print(f"Warning: Error parsing {log_path}: {e}")
    
    return metrics


def extract_scalability_metrics():
    """Extract metrics from all scalability experiment result directories."""
    results = []

    base_dir = './online_dmpc/cpp/results/scalability'
    
    # Scalability scenarios
    # scenarios = ['scenario_scale_4', 'scenario_scale_8', 'scenario_scale_16', 
    #              'scenario_scale_32', 'scenario_scale_64', 'scenario_scale_128']

    scenarios = ['scenario_scale_4', 'scenario_scale_8', 'scenario_scale_16', 
                 'scenario_scale_32', 'scenario_scale_64']
    
    # Extract agent count from scenario name
    agent_counts = {scenario: int(scenario.split('_')[-1]) for scenario in scenarios}
    
    # Methods and collision avoidance methods used in the experiment script
    methods = ['static', 'reactive', 'predictive']
    collision_methods = ['BVC', 'on-demand']
    
    # Number of runs per experiment (from RUNS=3 in bash script)
    num_runs = 3

    print("="*60)
    print("EXTRACTING SCALABILITY METRICS")
    print("="*60)

    for scenario in scenarios:
        for method in methods:
            for collision_method in collision_methods:
                for run in range(1, num_runs + 1):
                    run_dir = f'{base_dir}/{scenario}/{method}/{collision_method}/run_{run}'

                    if not os.path.exists(run_dir):
                        print(f"⚠ {run_dir} not found")
                        continue

                    console_log = f'{run_dir}/console.log'
                    console_metrics = parse_console_log(console_log) 
                    
                    reallocation_log = f'{run_dir}/reallocation_log.csv'
                    realloc_metrics = parse_reallocation_log(reallocation_log, method=method)

                    result = {
                        'scenario': scenario,
                        'num_agents': agent_counts[scenario],
                        'method': method,
                        'collision_method': collision_method,
                        'run': run,
                        **console_metrics,
                        **realloc_metrics
                    }

                    results.append(result)

                    print(f"✓ Extracted: {scenario} ({agent_counts[scenario]} agents) - {method}/{collision_method} - run {run}")

    if not results:
        print("\n❌ No results found! Have you run the scalability experiments?")
        print("   Run: ./run_scalability_experiments.sh")
        return None

    results_df = pd.DataFrame(results)
    output_path = f'{base_dir}/scalability_metrics.csv'
    results_df.to_csv(output_path, index=False)
    print(f"\n✓ Aggregated metrics saved to {output_path}")

    return results_df


def plot_scalability_analysis(df):
    """Generate comprehensive scalability analysis plots."""
    
    output_dir = Path("./online_dmpc/cpp/results/scalability/figures")
    output_dir.mkdir(exist_ok=True)
    
    print("\n" + "="*60)
    print("GENERATING SCALABILITY PLOTS")
    print("="*60)

    # Group by number of agents, method, and collision_method
    grouped = df.groupby(['num_agents', 'method', 'collision_method']).agg({
        'avg_solving_frequency': ['mean', 'std'],
        'num_reallocations': ['mean', 'std'],
        'all_goals_reached': 'mean',
        'collisions': 'mean',
        'avg_distance_to_goal': ['mean', 'std'],
        'final_assignment_cost': ['mean', 'std'],
        'total_reallocation_distance': ['mean', 'std'],
        'avg_reallocation_distance': ['mean', 'std']
    }).reset_index()

    # Flatten column names
    grouped.columns = ['_'.join(col).strip('_') if col[1] else col[0] for col in grouped.columns.values]

    # For plotting, we'll focus on BVC collision method (or aggregate across collision methods)
    # Filter to BVC if available, otherwise use all data
    if 'collision_method' in grouped.columns:
        plot_data = grouped[grouped['collision_method'] == 'BVC'] if 'BVC' in grouped['collision_method'].values else grouped
    else:
        plot_data = grouped

    # 1. Solving Frequency vs Team Size
    fig, ax = plt.subplots(figsize=(10, 6))
    
    for method in ['static', 'reactive', 'predictive']:
        method_data = plot_data[plot_data['method'] == method]
        if method_data.empty:
            continue
        ax.errorbar(method_data['num_agents'], 
                   method_data['avg_solving_frequency_mean'],
                   yerr=method_data['avg_solving_frequency_std'],
                   marker='o', linewidth=2, markersize=8,
                   capsize=5, label=method.capitalize())
    
    ax.set_xlabel('Number of Agents', fontsize=12, fontweight='bold')
    ax.set_ylabel('Average Solving Frequency (Hz)', fontsize=12, fontweight='bold')
    ax.set_title('Computational Scalability: Solving Frequency vs Team Size', 
                fontsize=14, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    if not plot_data.empty:
        ax.set_xticks(sorted(plot_data['num_agents'].unique()))
    ax.set_xscale('log', base=2)
    
    plt.tight_layout()
    plt.savefig(output_dir / "scalability_01_solving_frequency.png", dpi=300, bbox_inches='tight')
    print("✓ Saved: scalability_01_solving_frequency.png")
    plt.close()

    # 2. Number of Reallocations vs Team Size
    fig, ax = plt.subplots(figsize=(10, 6))
    
    for method in ['reactive', 'predictive']:
        method_data = plot_data[plot_data['method'] == method]
        if method_data.empty:
            continue
        ax.errorbar(method_data['num_agents'], 
                   method_data['num_reallocations_mean'],
                   yerr=method_data['num_reallocations_std'],
                   marker='s', linewidth=2, markersize=8,
                   capsize=5, label=method.capitalize())
    
    ax.set_xlabel('Number of Agents', fontsize=12, fontweight='bold')
    ax.set_ylabel('Number of Reallocations', fontsize=12, fontweight='bold')
    ax.set_title('Reallocation Frequency vs Team Size', 
                fontsize=14, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    if not plot_data.empty:
        ax.set_xticks(sorted(plot_data['num_agents'].unique()))
    ax.set_xscale('log', base=2)
    
    plt.tight_layout()
    plt.savefig(output_dir / "scalability_02_num_reallocations.png", dpi=300, bbox_inches='tight')
    print("✓ Saved: scalability_02_num_reallocations.png")
    plt.close()

    # 3. Success Rate vs Team Size
    fig, ax = plt.subplots(figsize=(10, 6))
    
    for method in ['static', 'reactive', 'predictive']:
        method_data = plot_data[plot_data['method'] == method]
        if method_data.empty:
            continue
        ax.plot(method_data['num_agents'], 
               method_data['all_goals_reached_mean'] * 100,
               marker='o', linewidth=2, markersize=8,
               label=method.capitalize())
    
    ax.set_xlabel('Number of Agents', fontsize=12, fontweight='bold')
    ax.set_ylabel('Success Rate (%)', fontsize=12, fontweight='bold')
    ax.set_title('Mission Success Rate vs Team Size', 
                fontsize=14, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    if not plot_data.empty:
        ax.set_xticks(sorted(plot_data['num_agents'].unique()))
    ax.set_xscale('log', base=2)
    ax.set_ylim([0, 105])
    
    plt.tight_layout()
    plt.savefig(output_dir / "scalability_03_success_rate.png", dpi=300, bbox_inches='tight')
    print("✓ Saved: scalability_03_success_rate.png")
    plt.close()

    # 4. Average Distance to Goal vs Team Size
    fig, ax = plt.subplots(figsize=(10, 6))
    
    for method in ['static', 'reactive', 'predictive']:
        method_data = plot_data[plot_data['method'] == method]
        if method_data.empty:
            continue
        ax.errorbar(method_data['num_agents'], 
                   method_data['avg_distance_to_goal_mean'],
                   yerr=method_data['avg_distance_to_goal_std'],
                   marker='d', linewidth=2, markersize=8,
                   capsize=5, label=method.capitalize())
    
    ax.set_xlabel('Number of Agents', fontsize=12, fontweight='bold')
    ax.set_ylabel('Average Distance to Goal (m)', fontsize=12, fontweight='bold')
    ax.set_title('Goal Tracking Performance vs Team Size', 
                fontsize=14, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    if not plot_data.empty:
        ax.set_xticks(sorted(plot_data['num_agents'].unique()))
    ax.set_xscale('log', base=2)
    
    plt.tight_layout()
    plt.savefig(output_dir / "scalability_04_distance_to_goal.png", dpi=300, bbox_inches='tight')
    print("✓ Saved: scalability_04_distance_to_goal.png")
    plt.close()

    # 5. Total Reallocation Distance vs Team Size
    fig, ax = plt.subplots(figsize=(10, 6))
    
    for method in ['reactive', 'predictive']:
        method_data = plot_data[plot_data['method'] == method]
        if method_data.empty:
            continue
        ax.errorbar(method_data['num_agents'], 
                   method_data['total_reallocation_distance_mean'],
                   yerr=method_data['total_reallocation_distance_std'],
                   marker='^', linewidth=2, markersize=8,
                   capsize=5, label=method.capitalize())
    
    ax.set_xlabel('Number of Agents', fontsize=12, fontweight='bold')
    ax.set_ylabel('Total Reallocation Distance (m)', fontsize=12, fontweight='bold')
    ax.set_title('Reallocation Overhead vs Team Size', 
                fontsize=14, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    if not plot_data.empty:
        ax.set_xticks(sorted(plot_data['num_agents'].unique()))
    ax.set_xscale('log', base=2)
    
    plt.tight_layout()
    plt.savefig(output_dir / "scalability_05_reallocation_distance.png", dpi=300, bbox_inches='tight')
    print("✓ Saved: scalability_05_reallocation_distance.png")
    plt.close()

    # 6. Comprehensive Comparison (2x2 subplot)
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Subplot 1: Solving Frequency
    for method in ['static', 'reactive', 'predictive']:
        method_data = plot_data[plot_data['method'] == method]
        if method_data.empty:
            continue
        axes[0, 0].plot(method_data['num_agents'], 
                       method_data['avg_solving_frequency_mean'],
                       marker='o', linewidth=2, markersize=6, label=method.capitalize())
    axes[0, 0].set_xlabel('Number of Agents', fontweight='bold')
    axes[0, 0].set_ylabel('Solving Frequency (Hz)', fontweight='bold')
    axes[0, 0].set_title('(a) Computational Performance', fontweight='bold')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    if not plot_data.empty:
        axes[0, 0].set_xticks(sorted(plot_data['num_agents'].unique()))
    axes[0, 0].set_xscale('log', base=2)
    
    # Subplot 2: Success Rate
    for method in ['static', 'reactive', 'predictive']:
        method_data = plot_data[plot_data['method'] == method]
        if method_data.empty:
            continue
        axes[0, 1].plot(method_data['num_agents'], 
                       method_data['all_goals_reached_mean'] * 100,
                       marker='s', linewidth=2, markersize=6, label=method.capitalize())
    axes[0, 1].set_xlabel('Number of Agents', fontweight='bold')
    axes[0, 1].set_ylabel('Success Rate (%)', fontweight='bold')
    axes[0, 1].set_title('(b) Mission Success', fontweight='bold')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    if not plot_data.empty:
        axes[0, 1].set_xticks(sorted(plot_data['num_agents'].unique()))
    axes[0, 1].set_xscale('log', base=2)
    axes[0, 1].set_ylim([0, 105])
    
    # Subplot 3: Number of Reallocations
    for method in ['reactive', 'predictive']:
        method_data = plot_data[plot_data['method'] == method]
        if method_data.empty:
            continue
        axes[1, 0].plot(method_data['num_agents'], 
                       method_data['num_reallocations_mean'],
                       marker='d', linewidth=2, markersize=6, label=method.capitalize())
    axes[1, 0].set_xlabel('Number of Agents', fontweight='bold')
    axes[1, 0].set_ylabel('Number of Reallocations', fontweight='bold')
    axes[1, 0].set_title('(c) Reallocation Frequency', fontweight='bold')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    if not plot_data.empty:
        axes[1, 0].set_xticks(sorted(plot_data['num_agents'].unique()))
    axes[1, 0].set_xscale('log', base=2)
    
    # Subplot 4: Average Distance to Goal
    for method in ['static', 'reactive', 'predictive']:
        method_data = plot_data[plot_data['method'] == method]
        if method_data.empty:
            continue
        axes[1, 1].plot(method_data['num_agents'], 
                       method_data['avg_distance_to_goal_mean'],
                       marker='^', linewidth=2, markersize=6, label=method.capitalize())
    axes[1, 1].set_xlabel('Number of Agents', fontweight='bold')
    axes[1, 1].set_ylabel('Avg Distance to Goal (m)', fontweight='bold')
    axes[1, 1].set_title('(d) Goal Tracking Performance', fontweight='bold')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    if not plot_data.empty:
        axes[1, 1].set_xticks(sorted(plot_data['num_agents'].unique()))
    axes[1, 1].set_xscale('log', base=2)
    
    plt.suptitle('Scalability Analysis: Comprehensive Comparison', 
                fontsize=16, fontweight='bold', y=0.995)
    plt.tight_layout()
    plt.savefig(output_dir / "scalability_00_comprehensive.png", dpi=300, bbox_inches='tight')
    print("✓ Saved: scalability_00_comprehensive.png")
    plt.close()

    print(f"\n✓ All plots saved to: {output_dir}")


def print_summary_statistics(df):
    """Print summary statistics for scalability analysis."""
    
    print("\n" + "="*60)
    print("SCALABILITY SUMMARY STATISTICS")
    print("="*60)
    
    for num_agents in sorted(df['num_agents'].unique()):
        print(f"\n{'─'*60}")
        print(f"  {num_agents} AGENTS")
        print(f"{'─'*60}")
        
        agent_df = df[df['num_agents'] == num_agents]
        
        for method in ['static', 'reactive', 'predictive']:
            for collision_method in df['collision_method'].unique():
                method_df = agent_df[(agent_df['method'] == method) & 
                                    (agent_df['collision_method'] == collision_method)]
            
                if method_df.empty:
                    continue
                
                print(f"\n  {method.upper()} ({collision_method}):")
                print(f"    Solving Frequency: {method_df['avg_solving_frequency'].mean():.2f} ± {method_df['avg_solving_frequency'].std():.2f} Hz")
                print(f"    Success Rate: {method_df['all_goals_reached'].mean()*100:.1f}%")
                print(f"    Collision Rate: {method_df['collisions'].mean()*100:.1f}%")
                print(f"    Avg Distance to Goal: {method_df['avg_distance_to_goal'].mean():.3f} ± {method_df['avg_distance_to_goal'].std():.3f} m")
                
                if method != 'static':
                    print(f"    Num Reallocations: {method_df['num_reallocations'].mean():.1f} ± {method_df['num_reallocations'].std():.1f}")
                    print(f"    Total Realloc Distance: {method_df['total_reallocation_distance'].mean():.2f} ± {method_df['total_reallocation_distance'].std():.2f} m")
    
    # Performance degradation analysis
    print(f"\n{'='*60}")
    print("PERFORMANCE DEGRADATION ANALYSIS")
    print(f"{'='*60}")
    
    grouped = df.groupby(['num_agents', 'method', 'collision_method'])['avg_solving_frequency'].mean().reset_index()
    
    for method in ['static', 'reactive', 'predictive']:
        for collision_method in df['collision_method'].unique():
            method_data = grouped[(grouped['method'] == method) & 
                                 (grouped['collision_method'] == collision_method)].sort_values('num_agents')
            
            if len(method_data) < 2:
                continue
            
            min_agents = method_data['num_agents'].min()
            max_agents = method_data['num_agents'].max()
            
            freq_min = method_data[method_data['num_agents'] == min_agents]['avg_solving_frequency'].values
            freq_max = method_data[method_data['num_agents'] == max_agents]['avg_solving_frequency'].values
            
            if len(freq_min) > 0 and len(freq_max) > 0:
                degradation = ((freq_min[0] - freq_max[0]) / freq_min[0]) * 100
                print(f"\n  {method.capitalize()} ({collision_method}): {degradation:.1f}% frequency reduction ({min_agents}→{max_agents} agents)")
                print(f"    {min_agents} agents:  {freq_min[0]:.2f} Hz")
                print(f"    {max_agents} agents: {freq_max[0]:.2f} Hz")


def main():
    """Main execution function."""
    print("\n" + "="*60)
    print("  SCALABILITY ANALYSIS FOR DMPC EXPERIMENTS")
    print("="*60 + "\n")
    
    # Check if results exist, if not extract them
    metrics_file = Path("./online_dmpc/cpp/results/scalability/scalability_metrics.csv")
    
    if metrics_file.exists():
        print(f"Loading existing metrics from: {metrics_file}")
        df = pd.read_csv(metrics_file)
    else:
        print("Metrics file not found. Extracting from experiment results...")
        df = extract_scalability_metrics()
        
        if df is None:
            return
    
    # Print summary statistics
    print_summary_statistics(df)
    
    # Generate plots
    plot_scalability_analysis(df)
    
    print("\n" + "="*60)
    print("  ANALYSIS COMPLETE!")
    print("="*60)
    print(f"\nResults:")
    print(f"  • Metrics CSV: ./online_dmpc/cpp/results/scalability/scalability_metrics.csv")
    print(f"  • Figures: ./online_dmpc/cpp/results/scalability/figures/")
    print("\n")


if __name__ == "__main__":
    main()
