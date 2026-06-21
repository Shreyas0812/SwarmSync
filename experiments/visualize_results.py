#!/usr/bin/env python3
"""
Visualization script for DMPC experiment results
Compares static vs. with_realloc methods across different scenarios
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import json
import ast
from pathlib import Path
import seaborn as sns

# Set style
plt.style.use('seaborn-v0_8-darkgrid')
sns.set_palette("husl")

# Load data
data_file = Path(__file__).parent / "online_dmpc/cpp/results/experiments/aggregated_metrics.csv"
df = pd.read_csv(data_file)

# Parse cost_over_time column (it's stored as string representation of list)
df['cost_over_time'] = df['cost_over_time'].apply(lambda x: ast.literal_eval(x) if x != '[]' else [])

# Create output directory
output_dir = Path(__file__).parent / "online_dmpc/cpp/results/experiments/figures"
output_dir.mkdir(exist_ok=True)

print("="*60)
print("DMPC EXPERIMENT VISUALIZATION")
print("="*60)
print(f"\nTotal runs: {len(df)}")
print(f"Scenarios: {df['scenario'].unique()}")
print(f"Methods: {df['method'].unique()}")
print(f"Runs per scenario-method: {df.groupby(['scenario', 'method']).size().unique()}")


# ============================================================================
# 1. PERFORMANCE COMPARISON: Solving Frequency
# ============================================================================
def plot_solving_frequency():
    """Compare average solving frequency between methods"""
    fig, ax = plt.subplots(figsize=(12, 6))
    
    # Group by scenario and method, calculate mean and std
    grouped = df.groupby(['scenario', 'method'])['avg_solving_frequency'].agg(['mean', 'std']).reset_index()
    
    scenarios = grouped['scenario'].unique()
    x = np.arange(len(scenarios))
    width = 0.25
    
    static_data = grouped[grouped['method'] == 'static']
    reactive_data = grouped[grouped['method'] == 'reactive']
    predictive_data = grouped[grouped['method'] == 'predictive']
    
    bars1 = ax.bar(x - width, static_data['mean'], width, 
                   yerr=static_data['std'], label='Static', 
                   capsize=5, alpha=0.8)
    bars2 = ax.bar(x, reactive_data['mean'], width, 
                   yerr=reactive_data['std'], label='Reactive',
                   capsize=5, alpha=0.8)
    bars3 = ax.bar(x + width, predictive_data['mean'], width, 
                   yerr=predictive_data['std'], label='Predictive',
                   capsize=5, alpha=0.8)
    
    ax.set_xlabel('Scenario', fontsize=12, fontweight='bold')
    ax.set_ylabel('Avg Solving Frequency (Hz)', fontsize=12, fontweight='bold')
    ax.set_title('MPC Solving Frequency Comparison', fontsize=14, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(scenarios)
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / "01_solving_frequency.png", dpi=300, bbox_inches='tight')
    print("\n✓ Saved: 01_solving_frequency.png")
    plt.close()


# ============================================================================
# 2. GOAL ACHIEVEMENT METRICS
# ============================================================================
def plot_goal_metrics():
    """Plot average distance to goal and goal achievement"""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Distance to goal
    grouped = df.groupby(['scenario', 'method'])['avg_distance_to_goal'].agg(['mean', 'std']).reset_index()
    scenarios = grouped['scenario'].unique()
    x = np.arange(len(scenarios))
    width = 0.25
    
    static_data = grouped[grouped['method'] == 'static']
    reactive_data = grouped[grouped['method'] == 'reactive']
    predictive_data = grouped[grouped['method'] == 'predictive']
    
    axes[0].bar(x - width, static_data['mean'], width, 
                yerr=static_data['std'], label='Static', 
                capsize=5, alpha=0.8)
    axes[0].bar(x, reactive_data['mean'], width, 
                yerr=reactive_data['std'], label='Reactive',
                capsize=5, alpha=0.8)
    axes[0].bar(x + width, predictive_data['mean'], width, 
                yerr=predictive_data['std'], label='Predictive',
                capsize=5, alpha=0.8)
    
    axes[0].set_xlabel('Scenario', fontsize=11, fontweight='bold')
    axes[0].set_ylabel('Avg Distance to Goal', fontsize=11, fontweight='bold')
    axes[0].set_title('Average Distance to Goal', fontsize=12, fontweight='bold')
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(scenarios)
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # Success rate
    success_rate = df.groupby(['scenario', 'method'])['all_goals_reached'].mean().reset_index()
    success_rate['all_goals_reached'] *= 100  # Convert to percentage
    
    for i, scenario in enumerate(scenarios):
        scenario_data = success_rate[success_rate['scenario'] == scenario]
        static_val = scenario_data[scenario_data['method'] == 'static']['all_goals_reached'].values[0]
        reactive_val = scenario_data[scenario_data['method'] == 'reactive']['all_goals_reached'].values[0]
        predictive_val = scenario_data[scenario_data['method'] == 'predictive']['all_goals_reached'].values[0]
        
        axes[1].bar(x[i] - width, static_val, width, alpha=0.8, color='C0')
        axes[1].bar(x[i], reactive_val, width, alpha=0.8, color='C1')
        axes[1].bar(x[i] + width, predictive_val, width, alpha=0.8, color='C2')
    
    axes[1].set_xlabel('Scenario', fontsize=11, fontweight='bold')
    axes[1].set_ylabel('Success Rate (%)', fontsize=11, fontweight='bold')
    axes[1].set_title('Goal Achievement Rate', fontsize=12, fontweight='bold')
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(scenarios)
    axes[1].set_ylim([0, 105])
    axes[1].grid(True, alpha=0.3)
    
    # Add legend to second plot
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor='C0', alpha=0.8, label='Static'),
                      Patch(facecolor='C1', alpha=0.8, label='Reactive'),
                      Patch(facecolor='C2', alpha=0.8, label='Predictive')]
    axes[1].legend(handles=legend_elements)
    
    plt.tight_layout()
    plt.savefig(output_dir / "02_goal_metrics.png", dpi=300, bbox_inches='tight')
    print("✓ Saved: 02_goal_metrics.png")
    plt.close()


# ============================================================================
# 3. COST CONVERGENCE OVER TIME
# ============================================================================
def plot_cost_convergence():
    """Plot cost convergence over time for scenarios with reallocation"""
    # Show both reactive and predictive methods
    realloc_df = df[df['method'].isin(['reactive', 'predictive'])].copy()
    
    scenarios = realloc_df['scenario'].unique()
    fig, axes = plt.subplots(3, 3, figsize=(16, 12))
    axes = axes.flatten()
    
    for idx, scenario in enumerate(scenarios):
        scenario_data = realloc_df[realloc_df['scenario'] == scenario]
        
        ax = axes[idx]
        
        for _, row in scenario_data.iterrows():
            cost_series = row['cost_over_time']
            if len(cost_series) > 0:
                ax.plot(cost_series, alpha=0.5, linewidth=1)
        
        # Plot mean
        all_costs = [row['cost_over_time'] for _, row in scenario_data.iterrows()]
        max_len = max(len(c) for c in all_costs)
        
        # Pad shorter sequences with NaN
        padded_costs = np.array([c + [np.nan]*(max_len - len(c)) for c in all_costs])
        mean_cost = np.nanmean(padded_costs, axis=0)
        
        ax.plot(mean_cost, 'k-', linewidth=2.5, label='Mean')
        ax.set_yscale('log')
        ax.set_xlabel('Time Step', fontsize=10, fontweight='bold')
        ax.set_ylabel('Assignment Cost (log scale)', fontsize=10, fontweight='bold')
        ax.set_title(f'{scenario}', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend()
    
    fig.suptitle('Cost Convergence with Reallocation', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()
    plt.savefig(output_dir / "03_cost_convergence.png", dpi=300, bbox_inches='tight')
    print("✓ Saved: 03_cost_convergence.png")
    plt.close()


# ============================================================================
# 4. REALLOCATION STATISTICS
# ============================================================================
def plot_reallocation_stats():
    """Plot reallocation distance statistics"""
    # Only analyze reactive and predictive methods (which use reallocation)
    realloc_df = df[df['method'].isin(['reactive', 'predictive'])].copy()
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    scenarios = realloc_df['scenario'].unique()
    x = np.arange(len(scenarios))
    
    # Total reallocation distance
    grouped = realloc_df.groupby('scenario')['total_reallocation_distance'].agg(['mean', 'std']).reset_index()
    axes[0, 0].bar(x, grouped['mean'], yerr=grouped['std'], capsize=5, alpha=0.8, color='steelblue')
    axes[0, 0].set_xlabel('Scenario', fontweight='bold')
    axes[0, 0].set_ylabel('Distance', fontweight='bold')
    axes[0, 0].set_title('Total Reallocation Distance', fontweight='bold')
    axes[0, 0].set_xticks(x)
    axes[0, 0].set_xticklabels(scenarios)
    axes[0, 0].grid(True, alpha=0.3)
    
    # Average reallocation distance
    grouped = realloc_df.groupby('scenario')['avg_reallocation_distance'].agg(['mean', 'std']).reset_index()
    axes[0, 1].bar(x, grouped['mean'], yerr=grouped['std'], capsize=5, alpha=0.8, color='coral')
    axes[0, 1].set_xlabel('Scenario', fontweight='bold')
    axes[0, 1].set_ylabel('Distance', fontweight='bold')
    axes[0, 1].set_title('Average Reallocation Distance', fontweight='bold')
    axes[0, 1].set_xticks(x)
    axes[0, 1].set_xticklabels(scenarios)
    axes[0, 1].grid(True, alpha=0.3)
    
    # Max reallocation distance
    grouped = realloc_df.groupby('scenario')['max_reallocation_distance'].agg(['mean', 'std']).reset_index()
    axes[1, 0].bar(x, grouped['mean'], yerr=grouped['std'], capsize=5, alpha=0.8, color='mediumseagreen')
    axes[1, 0].set_xlabel('Scenario', fontweight='bold')
    axes[1, 0].set_ylabel('Distance', fontweight='bold')
    axes[1, 0].set_title('Maximum Reallocation Distance', fontweight='bold')
    axes[1, 0].set_xticks(x)
    axes[1, 0].set_xticklabels(scenarios)
    axes[1, 0].grid(True, alpha=0.3)
    
    # Number of agents reallocated
    grouped = realloc_df.groupby('scenario')['num_agents_reallocated'].agg(['mean', 'std']).reset_index()
    axes[1, 1].bar(x, grouped['mean'], yerr=grouped['std'], capsize=5, alpha=0.8, color='mediumpurple')
    axes[1, 1].set_xlabel('Scenario', fontweight='bold')
    axes[1, 1].set_ylabel('Number of Agents', fontweight='bold')
    axes[1, 1].set_title('Agents Reallocated', fontweight='bold')
    axes[1, 1].set_xticks(x)
    axes[1, 1].set_xticklabels(scenarios)
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / "04_reallocation_stats.png", dpi=300, bbox_inches='tight')
    print("✓ Saved: 04_reallocation_stats.png")
    plt.close()


# ============================================================================
# 5. FINAL ASSIGNMENT COST COMPARISON
# ============================================================================
def plot_final_cost():
    """Compare final assignment costs"""
    fig, ax = plt.subplots(figsize=(12, 6))
    
    grouped = df.groupby(['scenario', 'method'])['final_assignment_cost'].agg(['mean', 'std']).reset_index()
    
    scenarios = grouped['scenario'].unique()
    x = np.arange(len(scenarios))
    width = 0.25
    
    static_data = grouped[grouped['method'] == 'static']
    reactive_data = grouped[grouped['method'] == 'reactive']
    predictive_data = grouped[grouped['method'] == 'predictive']
    
    bars1 = ax.bar(x - width, static_data['mean'], width, 
                   yerr=static_data['std'], label='Static', 
                   capsize=5, alpha=0.8)
    bars2 = ax.bar(x, reactive_data['mean'], width, 
                   yerr=reactive_data['std'], label='Reactive',
                   capsize=5, alpha=0.8)
    bars3 = ax.bar(x + width, predictive_data['mean'], width, 
                   yerr=predictive_data['std'], label='Predictive',
                   capsize=5, alpha=0.8)
    
    ax.set_xlabel('Scenario', fontsize=12, fontweight='bold')
    ax.set_ylabel('Final Assignment Cost', fontsize=12, fontweight='bold')
    ax.set_title('Final Assignment Cost Comparison', fontsize=14, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(scenarios)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_yscale('log')
    
    plt.tight_layout()
    plt.savefig(output_dir / "05_final_cost.png", dpi=300, bbox_inches='tight')
    print("✓ Saved: 05_final_cost.png")
    plt.close()


# ============================================================================
# 6. COLLISION AND SAFETY METRICS
# ============================================================================
def plot_safety_metrics():
    """Plot collision rates"""
    fig, ax = plt.subplots(figsize=(10, 6))
    
    collision_rate = df.groupby(['scenario', 'method'])['collisions'].mean().reset_index()
    collision_rate['collisions'] *= 100  # Convert to percentage
    
    scenarios = collision_rate['scenario'].unique()
    x = np.arange(len(scenarios))
    width = 0.25
    
    for i, scenario in enumerate(scenarios):
        scenario_data = collision_rate[collision_rate['scenario'] == scenario]
        static_val = scenario_data[scenario_data['method'] == 'static']['collisions'].values[0]
        reactive_val = scenario_data[scenario_data['method'] == 'reactive']['collisions'].values[0]
        predictive_val = scenario_data[scenario_data['method'] == 'predictive']['collisions'].values[0]
        
        ax.bar(x[i] - width, static_val, width, alpha=0.8, color='C0')
        ax.bar(x[i], reactive_val, width, alpha=0.8, color='C1')
        ax.bar(x[i] + width, predictive_val, width, alpha=0.8, color='C2')
    
    ax.set_xlabel('Scenario', fontsize=12, fontweight='bold')
    ax.set_ylabel('Collision Rate (%)', fontsize=12, fontweight='bold')
    ax.set_title('Safety: Collision Rate Comparison', fontsize=14, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(scenarios)
    ax.grid(True, alpha=0.3)
    
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor='C0', alpha=0.8, label='Static'),
                      Patch(facecolor='C1', alpha=0.8, label='Reactive'),
                      Patch(facecolor='C2', alpha=0.8, label='Predictive')]
    ax.legend(handles=legend_elements)
    
    plt.tight_layout()
    plt.savefig(output_dir / "06_collision_rate.png", dpi=300, bbox_inches='tight')
    print("✓ Saved: 06_collision_rate.png")
    plt.close()


# ============================================================================
# 7. SUMMARY STATISTICS TABLE
# ============================================================================
def generate_summary_table():
    """Generate a summary statistics table"""
    summary_data = []
    
    for scenario in df['scenario'].unique():
        for method in df['method'].unique():
            subset = df[(df['scenario'] == scenario) & (df['method'] == method)]
            
            summary_data.append({
                'Scenario': scenario,
                'Method': method,
                'Avg Freq (Hz)': f"{subset['avg_solving_frequency'].mean():.2f} ± {subset['avg_solving_frequency'].std():.2f}",
                'Success Rate': f"{subset['all_goals_reached'].mean()*100:.1f}%",
                'Collision Rate': f"{subset['collisions'].mean()*100:.1f}%",
                'Avg Dist to Goal': f"{subset['avg_distance_to_goal'].mean():.4f}",
                'Final Cost': f"{subset['final_assignment_cost'].mean():.2e}",
                'Total Realloc Dist': f"{subset['total_reallocation_distance'].mean():.2f}",
            })
    
    summary_df = pd.DataFrame(summary_data)
    
    # Save to CSV
    summary_df.to_csv(output_dir / "summary_statistics.csv", index=False)
    print("✓ Saved: summary_statistics.csv")
    
    # Print to console
    print("\n" + "="*100)
    print("SUMMARY STATISTICS")
    print("="*100)
    print(summary_df.to_string(index=False))
    print("="*100)


# ============================================================================
# MAIN EXECUTION
# ============================================================================
if __name__ == "__main__":
    print("\nGenerating visualizations...")
    print("-" * 60)
    
    plot_solving_frequency()
    plot_goal_metrics()
    plot_cost_convergence()
    plot_reallocation_stats()
    plot_final_cost()
    plot_safety_metrics()
    generate_summary_table()
    
    print("\n" + "="*60)
    print(f"✓ All visualizations saved to: {output_dir}")
    print("="*60)
    print("\nGenerated files:")
    print("  1. 01_solving_frequency.png    - MPC computational performance")
    print("  2. 02_goal_metrics.png         - Goal achievement analysis")
    print("  3. 03_cost_convergence.png     - Cost evolution over time")
    print("  4. 04_reallocation_stats.png   - Reallocation distance metrics")
    print("  5. 05_final_cost.png           - Final assignment cost comparison")
    print("  6. 06_collision_rate.png       - Safety metrics")
    print("  7. summary_statistics.csv      - Numerical summary table")
    print("="*60)
