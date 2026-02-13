#!/usr/bin/env python3
"""
Plot fitness curves from evolution log file.

Usage:
    python plot_fitness.py evolution_20260213_143052.log
    python plot_fitness.py evolution_20260213_143052.log --save output.png
    python plot_fitness.py evolution_20260213_143052.log --title "My Experiment"
"""

import re
import sys
import argparse
import matplotlib.pyplot as plt
import numpy as np


def parse_evolution_log(filepath):
    """
    Parse evolution log file and extract fitness data.
    
    Args:
        filepath: Path to evolution log file
        
    Returns:
        Tuple of (generations, best_fitness, avg_fitness)
    """
    generations = []
    best_fitness = []
    avg_fitness = []
    
    with open(filepath, 'r') as f:
        for line in f:
            # Match: "Gen    0 | best: 248.334 | avg: -180.908 | rule: [...]"
            match = re.match(r'Gen\s+(\d+)\s+\|\s+best:\s+([\d.-]+)\s+\|\s+avg:\s+([\d.-]+)', line)
            if match:
                gen = int(match.group(1))
                best = float(match.group(2))
                avg = float(match.group(3))
                
                generations.append(gen)
                best_fitness.append(best)
                avg_fitness.append(avg)
    
    return generations, best_fitness, avg_fitness


def plot_fitness(generations, best_fitness, avg_fitness, title=None, save_path=None):
    """
    Plot fitness curves.
    
    Args:
        generations: List of generation numbers
        best_fitness: List of best fitness values
        avg_fitness: List of average fitness values
        title: Optional plot title
        save_path: Optional path to save figure (if None, displays plot)
    """
    if len(generations) == 0:
        print("ERROR: No fitness data found in log file!")
        print("Make sure the log file contains lines like:")
        print("  Gen    0 | best: 248.334 | avg: -180.908 | rule: [...]")
        return
    
    plt.figure(figsize=(12, 7))
    
    # Plot best fitness
    plt.plot(generations, best_fitness, 'b-', linewidth=2.5, label='Best Fitness', marker='o', 
             markersize=4, markevery=max(1, len(generations)//20))
    
    # Plot average fitness
    plt.plot(generations, avg_fitness, 'r--', linewidth=2, label='Average Fitness', marker='s',
             markersize=3, markevery=max(1, len(generations)//20))
    
    # Add horizontal line at y=0 for reference
    plt.axhline(y=0, color='gray', linestyle=':', linewidth=1, alpha=0.5)
    
    plt.xlabel('Generation', fontsize=12)
    plt.ylabel('Fitness', fontsize=12)
    
    if title:
        plt.title(title, fontsize=14, fontweight='bold')
    else:
        plt.title('Evolution Progress', fontsize=14, fontweight='bold')
    
    plt.legend(fontsize=11, loc='best')
    plt.grid(True, alpha=0.3, linestyle='--')
    plt.tight_layout()
    
    # Add statistics text box
    final_best = best_fitness[-1]
    final_avg = avg_fitness[-1]
    max_best = max(best_fitness)
    max_best_gen = generations[best_fitness.index(max_best)]
    
    stats_text = f"Final Gen: {generations[-1]}\n"
    stats_text += f"Final Best: {final_best:.2f}\n"
    stats_text += f"Final Avg: {final_avg:.2f}\n"
    stats_text += f"Max Best: {max_best:.2f} (Gen {max_best_gen})"
    
    plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes,
             fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"✅ Plot saved to: {save_path}")
    else:
        print("📊 Displaying plot (close window to exit)...")
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Plot fitness curves from evolution log file',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python plot_fitness.py evolution_20260213_143052.log
  python plot_fitness.py evolution_20260213_143052.log --save fitness_plot.png
  python plot_fitness.py best_chromosomes/evolution_20260213_143052.log --title "5.5cm Sensors"
        """
    )
    
    parser.add_argument('logfile', help='Path to evolution log file')
    parser.add_argument('--save', metavar='PATH', help='Save plot to file instead of displaying')
    parser.add_argument('--title', metavar='TEXT', help='Custom plot title')
    
    args = parser.parse_args()
    
    # Check if file exists
    try:
        with open(args.logfile, 'r') as f:
            pass
    except FileNotFoundError:
        print(f"ERROR: File not found: {args.logfile}")
        sys.exit(1)
    
    print(f"📂 Loading fitness data from: {args.logfile}")
    
    # Parse log file
    generations, best_fitness, avg_fitness = parse_evolution_log(args.logfile)
    
    if len(generations) == 0:
        print("ERROR: No fitness data found in log file!")
        sys.exit(1)
    
    print(f"✅ Loaded {len(generations)} generations")
    print(f"   First generation: {generations[0]}")
    print(f"   Last generation: {generations[-1]}")
    print(f"   Best fitness: {max(best_fitness):.3f} (Gen {generations[best_fitness.index(max(best_fitness))]})")
    print(f"   Final best: {best_fitness[-1]:.3f}")
    print(f"   Final avg: {avg_fitness[-1]:.3f}")
    print()
    
    # Plot
    plot_fitness(generations, best_fitness, avg_fitness, title=args.title, save_path=args.save)


if __name__ == "__main__":
    main()
