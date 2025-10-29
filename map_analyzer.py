#!/usr/bin/env python3
"""
ROS2 Map Area Analyzer
Calculates and visualizes explored area from a saved map
"""

import yaml
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

def load_map_from_pgm(yaml_file):
    """Load map from YAML and PGM files"""
    # Load YAML metadata
    with open(yaml_file, 'r') as f:
        map_metadata = yaml.safe_load(f)
    
    # Get PGM filename
    pgm_file = yaml_file.replace('.yaml', '.pgm')
    
    # Load PGM image
    img = Image.open(pgm_file)
    map_data = np.array(img)
    
    return map_data, map_metadata

def analyze_map(map_data, resolution):
    """Analyze map and calculate areas"""
    total_cells = map_data.size
    
    # Count different cell types
    # In PGM: 254=free, 0=occupied, 205=unknown
    free_cells = np.sum(map_data == 254)
    occupied_cells = np.sum(map_data == 0)
    unknown_cells = np.sum(map_data == 205)
    
    # Calculate areas
    cell_area = resolution * resolution
    total_area = total_cells * cell_area
    free_area = free_cells * cell_area
    occupied_area = occupied_cells * cell_area
    unknown_area = unknown_cells * cell_area
    explored_area = free_area + occupied_area
    
    return {
        'total_area': total_area,
        'free_area': free_area,
        'occupied_area': occupied_area,
        'unknown_area': unknown_area,
        'explored_area': explored_area,
        'free_cells': free_cells,
        'occupied_cells': occupied_cells,
        'unknown_cells': unknown_cells,
        'total_cells': total_cells,
        'exploration_percentage': (explored_area / total_area) * 100
    }

def visualize_map(map_data, metadata, stats):
    """Visualize the map with statistics"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
    
    # Plot map
    ax1.imshow(map_data, cmap='gray', origin='lower')
    ax1.set_title('Map Visualization')
    ax1.set_xlabel(f'X (cells) - Resolution: {metadata["resolution"]} m/cell')
    ax1.set_ylabel('Y (cells)')
    
    # Add grid
    ax1.grid(True, alpha=0.3)
    
    # Plot statistics
    labels = ['Free Space', 'Occupied', 'Unknown']
    sizes = [stats['free_area'], stats['occupied_area'], stats['unknown_area']]
    colors = ['lightgreen', 'red', 'gray']
    explode = (0.1, 0, 0)
    
    ax2.pie(sizes, explode=explode, labels=labels, colors=colors,
            autopct='%1.1f%%', shadow=True, startangle=90)
    ax2.set_title('Area Distribution')
    
    # Add text with statistics
    stats_text = f"""
    Map Statistics:
    ─────────────────────
    Total Area: {stats['total_area']:.2f} m²
    Explored: {stats['explored_area']:.2f} m² ({stats['exploration_percentage']:.1f}%)
    
    Free Space: {stats['free_area']:.2f} m²
    Occupied: {stats['occupied_area']:.2f} m²
    Unknown: {stats['unknown_area']:.2f} m²
    
    Map Dimensions:
    Width: {map_data.shape[1]} cells ({map_data.shape[1] * metadata['resolution']:.2f} m)
    Height: {map_data.shape[0]} cells ({map_data.shape[0] * metadata['resolution']:.2f} m)
    Resolution: {metadata['resolution']} m/cell
    """
    
    plt.figtext(0.5, 0.02, stats_text, ha='center', fontsize=10, 
                family='monospace', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.25)
    plt.savefig('map_analysis.png', dpi=150, bbox_inches='tight')
    print("Saved visualization to map_analysis.png")
    plt.show()

def main():
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python3 map_analyzer.py <path_to_map.yaml>")
        print("Example: python3 map_analyzer.py ~/robot_mapping_ws/maps/my_map.yaml")
        sys.exit(1)
    
    yaml_file = sys.argv[1]
    
    print(f"Loading map from: {yaml_file}")
    map_data, metadata = load_map_from_pgm(yaml_file)
    
    print(f"\nAnalyzing map...")
    stats = analyze_map(map_data, metadata['resolution'])
    
    # Print results
    print("\n" + "="*50)
    print("MAP ANALYSIS RESULTS")
    print("="*50)
    print(f"Total Map Area:      {stats['total_area']:>10.2f} m²")
    print(f"Explored Area:       {stats['explored_area']:>10.2f} m² ({stats['exploration_percentage']:.1f}%)")
    print(f"  - Free Space:      {stats['free_area']:>10.2f} m²")
    print(f"  - Occupied:        {stats['occupied_area']:>10.2f} m²")
    print(f"Unknown Area:        {stats['unknown_area']:>10.2f} m²")
    print("="*50)
    print(f"Resolution:          {metadata['resolution']} m/cell")
    print(f"Map Size:            {map_data.shape[1]} × {map_data.shape[0]} cells")
    print(f"Origin:              ({metadata['origin'][0]:.2f}, {metadata['origin'][1]:.2f})")
    print("="*50)
    
    # Visualize
    print("\nGenerating visualization...")
    visualize_map(map_data, metadata, stats)

if __name__ == "__main__":
    main()
