#!/usr/bin/env python3
"""ROS2 Map Area Analyzer - Package Module Version"""

import yaml
from PIL import Image
import numpy as np

def analyze_map(yaml_file):
    """Analyze map and return statistics"""
    with open(yaml_file, 'r') as f:
        metadata = yaml.safe_load(f)
    
    pgm_file = yaml_file.replace('.yaml', '.pgm')
    img = Image.open(pgm_file)
    map_data = np.array(img)
    
    resolution = metadata['resolution']
    total_cells = map_data.size
    
    free_cells = np.sum(map_data == 254)
    occupied_cells = np.sum(map_data == 0)
    unknown_cells = np.sum(map_data == 205)
    
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
        'exploration_percentage': (explored_area / total_area) * 100,
        'width': map_data.shape[1],
        'height': map_data.shape[0],
        'resolution': resolution,
        'origin': metadata['origin']
    }

def main():
    import sys
    if len(sys.argv) < 2:
        print("Usage: ros2 run autonomous_explorer map_analyzer <map.yaml>")
        sys.exit(1)
    
    yaml_file = sys.argv[1]
    stats = analyze_map(yaml_file)
    
    print("\n" + "="*60)
    print(f"MAP ANALYSIS: {yaml_file}")
    print("="*60)
    print(f"Map Dimensions:      {stats['width']} × {stats['height']} cells")
    print(f"Physical Size:       {stats['width']*stats['resolution']:.2f} × {stats['height']*stats['resolution']:.2f} m")
    print(f"Resolution:          {stats['resolution']} m/cell")
    print("-"*60)
    print(f"Total Area:          {stats['total_area']:>10.2f} m²")
    print(f"Explored Area:       {stats['explored_area']:>10.2f} m² ({stats['exploration_percentage']:.1f}%)")
    print(f"  - Free Space:      {stats['free_area']:>10.2f} m²")
    print(f"  - Occupied:        {stats['occupied_area']:>10.2f} m²")
    print(f"Unknown Area:        {stats['unknown_area']:>10.2f} m²")
    print("="*60)

if __name__ == "__main__":
    main()
