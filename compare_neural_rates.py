#!/usr/bin/env python3

"""
Compare Neural Rates
Compares neural rates from vector_field_data.json with stored JSON files in cells_kernels directory
"""

import json
import os
import numpy as np
from typing import Dict, List, Tuple, Optional
from cell_configs import cell_ls


def find_cell(position: np.ndarray, cell_ls: List) -> int:
    """
    Find which cell a position belongs to.
    
    Args:
        position: Position array [x, y]
        cell_ls: List of cell objects
        
    Returns:
        Cell index (cell_id)
    """
    position = np.reshape(position, (1, 2))
    ls_flag = []
    for i in range(len(cell_ls)):
        ls_flag.append(cell_ls[i].check_in_polygon(position))
    
    cell_indices = [i for i, x in enumerate(ls_flag) if x]
    if cell_indices:
        return cell_indices[0]
    else:
        raise ValueError(f"Position {position} not in any defined cell")


def find_matching_json_file(cell_id: int, heading: float, position: List[float], 
                           cells_kernels_dir: str = 'cells_kernels', 
                           tolerance: float = 0.01) -> Optional[str]:
    """
    Find the JSON file in cells_kernels that matches the given position and heading.
    
    Args:
        cell_id: Cell identifier
        heading: Heading direction in degrees
        position: Position [x, y]
        cells_kernels_dir: Base directory for cells_kernels
        tolerance: Position matching tolerance
        
    Returns:
        Path to matching JSON file or None if not found
    """
    heading_str = f"{heading:g}"
    directory = os.path.join(cells_kernels_dir, f'c{int(cell_id)}', f'deg{heading_str}')
    
    if not os.path.exists(directory):
        return None
    
    x, y = position[0], position[1]
    
    # Try to find exact match first
    expected_filename = f'nr_{x:.2f}_y{y:.2f}_HD{heading_str}.json'
    expected_path = os.path.join(directory, expected_filename)
    
    if os.path.exists(expected_path):
        return expected_path
    
    # If exact match not found, search for closest match within tolerance
    best_match = None
    min_distance = float('inf')
    
    for filename in os.listdir(directory):
        if not filename.endswith('.json'):
            continue
        
        # Parse filename: nr_{x}_y{y}_HD{heading}.json
        try:
            if filename.startswith('nr_') and '_y' in filename and '_HD' in filename:
                parts = filename.replace('.json', '').split('_')
                file_x = float(parts[1])
                file_y = float(parts[2].replace('y', ''))
                file_hd = float(parts[3].replace('HD', ''))
                
                # Check if heading matches
                if abs(file_hd - heading) > tolerance:
                    continue
                
                # Calculate distance
                distance = np.sqrt((file_x - x)**2 + (file_y - y)**2)
                
                if distance < min_distance and distance <= tolerance:
                    min_distance = distance
                    best_match = os.path.join(directory, filename)
        except (ValueError, IndexError):
            continue
    
    return best_match


def compare_neural_rates(vector_field_json_path: str = 'trj/vector_field_data.json',
                        cells_kernels_dir: str = 'cells_kernels',
                        tolerance: float = 0.01,
                        verbose: bool = True) -> Dict:
    """
    Compare neural rates from vector_field_data.json with stored JSON files in cells_kernels.
    
    Args:
        vector_field_json_path: Path to vector_field_data.json
        cells_kernels_dir: Base directory for cells_kernels
        tolerance: Position matching tolerance
        verbose: Whether to print detailed comparison results
        
    Returns:
        Dictionary containing comparison results:
        {
            'total_entries': int,
            'matched_entries': int,
            'unmatched_entries': int,
            'comparisons': List[Dict],
            'statistics': Dict
        }
    """
    # Load vector field data
    if not os.path.exists(vector_field_json_path):
        raise FileNotFoundError(f"Vector field data file not found: {vector_field_json_path}")
    
    with open(vector_field_json_path, 'r') as f:
        vector_field_data = json.load(f)
    
    results = {
        'total_entries': len(vector_field_data),
        'matched_entries': 0,
        'unmatched_entries': 0,
        'comparisons': [],
        'statistics': {
            'mean_absolute_error': [],
            'mean_squared_error': [],
            'max_absolute_error': [],
            'correlation': []
        }
    }
    
    if verbose:
        print(f"Comparing {results['total_entries']} entries from vector_field_data.json")
        print(f"Searching in {cells_kernels_dir} directory\n")
    
    for idx, entry in enumerate(vector_field_data):
        position = entry['position']
        heading = entry['heading']
        neural_rate_vf = np.array(entry['neural_rate']).flatten()
        
        # Find cell_id for this position
        try:
            # cell_id = find_cell(np.array(position), cell_ls)
            cell_id = 48
        except ValueError as e:
            if verbose:
                print(f"Entry {idx}: {e}")
            results['unmatched_entries'] += 1
            results['comparisons'].append({
                'index': idx,
                'position': position,
                'heading': heading,
                'status': 'cell_not_found',
                'error': str(e)
            })
            continue
        
        # Find matching JSON file
        json_file_path = find_matching_json_file(cell_id, heading, position, 
                                                cells_kernels_dir, tolerance)
        
        if json_file_path is None:
            if verbose:
                print(f"Entry {idx}: No matching JSON file found for cell {cell_id}, "
                      f"position {position}, heading {heading}")
            results['unmatched_entries'] += 1
            results['comparisons'].append({
                'index': idx,
                'cell_id': cell_id,
                'position': position,
                'heading': heading,
                'status': 'file_not_found'
            })
            continue
        
        # Load neural rate from cells_kernels JSON file
        try:
            with open(json_file_path, 'r') as f:
                kernel_data = json.load(f)
            
            neural_rate_kernel = np.array(kernel_data['neural_rate']).flatten()
            
            # Compare neural rates
            mae = np.mean(np.abs(neural_rate_vf - neural_rate_kernel))
            mse = np.mean((neural_rate_vf - neural_rate_kernel)**2)
            max_ae = np.max(np.abs(neural_rate_vf - neural_rate_kernel))
            
            # Calculate correlation
            if np.std(neural_rate_vf) > 0 and np.std(neural_rate_kernel) > 0:
                correlation = np.corrcoef(neural_rate_vf, neural_rate_kernel)[0, 1]
            else:
                correlation = np.nan
            
            results['matched_entries'] += 1
            results['statistics']['mean_absolute_error'].append(mae)
            results['statistics']['mean_squared_error'].append(mse)
            results['statistics']['max_absolute_error'].append(max_ae)
            results['statistics']['correlation'].append(correlation)
            
            comparison_entry = {
                'index': idx,
                'cell_id': cell_id,
                'position': position,
                'heading': heading,
                'status': 'matched',
                'json_file': json_file_path,
                'metrics': {
                    'mean_absolute_error': float(mae),
                    'mean_squared_error': float(mse),
                    'max_absolute_error': float(max_ae),
                    'correlation': float(correlation) if not np.isnan(correlation) else None
                }
            }
            
            results['comparisons'].append(comparison_entry)
            
            if verbose and (idx % 10 == 0 or mae > 1.0):
                print(f"Entry {idx}: Cell {cell_id}, Pos {position}, HD {heading:.1f}°")
                print(f"  MAE: {mae:.4f}, MSE: {mse:.4f}, Max AE: {max_ae:.4f}, Corr: {correlation:.4f}")
        
        except Exception as e:
            if verbose:
                print(f"Entry {idx}: Error loading/comparing {json_file_path}: {e}")
            results['unmatched_entries'] += 1
            results['comparisons'].append({
                'index': idx,
                'cell_id': cell_id,
                'position': position,
                'heading': heading,
                'status': 'error',
                'error': str(e),
                'json_file': json_file_path
            })
    
    # Calculate summary statistics
    if results['statistics']['mean_absolute_error']:
        results['summary'] = {
            'mean_mae': float(np.mean(results['statistics']['mean_absolute_error'])),
            'std_mae': float(np.std(results['statistics']['mean_absolute_error'])),
            'mean_mse': float(np.mean(results['statistics']['mean_squared_error'])),
            'std_mse': float(np.std(results['statistics']['mean_squared_error'])),
            'mean_max_ae': float(np.mean(results['statistics']['max_absolute_error'])),
            'std_max_ae': float(np.std(results['statistics']['max_absolute_error'])),
            'mean_correlation': float(np.nanmean(results['statistics']['correlation'])),
            'std_correlation': float(np.nanstd(results['statistics']['correlation']))
        }
    else:
        results['summary'] = None
    
    if verbose:
        print(f"\n{'='*60}")
        print(f"Comparison Summary:")
        print(f"  Total entries: {results['total_entries']}")
        print(f"  Matched: {results['matched_entries']}")
        print(f"  Unmatched: {results['unmatched_entries']}")
        if results['summary']:
            print(f"\n  Statistics (across all matched entries):")
            print(f"    Mean Absolute Error: {results['summary']['mean_mae']:.4f} ± {results['summary']['std_mae']:.4f}")
            print(f"    Mean Squared Error: {results['summary']['mean_mse']:.4f} ± {results['summary']['std_mse']:.4f}")
            print(f"    Max Absolute Error: {results['summary']['mean_max_ae']:.4f} ± {results['summary']['std_max_ae']:.4f}")
            print(f"    Correlation: {results['summary']['mean_correlation']:.4f} ± {results['summary']['std_correlation']:.4f}")
        print(f"{'='*60}\n")
    
    return results


def save_comparison_results(results: Dict, output_path: str = 'trj/neural_rate_comparison.json'):
    """
    Save comparison results to a JSON file.
    
    Args:
        results: Comparison results dictionary from compare_neural_rates
        output_path: Path to save the results
    """
    # Convert numpy arrays to lists for JSON serialization
    results_copy = results.copy()
    results_copy['statistics'] = {
        'mean_absolute_error': [float(x) for x in results['statistics']['mean_absolute_error']],
        'mean_squared_error': [float(x) for x in results['statistics']['mean_squared_error']],
        'max_absolute_error': [float(x) for x in results['statistics']['max_absolute_error']],
        'correlation': [float(x) if not np.isnan(x) else None for x in results['statistics']['correlation']]
    }
    
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(results_copy, f, indent=2)
    
    print(f"Comparison results saved to {output_path}")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Compare neural rates from vector_field_data.json with cells_kernels')
    parser.add_argument('--vector-field', type=str, default='trj/vector_field_data.json',
                       help='Path to vector_field_data.json')
    parser.add_argument('--cells-kernels', type=str, default='cells_kernels',
                       help='Base directory for cells_kernels')
    parser.add_argument('--tolerance', type=float, default=0.01,
                       help='Position matching tolerance')
    parser.add_argument('--output', type=str, default='trj/neural_rate_comparison.json',
                       help='Output path for comparison results')
    parser.add_argument('--quiet', action='store_true',
                       help='Suppress verbose output')
    
    args = parser.parse_args()
    
    results = compare_neural_rates(
        vector_field_json_path=args.vector_field,
        cells_kernels_dir=args.cells_kernels,
        tolerance=args.tolerance,
        verbose=not args.quiet
    )
    
    save_comparison_results(results, args.output)

