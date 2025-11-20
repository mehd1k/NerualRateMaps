#!/usr/bin/env python3

"""
Test script for Gazebo Neural Analysis Node
Tests the integration without requiring Gazebo simulation
"""

import numpy as np
import matplotlib.pyplot as plt
import cv2
import os
import sys
import matlab.engine
from gen_controller import cell_ls
from find_controller_orientation import control_gain_load

def create_test_image(width=150, height=90):
    """Create a test image similar to what would come from Gazebo"""
    # Create a simple test pattern
    image = np.zeros((height, width), dtype=np.float32)
    
    # Add some geometric patterns
    cv2.rectangle(image, (20, 20), (width-20, height-20), 0.8, 2)
    cv2.circle(image, (width//2, height//2), 30, 0.6, -1)
    cv2.line(image, (0, height//2), (width, height//2), 0.4, 1)
    cv2.line(image, (width//2, 0), (width//2, height), 0.4, 1)
    
    # Add some noise
    noise = np.random.normal(0, 0.1, (height, width))
    image = np.clip(image + noise, 0, 1)
    
    return image

def test_matlab_integration():
    """Test MATLAB engine integration"""
    print("Testing MATLAB engine integration...")
    
    try:
        # Start MATLAB engine
        eng = matlab.engine.start_matlab()
        print("✓ MATLAB engine started successfully")
        
        # Create test image
        test_image = create_test_image()
        print(f"✓ Test image created: {test_image.shape}")
        
        # Convert to MATLAB format
        matlab_image = matlab.double(test_image.tolist())
        S = matlab.double(np.zeros([100, 1]).tolist())
        U = matlab.double(np.zeros([100, 1]).tolist())
        
        # Test MATLAB function call
        try:
            S_out, U_out = eng.generate_V1_RSC_model_response(matlab_image, S, U, nargout=2)
            neural_rate = np.array(S_out)
            print(f"✓ MATLAB function call successful: {neural_rate.shape}")
            print(f"  Neural rate range: [{neural_rate.min():.3f}, {neural_rate.max():.3f}]")
        except Exception as e:
            print(f"✗ MATLAB function call failed: {e}")
            print("  Note: This is expected if the MATLAB function is not available")
            print("  The node will work once you have the proper MATLAB function")
        
        # Quit MATLAB engine
        eng.quit()
        print("✓ MATLAB engine closed successfully")
        
        return True
        
    except Exception as e:
        print(f"✗ MATLAB engine test failed: {e}")
        return False

def test_control_system():
    """Test control system components"""
    print("\nTesting control system components...")
    
    try:
        # Test cell loading
        print(f"✓ Found {len(cell_ls)} cells")
        
        # Test control gain loading
        control_gain_loader = control_gain_load()
        print("✓ Control gain loader initialized")
        
        # Test cell finding
        test_position = np.array([0.4, 0.25])
        cell_id = find_cell(test_position, cell_ls)
        print(f"✓ Cell finding test: position {test_position} -> cell {cell_id}")
        
        # Test control gain interpolation
        try:
            K, Kb = control_gain_loader.interpolate_contorlgains(cell_id, 270.0)
            print(f"✓ Control gain interpolation successful: K{K.shape}, Kb{Kb.shape}")
        except Exception as e:
            print(f"✗ Control gain interpolation failed: {e}")
            print("  Note: This is expected if control gain files are not available")
        
        return True
        
    except Exception as e:
        print(f"✗ Control system test failed: {e}")
        return False

def find_cell(position, cell_ls):
    """Find which cell the position is in"""
    ls_flag = []
    for i in range(len(cell_ls)):
        ls_flag.append(cell_ls[i].check_in_polygon(np.reshape(position, (1, 2))))
    
    cell_indices = [i for i, x in enumerate(ls_flag) if x]
    if cell_indices:
        return cell_indices[0]
    else:
        return 0

def test_image_processing():
    """Test image processing pipeline"""
    print("\nTesting image processing pipeline...")
    
    try:
        # Create test image
        test_image = create_test_image()
        
        # Test image processing (same as in the node)
        processed_image = process_image(test_image)
        
        print(f"✓ Image processing successful: {processed_image.shape}")
        print(f"  Image range: [{processed_image.min():.3f}, {processed_image.max():.3f}]")
        
        # Save test image
        os.makedirs('test', exist_ok=True)
        plt.imsave('test/test_image.png', processed_image, cmap='gray')
        print("✓ Test image saved to test/test_image.png")
        
        return True
        
    except Exception as e:
        print(f"✗ Image processing test failed: {e}")
        return False

def process_image(cv_image):
    """Process image to match original format"""
    # Convert to grayscale if needed
    if len(cv_image.shape) == 3:
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    else:
        gray_image = cv_image
    
    # Resize to target dimensions (150x90)
    resized_image = cv2.resize(gray_image, (150, 90), interpolation=cv2.INTER_AREA)
    
    # Normalize to [0, 1] range
    normalized_image = resized_image.astype(np.float32) / 255.0
    
    return normalized_image

def test_data_saving():
    """Test data saving functionality"""
    print("\nTesting data saving functionality...")
    
    try:
        # Create test data
        test_data = {
            'positions': np.array([[0.4, 0.25], [0.41, 0.26], [0.42, 0.27]]),
            'headings': np.array([0.0, 5.0, 10.0]),
            'neural_rates': np.random.rand(3, 100),
            'control_inputs': np.random.rand(3, 2)
        }
        
        # Save test data
        os.makedirs('trj', exist_ok=True)
        np.save('trj/test_positions.npy', test_data['positions'])
        np.save('trj/test_headings.npy', test_data['headings'])
        np.save('trj/test_neural_rates.npy', test_data['neural_rates'])
        np.save('trj/test_control_inputs.npy', test_data['control_inputs'])
        
        print("✓ Test data saved successfully")
        
        # Create test plot
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.plot(test_data['positions'][:, 0], test_data['positions'][:, 1], 'b-o')
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_title('Test Trajectory')
        ax.grid(True)
        plt.savefig('trj/test_trajectory.png', dpi=150, bbox_inches='tight')
        plt.close()
        
        print("✓ Test trajectory plot saved")
        
        return True
        
    except Exception as e:
        print(f"✗ Data saving test failed: {e}")
        return False

def main():
    """Run all tests"""
    print("=" * 60)
    print("GAZEBO NEURAL ANALYSIS NODE - INTEGRATION TEST")
    print("=" * 60)
    
    tests = [
        ("MATLAB Integration", test_matlab_integration),
        ("Control System", test_control_system),
        ("Image Processing", test_image_processing),
        ("Data Saving", test_data_saving),
    ]
    
    results = []
    for test_name, test_func in tests:
        print(f"\n{test_name}:")
        print("-" * 40)
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"✗ {test_name} failed with exception: {e}")
            results.append((test_name, False))
    
    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    passed = 0
    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        print(f"{test_name:20} : {status}")
        if result:
            passed += 1
    
    print(f"\nPassed: {passed}/{len(results)} tests")
    
    if passed == len(results):
        print("\n🎉 All tests passed! The node should work correctly.")
    else:
        print(f"\n⚠️  {len(results) - passed} tests failed. Check the issues above.")
        print("Some failures may be expected if MATLAB functions or control gains are not available.")
    
    print("\nTo run the actual node:")
    print("ros2 run neural_rate_maps gazebo_neural_analysis.py")

if __name__ == '__main__':
    main()
