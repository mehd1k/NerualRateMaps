#!/bin/bash

# Shell script to run gazebo_neural_analysis.py in gen_data mode
# Iterates through cells 0-47 and headings 0-350 in steps of 10

# Generate heading list: 0, 10, 20, ..., 350
headings=()
for h in {0..350..10}; do
    headings+=($h)
done

# Convert headings array to space-separated string
headings_str="${headings[@]}"

# Iterate through cells 0 to 47
for cell in {0..47}; do
    echo "=========================================="
    echo "Processing cell: $cell"
    echo "Headings: $headings_str"
    echo "=========================================="
    
    # Run the Python script with current cell and all headings
    python3 gazebo_neural_analysis.py \
        --mode gen_data \
        --cells $cell \
        --headings $headings_str
    
    # Check if the command was successful
    if [ $? -ne 0 ]; then
        echo "Error: Failed to process cell $cell"
        echo "Continuing with next cell..."
    else
        echo "Successfully completed cell $cell"
    fi
    
    echo ""
done

echo "=========================================="
echo "All cells processed!"
echo "=========================================="

