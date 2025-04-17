#!/bin/bash

# Loop over cell_id from 0 to 47
for cell_id in {0..47}
do
    echo "Running MyApp with cell_id=$cell_id"
    python merged.py --cell_id $cell_id
done
