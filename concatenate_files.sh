#!/bin/bash

# Output file
OUTPUT_FILE="$HOME/Desktop/all_code4.txt"

# Clear output file
> "$OUTPUT_FILE"

# Define extensions to include
EXTENSIONS="\\( -name \"*.cpp\" -o -name \"*.hpp\" -o -name \"*.py\" -o -name \"*.json\" -o -name \"*.yaml\" \\)"

# Use eval to safely evaluate escaped parentheses
eval "find . -type f $EXTENSIONS" | while IFS= read -r file; do
    echo "================== $file ==================" >> "$OUTPUT_FILE"
    cat "$file" >> "$OUTPUT_FILE"
    echo >> "$OUTPUT_FILE"  # Add spacing
done

echo "✅ All code files have been concatenated into: $OUTPUT_FILE"
