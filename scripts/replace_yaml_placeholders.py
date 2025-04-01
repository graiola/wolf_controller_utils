#!/usr/bin/env python3

import yaml
import sys
import re

def replace_placeholders(data, replacements):
    if isinstance(data, dict):
        return {k: replace_placeholders(v, replacements) for k, v in data.items()}
    elif isinstance(data, list):
        return [replace_placeholders(item, replacements) for item in data]
    elif isinstance(data, str):
        for key, value in replacements.items():
            data = re.sub(rf'\$\{{{key}\}}', str(value), data)
        return data
    else:
        return data

def main():
    if len(sys.argv) < 4:
        print("Usage: replace_yaml_placeholders.py <input_yaml> <output_yaml> <key=value> [<key=value> ...]")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    replacements = dict(arg.split("=", 1) for arg in sys.argv[3:])

    with open(input_file, 'r') as f:
        yaml_data = yaml.safe_load(f)

    new_yaml = replace_placeholders(yaml_data, replacements)

    with open(output_file, 'w') as f:
        yaml.safe_dump(new_yaml, f)

if __name__ == "__main__":
    main()

