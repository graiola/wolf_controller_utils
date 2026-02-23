#!/usr/bin/env python3

import yaml
import sys
import re


def normalize_slashes(value):
    """Collapse repeated forward slashes while preserving URL schemes."""
    if not isinstance(value, str):
        return value
    return re.sub(r'(?<!:)/{2,}', '/', value)


def try_parse_value(value):
    """Convert a string to int, float, or bool if possible."""
    if isinstance(value, str):
        if value.lower() in ("true", "false"):
            return value.lower() == "true"
        try:
            return float(value) if '.' in value else int(value)
        except ValueError:
            return value
    return value

def replace_placeholders(data, replacements):
    """Recursively replace placeholders in both dict keys and values."""
    if isinstance(data, dict):
        new_dict = {}
        for key, val in data.items():
            new_key = replace_placeholders(key, replacements) if isinstance(key, str) else key
            new_val = replace_placeholders(val, replacements)
            new_dict[new_key] = new_val
        return new_dict
    elif isinstance(data, list):
        return [replace_placeholders(item, replacements) for item in data]
    elif isinstance(data, str):
        result = data
        for k, v in replacements.items():
            result = re.sub(rf'\$\{{{k}\}}', str(v), result)
        result = normalize_slashes(result)
        return try_parse_value(result)
    else:
        return data

def main():
    if len(sys.argv) < 4:
        print("Usage: replace_yaml_placeholders.py <input_yaml> <output_yaml> KEY=VALUE ...")
        sys.exit(1)

    infile = sys.argv[1]
    outfile = sys.argv[2]
    repl = {k: try_parse_value(v) for k, v in [arg.split("=",1) for arg in sys.argv[3:]]}

    with open(infile) as f:
        data = yaml.safe_load(f)

    new = replace_placeholders(data, repl)

    with open(outfile, "w") as f:
        yaml.safe_dump(new, f, sort_keys=False)

if __name__ == "__main__":
    main()
