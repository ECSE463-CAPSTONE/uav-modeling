import inspect
import datetime
import csv

def log(variables):
    """
    Log selected variables in a dictionary with their variable names as keys.
    
    Args:
        variables (list): List of variables to log.
    
    Returns:
        dict: Dictionary with variable names as keys and their values.
    """
    frame = inspect.currentframe().f_back
    names = {id(v): k for k, v in frame.f_locals.items()}  # Map variable IDs to names
    return {names[id(var)]: var for var in variables if id(var) in names}

# def consolidate_logs(logs):
#     """
#     Consolidate multiple logs into a dictionary indexed by timesteps or iterations.
    
#     Args:
#         logs (dict): Dictionary where keys are timesteps/iterations and values are dictionaries of logged data.
#         save_csv (bool): If True, saves the consolidated log to a CSV file.
    
#     Returns:
#         dict: Consolidated dictionary of logs.
#     """
#     consolidated = {"timestep": list(logs.keys())}
#     for timestep, log_data in logs.items():
#         for key, value in log_data.items():
#             if key not in consolidated:
#                 consolidated[key] = []
#             consolidated[key].append(value)
    
#     return consolidated

def save_consolidated_logs(consolidated):
    """
    Save logs to a CSV file.

    Args:
        logs (dict): Dictionary of logs to save.
    """
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_logs.csv"
    with open(filename, mode='w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=consolidated.keys())
        writer.writeheader()
        for i in range(len(consolidated["timestep"])):
            row = {key: consolidated[key][i] for key in consolidated}
            writer.writerow(row)

def consolidate_logs(logs):
    """
    Consolidate multiple logs into a dictionary indexed by timesteps or iterations.

    Args:
        logs (dict): Dictionary where keys are timesteps/iterations and values are dictionaries of logged data.

    Returns:
        dict: Consolidated dictionary of logs.
    """
    consolidated = {"timestep": list(logs.keys())}
    
    # Find all unique keys across all timesteps
    all_keys = set()
    for log_data in logs.values():
        all_keys.update(log_data.keys())

    # Initialize lists for each key
    for key in all_keys:
        consolidated[key] = []

    # Fill lists with corresponding values or None for missing keys
    for timestep in logs:
        for key in all_keys:
            consolidated[key].append(logs[timestep].get(key, None))

    return consolidated

def flatten_logs(nested_dict, parent_key='', separator='_'):
    """
    Flattens a nested dictionary by concatenating keys.
    
    Args:
        nested_dict (dict): The nested dictionary to flatten.
        parent_key (str, optional): The prefix for keys (used for recursion).
        separator (str, optional): The separator used to concatenate keys.

    Returns:
        dict: A flattened dictionary with no nested structures.
    """
    flattened_dict = {}

    def recurse(current_dict, current_key):
        for key, value in current_dict.items():
            # Remove 'control_forces' from the key path
            if current_key == 'control_forces':
                new_key = key
            else:
                new_key = f"{current_key}{separator}{key}" if current_key else key
            
            if isinstance(value, dict):
                recurse(value, new_key)
            else:
                flattened_dict[new_key] = value

    recurse(nested_dict, parent_key)
    return flattened_dict

