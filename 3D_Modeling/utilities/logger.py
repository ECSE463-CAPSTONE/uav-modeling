import inspect

def log(variables):
    """
    Log selected variables in a dictionary with their variable names as keys.
    
    Args:
        variables (list): List of variables to log.
    
    Returns:
        dict: Dictionary with variable names as keys and their values.
    """
    frame = inspect.currentframe().f_back
    return {var_name: frame.f_locals[var_name] for var_name in variables}

def consolidate_logs(logs):
    """
    Consolidate multiple logs into a dictionary indexed by timesteps or iterations.
    
    Args:
        logs (dict): Dictionary where keys are timesteps/iterations and values are dictionaries of logged data.
    
    Returns:
        dict: Consolidated dictionary of logs.
    """
    consolidated = {}
    for timestep, log_data in logs.items():
        for key, value in log_data.items():
            if key not in consolidated:
                consolidated[key] = []
            consolidated[key].append(value)
    return consolidated
