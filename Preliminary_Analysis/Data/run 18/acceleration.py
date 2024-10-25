import matplotlib.pyplot as plt
import pandas as pd
def central_difference_derivative(csv_file, column_name):
    """
    Read a CSV file, apply central difference to differentiate a specified column,
    and return a DataFrame named 'acceleration' containing only the derivative of the specified column.

    Parameters:
    csv_file (str): Path to the CSV file.
    column_name (str): Name of the column to differentiate.

    Returns:
    pd.DataFrame: DataFrame named 'acceleration' with the derivative of the specified column.
    """
    # Load the CSV file into a DataFrame
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print("The specified CSV file was not found.")
        return None

    # Check if the column exists in the DataFrame
    if column_name not in df.columns:
        print("The specified column does not exist in the DataFrame.")
        return None

    # Get the values of the specified column
    values = df[column_name].values
    n = len(values)
    
    # Step size
    h = 1  # Assuming equally spaced values, adjust this if necessary

    # Calculate the central difference derivative
    # Initialize the derivative array with NaN for boundary values
    derivative = pd.Series([float('nan')] * n)

    # Apply central difference method
    derivative[1:-1] = (values[2:] - values[:-2]) / (2 * h)

    # Create a new DataFrame called 'acceleration' containing only the derivative
    acceleration = pd.DataFrame({"Time":df["TimeMs"],
        f'{column_name}_derivative': derivative
    })

    return acceleration

# Example usage:
acceleration = central_difference_derivative("combined_file.csv", 'hSpeed')
print(acceleration)
plt.figure(figsize=(10, 6))
plt.plot(acceleration["Time"],acceleration["hSpeed_derivative"])
plt.show()
