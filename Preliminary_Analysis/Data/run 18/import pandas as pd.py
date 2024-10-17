import pandas as pd
from scipy.stats import linregress
import matplotlib.pyplot as plt

import pandas as pd
import numpy as np

def lol(csv_file, col_x, col_y):
    """
    Calculates the slope between each pair of subsequent points in two columns and stores the result in a new DataFrame.

    Parameters:
    df (pd.DataFrame): The DataFrame containing the data.
    col_x (str): The column name to use as the x values.
    col_y (str): The column name to use as the y values.

    Returns:
    pd.DataFrame: A DataFrame containing the slopes for each pair of subsequent points.
    """
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print("The specified CSV file was not found.")
        return None
    # Ensure the DataFrame has at least two rows to calculate slopes between subsequent points
    if len(df) < 2:
        raise ValueError("The DataFrame must contain at least two rows to calculate slopes between subsequent points.")

    # Calculate the slopes between each pair of subsequent points
    slopes = []
    for i in range(len(df) - 1):
        # Calculate the change in x and y
        delta_x = df[col_x].iloc[i + 1] - df[col_x].iloc[i]
        delta_y = df[col_y].iloc[i + 1] - df[col_y].iloc[i]
        
        # Calculate the slope (delta_y / delta_x) and handle division by zero
        slope = delta_y / delta_x if delta_x != 0 else None
        try:
            normalized=slope/(df["hSpeed"][i])
        except:
            slope=0

        slopes.append(slope)

    # Store the result in a DataFrame
    result_df = pd.DataFrame({'Time': df["TimeMs"][1:],
                              'Slope': slopes})
    return result_df

plot=lol("combined_file.csv","longitudeDeg","latitudeDeg")
print(plot)
plt.figure(figsize=(10, 6))
plt.plot(plot["Time"],plot["Slope"])
plt.show()

