import matplotlib.pyplot as plt
import pandas as pd
import pandas as pd

def central_difference_derivative(csv_file,additional_column,column_name):

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

    # Create a new DataFrame called 'acceleration' with only the original column and its derivative
    acceleration = pd.DataFrame({additional_column: df[additional_column],
        f'{column_name}_derivative': derivative
        
    })
 




acc=central_difference_derivative("combined_file.csv","longitudeDeg","latitudeDeg")
print(acc)
plt.figure(figsize=(10, 6))
plt.plot(acc["longitudeDeg"],acc["latitudeDeg"])
plt.show()


