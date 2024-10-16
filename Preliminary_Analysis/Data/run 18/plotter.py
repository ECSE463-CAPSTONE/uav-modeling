import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
def plot_data_analysis(csv_file, col1, col2):
    # Load the CSV file into a DataFrame
    csv_file=str(csv_file)
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print("The specified CSV file was not found.")
        return

    # Check if the columns exist in the DataFrame
    if col1 not in df.columns or col2 not in df.columns:
        print("One or both column names do not exist in the DataFrame.")
        return
    
    rms_col2 = np.sqrt(np.mean(df[col2] ** 2))  # Root Mean Square
    avg_col2 = df[col2].mean()  # Average
    max_col2 = df[col2].max()  # Maximum
    min_col2 = df[col2].min()  # Minimum
    var1 = df[col1].var()  # Variance of col1

    # Calculate variance
    var1 = df[col1].var()
    var2 = df[col2].var()
    print(var1)
    print(f"Variance of {col1}: {var1}")
    print(f"Variance of {col2}: {var2}")

    # Calculate frequency
    freq1 = df[col1].value_counts()
    freq2 = df[col2].value_counts()
    print(f"\nFrequency of {col1}:\n{freq1}")
    print(f"\nFrequency of {col2}:\n{freq2}")

    # Plot scatter plot of the two columns
    plt.figure(figsize=(10, 6))
    sns.lineplot(x=df[col1], y=df[col2])
    plt.title(f'Scatter Plot of {col1} vs {col2}')
    plt.xlabel(col1)
    plt.ylabel(col2)
    text_str = (f'RMS: {rms_col2:.2f}\n'
                f'Average: {avg_col2:.2f}\n'
                f'Maximum: {max_col2:.2f}\n'
                f'Minimum: {min_col2:.2f}')
    plt.text(0.05, 0.95, text_str, transform=plt.gca().transAxes, 
             fontsize=10, verticalalignment='top', bbox=dict(boxstyle="round,pad=0.3", edgecolor="gray", facecolor="lightyellow"))

    plt.show()

    plt.show()

    # Plot frequency distribution for both columns
    plt.figure(figsize=(14, 6))
    sns.histplot(df[col2], kde=True, bins=20)
    plt.title('Histogram')
    plt.tight_layout()
    plt.show()
    
    #frequency
    #plt.figure(figsize=(10, 6))
    #data = df[col1].dropna().values  # Drop NaN values if any
    #n = len(data)
    #freq = np.fft.fftfreq(n)  # Frequency range
    #fft_values = np.fft.fft(data)  # Fourier Transform
    #plt.plot(freq[:n // 2], np.abs(fft_values)[:n // 2])  # Plot positive frequencies
    #plt.title(f'Fourier Transform of {col2}')
    #plt.xlabel('Frequency')
    #plt.ylabel('Amplitude')
    #plt.show()
#plot_data_analysis("combined_file.csv","TimeMs","rollDeg")
plot_data_analysis("combined_file.csv","TimeMs", "H.Speed")