import pandas as pd
import numpy as np

# Load your CSV files
df_small = pd.read_csv('echologger_data_18_15_20-32_1268.csv')
df_large = pd.read_csv('skymate_logger_18_07_14.csv')

# Strip whitespace from column names to avoid any hidden spaces
df_small.columns = df_small.columns.str.strip()
df_large.columns = df_large.columns.str.strip()

# Interpolate the small DataFrame to match the large DataFrame's systemTimeMs column
df_small_interpolated = df_small.set_index('Time').reindex(df_large['systemTimeMs']).interpolate(method='index').reset_index()

# Rename the index column to 'systemTimeMs' to match the large DataFrame for merging
df_small_interpolated.rename(columns={'index': 'systemTimeMs'}, inplace=True)

# Merge the DataFrames on 'systemTimeMs' column
df_combined = pd.merge(df_large, df_small_interpolated, on='systemTimeMs', suffixes=('_large', '_small'))

# Save the combined DataFrame
df_combined.to_csv('combined_file.csv', index=False)

print("Combined CSV saved as 'combined_file.csv'")
