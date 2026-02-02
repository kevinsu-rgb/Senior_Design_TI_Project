import numpy as np
import matplotlib.pyplot as plt
import os


def generate_average_heatmap(directory_path, output_png_name):
    """
    Averages all CSV files in a directory and saves a heatmap image.
    """
    # 1. Get list of all CSV files
    if not os.path.exists(directory_path):
        print(f"Error: Directory '{directory_path}' not found.")
        return

    csv_files = [f for f in os.listdir(directory_path) if f.endswith(".csv")]

    if not csv_files:
        print(f"No CSV files found in {directory_path}.")
        return

    print(f"Averaging {len(csv_files)} frames from {directory_path}...")

    # 2. Accumulate the data
    sum_heatmap = None
    count = 0

    for file_name in csv_files:
        try:
            # Load the 128x8 data
            data = np.loadtxt(os.path.join(directory_path, file_name), delimiter=",")

            if sum_heatmap is None:
                sum_heatmap = np.zeros_like(data)

            sum_heatmap += data
            count += 1
        except Exception as e:
            print(f"Skipping {file_name} due to error: {e}")

    if count == 0:
        return

    # 3. Calculate the Mean
    average_heatmap = sum_heatmap / count

    # 4. Generate the Visualization
    plt.clf()  # Clear previous plots
    plt.imshow(
        average_heatmap, aspect="auto", cmap="magma"
    )  # 'magma' or 'viridis' look great for radar
    plt.colorbar(label="Normalized Intensity")
    plt.title(f"Average Signature: {os.path.basename(directory_path)}")
    plt.xlabel("Azimuth (Angle)")
    plt.ylabel("Range (Distance)")

    # 5. Save the file
    plt.savefig(output_png_name, dpi=300)
    plt.close()
    print(f"Successfully saved: {output_png_name}")


# --- HOW TO RUN ---
# Replace folder names with your actual training folders
generate_average_heatmap("training/SITTING", "SITTING_AVERAGE_MINE.png")
generate_average_heatmap("../dataset/classes/SITTING", "SITTING_AVERAGE_SEB.png")
generate_average_heatmap("training/STANDING", "STANDING_AVERAGE_MINE.png")
generate_average_heatmap("../dataset/classes/STANDING", "STANDING_AVERAGE_SEB.png")
