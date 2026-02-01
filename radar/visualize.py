import matplotlib.pyplot as plt
import sys
import numpy as np


def visualize_heatmap(heatmap_2d, title="Radar Heatmap", save_path=None):
    """
    Visualizes a 2D numpy array as a heatmap.
    """
    raw_data = np.loadtxt(heatmap_2d, delimiter=",", dtype=str)
    heatmap_2d = raw_data.astype(float)
    plt.figure(figsize=(8, 5))
    # 'origin=lower' usually matches radar range/azimuth orientation
    plt.imshow(heatmap_2d, aspect="auto", cmap="viridis", origin="lower")
    plt.colorbar(label="Intensity")
    plt.title(title)
    plt.xlabel("Azimuth Bins")
    plt.ylabel("Range Bins")

    output_name = "latest_heatmap.png"
    plt.savefig(output_name)
    plt.close()


if __name__ == "__main__":
    # Usage: python view_csv.py data/10.csv
    if len(sys.argv) > 1:
        visualize_heatmap(sys.argv[1])
    else:
        print("Please provide a path to a CSV file.")
