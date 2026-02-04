import json
import numpy as np
import os

def process_all_heatmaps(json_file, output_folder):
    with open(json_file, 'r') as f:
        data = json.load(f)

    # Validate file
    if "data" not in data:
        print(f"No 'data' list found in {json_file}")
        return

    frames = data["data"]
    print(f"Found {len(frames)} frames. Starting processing...")

    # Loop through every frame found in the file
    for i, item in enumerate(frames):
        # Get frame
        frame_data = item["frameData"]
        
        # Use frame number for the filename
        frame_num = frame_data.get("frameNum", i)

        if "rangeAzimuthHeatmapMinor" not in frame_data:
            print(f"Skipping frame {frame_num}: 'rangeAzimuthHeatmapMinor' key not found")
            continue
            
        raw_data = np.array(frame_data["rangeAzimuthHeatmapMinor"], dtype=np.float32)

        # Normalize to 0-1 range
        if raw_data.max() == raw_data.min():
            normalized_data = np.zeros_like(raw_data)
        else:
            normalized_data = (raw_data - raw_data.min()) / (raw_data.max() - raw_data.min())

        # Reshape (1024 -> 128x8)
        reshaped_data = normalized_data.reshape(128, 8)

        # Generate dynamic filename and save
        csv_filename = f"heatmap_frame_{frame_num}.csv"
        full_output_path = os.path.join(output_folder, csv_filename)

        np.savetxt(full_output_path, reshaped_data, delimiter=",", fmt='%.3f')
        
        print(f"Saved: {csv_filename}")

# config
input_filename = "binData/2026-02-01_16-26-55/replay_2026-02-01_16-26-55.json" # dont forget to update this
output_directory = "dataset/classes/SITTING/"

if __name__ == "__main__":        
    process_all_heatmaps(input_filename, output_directory)