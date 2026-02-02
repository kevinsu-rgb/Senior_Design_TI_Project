import torch
import numpy as np
import os
from nn import NeuralNetwork

# --- SETTINGS ---
MODEL_PATH = "model.pth"
TEST_DATA_DIR = "ourdata/STANDING"
CLASS_NAMES = ["SITTING", "STANDING"]
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def load_model():
    model = NeuralNetwork().to(device)
    model.load_state_dict(torch.load("model.pth", map_location=device))
    model.eval()
    return model


def test_offline():
    model = load_model()
    print(f"Loaded model from {MODEL_PATH}")
    print("-" * 30)

    # List all CSV files in your data folder
    csv_files = [f for f in os.listdir(TEST_DATA_DIR) if f.endswith(".csv")]

    if not csv_files:
        print("No CSV files found in the data directory!")
        return

    for file_name in sorted(csv_files):
        file_path = os.path.join(TEST_DATA_DIR, file_name)

        try:
            # 1. Load the raw data
            raw_data = np.loadtxt(file_path, delimiter=",")

            # 2. Normalize (exactly like your real-time script)
            # Use 1e-8 to prevent division by zero
            denom = raw_data.max() - raw_data.min() + 1e-8
            normalized_data = (raw_data - raw_data.min()) / denom

            # 3. Prepare for model (Tensor + Reshape)
            input_tensor = torch.from_numpy(normalized_data).float().to(device)
            input_tensor = input_tensor.view(
                1, 1, 128, 8
            )  # Match your model input shape

            # 4. Inference
            with torch.no_grad():
                logits = model(input_tensor)
                probs = torch.softmax(logits, dim=1)
                conf, pred = torch.max(probs, dim=1)

            print(f"FILE: {file_name}")
            print(f"  -> Prediction: {CLASS_NAMES[pred.item()]}")
            print(f"  -> Confidence: {conf.item():.2%}")
            print(f"  -> Raw Max/Min: {raw_data.max():.2e} / {raw_data.min():.2e}")
            print("-" * 30)

        except Exception as e:
            print(f"Error processing {file_name}: {e}")


if __name__ == "__main__":
    test_offline()
