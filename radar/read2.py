import onnxruntime as ort
import sys
import pandas as pd
from collections import deque, Counter
import serial
import threading
import time
import struct
import numpy as np
import queue

q: queue.Queue = queue.Queue(1)

MINIMUM_POINTS = 5

RECORD_MODE = False

pressed = False

frames = []
frame_count = 14;

import tkinter as tk
def launch_recorder_ui():
    global pressed

    root = tk.Tk()
    root.title("Radar Recorder")
    root.geometry("220x140")
    root.resizable(False, False)

    def toggle():
        global pressed
        pressed = not pressed
        if pressed:
            btn.config(text="stop recording", bg="#e74c3c", fg="white")
            status.config(text="recording...", fg="#e74c3c")
        else:
            btn.config(text="start recording", bg="#2ecc71", fg="white")
            status.config(text="idle", fg="#7f8c8d")

    def save():
        global frames
        global frame_count
        # import here or at top of file
        if len(frames) > 0:
            columns = [
                'posz', 'velx', 'vely', 'velz', 'accx', 'accy', 'accz',
                'p1x', 'p1y', 'p1z', 'p2x', 'p2y', 'p2z', 'p3x', 'p3y', 'p3z',
                'p4x', 'p4y', 'p4z', 'p5x', 'p5y', 'p5z', 'heatmap'
            ]
            big_df = pd.DataFrame(frames, columns=columns)
            big_df.to_csv(f"data/classes/TEST/frames{frame_count}.csv", index=False)
            status.config(text=f"Saved {len(frames)} frames!", fg="#2980b9")
            print(f"Saved {len(frames)} frames to cs1v")
            frames = []
            frame_count += 1
        else:
            status.config(text="Nothing to save.", fg="#e67e22")

    status = tk.Label(root, text="Idle", fg="#7f8c8d", font=("Arial", 11))
    status.pack(pady=(12, 4))

    btn = tk.Button(
        root, text="start recording",
        bg="#2ecc71", fg="white",
        font=("Arial", 11, "bold"),
        relief="flat", padx=12, pady=6,
        command=toggle
    )
    btn.pack(pady=4)

    save_btn = tk.Button(
        root, text="save csv",
        bg="#3498db", fg="white",
        font=("Arial", 11, "bold"),
        relief="flat", padx=12, pady=6,
        command=save
    )
    save_btn.pack(pady=4)

    root.mainloop()

# returns the baud rate the config is using
def send_cfg(cfg_path: str, cli_baud_rate: int, cli_port: str, data_port: str):
    cli = serial.Serial(cli_port, cli_baud_rate, timeout=1)

    # Read the config file
    with open(cfg_path, "r") as f:
        cfg = [line.strip() for line in f if line.strip() and not line.startswith("%")]

    # test a dummy write to the radar
    _ = cli.write(b"\n\n")
    time.sleep(0.1)
    cli.reset_input_buffer()

    for line in cfg:
        print(f"sending {line}")

        # send the data to the radar
        _ = cli.write((line).encode())
        time.sleep(0.01)
        _ = cli.write(b"\n")

        if line.split(" ")[0] == "baudRate" and cli_port == data_port:
            new_baud_rate = int(line.split(" ")[1])
            cli.baudrate = new_baud_rate
            cli_baud_rate = new_baud_rate
            time.sleep(0.5)

        time.sleep(0.1)

    print("Sent.")
    cli.close()


MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"


def read_uart(_x: str, data_port: str, baud_rate: int):
    global q
    print("starting read")
    ser = serial.Serial(data_port, baud_rate, timeout=1)
    buffer = bytearray()

    while True:
        bytecount = ser.in_waiting

        if bytecount <= 0:
            continue

        data = ser.read(bytecount)
        # print(data)
        buffer.extend(data)

        magic_index = buffer.find(MAGIC_WORD)

        # weve detected the magic index
        if magic_index != -1:
            if magic_index > 0:
                buffer = buffer[magic_index:]
                magic_index = 0

            if len(buffer) >= 40:

                # read in our frame header
                frame_header_raw = buffer[:40]
                num_tlvs = int.from_bytes(frame_header_raw[32:36], byteorder="little")
                frame_num = int.from_bytes(frame_header_raw[20:24], byteorder="little")
                total_packet_len = int.from_bytes(
                    frame_header_raw[12:16], byteorder="little"
                )

                tlv_offset = 40

                # make sure we have enough data in our buffer to read all the tlvs
                while len(buffer) < total_packet_len:
                    bytecount = ser.in_waiting
                    if bytecount > 0:
                        data = ser.read(bytecount)
                        buffer.extend(data)

                # tid posx	posy	posz	velx	vely	velz	accx	accy	accz

                frame = {
                    "posx": 0,
                    "tid": 0,
                    "posy": 0,
                    "posz": 0,
                    "velx": 0,
                    "vely": 0,
                    "velz": 0,
                    "accx": 0,
                    "accy": 0,
                    "accz": 0,
                    # later we will put the heatmap here
                    "list_of_points": [],  # this is a list of dictionaries with pointx, pointy, pointz and snr
                    "heatmap": [],
                }

                found_points = False
                found_target = False
                found_heatmap = False

                # print(f"num tlvs is {num_tlvs}")

                for _ in range(num_tlvs):

                    tlv_header_raw = buffer[tlv_offset : tlv_offset + 8]
                    tlv_type = int.from_bytes(tlv_header_raw[0:4], byteorder="little")
                    # print(f"TLV TYPE {tlv_type}")
                    tlv_length = int.from_bytes(tlv_header_raw[4:8], byteorder="little")

                    tlv_data = buffer[tlv_offset + 8 : tlv_offset + 8 + tlv_length]
                    tlv_offset += tlv_length + 8

                    #print(f"TLV TYPE: {tlv_type} is {tlv_length} bytes long.")

                    MMWDEMO_OUTPUT_EXT_MSG_DETECTED_POINTS = 301
                    MMWDEMO_OUTPUT_EXT_MSG_TARGET_LIST = 308
                    MMWDEMO_OUTPUT_EXT_MSG_TARGET_INDEX = 309
                    MMWDEMO_OUTPUT_EXT_MSG_RANGE_AZIMUTH_HEAT_MAP_MAJOR = 304

                    i = 1

                    if tlv_type == MMWDEMO_OUTPUT_EXT_MSG_DETECTED_POINTS:
                        # xyz_unit = float.from_number(int.from_bytes(tlv_data[0:4]))
                        xyz_unit = struct.unpack("<f", tlv_data[0:4])[0]
                        doppler_unit = struct.unpack("<f", tlv_data[4:8])[0]
                        snr_unit = struct.unpack("<f", tlv_data[8:12])[0]
                        noise_unit = struct.unpack("<f", tlv_data[12:16])[0]
                        num_detected_points = int.from_bytes(tlv_data[16:18], "little")

                        if num_detected_points < MINIMUM_POINTS:
                            continue

                        found_points = True
                        for j in range(num_detected_points):
                            offset = 20 + (j * 10)

                            x = (
                                int.from_bytes(
                                    tlv_data[offset : offset + 2], "little", signed=True
                                )
                                * xyz_unit
                            )
                            y = (
                                int.from_bytes(
                                    tlv_data[offset + 2 : offset + 4],
                                    "little",
                                    signed=True,
                                )
                                * xyz_unit
                            )
                            z = (
                                int.from_bytes(
                                    tlv_data[offset + 4 : offset + 6],
                                    "little",
                                    signed=True,
                                )
                                * xyz_unit
                            )

                            doppler = (
                                int.from_bytes(
                                    tlv_data[offset + 6 : offset + 8],
                                    "little",
                                    signed=True,
                                )
                                * doppler_unit
                            )

                            snr = (
                                int.from_bytes(
                                    tlv_data[offset + 8 : offset + 9],
                                    "little",
                                    signed=True,
                                )
                                * snr_unit
                            )

                            _ = {
                                int.from_bytes(tlv_data[29:30], "little", signed=True)
                                * snr_unit
                            }

                            # print(f"x: {x}\n y: {y}\n z : {z}\n snr: {snr}")
                            point = [x, y, z, doppler, snr]
                            frame["list_of_points"].append(point)

                            i += 1
                            pass
                    elif tlv_type == MMWDEMO_OUTPUT_EXT_MSG_RANGE_AZIMUTH_HEAT_MAP_MAJOR:
                        num_range_bins = 32
                        num_azimuth_bins = 32
                        expected_size = num_range_bins * num_azimuth_bins * 4

                        heatmap_1d = np.frombuffer(tlv_data[:expected_size], dtype=np.uint32)
                        heatmap_2d = heatmap_1d.reshape(num_range_bins, num_azimuth_bins)

                        found_heatmap = True
                        frame["heatmap"] = heatmap_2d

                    elif tlv_type == MMWDEMO_OUTPUT_EXT_MSG_TARGET_LIST:
                        found_target = True
                        vals = struct.unpack("<I9f", tlv_data[:40])

                        frame["tid"] = vals[0]
                        frame["posx"], frame["posy"], frame["posz"] = (
                            vals[1],
                            vals[2],
                            vals[3],
                        )
                        frame["velx"], frame["vely"], frame["velz"] = (
                            vals[4],
                            vals[5],
                            vals[6],
                        )
                        frame["accx"], frame["accy"], frame["accz"] = (
                            vals[7],
                            vals[8],
                            vals[9],
                        )
                    elif tlv_type == MMWDEMO_OUTPUT_EXT_MSG_TARGET_INDEX:
                        pass
                    elif tlv_type == 1031:
                        print("1031 found")

                if found_target and found_points and found_heatmap:
                    try:
                        q.put(frame, block=False)
                        # print("put in queue")
                    except queue.Full:
                        pass

                buffer = buffer[total_packet_len:]


def infer(X_pc, X_hm):
    #print(X_pc.dtype, X_pc.shape)
    #print(X_hm.dtype, X_hm.shape)
    #torch_out = model(torch.tensor(X_pc), torch.tensor(X_hm))
    outputs = ort_session.run(None, {
        "point_cloud_input": X_pc, 
        "heatmap_input": X_hm
    })

    logits = outputs[0]

    e_x = np.exp(logits - np.max(logits, axis=1, keepdims=True))
    probs = e_x / e_x.sum(axis=1, keepdims=True)

    return np.argmax(probs[0])


def process(data_dict):
    posy = data_dict.get("posy", 0.0)
    base_features = [
        data_dict.get("posz", 0.0),
        data_dict.get("velx", 0.0),
        data_dict.get("vely", 0.0),
        data_dict.get("velz", 0.0),
        data_dict.get("accx", 0.0),
        data_dict.get("accy", 0.0),
        data_dict.get("accz", 0.0),
    ]

    list_of_points = data_dict.get("list_of_points", [])
    raw_points = [[p[1] - posy, p[2], p[4]] for p in list_of_points]
    raw_points = [p for p in raw_points if -4 <= p[0] <= 5 and -4 <= p[1] <= 3]

    raw_points.sort(key=lambda x: x[1])


    selected = raw_points[-5:]
    if len(selected) < 5:
        selected = selected + [[0.0, 0.0, 0.0]] * (5 - len(selected))

    if len(raw_points) > 0:
        top_point = max(raw_points, key=lambda x: x[1])
        # print(f"Detected Head Height: {top_point[1]:.2f} meters")

    selected.sort(key=lambda x: x[0])

    flat_points = [val for pt in selected for val in pt]

    heatmap = data_dict.get("heatmap")
    heatmap_list = [heatmap.flatten().tolist()]

    return base_features + flat_points + heatmap_list



def predict(status_out_queue: queue.Queue | None = None):
    global q
    global pressed
    global frames

    WINDOW_SIZE = 8
    FEATURE_COUNT = 22
    window = deque(maxlen=WINDOW_SIZE)

    class_data = {0: 'SITTING', 1: 'FALLING', 2: 'WALKING', 3: 'STS', 4: 'STANDING'}

    pred_window = deque(maxlen=3)
    curr_status = 2

    columns = [
        'posz', 'velx', 'vely', 'velz', 'accx', 'accy', 'accz',
        'p1x', 'p1y', 'p1z', 'p2x', 'p2y', 'p2z', 'p3x', 'p3y', 'p3z',
        'p4x', 'p4y', 'p4z', 'p5x', 'p5y', 'p5z', 'heatmap'
    ]

    i = 0

    while True: 
        raw_data = q.get()
        processed_row = process(raw_data)

        if RECORD_MODE and pressed:
            frames.append(processed_row)
            print(f"frame saved {i}")
            i += 1
            continue
        elif RECORD_MODE:
            continue

        window.append(processed_row)

        if len(window) < WINDOW_SIZE:
            continue

        pc_list = [f[:22] for f in window]
        hm_list = [f[-1] for f in window]

        X_pc = np.array(pc_list, dtype=np.float32).reshape(1, 8, 22)
        X_hm = np.array(hm_list, dtype=np.float32).reshape(1, 8, 32, 32)
        X_hm = X_hm[:, np.newaxis, :, :, :]

        result = int(infer(X_pc, X_hm))

        pred_window.append(result)
        if len(set(pred_window)) == 1:
            if class_data[pred_window[0]] != 'STS':
                curr_status = pred_window[0]
            else:
                curr_status = 0

        print(f"status {class_data[curr_status]}")
        if status_out_queue is not None:
            status_out_queue.put_nowait(class_data[curr_status])


def main():
    cli_port = "/dev/ttyACM0"
    data_port = "/dev/ttyACM0"
    cli_baud_rate = 115200

    start_p = lambda: read_uart("", data_port, 1250000)

    send_cfg("config.cfg", cli_baud_rate, cli_port, data_port)


    pt = threading.Thread(target=start_p, name="read uart", daemon=True)
    ct = threading.Thread(target=predict, name="predict", daemon=True)

    pt.start()
    ct.start()

    if RECORD_MODE:
        launch_recorder_ui()
    else:
        ct.join()

    pt.join()


# device = "cuda" if torch.cuda.is_available() else "cpu"
# device = "cpu"
# model = NeuralNetwork(176, 5).to(device)
# model.load_state_dict(torch.load("model.pth", map_location=device))
# model.eval()
#
# dummy_input = torch.randn(1, 176)
#
## 3. Export to ONNX
# torch.onnx.export(
#    model,
#    dummy_input,
#    "model.onnx",
#    export_params=True,
#    opset_version=12,
#    do_constant_folding=True,
#    input_names=["input"],
#    output_names=["output"],
# )
# print("Model exported to model.onnx")

ort_session = ort.InferenceSession("model.pth")

if __name__ == "__main__":
    main()
