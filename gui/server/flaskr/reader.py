from pathlib import Path
import onnxruntime as ort
from collections import deque
import serial
import threading
import time
import struct
import numpy as np
import queue
import os

q: queue.Queue = queue.Queue(1)

from . import status_queue

MINIMUM_POINTS = 5


# returns the baud rate the config is using
def send_cfg(cfg_path: str, cli_baud_rate: int, cli_port: str, data_port: str):

    print(f"Current Working Directory: {os.getcwd()}")
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
                    "accx": 0,
                    "accy": 0,
                    "accz": 0,
                    # later we will put the heatmap here
                    "list_of_points": [],  # this is a list of dictionaries with pointx, pointy, pointz and snr
                }

                found_points = False
                found_target = False

                # print(f"num tlvs is {num_tlvs}")

                for _ in range(num_tlvs):

                    tlv_header_raw = buffer[tlv_offset : tlv_offset + 8]
                    tlv_type = int.from_bytes(tlv_header_raw[0:4], byteorder="little")
                    # print(f"TLV TYPE {tlv_type}")
                    tlv_length = int.from_bytes(tlv_header_raw[4:8], byteorder="little")

                    tlv_data = buffer[tlv_offset + 8 : tlv_offset + 8 + tlv_length]
                    tlv_offset += tlv_length + 8

                    # print(f"TLV TYPE: {tlv_type} is {tlv_length} bytes long.")

                    MMWDEMO_OUTPUT_EXT_MSG_DETECTED_POINTS = 301
                    MMWDEMO_OUTPUT_EXT_MSG_TARGET_LIST = 308
                    MMWDEMO_OUTPUT_EXT_MSG_TARGET_INDEX = 309

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

                    if found_target and found_points:
                        try:
                            q.put(frame, block=False)
                            # print("put in queue")
                        except queue.Full:
                            pass

                buffer = buffer[total_packet_len:]


def infer(X, ort_session):
    input_tensor = np.array(X, dtype=np.float32).reshape(1, -1)
    outputs = ort_session.run(None, {"input": input_tensor})
    return np.argmax(outputs[0], axis=1)[0]


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

    raw_points.sort(key=lambda x: x[1])

    if len(raw_points) >= 5:
        selected = raw_points[-3:] + raw_points[:2]
    else:
        selected = raw_points + [[0.0, 0.0, 0.0]] * (5 - len(raw_points))

    if len(raw_points) > 0:
        top_point = max(raw_points, key=lambda x: x[1])
        # print(f"Detected Head Height: {top_point[1]:.2f} meters")

    selected.sort(key=lambda x: x[0])

    flat_points = [val for pt in selected for val in pt]

    return base_features + flat_points


def predict():
    global q  # this is the queue which gets the point clouds
    global status_queue  # this is the queue to send to flask
    WINDOW_SIZE = 8
    FEATURE_COUNT = 22
    window = deque(maxlen=WINDOW_SIZE)

    for _ in range(WINDOW_SIZE):
        window.append(np.zeros(FEATURE_COUNT))

    path = Path(__file__).resolve().parent / "models" / "model.onnx"
    ort_session = ort.InferenceSession(path)

    class_data = {0: "STANDING", 1: "SITTING", 2: "LYING", 3: "FALLING", 4: "WALKING"}
    class_predicted = 0

    while True:
        raw_data = q.get()
        processed_row = process(raw_data)
        window.append(processed_row)

        X = np.array(window).T.flatten().astype(np.float32)

        result = infer(X, ort_session)

        if class_predicted == 3:
            if result == 4:
                class_predicted = result
        else:
            class_predicted = result

        status_queue.put(class_data[int(class_predicted)])
        # print(f"Status: {class_data[int(class_predicted)]}")


def main():
    cli_port = "/dev/ttyACM0"
    data_port = "/dev/ttyACM0"
    cli_baud_rate = 115200

    start_p = lambda: read_uart("", data_port, 1250000)

    send_cfg("config.cfg", cli_baud_rate, cli_port, data_port)
    pt = threading.Thread(target=start_p, name="read uart", daemon=True)
    ct = threading.Thread(target=predict, name="predict data", daemon=True)

    pt.start()
    ct.start()

    pt.join()
    ct.join()


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
