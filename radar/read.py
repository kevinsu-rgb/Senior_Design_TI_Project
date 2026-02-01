import serial
import time
import glob
import struct
import numpy as np
import threading
import torch
from nn import NeuralNetwork

import queue

q: queue.Queue[np.ndarray[tuple[int, int], np.dtype[np.int64]]] = queue.Queue(1)

device = "cuda" if torch.cuda.is_available() else "cpu"
model = NeuralNetwork().to(device)
model.load_state_dict(torch.load("model.pth", map_location=device))
model.eval()


# returns the baud rate the config is using
def send_cfg(cfg_path: str, cli_baud_rate: int, cli_port: str) -> int:
    cli = serial.Serial(cli_port, cli_baud_rate, timeout=1)

    # Read the config file
    with open(cfg_path, "r") as f:
        cfg = [line.strip() for line in f if line.strip() and not line.startswith("%")]

    # test a dummy write to the radar
    _ = cli.write(b"\n\n")
    time.sleep(0.1)
    cli.reset_input_buffer()

    for line in cfg:
        print("sending {}", line)

        # send the data to the radar
        _ = cli.write((line).encode())
        time.sleep(0.01)
        _ = cli.write(b"\n")

        if line.split(" ")[0] == "baudRate":
            new_baud_rate = int(line.split(" ")[1])
            cli.baudrate = new_baud_rate
            cli_baud_rate = new_baud_rate
            time.sleep(0.5)

        time.sleep(0.1)
        while cli.in_waiting:
            response = cli.readline().decode(errors="ignore").strip()
            if response:
                print(f"Response: {response}")

    print("Sent.")
    cli.close()
    return cli_baud_rate


MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"


def read_uart(_x: str, data_port: str, baud_rate: int):
    ser = serial.Serial(data_port, baud_rate, timeout=1)
    buffer = bytearray()

    while True:
        bytecount = ser.in_waiting

        if bytecount <= 0:
            continue

        data = ser.read(bytecount)
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
                _ = int.from_bytes(frame_header_raw[20:24], byteorder="little")
                total_packet_len = int.from_bytes(
                    frame_header_raw[12:16], byteorder="little"
                )

                tlv_offset = 40

                # make sure we have enought data in our buffer to read all the tlvs
                while len(buffer) < total_packet_len:
                    bytecount = ser.in_waiting
                    if bytecount > 0:
                        data = ser.read(bytecount)
                        buffer.extend(data)

                for _ in range(num_tlvs):
                    tlv_header_raw = buffer[tlv_offset : tlv_offset + 8]
                    tlv_type = int.from_bytes(tlv_header_raw[0:4], byteorder="little")
                    tlv_length = int.from_bytes(tlv_header_raw[4:8], byteorder="little")

                    tlv_data = buffer[tlv_offset + 8 : tlv_offset + 8 + tlv_length]
                    tlv_offset += tlv_length + 8

                    # print(
                    #    f"TLV Type: {tlv_type}, Length: {tlv_length}, Data: {tlv_data}"
                    # )

                    # HEATMAP_TLV = 304
                    HEATMAP_TLV = 304

                    NUM_RANGE_BINS = 64
                    NUM_AZIMUTH_BINS = 8

                    if tlv_type == HEATMAP_TLV:
                        expected_size = 4 * NUM_RANGE_BINS * NUM_AZIMUTH_BINS
                        num_elements = NUM_RANGE_BINS * NUM_AZIMUTH_BINS
                        heatmap_data = struct.unpack(
                            f"<{num_elements}I", tlv_data[:expected_size]
                        )
                        heatmap_2d = np.array(heatmap_data).reshape(
                            NUM_RANGE_BINS, NUM_AZIMUTH_BINS
                        )

                        print(heatmap_2d.ndim)

                        # lets push this heatmap to a queue, the max size of the queue is 1 but if it is full we do not put the data.
                        # print(heatmap_2d)
                        try:
                            q.put_nowait(heatmap_2d)
                        except queue.Full:
                            continue

                buffer = buffer[magic_index + 40 :]


def save_csv():
    global q

    i = 0
    while True:
        heatmap = q.get()
        filename = f"data/{i}.csv"
        np.savetxt(filename, heatmap, delimiter=",", dtype=np.float64)
        print(f"csv {filename} saved.")
        i += 1


def predict():
    CLASS_NAMES = ["FALLING", "SITTING", "STANDING", "WALKING"]

    while True:
        heatmap_2d = q.get()

        input_tensor = torch.from_numpy(heatmap_2d).float().unsqueeze(0).to(device)

        with torch.no_grad():
            logits = model(input_tensor)
            prediction = torch.argmax(logits, dim=1).item()

        print(f"Detected Class: {CLASS_NAMES[int(prediction)]}")
        q.task_done()


def check(data_port: str, data_baud_rate: int):
    try:
        data_ser = serial.Serial(data_port, data_baud_rate, timeout=1)
        print(f"Listening on {data_port}...")

        while True:
            # Read a chunk of bytes
            raw_data = data_ser.read(1024)
            if raw_data:
                print(
                    f"Received {len(raw_data)} bytes. First few: {raw_data[:8].hex()}"
                )
            else:
                print("Waiting for data... (Is the radar on and started?)")

    except Exception as e:
        print(f"Error: {e}")


def reset_ports():
    ports = glob.glob("/dev/ttyACM*")
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            print(f"Manually closed: {port}")
        except:
            print(f"Could not access {port} (maybe already closed or in use by system)")


def main():
    cli_port = "/dev/ttyACM0"
    data_port = "/dev/ttyACM0"
    cli_baud_rate = 115200

    reset_ports()
    _ = send_cfg("config.cfg", cli_baud_rate, cli_port)

    start_p = lambda: read_uart("", data_port, 1250000)

    pt = threading.Thread(target=start_p, name="read uart")
    ct = threading.Thread(target=predict, name="predict data", daemon=True)

    # read_uart("", data_port, 1250000)
    # check(data_port, 1250000)
    #
    pt.start()
    ct.start()

    pt.join()


main()
