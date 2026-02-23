import serial
import matplotlib.pyplot as plt
import time
import glob
import struct
import numpy as np
import threading
import torch
from nn import NeuralNetwork
import os
import queue
from pynput import keyboard
import signal
import sys

q: queue.Queue[np.ndarray[tuple[int, int], np.dtype[np.float32]]] = queue.Queue(1)
q_pointcloud: queue.Queue[np.ndarray] = queue.Queue(1)
q_tracker: queue.Queue[np.ndarray] = queue.Queue(1)  # NEW: Queue for tracker data

space_pressed = False
shutdown_flag = threading.Event()

def signal_handler(sig, frame):
    print('\nShutting down')
    shutdown_flag.set()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def on_press(key):
    global space_pressed
    
    if key == keyboard.Key.space:
        space_pressed = not space_pressed
        print(f"Recording status: {space_pressed}")

def listen_for_spacebar():
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

# returns the baud rate the config is using
def send_cfg(cfg_path: str, cli_baud_rate: int, cli_port: str, data_port: str) -> int:
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

        if line.split(" ")[0] == "baudRate" and cli_port == data_port:
            new_baud_rate = int(line.split(" ")[1])
            cli.baudrate = new_baud_rate
            cli_baud_rate = new_baud_rate
            time.sleep(0.5)

        time.sleep(0.1)

    print("Sent.")
    cli.close()
    return cli_baud_rate


MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"


def read_uart(_x: str, data_port: str, baud_rate: int):
    print("starting read")
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

                # variables to store frame data
                current_pointcloud = None
                current_tracker = None

                for _ in range(num_tlvs):
                    tlv_header_raw = buffer[tlv_offset : tlv_offset + 8]
                    tlv_type = int.from_bytes(tlv_header_raw[0:4], byteorder="little")
                    tlv_length = int.from_bytes(tlv_header_raw[4:8], byteorder="little")

                    tlv_data = buffer[tlv_offset + 8 : tlv_offset + 8 + tlv_length]
                    tlv_offset += tlv_length + 8

                    # TLV type constants
                    POINT_CLOUD_EXT_TLV = 301
                    TRACKER_TLV = 308

                    # parse Point Cloud TLV
                    if tlv_type == POINT_CLOUD_EXT_TLV:
                        try:
                            # struct formats (from TI reference)
                            pUnitStruct = '4f2h'  # 4 floats, 2 shorts (decompression units)
                            pointStruct = '4h2B'  # 4 shorts (x,y,z,doppler), 2 bytes (snr, noise)
                            
                            pUnitSize = struct.calcsize(pUnitStruct)
                            pointSize = struct.calcsize(pointStruct)
                            
                            # parse decompression factors
                            pUnit = struct.unpack(pUnitStruct, tlv_data[:pUnitSize])
                            
                            # calculate number of points
                            numPoints = int((tlv_length - pUnitSize) / pointSize)
                            
                            if numPoints > 0:
                                # initialize point cloud array [y, z, snr] only
                                pointCloud = np.zeros((numPoints, 3), dtype=np.float32)
                                
                                # skip past decompression units
                                data_ptr = pUnitSize
                                
                                # parse each point
                                for i in range(numPoints):
                                    try:
                                        x, y, z, doppler, snr, noise = struct.unpack(
                                            pointStruct, 
                                            tlv_data[data_ptr:data_ptr + pointSize]
                                        )
                                        
                                        # decompress values - only save y, z, snr
                                        pointCloud[i, 0] = y * pUnit[0]       # y
                                        pointCloud[i, 1] = z * pUnit[0]       # z
                                        pointCloud[i, 2] = snr * pUnit[2]     # SNR
                                        
                                        data_ptr += pointSize
                                        
                                    except struct.error:
                                        print(f"Failed to parse point {i}")
                                        break
                                
                                current_pointcloud = pointCloud
                                
                        except Exception as e:
                            print(f"Point Cloud parsing error: {e}")

                    # parse Tracker TLV
                    elif tlv_type == TRACKER_TLV:
                        try:
                            targetStruct = 'I27f'
                            targetSize = struct.calcsize(targetStruct)
                            numDetectedTargets = int(tlv_length / targetSize)
                            
                            if numDetectedTargets > 0:
                                # We only care about the first target
                                targetData = struct.unpack(targetStruct, tlv_data[:targetSize])
                                
                                # tid, posx, posy, posz, velx, vely, velz, accx, accy, accz
                                tracker_info = np.array([
                                    targetData[0],   # Target ID
                                    targetData[1],   # X Position
                                    targetData[2],   # Y Position
                                    targetData[3],   # Z Position
                                    targetData[4],   # X Velocity
                                    targetData[5],   # Y Velocity
                                    targetData[6],   # Z Velocity
                                    targetData[7],   # X Acceleration
                                    targetData[8],   # Y Acceleration
                                    targetData[9]    # Z Acceleration
                                ], dtype=np.float32)
                                
                                current_tracker = tracker_info
                                
                        except Exception as e:
                            print(f"Tracker parsing error: {e}")

                # put data in queues after parsing all TLVs in frame
                if current_pointcloud is not None:
                    try:
                        q_pointcloud.put_nowait((frame_num, current_pointcloud))
                    except queue.Full:
                        pass
                
                if current_tracker is not None:
                    try:
                        q_tracker.put_nowait(current_tracker)
                    except queue.Full:
                        pass

                buffer = buffer[total_packet_len:]


def predict():
    global space_pressed
    global shutdown_flag

    record_pose = "STANDING"

    # create directories for combined data
    os.makedirs(f"combined/data/{record_pose}", exist_ok=True)

    frame_count = 0
    
    # Open CSV file for writing
    csv_filepath = f"combined/data/{record_pose}/tracker_pointcloud_data.csv"
    
    # Estimate max points (you can adjust this based on your radar config)
    MAX_POINTS_ESTIMATE = 50
    
    # Write header with estimated point columns
    # Format matches notebook expectations: tid, posx, posy, posz, velx, vely, velz, accx, accy, accz, pointy0, pointz0, snr0, ...
    with open(csv_filepath, 'w') as csv_file:
        header = "Frame count,tid,posx,posy,posz,velx,vely,velz,accx,accy,accz"
        for i in range(MAX_POINTS_ESTIMATE):
            header += f",pointy{i},pointz{i},snr{i}"
        csv_file.write(header + '\n')
    
    # Open in append mode
    csv_file = open(csv_filepath, 'a')
    
    try:
        while not shutdown_flag.is_set():
            # check if we have data in queues
            pointcloud_data = None
            tracker_data = None
            frame_num = None
            
            try:
                frame_num, pointcloud_data = q_pointcloud.get_nowait()
            except queue.Empty:
                pass
            
            try:
                tracker_data = q_tracker.get_nowait()
            except queue.Empty:
                pass
            
            # if no data at all, wait a bit and continue
            if pointcloud_data is None and tracker_data is None:
                time.sleep(0.01)
                continue

            # Only record when spacebar is pressed
            if not space_pressed:
                continue

            # Only save if we have tracker data
            if tracker_data is not None:
                frame_count += 1
                
                # Start building the row
                row_data = [frame_count]
                
                # Add tracker data: tid, posx, posy, posz, velx, vely, velz, accx, accy, accz
                row_data.extend(tracker_data.tolist())
                
                # Add point cloud data if available
                if pointcloud_data is not None:
                    # Flatten point cloud: pointy0, pointz0, snr0, pointy1, pointz1, snr1, ...
                    for point in pointcloud_data:
                        row_data.extend(point.tolist())  # [y, z, snr]
                    
                    print(f"Frame {frame_count} - Tracker: tid={int(tracker_data[0])}, Points: {pointcloud_data.shape[0]}")
                else:
                    print(f"Frame {frame_count} - Tracker: tid={int(tracker_data[0])}, Points: 0")
                
                # Write row to CSV
                csv_file.write(','.join(map(str, row_data)) + '\n')
                csv_file.flush()  # Ensure data is written immediately
    
    finally:
        csv_file.close()
        print("CSV file closed.")


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


def main():
    cli_port = "COM5"
    data_port = "COM5"
    cli_baud_rate = 115200

    # create directory structure
    for pose in ["SITTING", "STANDING"]:
        os.makedirs(f"combined/data/{pose}", exist_ok=True)

    _ = send_cfg("config.cfg", cli_baud_rate, cli_port, data_port)

    start_p = lambda: read_uart("", data_port, 1250000)

    pt = threading.Thread(target=start_p, name="read uart", daemon=True)
    ct = threading.Thread(target=predict, name="predict data", daemon=True)
    lt = threading.Thread(target=listen_for_spacebar, daemon=True)
    lt.start()

    pt.start()
    ct.start()

    # Keep main thread alive but allow Ctrl+C to work
    try:
        while not shutdown_flag.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting")
        shutdown_flag.set()


device = "cuda" if torch.cuda.is_available() else "cpu"
model = NeuralNetwork(176, 5).to(device)
model.load_state_dict(torch.load("model.pth", map_location=device))
model.eval()

main()
