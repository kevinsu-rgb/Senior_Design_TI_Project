from flask.cli import cli
import serial
import random
import numpy as np
import struct
import os
import time
from contextlib import suppress
import subprocess
import queue

status_queue = queue.Queue()


def send_cfg(cli_port, cfg_path):
    # 1. Clean and Read CFG
    if not os.path.isfile(cfg_path):
        print(f"File not found: {cfg_path}")
        return 2

    with open(cfg_path, "r") as f:
        cfg = [line.strip() for line in f if line.strip() and not line.startswith("%")]

    cli = None
    try:
        # 2. Open ONLY CLI Port (avoids Access Denied on Data Port)
        cli = serial.Serial(cli_port, 115200, timeout=1)
        cli.write(b"\n\n")  # Wake up radar
        time.sleep(0.1)
        cli.reset_input_buffer()

        for line in cfg:
            print(f"Sending: {line}")

            # 3. Send character-by-character (Prevents 'command not recognized' errors)
            for char in line + "\n":
                cli.write(char.encode())
                time.sleep(0.001)

            # 4. Handle Baud Rate Jump in the CFG
            if "baudRate" in line:
                try:
                    new_baud = int(line.split()[1])
                    time.sleep(0.1)  # Wait for radar to switch
                    cli.baudrate = new_baud
                    print(f"--- CLI Speed Switched to {new_baud} ---")
                except:
                    print("Baudrate switch failed")

            # 5. Read all responses (handles 'Done', 'Error', or Warnings)
            time.sleep(0.1)
            while cli.in_waiting:
                response = cli.readline().decode(errors="ignore").strip()
                if response:
                    print(f"   Radar: {response}")

        print("Configuration complete.")
        return 0

    except Exception as e:
        print(f"Failed: {e}")
        return 4
    finally:
        if cli:
            cli.close()


MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"
prev_status = "falling"


def read_uart(cli_port="COM7", data_port="COM8"):
    global prev_status
    # global queue
    ser = serial.Serial(data_port, baudrate=1250000)
    print("READING")

    buffer = bytearray()

    while True:
        bytecount = ser.in_waiting

        if bytecount <= 0:
            print("BUFFER EMPTYY")
            continue

        data = ser.read(bytecount)
        buffer.extend(data)

        print("BUFFER LENGTH:", len(buffer))

        magic_index = buffer.find(MAGIC_WORD)

        # weve detected the magic index
        if magic_index != -1:
            print("MAGIC WORD DETECTED")
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

                # make sure we have enought data in our buffer to read all the tlvs
                while len(buffer) < total_packet_len:
                    bytecount = ser.in_waiting
                    if bytecount > 0:
                        data = ser.read(bytecount)
                        buffer.extend(data)

                for _ in range(num_tlvs):
                    print("READING TLV")
                    tlv_header_raw = buffer[tlv_offset : tlv_offset + 8]
                    tlv_type = int.from_bytes(tlv_header_raw[0:4], byteorder="little")
                    tlv_length = int.from_bytes(tlv_header_raw[4:8], byteorder="little")

                    tlv_data = buffer[tlv_offset + 8 : tlv_offset + 8 + tlv_length]
                    tlv_offset += tlv_length + 8

                    print(
                        f"TLV Type: {tlv_type}, Length: {tlv_length}, Data: {tlv_data}"
                    )

                    MMWDEMO_OUTPUT_MSG_PRESCENCE_INDICATION = 1021
                    MMWDEMO_OUTPUT_MSG_TARGET_HEIGHT = 1012
                    HEATMAP_TLV = 304

                    NUM_RANGE_BINS = 64
                    NUM_AZIMUTH_BINS = 8

                    if tlv_type == MMWDEMO_OUTPUT_MSG_TARGET_HEIGHT:
                        maxZ = struct.unpack("<f", tlv_data[4:8])[0]
                        minZ = struct.unpack("<f", tlv_data[8:12])[0]
                        status = "standing" if (maxZ) > 1.6 else "falling"
                        # print(maxZ)

                        if prev_status != status:
                            # print(f"Status changed: {prev_status} -> {status}")
                            prev_status = status
                            if queue.qsize() < 10:
                                queue.put(status)
                            else:
                                print("Queue full, dropping status update")

                    elif tlv_type == MMWDEMO_OUTPUT_MSG_PRESCENCE_INDICATION:
                        pass
                    elif tlv_type == HEATMAP_TLV:
                        print("I CAN REAED THE TLV")
                        expected_size = 4 * NUM_RANGE_BINS * NUM_AZIMUTH_BINS
                        num_elements = NUM_RANGE_BINS * NUM_AZIMUTH_BINS
                        heatmap_data = struct.unpack(
                            f"<{num_elements}I", tlv_data[:expected_size]
                        )
                        heatmap_2d = np.array(heatmap_data).reshape(
                            NUM_RANGE_BINS, NUM_AZIMUTH_BINS
                        )
                        print("HEATMAP IS:", heatmap_2d)

                buffer = buffer[magic_index + 40 :]


def testuart(data_port="/dev/ttyACM1"):
    # Open with a short timeout
    ser = serial.Serial(data_port, baudrate=1250000, timeout=5.0)
    print("Forcing a 2-second listen...")

    # Read anything available, even if it's 0 bytes
    raw_data = ser.read(100)

    if len(raw_data) == 0:
        print("RESULT: Absolute Silence. The radar is NOT transmitting.")
        print(
            "Check: Is the green LED on the board blinking? Is 'sensorStart' returning 'Done'?"
        )
    else:
        print(f"RESULT: Received {len(raw_data)} bytes. Data is flowing!")
        print(f"Hex: {raw_data.hex()}")


def main():
    cli = "/dev/ttyACM0"
    data = "/dev/ttyACM1"
    send_cfg(cli_port=cli, cfg_path="ufcfg.cfg")
    time.sleep(1)
    testuart(data_port=data)


main()
