import serial
import random
import numpy as np
import struct
import os
import time
from contextlib import suppress
from . import queue


# TODO: REPLACE THIS WITH ~/radar/read.py
def send_cfg(
    cli_port,
    cfg_path,
    data_port=None,
    device="xWR6843",
    single_com=False,
    cli_baud=115200,
):

    # Read cfg file
    if not os.path.isfile(cfg_path):
        print("cfg file not found: %s", cfg_path)
        return 2

    with open(cfg_path, "r", encoding="utf-8", errors="replace") as fh:
        cfg_lines = fh.readlines()

    # Clean lines similar to GUI: remove blank lines, ensure trailing newline, remove commented lines
    cfg = [line for line in cfg_lines if line != "\n"]
    cfg = [line + "\n" if not line.endswith("\n") else line for line in cfg]
    cfg = [line for line in cfg if line.strip() and not line.lstrip().startswith("%")]

    cli = None
    data = None
    is_low_power = False
    try:
        if single_com:
            # Single COM devices open CLI at requested baud
            cli = serial.Serial(
                cli_port,
                cli_baud,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.6,
            )
            cli.reset_output_buffer()
            is_low_power = True
        else:
            # Double COM: CLI at 115200, DATA at device-specific baud
            cli = serial.Serial(
                cli_port,
                115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.6,
            )
            if device == "xWRL6844":
                data = serial.Serial(
                    data_port,
                    1250000,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.6,
                )
            else:
                data = serial.Serial(
                    data_port,
                    921600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.6,
                )
            data.reset_output_buffer()
            cli.reset_output_buffer()
            is_low_power = False
    except Exception as e:
        print("Failed to open serial ports: %s", e)
        with suppress(Exception):
            if cli is not None:
                cli.close()
        with suppress(Exception):
            if data is not None:
                data.close()
        return 4

    # Send lines
    try:
        for line in cfg:
            time.sleep(0.03)

            if cli.baudrate == 1250000:
                for char in [*line]:
                    time.sleep(0.001)
                    cli.write(char.encode())
            else:
                cli.write(line.encode())

            ack = cli.readline()
            if len(ack) == 0:
                print("ERROR: No data detected on COM Port, read timed out")
                print(
                    "\tBe sure that the device is in the proper SOP mode after flashing with the correct binary, and that the cfg you are sending is valid"
                )
                return 5

            print(ack, flush=True)

            ack = cli.readline()
            print(ack, flush=True)

            if is_low_power:
                ack = cli.readline()
                print(ack, flush=True)
                ack = cli.readline()
                print(ack, flush=True)

            splitLine = line.split()
            if splitLine and splitLine[0] == "baudRate":
                try:
                    cli.baudrate = int(splitLine[1])
                except Exception:
                    print(
                        "Error - Invalid baud rate: %s"
                        % (splitLine[1] if len(splitLine) > 1 else None)
                    )
                    with suppress(Exception):
                        cli.close()
                    with suppress(Exception):
                        if data is not None:
                            data.close()
                    return 6

        # Give buffer time to clear
        time.sleep(0.03)
        with suppress(Exception):
            cli.reset_input_buffer()
        return 0
    except Exception as e:
        print("Failed while sending cfg: %s" % e)
        return 7
    finally:
        with suppress(Exception):
            if cli is not None:
                cli.close()
        with suppress(Exception):
            if data is not None:
                data.close()


MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"
prev_status = "falling"


def read_uart(cli_port="COM7", data_port="COM8"):
    global prev_status
    global queue
    ser = serial.Serial(data_port, baudrate=921600)
    print("READING")

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

                # make sure we have enought data in our buffer to read all the tlvs
                while len(buffer) < total_packet_len:
                    bytecount = ser.in_waiting
                    if bytecount > 0:
                        data = ser.read(bytecount)
                        buffer.extend(data)

                for i in range(num_tlvs):
                    tlv_header_raw = buffer[tlv_offset : tlv_offset + 8]
                    tlv_type = int.from_bytes(tlv_header_raw[0:4], byteorder="little")
                    tlv_length = int.from_bytes(tlv_header_raw[4:8], byteorder="little")

                    tlv_data = buffer[tlv_offset + 8 : tlv_offset + 8 + tlv_length]
                    tlv_offset += tlv_length + 8

                    # print(f"TLV Type: {tlv_type}, Length: {tlv_length}, Data: {tlv_data}")

                    MMWDEMO_OUTPUT_MSG_PRESCENCE_INDICATION = 1021
                    MMWDEMO_OUTPUT_MSG_TARGET_HEIGHT = 1012

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

                buffer = buffer[magic_index + 40 :]
