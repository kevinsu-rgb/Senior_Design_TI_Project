from sys import byteorder
import serial
import threading
import time
import struct


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

                # tid posx	posy	posz	velx	vely	velz	accx	accy	accz

                print(f"TVLS DETECTED {num_tlvs}")
                for _ in range(num_tlvs):

                    frame = {
                        "tid": 0,
                        "posx": 0,
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

                    tlv_header_raw = buffer[tlv_offset : tlv_offset + 8]
                    tlv_type = int.from_bytes(tlv_header_raw[0:4], byteorder="little")
                    tlv_length = int.from_bytes(tlv_header_raw[4:8], byteorder="little")

                    tlv_data = buffer[tlv_offset + 8 : tlv_offset + 8 + tlv_length]
                    tlv_offset += tlv_length + 8

                    print(f"TLV TYPE: {tlv_type} is {tlv_length} bytes long.")

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

                        # print(xyz_unit)

                        for _ in range(num_detected_points):

                            x = (
                                int.from_bytes(tlv_data[20:22], "little", signed=True)
                                * xyz_unit
                            )
                            y = (
                                int.from_bytes(tlv_data[22:24], "little", signed=True)
                                * xyz_unit
                            )
                            z = (
                                int.from_bytes(tlv_data[24:26], "little", signed=True)
                                * xyz_unit
                            )

                            doppler = (
                                int.from_bytes(tlv_data[26:28], "little", signed=True)
                                * doppler_unit
                            )

                            snr = (
                                int.from_bytes(tlv_data[28:29], "little", signed=True)
                                * snr_unit
                            )

                            _ = {
                                int.from_bytes(tlv_data[29:30], "little", signed=True)
                                * snr_unit
                            }

                            # print(f"x: {x}\n y: {y}\n z : {z}\n snr: {snr}")
                            point = {
                                f"pointx{i}": x,
                                f"pointy{i}": y,
                                f"pointz{i}": z,
                                f"pointz{i}": doppler,
                                f"pointz{i}": snr,
                            }

                            i += 1

                            frame["list_of_points"] += [point]

                            pass
                    elif tlv_type == MMWDEMO_OUTPUT_EXT_MSG_TARGET_LIST:
                        frame["tid"] = int.from_bytes(tlv_data[0:4], "little")
                        frame["posx"] = struct.unpack("<f", tlv_data[4:8])[0]
                        frame["posy"] = struct.unpack("<f", tlv_data[8:12])[0]
                        frame["posz"] = struct.unpack("<f", tlv_data[12:16])[0]
                        frame["velx"] = struct.unpack("<f", tlv_data[16:20])[0]
                        frame["vely"] = struct.unpack("<f", tlv_data[20:24])[0]
                        frame["velz"] = struct.unpack("<f", tlv_data[24:28])[0]
                        frame["accx"] = struct.unpack("<f", tlv_data[28:32])[0]
                        frame["accy"] = struct.unpack("<f", tlv_data[32:36])[0]
                        frame["accz"] = struct.unpack("<f", tlv_data[36:40])[0]
                        pass
                    elif tlv_type == MMWDEMO_OUTPUT_EXT_MSG_TARGET_INDEX:
                        pass

                    if num_tlvs == 3:
                        print(frame)

                buffer = buffer[total_packet_len:]


def main():
    cli_port = "/dev/ttyACM0"
    data_port = "/dev/ttyACM0"
    cli_baud_rate = 115200

    start_p = lambda: read_uart("", data_port, 1250000)

    send_cfg("config.cfg", cli_baud_rate, cli_port, data_port)
    pt = threading.Thread(target=start_p, name="read uart", daemon=True)

    pt.start()
    while True:
        continue


if __name__ == "__main__":
    main()
