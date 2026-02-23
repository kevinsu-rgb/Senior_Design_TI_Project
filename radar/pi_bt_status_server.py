import argparse
import json
import queue
import socket
import threading
import time

import read2


def run_bt_server(status_q: queue.Queue, channel: int):
    while True:
        server_sock = None
        client_sock = None
        try:
            server_sock = socket.socket(
                socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM
            )
            server_sock.bind((socket.BDADDR_ANY, channel))
            server_sock.listen(1)
            print(f"[BT] Waiting for laptop connection on RFCOMM channel {channel}...")

            client_sock, client_info = server_sock.accept()
            print(f"[BT] Laptop connected: {client_info}")

            while True:
                status = status_q.get()
                payload = {
                    "status": status,
                    "ts": time.time(),
                }
                line = json.dumps(payload) + "\n"
                client_sock.sendall(line.encode("utf-8"))
        except Exception as exc:
            print(f"[BT] Connection dropped or failed: {exc}. Restarting in 2s...")
            time.sleep(2)
        finally:
            try:
                if client_sock is not None:
                    client_sock.close()
            except Exception:
                pass
            try:
                if server_sock is not None:
                    server_sock.close()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(
        description="Run radar ML on Pi and stream statuses via Bluetooth RFCOMM."
    )
    parser.add_argument("--cli-port", default="/dev/ttyACM0")
    parser.add_argument("--data-port", default="/dev/ttyACM0")
    parser.add_argument("--cfg-path", default="config.cfg")
    parser.add_argument("--cli-baud", type=int, default=115200)
    parser.add_argument("--data-baud", type=int, default=1250000)
    parser.add_argument("--bt-channel", type=int, default=1)
    args = parser.parse_args()

    status_q: queue.Queue = queue.Queue(maxsize=4)

    read2.send_cfg(args.cfg_path, args.cli_baud, args.cli_port, args.data_port)

    reader_thread = threading.Thread(
        target=read2.read_uart,
        args=("", args.data_port, args.data_baud),
        daemon=True,
        name="radar-read-uart",
    )
    predict_thread = threading.Thread(
        target=read2.predict,
        kwargs={"status_out_queue": status_q},
        daemon=True,
        name="radar-predict",
    )
    bt_thread = threading.Thread(
        target=run_bt_server,
        args=(status_q, args.bt_channel),
        daemon=True,
        name="bt-rfcomm-server",
    )

    reader_thread.start()
    predict_thread.start()
    bt_thread.start()

    reader_thread.join()
    predict_thread.join()
    bt_thread.join()


if __name__ == "__main__":
    main()
