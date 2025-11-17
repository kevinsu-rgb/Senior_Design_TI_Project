import serial
import os
import sys
import logging
import time
from contextlib import suppress

log = logging.getLogger(__name__)


def parse_and_send_cfg(cli_port, cfg_path, data_port=None, device="xWR6843", single_com=False, cli_baud=115200):

	# Read cfg file
	if not os.path.isfile(cfg_path):
		log.error("cfg file not found: %s", cfg_path)
		return 2

	with open(cfg_path, "r", encoding="utf-8", errors="replace") as fh:
		cfg_lines = fh.readlines()

	# Clean lines similar to GUI: remove blank lines, ensure trailing newline, remove commented lines
	cfg = [line for line in cfg_lines if line != '\n']
	cfg = [line + '\n' if not line.endswith('\n') else line for line in cfg]
	cfg = [line for line in cfg if line.strip() and not line.lstrip().startswith('%')]

	cli = None
	data = None
	is_low_power = False
	try:
		if single_com:
			# Single COM devices open CLI at requested baud
			cli = serial.Serial(cli_port, cli_baud, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.6)
			cli.reset_output_buffer()
			is_low_power = True
		else:
			# Double COM: CLI at 115200, DATA at device-specific baud
			cli = serial.Serial(cli_port, 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.6)
			if device == "xWRL6844":
				data = serial.Serial(data_port, 1250000, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.6)
			else:
				data = serial.Serial(data_port, 921600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.6)
			data.reset_output_buffer()
			cli.reset_output_buffer()
			is_low_power = False
	except Exception as e:
		log.exception("Failed to open serial ports: %s", e)
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
			time.sleep(.03)

			if cli.baudrate == 1250000:
				for char in [*line]:
					time.sleep(.001)
					cli.write(char.encode())
			else:
				cli.write(line.encode())

			ack = cli.readline()
			if len(ack) == 0:
				log.error("ERROR: No data detected on COM Port, read timed out")
				log.error("\tBe sure that the device is in the proper SOP mode after flashing with the correct binary, and that the cfg you are sending is valid")
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
					log.error("Error - Invalid baud rate: %s", splitLine[1] if len(splitLine) > 1 else None)
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
		log.exception("Failed while sending cfg: %s", e)
		return 7
	finally:
		with suppress(Exception):
			if cli is not None:
				cli.close()
		with suppress(Exception):
			if data is not None:
				data.close()






def _main():
	import argparse

	logging.basicConfig(level=logging.INFO)

	p = argparse.ArgumentParser(description="Send a .cfg file to an mmWave board")
	p.add_argument("--cli", required=True, help="CLI serial port (e.g. COM3 or /dev/ttyUSB0)")
	p.add_argument("--cfg", required=True, help="Path to .cfg file to send")
	p.add_argument("--data", required=False, help="DATA port for double-com devices")
	p.add_argument("--device", default="xWR6843", help="Device name string to set in parser")
	p.add_argument("--single", action="store_true", help="Use single-COM parser (xWRLx432 family)")
	p.add_argument("--baud", type=int, default=115200, help="CLI baud for opening serial port")

	args = p.parse_args()

	rc = parse_and_send_cfg(args.cli, args.cfg, data_port=args.data, device=args.device, single_com=args.single, cli_baud=args.baud)
	if rc != 0:
		log.error("Failed to send cfg (rc=%d)", rc)
		sys.exit(rc)
	print("cfg sent successfully")


if __name__ == "__main__":
	_main()

