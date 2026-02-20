import unittest
import time
from queue import Queue

from radar import radar_list

"""
AI was used to help generate these tests.
"""


class TestRadarLogic(unittest.TestCase):

    def test_uptime_calculation(self):
        uptime = 90061

        days = uptime // 86400
        hours = (uptime % 86400) // 3600
        minutes = (uptime % 3600) // 60
        seconds = uptime % 60

        result = f"{days}d {hours}h {minutes}m {seconds}s"
        self.assertEqual(result, "1d 1h 1m 1s")

    def test_queue_status_update(self):
        status_queue = Queue()
        status_queue.put("falling")

        current_status = "unknown"
        while not status_queue.empty():
            current_status = status_queue.get_nowait().lower()

        self.assertEqual(current_status, "falling")

    def test_activity_log(self):
        activity_log = [1, 2, 3, 4, 5, 6, 7, 8]
        sliced_log = activity_log[-5:]

        self.assertEqual(len(sliced_log), 5)
        self.assertEqual(sliced_log, [4, 5, 6, 7, 8])

    def test_radar_list_structure(self):
        response = radar_list()

        self.assertIn("radars", response)

        self.assertEqual(len(response["radars"]), 1)

        first_radar = response["radars"][0]
        self.assertEqual(first_radar["radar_id"], 1)
        self.assertEqual(first_radar["name"], "Radar A")  # chr(64 + 1) is 'A'

    def test_radar_name_logic(self):
        radar_id = 2
        name = "Radar " + chr(64 + radar_id)
        self.assertEqual(name, "Radar B")


if __name__ == "__main__":
    unittest.main()
