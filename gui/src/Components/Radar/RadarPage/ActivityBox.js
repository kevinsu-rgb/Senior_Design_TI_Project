import { useEffect, useState } from "react";

export default function ActivityBox({ radar }) {
	const [activityLog, setActivityLog] = useState([]);

	useEffect(() => {
		async function fetchActivityLog() {
			try {
				const response = await fetch(`/api/radar/activity/${radar.radar_id}`);
				if (response.ok) {
					const data = await response.json();
					setActivityLog(data?.activity || []);
				}
			} catch (err) {
				console.error("Error getting activity log: ", err);
			}
		}
		fetchActivityLog();
	}, [radar.radar_id]);

	return (
		<div className="bg-bg2 rounded-lg p-8 overflow-hidden h-full flex flex-col">
			<h3 className="text-2xl font-bold text-white">Activity</h3>

			<div className="flex flex-col gap-3  overflow-y-auto mt-4">
				{activityLog.map((log, index) => (
					<div
						key={index}
						className="border-2 border-gray-600 rounded-lg bg-gray-700 p-4 flex items-center"
					>
						<span className="text-white font-mono text-lg min-w-[100px]">
							{log.time}
						</span>
						<span className="text-white text-lg ml-8">{log.event}</span>
					</div>
				))}
			</div>
		</div>
	);
}
