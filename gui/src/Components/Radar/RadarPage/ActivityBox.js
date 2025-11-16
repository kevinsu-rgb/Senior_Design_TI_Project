import { useGetRadarById } from "../../../hooks/useRadar";

export default function ActivityBox({ radarId }) {
	const radar = useGetRadarById(radarId);

	if (!radar) return null;

	return (
		<div className="bg-bg2 rounded-lg p-8 overflow-hidden h-full flex flex-col">
			<h3 className="text-2xl font-bold text-white">Activity</h3>

			<div className="flex flex-col gap-3  overflow-y-auto mt-4">
				{radar?.activity_log?.map((log, index) => (
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
