import { useNavigate } from "react-router-dom";
import RadarListElement from "./RadarListElement";
import { useRadar } from "../../hooks/useRadar";

export default function RadarList() {
	const { radarList } = useRadar();
	const navigate = useNavigate();

	if (!radarList || radarList.length === 0) {
		return (
			<div className="h-full flex items-center justify-center">
				<p className="text-neutral-400 text-lg">No radars available.</p>
			</div>
		);
	}

	return (
		<div className="max-w-5xl mx-auto p-8 space-y-4 h-full">
			{radarList?.map((radar) => (
				<div
					key={radar.radar_id}
					onClick={() =>
						navigate(`/radar/${radar.radar_id}`, {
							state: { radarId: radar.radar_id },
						})
					}
					className="cursor-pointer hover:opacity-80 transition"
				>
					<RadarListElement radarId={radar.radar_id} />
				</div>
			))}
		</div>
	);
}
