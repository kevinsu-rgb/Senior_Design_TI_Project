import { useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import RadarCard from "./RadarCard";

export default function RadarList() {
	const [radarList, setRadarList] = useState(null);
	const navigate = useNavigate();

	useEffect(() => {
		async function fetchRadarList() {
			try {
				const response = await fetch(`/api/radar/list`);
				if (response.ok) {
					const data = await response.json();
					setRadarList(data?.radars);
				}
			} catch (err) {
				console.error("Error getting radar list: ", err);
			}
		}
		fetchRadarList();
	}, []);

	return (
		<div className="max-w-5xl mx-auto p-8 space-y-4 h-full">
			{radarList?.map((radar) => (
				<div
					key={radar.radar_id}
					onClick={() =>
						navigate(`/radar/${radar.radar_id}`, { state: { radar } })
					}
					className="cursor-pointer hover:opacity-80 transition"
				>
					<RadarCard radar={radar} />
				</div>
			))}
		</div>
	);
}
