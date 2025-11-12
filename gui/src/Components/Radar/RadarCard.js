export default function RadarCard({ radar }) {
	const statusConfig = {
		active: {
			bg: "bg-neutral-700",
			border: "border-neutral-600",
			badge: "bg-green-600",
			text: "Standing",
		},
		fall: {
			bg: "bg-red-900/40",
			border: "border-red-600",
			badge: "bg-red-600",
			text: "Fall detected",
		},
	};

	const config = statusConfig[radar?.status] || statusConfig.active;

	return (
		<div
			className={`${config.bg} ${config.border} h-full overflow-hidden border-2 rounded-lg p-5 flex items-center justify-between`}
		>
			<div>
				<h3 className="text-2xl font-normal text-white">{radar?.name}</h3>
				<p className="text-neutral-400 text-sm mt-1">ID: {radar?.radar_id}</p>
			</div>
			<div className={`${config.badge} px-4 py-2 rounded-md`}>
				<span className="text-white font-medium">{config.text}</span>
			</div>
		</div>
	);
}
