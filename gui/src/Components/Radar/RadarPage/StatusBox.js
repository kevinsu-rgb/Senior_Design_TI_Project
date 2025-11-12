export default function StatusBox({ radar }) {
	console.log(radar);
	return (
		<div className="h-full overflow-hidden bg-bg2 rounded-lg p-8 ">
			<div className="border-4 border-red-600 bg-bg3 rounded-lg  p-6">
				<div className="flex items-center justify-between ">
					<h2 className="text-5xl font-bold text-white">{radar.name}</h2>
				</div>

				<div className="flex items-center gap-2 ">
					<p className="text-white text-lg">ID: {radar.id}</p>
				</div>

				<div className="flex items-center gap-4">
					<p className="text-white text-2xl">
						Status:{" "}
						<span className="text-red-500 font-bold">{radar.status}</span>
					</p>
				</div>
			</div>

			<div className="flex gap-4 mt-4">
				<div className="flex-1 border-2 border-gray-600 rounded-lg bg-bg3 p-6">
					<p className="text-gray-400 text-lg ">Uptime</p>
					<p className="text-5xl font-bold text-white">{radar.uptime}</p>
				</div>

				{/* People detected box */}
				<div className="flex-1 border-2 border-gray-600 rounded-lg bg-bg3 p-6">
					<p className="text-gray-400 text-lg ">People detected</p>
					<p className="text-5xl font-bold text-white">1</p>
				</div>
			</div>
		</div>
	);
}
