import { useNavigate, useLocation } from "react-router-dom";
import BackButton from "./BackButton";
import ActivityBox from "./ActivityBox";
import StatusBox from "./StatusBox";
import { useGetRadarById, useRadar } from "../../../hooks/useRadar";

export default function RadarPage() {
	const navigate = useNavigate();
	const location = useLocation();
	const radarId = location.state?.radarId;
	const radar = useGetRadarById(radarId);
	const { clearRadarFault } = useRadar();

	return (
		<div className="h-full flex flex-col overflow-hidden">
			<div className="flex-none p-4 flex items-center justify-between">
				<BackButton onBack={() => navigate(-1)} />
				<button
					type="button"
					onClick={() => clearRadarFault?.(radarId)}
					disabled={!radar?.fault_latched}
					className={`px-4 py-2 rounded-md text-white font-medium transition ${
						radar?.fault_latched
							? "bg-red-600 hover:bg-red-500"
							: "bg-neutral-700 cursor-not-allowed"
					}`}
				>
					Clear Fault
				</button>
			</div>

			<div className="flex-1 p-4 flex flex-col gap-4 min-h-0">
				<div className="flex-none">
					<StatusBox radarId={radarId} />
				</div>

				<div className="flex-1 overflow-auto min-h-0">
					<ActivityBox radarId={radarId} />
				</div>
			</div>
		</div>
	);
}
