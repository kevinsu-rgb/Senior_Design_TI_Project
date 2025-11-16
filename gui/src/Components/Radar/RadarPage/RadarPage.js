import { useNavigate, useLocation } from "react-router-dom";
import BackButton from "./BackButton";
import ActivityBox from "./ActivityBox";
import StatusBox from "./StatusBox";
import { useContext } from "react";

export default function RadarPage() {
	const navigate = useNavigate();
	const location = useLocation();
	const radarId = location.state?.radarId;

	return (
		<div className="h-full flex flex-col overflow-hidden">
			<div className="flex-none p-4">
				<BackButton onBack={() => navigate(-1)} />
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
