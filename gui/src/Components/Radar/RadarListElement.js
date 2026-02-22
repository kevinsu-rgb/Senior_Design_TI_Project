import { useGetRadarById } from "../../hooks/useRadar";
import { getStatusTheme } from "../../statusTheme";

export default function RadarListElement({ radarId }) {
    const radar = useGetRadarById(radarId);

    const config = getStatusTheme(radar?.status);

    return (
        <div
            className={`${config.cardBg} ${config.cardBorder} h-full overflow-hidden border-2 rounded-lg p-5 flex items-center justify-between`}
        >
            <div>
                <h3 className="text-2xl font-normal text-white">{radar?.name}</h3>
                <p className="text-neutral-400 text-sm mt-1">ID: {radar?.radar_id}</p>
            </div>
            <div className={`${config.badgeBg} px-4 py-2 rounded-md`}>
                <span className="text-white font-medium">{config.label}</span>
            </div>
        </div>
    );
}
