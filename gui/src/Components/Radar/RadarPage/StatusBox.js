import { useGetRadarById } from "../../../hooks/useRadar";
import { getStatusTheme } from "../../../statusTheme";

export default function StatusBox({ radarId }) {
    const radar = useGetRadarById(radarId);

    if (!radar) return null;

    const theme = getStatusTheme(radar.status);
    const statusLabel = theme.label;

    return (
        <div className="h-full overflow-hidden bg-bg2 rounded-lg p-8 ">
            <div className={`border-4 ${theme.cardBorder} ${theme.cardBg} rounded-lg p-6`}>
                <div className="flex items-center justify-between ">
                    <h2 className="text-5xl font-bold text-white">{radar.name}</h2>
                </div>

                <div className="flex items-center gap-2 ">
                    <p className="text-white text-lg">ID: {radar.radar_id}</p>
                </div>

                <div className="flex items-center gap-4">
                    <p className="text-white text-2xl">
                        Status:{" "}
                        <span className={`${theme.statusText} font-bold`}>
                            {statusLabel}
                        </span>
                    </p>
                </div>
            </div>

            <div className="flex gap-4 mt-4">
                <div className="flex-1 border-2 border-gray-600 rounded-lg bg-bg3 p-6">
                    <p className="text-gray-400 text-lg ">Uptime</p>
                    <p className="text-5xl font-bold text-white">{radar.uptime}</p>
                </div>

                <div className="flex-1 border-2 border-gray-600 rounded-lg bg-bg3 p-6">
                    <p className="text-gray-400 text-lg ">People detected</p>
                    <p className="text-5xl font-bold text-white">{radar.people_count}</p>
                </div>
            </div>
        </div>
    );
}
