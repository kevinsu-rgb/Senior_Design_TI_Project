import { useGetRadarById } from "../../../hooks/useRadar";
import { getStatusTheme } from "../../../statusTheme";
import { useEffect, useState } from "react";

export default function StatusBox({ radarId }) {
    const radar = useGetRadarById(radarId);

    const [now, setNow] = useState(Date.now());

    useEffect(() => {
        const interval = setInterval(() => {
            setNow(Date.now());
        }, 1000);
        return () => clearInterval(interval);
    }, []);

    if (!radar) return null;

    function formatDuration(ms) {
        const totalSeconds = Math.floor(ms / 1000);
        const days = Math.floor(totalSeconds / 86400);
        const hours = Math.floor((totalSeconds % 86400) / 3600);
        const minutes = Math.floor((totalSeconds % 3600) / 60);
        const seconds = totalSeconds % 60;

        return `${days}d ${hours}h ${minutes}m ${seconds}s`;
    }

    const uptime =
        radar.is_connected && radar.connected_since
            ? formatDuration(now - new Date(radar.connected_since).getTime())
            : "Offline";

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
                    {radar.fault_latched && (
                        <span className="px-3 py-1 rounded-md bg-red-700 text-white font-semibold text-sm">
                            Fault latched
                        </span>
                    )}
                </div>
            </div>

            <div className="flex gap-4 mt-4">
                <div className="flex-1 border-2 border-gray-600 rounded-lg bg-bg3 p-6">
                    <p className="text-gray-400 text-lg ">Uptime</p>
                    <p className="text-5xl font-bold text-white">{uptime}</p>
                </div>

                <div className="flex-1 border-2 border-gray-600 rounded-lg bg-bg3 p-6">
                    <p className="text-gray-400 text-lg ">People detected</p>
                    <p className="text-5xl font-bold text-white">{radar.people_count}</p>
                </div>
            </div>
        </div>
    );
}
