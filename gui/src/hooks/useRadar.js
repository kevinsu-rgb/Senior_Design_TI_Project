import { useContext } from "react";
import { RadarContext } from "../context/RadarProvider";

export const useRadar = () => {
	return useContext(RadarContext);
};
export const useGetRadarById = (radarId) => {
	const { radarList } = useRadar();
	return radarList.find((r) => r.radar_id === radarId);
};
