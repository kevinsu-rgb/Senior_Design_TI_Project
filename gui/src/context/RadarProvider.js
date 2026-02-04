import { useEffect, useState, createContext } from "react";
import { io } from "socket.io-client";

export const RadarContext = createContext([]);

export default function RadarProvider({ children }) {
  const [radarList, setRadarList] = useState([]);
  const [ws, setWs] = useState(null);
  const [isConnected, setIsConnected] = useState(false);

  useEffect(() => {
    const ws = io("http://127.0.0.1:5000");

    ws.on("connect", () => {
      console.log("socket connected");
      setIsConnected(true);
    });

    ws.on("disconnect", () => {
      console.log("socket disconnected");
      setIsConnected(false);
    });

    setWs(ws);

    return () => {
      ws.disconnect();
    };
  }, []);

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
  }, [ws]);

  useEffect(() => {
    if (!isConnected) return;

    ws.on("radar_status_update", (data) => {
      const updates = data.updates;

      setRadarList((prev) =>
        prev.map((radar) =>
          radar.radar_id === updates.radar_id ? { ...radar, ...updates } : radar
        )
      );
    });

    return () => {
      ws.off("radar_status_update");
    };
  }, [ws, isConnected, radarList]);

  return (
    <RadarContext.Provider value={{ radarList }}>
      {children}
    </RadarContext.Provider>
  );
}
