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
    if (!ws || !isConnected) return;

    const onRadarStatusUpdate = (data) => {
      const updates = data.updates;

      setRadarList((prev) =>
        prev.map((radar) =>
          radar.radar_id === updates.radar_id ? { ...radar, ...updates } : radar
        )
      );
    };

    ws.on("radar_status_update", onRadarStatusUpdate);

    return () => {
      ws.off("radar_status_update", onRadarStatusUpdate);
    };
  }, [ws, isConnected]);

  const clearRadarFault = async (radarId) => {
    try {
      const response = await fetch(`/api/radar/${radarId}/clear-fault`, {
        method: "POST",
      });
      if (!response.ok) {
        throw new Error(`Failed to clear fault for radar ${radarId}`);
      }
    } catch (err) {
      console.error("Error clearing radar fault: ", err);
    }
  };

  return (
    <RadarContext.Provider value={{ radarList, clearRadarFault }}>
      {children}
    </RadarContext.Provider>
  );
}
