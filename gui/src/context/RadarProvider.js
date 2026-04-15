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
          setRadarList((prev) => {
            const nextRadars = data?.radars ?? [];
            const radarIndexByIp = new Map();
            const merged = prev.slice();
            console.log("Previous radar list: ", merged);
            console.log("Next radar list: ", nextRadars);

            for (let i = 0; i < merged.length; i += 1) {
              radarIndexByIp.set(merged[i].radar_ip, i);
            }

            for (let i = 0; i < nextRadars.length; i += 1) {
              const nextRadar = nextRadars[i];
              const existingIndex = radarIndexByIp.get(nextRadar.radar_ip);

              if (existingIndex === undefined) {
                radarIndexByIp.set(nextRadar.radar_ip, merged.length);
                merged.push(nextRadar);
              }
            }
            
            console.log("Merged radar list: ", merged);
            return merged;
          });
        }
      } catch (err) {
        console.error("Error getting radar list: ", err);
      }
    }

    fetchRadarList();

    // Refresh radar list every 2 seconds to get new radars
    const intervalId = setInterval(fetchRadarList, 2000);

    return () => {
      clearInterval(intervalId);
    };
  }, []);

  useEffect(() => {
    if (!ws || !isConnected) return;

    const onRadarStatusUpdate = (data) => {
      const updates = data.updates;

      

      setRadarList((prev) =>
        prev.map((radar) => {
          console.log("Comparing radar_ip:", radar.radar_ip, "with update_ip:", updates.radar_ip);
          return radar.radar_ip === updates.radar_ip
            ? { ...radar, ...updates }
            : radar;
        })
      );
    };

    ws.on("radar_status_update", onRadarStatusUpdate);

    return () => {
      ws.off("radar_status_update", onRadarStatusUpdate);
    };
  }, [ws, isConnected]);

  const clearRadarFault = async (radarIp) => {
    try {
      const response = await fetch(`/api/radar/${radarIp}/clear-fault`, {
        method: "POST",
      });
      if (!response.ok) {
        throw new Error(`Failed to clear fault for radar ${radarIp}`);
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
