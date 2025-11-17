import logo from "./assets/logo.png";
import "./App.css";
import Navbar from "./components/Navbar";
import RadarList from "./components/Radar/RadarList";
import RadarPage from "./components/Radar/RadarPage/RadarPage";
import RadarProvider from "./context/RadarProvider";
import { Routes, Route } from "react-router-dom";

function App() {
	return (
		<RadarProvider>
			<div className="App bg-bg1 h-screen overflow-hidden flex flex-col">
				<Navbar logo={logo} />
				<header className="App-header flex-1 overflow-hidden">
					<Routes>
						<Route path="/" element={<RadarList />} />
						<Route path="/radar/:radarId" element={<RadarPage />} />
					</Routes>
				</header>
			</div>
		</RadarProvider>
	);
}

export default App;
