import logo from "./assets/logo.png";
import "./App.css";
import Navbar from "./Components/Navbar";
import RadarList from "./Components/Radar/RadarList";
import RadarPage from "./Components/Radar/RadarPage/RadarPage";
import { Routes, Route } from "react-router-dom";

function App() {
	return (
		<div className="App bg-bg1 h-screen overflow-hidden flex flex-col">
			<Navbar logo={logo} />
			<header className="App-header flex-1 overflow-hidden">
				<Routes>
					<Route path="/" element={<RadarList />} />
					<Route path="/radar/:radarId" element={<RadarPage />} />
				</Routes>
			</header>
		</div>
	);
}

export default App;
