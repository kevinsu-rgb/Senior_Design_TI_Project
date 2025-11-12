export default function Navbar({ logo }) {
	return (
		<nav className="bg-bg2 px-8 py-4 border-b-2 border-primary">
			<div className="flex items-center gap-3">
				{logo && (
					<img src={logo} alt="Logo" className="w-8 h-8 object-contain" />
				)}
				<h1 className="text-white text-2xl font-normal tracking-wide">
					TEXAS INSTRUMENTS
				</h1>
			</div>
		</nav>
	);
}
