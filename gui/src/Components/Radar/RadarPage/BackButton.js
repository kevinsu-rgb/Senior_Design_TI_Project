import { useNavigate } from "react-router-dom";

export default function BackButton() {
	const navigator = useNavigate();
	return (
		<div>
			<button className=" text-xl text-white" onClick={() => navigator("/")}>
				Back to dashboard
			</button>
		</div>
	);
}
