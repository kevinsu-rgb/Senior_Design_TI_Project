const STATUS_THEME = {
  standing: {
    label: "Standing",
    cardBg: "bg-emerald-900/30",
    cardBorder: "border-emerald-600",
    badgeBg: "bg-emerald-600",
    statusText: "text-emerald-400",
  },
  sitting: {
    label: "Sitting",
    cardBg: "bg-blue-900/30",
    cardBorder: "border-blue-600",
    badgeBg: "bg-blue-600",
    statusText: "text-blue-400",
  },
  lying: {
    label: "Lying",
    cardBg: "bg-purple-900/30",
    cardBorder: "border-purple-600",
    badgeBg: "bg-purple-600",
    statusText: "text-purple-400",
  },
  walking: {
    label: "Walking",
    cardBg: "bg-amber-900/30",
    cardBorder: "border-amber-500",
    badgeBg: "bg-amber-500",
    statusText: "text-amber-400",
  },
  falling: {
    label: "Fall detected",
    cardBg: "bg-red-900/40",
    cardBorder: "border-red-600",
    badgeBg: "bg-red-600",
    statusText: "text-red-500",
  },
  unknown: {
    label: "Unknown",
    cardBg: "bg-neutral-700",
    cardBorder: "border-neutral-600",
    badgeBg: "bg-neutral-600",
    statusText: "text-yellow-400",
  },
};

export function getStatusTheme(rawStatus) {
  const key = (rawStatus || "").toLowerCase();
  return STATUS_THEME[key] || STATUS_THEME.unknown;
}
