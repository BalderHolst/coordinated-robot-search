COLORS = [
    "#5e81b5",
    "#e19c24",
    "#8fb032",
    "#eb6235",
    "#8778b3",
    "#c56e1a",
    "#5d9ec7",
    "#ffbf00",
    "#a5609d",
    "#929600",
    "#e95536",
    "#6685d9",
    "#f89f13",
    "#bc5b80",
    "#47b66d",
]

def get_color(index: int) -> str:
    return COLORS[index % len(COLORS)]
