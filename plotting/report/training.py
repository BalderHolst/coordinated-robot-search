import botplot as bp

FILES = [
    {
        "title": "Small Agent",
        "path": bp.repo_path("plotting", "data", "small-search-agent-training.ipc"),
    },
]

def main():
    for file in FILES:
        bp.plot_training(file["path"], file["title"])

if __name__ == "__main__":
    main()
