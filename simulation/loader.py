import argparse
from scripts.main import main

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Isaac Sim scene launcher")
    parser.add_argument(
        "--config",
        default="simulation/config/default.yaml",
        type=str,
        help="Path to the YAML configuration file for the simulation"
    )
    args = parser.parse_args()

    # Pass the path into your main function
    main(args.config)