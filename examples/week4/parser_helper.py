import argparse
parser = argparse.ArgumentParser(
    prog="motionplanning",
    description="Run motionplanning for given map and output results to file"
)
parser.add_argument(
    "-m", "--map",
    help="Path to map file",
    required=True
)
parser.add_argument(
    "-o", "--output",
    help="Path to output folder",
    required=True
)
parser.add_argument(
    "-cfg", "--config",
    help="Path to config file",
    required=True
)
