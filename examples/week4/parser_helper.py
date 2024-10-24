import argparse

from maps import str2map
parser = argparse.ArgumentParser(
    prog="motionPlanning",
    description="Run motionPlanning for given map and output results to file"
)
parser.add_argument(
    "-m", "--map",
    help="Name of the map",
    choices=str2map.keys(),
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

parser.add_argument(
    "-n" , "--name",
    help="Name of the run",
    required=True
)
parser.add_argument(
    "-s", "--seed",
    help="Planning seed for the run",
    required=True
)