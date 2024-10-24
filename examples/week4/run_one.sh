PYTHONPATH=../../src:$PYTHONPATH
python3 -u the_script.py -m standard_stones -o ./testrun -cfg ./vanilla_config.json -n "test_$1" -s "$2"