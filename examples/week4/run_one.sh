PYTHONPATH=../../src:$PYTHONPATH
python3 -u the_script.py -m empty -o ./testrun -cfg "$3" -n "test_$1" -s "$2"