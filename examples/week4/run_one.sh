PYTHONPATH=../../src:$PYTHONPATH
cfgname=$(basename $3 config.json)
python3 -u the_script.py -m empty -o "$cfgname" -cfg "$3" -n "test_$1" -s "$2"
