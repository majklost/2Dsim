PYTHONPATH=../../src:$PYTHONPATH
cfgname=$(basename $3 config.json)
resname="${cfgname}_${4}"
mkdir -p "output"
python3 -u the_script.py -m "$4" -o "output/${resname}" -cfg "$3" -n "test_$1" -s "$2"
