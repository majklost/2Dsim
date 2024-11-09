#!/bin/bash
# Number of seeds to generate
NUMTESTS=$1
CONFIGNAME=$2
MAPNAME=$3

echo "Generating $NUMTESTS random seeds..."
seeds=()
tnums=$(seq 1 $NUMTESTS)


for i in $(seq 1 $NUMTESTS); do
    # Generate a random seed (using $RANDOM in this example)
    seed=$(($(date +%s%N) + RANDOM + $(od -vAn -N4 -tu4 < /dev/urandom | tr -d ' ')))

    # Store the seed in the array
    seeds+=("$seed")
done
# shellcheck disable=SC2145
echo "seeds are ${seeds[*]}"
echo "tnums are ${tnums[*]}"
echo "configname is $CONFIGNAME"

echo "Running tests..."
# Run the tests
parallel -j 8 --lb ./run_one.sh ::: "${tnums[@]}" :::+ "${seeds[@]}" ::: "$CONFIGNAME" ::: "$MAPNAME"
#parallel ./dummy_run.sh ::: "${tnums[@]}" :::+ "${seeds[@]}"
