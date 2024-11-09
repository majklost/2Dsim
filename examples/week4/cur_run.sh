#!/bin/bash
# ./test.sh $TNUM vanilla_config.json
# ./test.sh $TNUM goal_bias_config.json
TNUM=10
# ./test.sh $TNUM subsampler_config.json non_convex
# ./test.sh $TNUM subsampler_config.json thick_stones
# ./test.sh $TNUM subsampler_config.json piped

# ./test.sh $TNUM vanilla_config.json non_convex
# ./test.sh $TNUM vanilla_config.json thick_stones
# ./test.sh $TNUM vanilla_config.json piped

# ./test.sh $TNUM goal_bias_config.json non_convex
# ./test.sh $TNUM goal_bias_config.json thick_stones
# ./test.sh $TNUM goal_bias_config.json piped

if [[ " $* " == *" --clear "* ]]; then
    echo "Clearing output directory"
    rm -rf output
fi


./test.sh $TNUM ./configs/subsampler_two_controlable_config.json standard_stones
