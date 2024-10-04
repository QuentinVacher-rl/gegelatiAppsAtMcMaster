#!/bin/bash

list_params=$(seq 0 1 | sed 's/^/..\/params_/' | sed 's/$/.json/')
seeds=$(seq 0 4)

for params in ${list_params[@]}; do
    echo "Params used $params"

    # Extraire l'index du fichier JSON, par exemple 'params_1.json' -> '1'
    index_param=$(basename "$params" | sed 's/[^0-9]*\([0-9]*\)\.json/\1/')

    for i in ${seeds[*]}; do
        ./Release/pendulum -s $i -p $params 1>dirtyLog 2>/dev/null
        
        # Print last line of log with only the index of params
        tail -n 1 "out.$i.p$index_param.std"
    done
done

echo Done


