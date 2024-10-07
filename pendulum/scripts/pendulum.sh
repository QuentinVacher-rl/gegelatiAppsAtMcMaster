#!/bin/bash

list_params=$(seq 0 0 | sed 's/^/params\/params_/' | sed 's/$/.json/')
seeds=$(seq 0 4)
veclocities=$(seq 0 0)
continuous=$(seq 1 1)

for params in ${list_params[@]}; do
    echo "Params used $params"

    # Extraire l'index du fichier JSON, par exemple 'params_1.json' -> '1'
    index_param=$(basename "$params" | sed 's/[^0-9]*\([0-9]*\)\.json/\1/')

    for v in ${veclocities[*]}; do
    echo "Velocity used $v"
        for c in ${continuous[*]}; do
        echo "Continuous used $c"
            for i in ${seeds[*]}; do
                echo "Seed used $i"
                ./bin/Release/pendulum -s $i -p $params -v $v -c $c 1>dirtyLog 2>/dev/null
                
                # Print last line of log with only the index of params
                tail -n 1 "out.$i.p$index_param.v$v.c$c.std"
            done
        done
    done
done

echo Done


