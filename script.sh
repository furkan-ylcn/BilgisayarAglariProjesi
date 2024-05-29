#!/bin/bash

script="scratch/experiments"

nDevices=200     # Change from 50 to 200
MobileNodeProbability=1
MinSpeed=5       # Change from 5 to 20
MaxSpeed=20       # Change from 5 to 20
folder="scratch/experiments/Test_${MaxSpeed}_${nDevices}/run"

echo -n "Running experiments: "

mkdir -p $folder
for r in `seq 1 30`;
do
  echo -n " $r"
  ./ns3 run "$script --RngRun=$r --nDevices=$nDevices --MobileNodeProbability=$MobileNodeProbability --MinSpeed=$MinSpeed --MaxSpeed=$MaxSpeed" >> "$folder/log.txt" 2>&1
done

echo " END"

