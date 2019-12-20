#!/bin/bash

cd ../build

for i in {0..5}
do
  for j in {0..6}
  do
     cd ../build
    ./2D_feature_tracking --roi 1 --detector $j --descriptor $i --matcher 0 --matcher-selector 1 --visualize 0
  done
done

