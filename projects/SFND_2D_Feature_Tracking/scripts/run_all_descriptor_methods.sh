#!/bin/bash

cd ../build

for i in {0..6}
do
   cd ../build
   ./2D_feature_tracking --roi 1 --detector $i --visualize 0
done

