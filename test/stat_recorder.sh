#!/bin/bash

rm $2
echo "x = [" > $2

i="0"

while [ $i -lt 30 ]
do
rostest robot_localization $1 output_final_position:=true output_location:=$2
i=$[$i+1]
sleep 3
done

echo "]" >> $2
echo "mean(x)" >> $2
echo "sqrt(sum((4 * std(x)).^2))" >> $2

