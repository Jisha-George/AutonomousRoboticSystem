#!/bin/bash
touch 7_2_train_hist.txt
for i in {1..25} 
do
	echo $i
	python hist.py 7 2 $i >> 7_2_train_hist.txt
done
