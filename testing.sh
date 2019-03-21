#!/bin/bash
touch 10_1.5_train_hist.txt
for i in {1..25} 
do
	echo $i
	python hist.py 10 1.5 $i >> 10_1.5_train_hist.txt
done
