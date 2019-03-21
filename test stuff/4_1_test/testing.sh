#!/bin/bash
touch 4_1_test_log.txt
for i in {1..25} 
do
	echo $i
	python hist.py 4 1 $i >> 4_1_test_log.txt
done
