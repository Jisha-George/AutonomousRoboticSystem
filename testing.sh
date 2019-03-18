#!/bin/bash
touch 10_1.5_test_log.txt
for i in {1..100} 
do
	echo $i
	python a.py 10 1.5 $i >> 10_1.5_test_log.txt
done
