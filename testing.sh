#!/bin/bash
touch 7_2_test_log.txt
for i in {1..100} 
do
	echo $i
	python a.py 7 2 $i >> 7_2_test_log.txt
done
