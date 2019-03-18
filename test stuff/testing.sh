#!/bin/bash
touch testlog.txt
for i in {1..100} 
do
	python a.py 10 1.5 $i >> testlog.txt
done
