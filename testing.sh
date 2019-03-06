#!/bin/bash
mkdir -p tests
for i in {1..100}
do
	touch tests/training$i.txt
	python a.py > tests/training$i.txt
done
