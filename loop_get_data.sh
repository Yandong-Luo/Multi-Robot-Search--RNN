#!/bin/bash
num=100
while(($num > 0));do
	processExist=ps | grep  "argos3" | grep -v "grep"
	if [ ["$processExist" != ""] ];then
		echo $num
		let num--
		argos3 -c experiments/PF_RNN-trials.argos
	fi
done
