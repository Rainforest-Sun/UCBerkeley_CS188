#!/bin/bash

while getopts "w:" opt; do
  case $opt in
    w)
      parameter=$OPTARG
      ;;
    \?)
      echo "无效的选项: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

if [ "$parameter" == "mini" ]; then
	python pacman.py -l tinyMaze -p SearchAgent
elif [ "$parameter" == "medium" ]; then
	python pacman.py -l mediumMaze -p SearchAgent
elif [ "$parameter" == "large" ]; then
	python pacman.py -l bigMaze -z .5 -p SearchAgent
else
	echo "未知的选项: -$parameter"
	exit 1
fi
