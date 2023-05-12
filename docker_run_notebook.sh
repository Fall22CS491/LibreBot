#! /bin/bash
if [ "$#" != "1" ]; then 
	echo "Please supply two arguments: a relative path to your preferred notebook directory"
	exit 1
else
	echo -e "\033[32m" Using $(pwd)$1 as your notebook root directory. "\033[0m"	
	docker pull russtedrake/manipulation:cfccef8
#	docker pull robotlocomotion/drake:focal-1.9.0

	docker run -it -p 8080:8080 -p 7000-7010:7000-7010 --rm -v "$(pwd)/$1"":"/psets \
		russtedrake/manipulation:cfccef8 /bin/bash -c "cd /psets && jupyter notebook --ip 0.0.0.0 --port 8080 --allow-root --no-browser"

	# apt-get -q update && apt-get -q install -y --no-install-recommends git nginx-light xvfb && apt-get -q clean 
fi
