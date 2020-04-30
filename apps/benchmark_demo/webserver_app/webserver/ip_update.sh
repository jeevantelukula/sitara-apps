#!/bin/bash

sed -i -E "s#([0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3})#$(ifconfig eth0 | grep 'inet addr' | cut -d ':' -f 2 | cut -d ' ' -f 1)#" /home/ti/ti-processor-sdk-am64xx/apps/benchmark_demo/webserver_app/app/index.html
sed -i -E "s#([0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3})#$(ifconfig eth0 | grep 'inet addr' | cut -d ':' -f 2 | cut -d ' ' -f 1)#" /home/ti/ti-processor-sdk-am64xx/apps/benchmark_demo/webserver_app/app/main.js
