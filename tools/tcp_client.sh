#!/bin/bash

search_cmd="llmnr-query --type A --interface enp0s31f6 iot_server.local"

connect_prog="netcat"

connect_port="10000"

search_function() 
{
   echo "searching..."
   $search_cmd
}

connect_function() 
{
   read -p "Enter IP address : " ipaddress
   echo "connecting..."
   connect_cmd="$connect_prog $ipaddress $connect_port"
   $connect_cmd
}

error_function() 
{
   echo "wrong command!"
}

clear
echo "Dummy TCP client tool..."
while [ 1 ]
do
   read -p "Enter command : " cmd
   if [ $cmd = s ]
           then search_function
   elif [ $cmd = c ]
           then connect_function
           else error_function
   fi
done

