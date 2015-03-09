#find busybox-1.18.5/ -name "*.dbg*" |while read i; do echo $i; rm $i; done 
#find busybox-1.18.5/ -name "*.err" |while read i; do echo $i; rm $i; done 
#find busybox-1.18.5/ -name "*.interface" |while read i; do echo $i; rm $i; done 
find busybox-1.18.5/ -name "*.pi" |while read i; do echo $i; rm $i; done 
find busybox-1.18.5/ -name "*.pi_dbg" |while read i; do echo $i; rm $i; done
#find busybox-1.18.5/ -name "*.time" |while read i; do echo $i; rm $i; done 
find busybox-1.18.5/ -name "*.c.xml" |while read i; do echo $i; rm $i; done
find busybox-1.18.5/ -name "*_ifdeftoif.c" |while read i; do echo $i; rm $i; done
find . -name "console_output.txt" |while read i; do echo$i; rm $i; done
find ../ifdeftoif/ -name "type_errors.txt" |while read i; do echo $i; rm $i; done
find ../ifdeftoif/ -name "top_level_statistics.csv" |while read i; do echo $i; rm $i; done
find ../ifdeftoif/ -name "statistics.csv" |while read i; do echo $i; rm $i; done
find ../ifdeftoif/ -name "featureMap.csv" |while read i; do echo $i; rm $i; done
find ../ifdeftoif/ -name "skipped_duplications.txt" |while read i; do echo $i; rm $i; done
