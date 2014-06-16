#!/bin/bash -e
#!/bin/bash -vxe

filesToProcess() {
  local listFile=busybox/busybox_files
  cat $listFile
}

path=$(cd "$(dirname "$0")"; pwd)
srcPath="busybox-1.18.5"

## Remove the MAIN_EXTERNALLY_VISIBLE macro from sources files
filesToProcess|while read i; do
  sed -i 's/ MAIN_EXTERNALLY_VISIBLE;/;/g' $srcPath/$i.c
  sed -i 's/ EXTERNALLY_VISIBLE;/;/g' $srcPath/$i.c
done

## Remove the MAIN_EXTERNALLY_VISIBLE macro from .h files
find busybox-1.18.5/ -type f -name "*.h" | xargs sed -i 's/ MAIN_EXTERNALLY_VISIBLE;/;/g'
find busybox-1.18.5/ -type f -name "*.h" | xargs sed -i 's/ EXTERNALLY_VISIBLE;/;/g'
