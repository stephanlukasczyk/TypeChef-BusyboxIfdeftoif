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
done
