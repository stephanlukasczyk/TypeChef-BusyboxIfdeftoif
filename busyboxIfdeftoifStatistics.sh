#!/bin/bash -e
#!/bin/bash -vxe

filesToProcess() {
  local listFile=busybox/busybox_files
  cat $listFile
  #awk -F: '$1 ~ /.c$/ {print gensub(/\.c$/, "", "", $1)}' < linux_2.6.33.3_pcs.txt
}

path=$(cd "$(dirname "$0")"; pwd)
extension="_ifdeftoif.c"
flags="-U HAVE_LIBDMALLOC -DCONFIG_FIND -U CONFIG_FEATURE_WGET_LONG_OPTIONS -U ENABLE_NC_110_COMPAT -U CONFIG_EXTRA_COMPAT -D_GNU_SOURCE"
srcPath="busybox-1.18.5"
export partialPreprocFlags="--bdd -x CONFIG_ --include busybox/config.h -I $srcPath/include --featureModelDimacs $path/BB_fm.dimacs --featureConfiguration $path/BusyBoxDefConfig.config --recordTiming --parserstatistics --debugInterface --ifdeftoifstatistics"

## Reset output
filesToProcess|while read i; do
  if [ ! -f $srcPath/$i$extension ]; then
# this script run transforms the input C file using ifdeftoif transformations
    ./jcpp.sh $srcPath/$i.c $flags

  else
    echo "Skipping $srcPath/$i.c"
  fi
done

