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
filePath=$(find $sourcePath -name $1.c |head -1)
echo "filepath: $filePath"
export partialPreprocFlags="--bdd -x CONFIG_ --include busybox/config.h -I $srcPath/include --featureModelDimacs busybox/featureModel.dimacs --recordTiming --parserstatistics --openFeat $path/openfeatures.txt --writePI --debugInterface --ifdeftoifstatistics"

## Reset output
echo "Starting process on file $1"
./jcpp.sh $filePath $flags

## Create id2i_optionstruct
./../Hercules/ifdeftoif.sh --featureConfig BusyBoxDefConfig.config
