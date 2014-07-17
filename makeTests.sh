#!/bin/bash
START_TIME=$SECONDS

filesToProcess() {
  local listFile=busybox/busybox_files
  cat $listFile
  #awk -F: '$1 ~ /.c$/ {print gensub(/\.c$/, "", "", $1)}' < linux_2.6.33.3_pcs.txt
}

path=$(cd "$(dirname "$0")"; pwd)
extension="_ifdeftoif.c"
flags="-U HAVE_LIBDMALLOC -DCONFIG_FIND -U CONFIG_FEATURE_WGET_LONG_OPTIONS -U ENABLE_NC_110_COMPAT -U CONFIG_EXTRA_COMPAT -D_GNU_SOURCE"
srcPath="busybox-1.18.5"
testOutputPath="test_results"
export partialPreprocFlags="--bdd -x CONFIG_ --include busybox/config.h -I $srcPath/include --featureModelDimacs busybox/featureModel.dimacs --recordTiming --parserstatistics --debugInterface --ifdeftoifstatistics"

if [ ! -d $testOutputPath ]; then
	mkdir $testOutputPath
fi

## Reset output
filesToProcess|while read i; do
  baseFileName="${i##*/}"
  if [ ! -f $testOutputPath/$baseFileName.test ]; then
# this script run transforms the input C file using ifdeftoif transformations
    ./jcpp.sh $srcPath/$i.c $flags
	
# Create id2i_optionstruct
	./../Hercules/ifdeftoif.sh --featureConfig BusyBoxDefConfig.config
# Swap original and transformed file
	mv "$srcPath/$i.c" "$srcPath/${i}.orig"
	mv "$srcPath/${i}_ifdeftoif.c" "$srcPath/$i.c"
	cd $srcPath
# Start making busybox
	echo "-=Make=- $i.c"
	make > ../$testOutputPath/$baseFileName.make 2>&1
	cd testsuite
# Start testing busybox
	echo "-=Test=- $i.c"
	./runtest > ../../$testOutputPath/$baseFileName.test 2>&1
	cd $path
# Swap files back to original state
	mv "$srcPath/$i.c" "$srcPath/${i}_ifdeftoif.c"
	mv "$srcPath/${i}.orig" "$srcPath/$i.c"
	# Reset busybox-1.18.5 folder
	git checkout -- $srcPath
	echo "-=Done=- $i.c"
  else
    echo "Skipping $srcPath/$i.c"
  fi
done

##Remove all but last line of .test files
#cd $path/$testOutputPath
#ls *.test | xargs sed -i '$!d'

ELAPSED_TIME=$(($SECONDS - $START_TIME))
echo "$(($ELAPSED_TIME/60)) min $(($ELAPSED_TIME%60)) sec"