#! /bin/bash
#********************************************************************
# 09.02.2012
# Alexander Kubicki
# I IA ATS 11
# Siemens AG
#
# Script for undo of renaming files (in clearcase) with same filenames,
# if the filesystem is not case-sensitive (e.g. Windows)
# --> run this script within the clearcase-view !
# 
# 1.parameter: list of filenames (with full path) to be changed
# 2.parameter: "undo" does a reverse file change 
#	(list of filenames has original filenames but clearcase has already changed filenames)
#
# 1.output-File: FileNameChanges.txt : previous and new (=changed) 
# 				       filenames 
#
#********************************************************************

tempdir="/tmp/ramdisk"
changedFiles="FileNameChanges.txt"
filelocation=`pwd`
tempdirsize="100M"
nameAddition="_2"
changedirection=0

echo ""
echo "-------------------------------------------------------------------------"
echo "Script for un-changing the filenames for a non-casesensitive filesystem. "
echo "-------------------------------------------------------------------------"

# reading in the clearcase-config_spec
echo `cleartool setcs -current`



if [ -n "$1" ]
then # with argument
	changelist=$1
	echo "The files are changed based on the list '$changelist'"
else # without argument
	echo "A list for changing the filenames must be provided as a parameter!"
	exit 1
fi

echo ""
echo "--> Script mut be run on a Clearcase-View !"
echo ""



if [ -e $tempdir ];then
	echo "		Temporary directory $tempdir exists."
else
	echo "		Temporary directory $tempdir will be created."
	mkdir $tempdir	

fi 

echo "		Ramdisk will be mounted as a temporary directory."
sudo mount -t tmpfs -o size=$tempdirsize tmpfs $tempdir
touch  $tempdir/$changedFiles

echo -e "previous filenames, actual filenames" >> $tempdir/$changedFiles
echo -e "---------------------------------------" >> $tempdir/$changedFiles
echo -e "" >> $tempdir/$changedFiles

for filename in $(cat $changelist)
do 
	

 
	path=`echo ${filename%/*}` # Path without filename 
	file=`echo ${filename##*/}` # $file: original filename in uppercase (without path)

	lowercase=`echo $file | (tr A-Z a-z)` # $lowercase: filename converted to lowercase
	filepre=`echo ${lowercase%.*}` # filename left of the dot
	filepost=`echo ${lowercase#*.}` # filename right of the dot
	# for "nameAddition" see head of file

	newfilename=$path/$filepre$nameAddition.$filepost 
					# $newfilename: new filename 

	if [ "$previouspath" != "$path" ];
	then
		if [ "$previouspath" != "" ];
		then
			echo `cleartool ci -c 'rechanges of filenames from a non-casesensitive-filesystem'  $previouspath/. `
		fi

		echo `cleartool co -nc $path/. `
	fi


	echo `cleartool mv -nc $newfilename $filename`

	
	# write changes to a file
	echo -e $newfilename , $filename >> $tempdir/$changedFiles 
	
	previouspath=$path


done

echo `cleartool ci -c 'rechanges of filenames from a non-casesensitive-filesystem'  $path/. `

cp $tempdir/$changedFiles $filelocation

echo "		Ramdisk will be unmounted"
sudo umount $tempdir

echo ""
echo "---------------------------------------------------------------------------------------------------"
echo "The changed filenames are listed up in '$filelocation/$changedFiles' ."
echo "---------------------------------------------------------------------------------------------------"
echo ""
