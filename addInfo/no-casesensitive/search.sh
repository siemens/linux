#! /bin/bash
#********************************************************************
# 09.02.2012
# Alexander Kubicki
# I IA ATS 11
# Siemens AG
#
# Script for searching of files with same filenames,
# if the filesystem is not case sensitive (e.g. Windows)
# 
# 1.parameter: starter-directory for the search (absolute pathname!)
# 2.parameter: search-depth (0=search only the starter directory)
#
# 1.output-File: scanningdirs.txt : all scanned directories
# 2.output-File: doubleFileNames.txt : filenames, which might cause
#		 errors on a not case-sensitive filesystem (e.g. Windows)
#
#********************************************************************

tempdir="/tmp/ramdisk"
tempfile="tempfile.txt"
doubleFiles="doubleFileNames.txt"
scanningdirs="scanningdirs.txt"
filelocation=`pwd`
searchdepth=2
tempdirsize="100M"

echo ""
echo "--------------------------------------------------------------------------------------------------------------------"
echo "Script for compatibility check of filenames with a non-casesensitive filesystem."
echo "--------------------------------------------------------------------------------------------------------------------"


if [ -n "$1" ]
then # with argument
	startdir=$1
	echo "Starting the search in directory '$startdir'"
else # without argument
	echo "Please provide a start-directory as a parameter!"
	exit 1
fi

if [ -n "$2" ]
then # with argument
	searchdepth=$2
else # without argument
	searchdepth=0
	echo "2. parameter missing: The search is done in the same directory"
fi



if [ -e $tempdir ];then
	echo "		Temporary directory $tempdir exists."
else
	echo "		Temporary directory $tempdir will be created."
	mkdir $tempdir	

fi 

echo "		Ramdisk will be mounted as a temporary directory"
sudo mount -t tmpfs -o size=$tempdirsize tmpfs $tempdir


touch  $tempdir/$tempfile
touch  $tempdir/$doubleFiles
touch  $tempdir/$scanningdirs


################################################
echo "Please wait! Directories up to the $searchdepth. subdirectory (including) will be searched" 
find $startdir -mindepth 0 -maxdepth $searchdepth -xdev -type d -printf %p\\n > $tempdir/$tempfile
################################################

temppath=`pwd`

for loop in $(cat $tempdir/$tempfile)
do 
	echo -e $loop >> $tempdir/$scanningdirs

done

rm $tempdir/$tempfile
touch $tempdir/$tempfile

#### get sub-directories ######################
for searchdir in $(cat $tempdir/$scanningdirs)
do

	cd $searchdir
	#echo -e "bearbeite: $searchdir"

	for filename in *
	do
		
		echo -e $filename >> $tempdir/$tempfile

		
	done

	for filename in $(cat $tempdir/$tempfile)
	do
		lowercase=`echo $filename | (tr A-Z a-z)` #change to lowercase

		if [ $filename != $lowercase ] && [ -e  $lowercase ];
		then
			echo -e "Critical filename found!->"
			echo -e `pwd`/$filename
			echo -e `pwd`/$filename >> $tempdir/$doubleFiles
		fi
	done
rm $tempdir/$tempfile
touch $tempdir/$tempfile

done


cp $tempdir/$doubleFiles $filelocation
cp $tempdir/$scanningdirs $filelocation



echo "		Ramdisk will be unmounted"
sudo umount $tempdir

echo ""
echo "--------------------------------------------------------------------------------------------------------------------"
echo "The critical filenames are listed up in '$filelocation/$doubleFiles'."
echo "--------------------------------------------------------------------------------------------------------------------"
echo ""