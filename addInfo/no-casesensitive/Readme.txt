Changing filenames and filecontents when using the Linux-Kernel on a non-casesensitive filesystem
=================================================================================================
10.02.2012
Alexander Kubicki
ATS11


Contents:
--------
1. Definition of critical files
2. Searching the critical files
3. Renaming the critical files
4. Undoing the file-renaming
5. Filechanges for kernel-compilation






1. Definition of critical files
-------------------------------
Unix uses casesensitive filesystems.
Windows filesystems preserve case, but are not case sensitive:
Filenames with same characters but different cases are critical files 
for conversion to a windows-filesystem.



2. Searching the critical files
-------------------------------
The 'search'-script is a bash-shellscript in Linux:

Program start:
- run on a local machine because of the temporary files
- 1.parameter: starter-directory for the search (absolute pathname!)
- 2.parameter: search-depth (0=search only the starter directory)
-> output-files are written to the actual directory

First all directories from the given start-directory to the given search depth are listed up.
This becomes the 1.output-File.

Second all directories listed in the fileare processed at looking for critical files:
	Convert the filename to lowercase and test, if this filename does already exist on the (unix-)filesystem.
	At existance the filename ist critical and listed up in the 2.output-File.

For performance reasons, the fileoperations are done on a temporary ramdisk (tmpfs).

Following parameters can be changed in the script: 

tempdir="/tmp/ramdisk"			-> temporary filesystem
tempfile="tempfile.txt"			-> temporary file
doubleFiles="doubleFileNames.txt"	-> name of 2.output-file (=critical-filenames)
scanningdirs="scanningdirs.txt"		-> name of 1.output-file (=listing of all processed (sub-)directories)
tempdirsize="100M"			-> amount of temporary ramdisk

The linux kenel has 10 subdirectories to scan.
The scanning lasts about 3 minutes



3. Renaming the critical files
------------------------------
The 'change'-script is a bash-shellscript in Linux.

Program start:
- run on a local machine because of the temporary files
- 1.parameter: 1.output-file of 'search'-script
- The script operates with clearcase commands and thus runs within an already set up clearcase-view.
-> output-files are written to the actual directory

All critical files are processed from a given list of filenames  (1.parameter).
This 1.parameter ist the 2.outputfile of the searchscript.
	At renaming the parent directory of the file is checkedout, the file is moved and the parent directory is checked in again.
	The previous and renamed filenames with their absolute paths are listed up in the 1.output-File.

Following parameters can be changed in the script: 

tempdir="/tmp/ramdisk"			-> temporary filesystem
changedFiles="FileNameChanges.txt"	-> name of 1.output-file (=listing of previous and actual filenames)
tempdirsize="100M"			-> amount of temporary ramdisk
nameAddition="_2"			-> addition to the filename 



4. Undoing the file-renaming
----------------------------
The 'unchange'-script is a bash-shellscript in Linux.

Program start:
- run on a local machine because of the temporary files
- 1.parameter: 1.output-file of 'search'-script
- The script operates with clearcase commands and thus runs within an already set up clearcase-view.
-> output-files are written to the actual directory

This script does the reverse operations of the 'change'-script(=undoing the filerenaming).
The 1.parameter is also the 2.outputfile of the 'search'-script as at the 'change'-script.




5. Filechanges for kernel-compilation
-------------------------------------
After file-renaming kernelconfig was done with:
make -f MakefileAudis linux_menuconfig_x686_embedded_ctrl_hr

Kernelcompilation was done with:
make -f MakefileAudis linux_build_x686_embedded_ctrl_hr

All renamed fieles are listed in 'FileNameChanges.txt'
All files with modified content are listed in 'ChangeFilecontent.txt'


