-- Binaries build for BE and LE.
-- Same kernel will work with SPI and NAND flash.
-- Config files are :
	mv88f4122_le_defconfig
	mv88f4122_be_defconfig
-- See mtd-utils section in LspReadme.txt how to format and use flashes.
-- Added simple test module to PPdrv.
	To use the module :
		Compile it with comm.sh script
		Run 'insmod PPdrv.ko'
		Run commands like 'echo '2040000' > /proc/ppdrv/read' and 'echo '2040000 69220511' > /proc/ppdrv/write'
-- This release not passed full QA, 
	tested BE/LE, SPI and NAND,
	PP register access, traffic and interrupts 
	on DB-24GE and DB-24FE boards.