#!/bin/bash -e

# parse commandline options
while [ ! -z "$1" ] ; do
        case $1 in
            --clean)
                echo "Clean module sources"
                make clean
                rm -f ${DTSDIR}/${MODNAME}.dtbo
                ;;
            --compile)
                echo "Build module"
                make
                ;;
            --dtbo)
                echo "Compile dtbo"
                ${KERNELDIR}/scripts/dtc/dtc -I dts -O dtb ${DTC_FLAGS} \
                -o ${DTSDIR}/${MODNAME}.dtbo \
                ${DTSDIR}/${MODNAME}.dtsi
                ;;
            --copymod)
                echo "Copy module to board"
                scp ${MODNAME}.ko ${ADDR_BOARD}:${MODULEDIR}
                ;;
            --copydtbo)
                echo "Copy overlay to board"
                scp ${DTSDIR}/${MODNAME}.dtbo \
                ${ADDR_BOARD}:${DTBDIR}/${MODNAME}.dtbo
                ;;
            --copysshid)
                echo "Copy ssh id to board"
                ssh-copy-id -i ~/.ssh/id_rsa.pub ${ADDR_BOARD}
                ;;
            --reboot)
                echo "The board reboot"
                ssh ${ADDR_BOARD} 'sudo reboot'
                ;;
            --compile_copy)
                echo "The module build and copy"
                make
                scp ${MODNAME}.ko ${ADDR_BOARD}:${MODULEDIR}
                ;;
            --compile_copy_reboot)
                echo "The module build, copy and reboot"
                make
                scp ${MODNAME}.ko ${ADDR_BOARD}:${MODULEDIR}
                sleep 1
                ssh ${ADDR_BOARD} 'sudo reboot'
                ;;
            --compile_copy_reload)
                echo "The module build, rmmod and insmod"
                make
                scp ${MODNAME}.ko ${ADDR_BOARD}:${MODULEDIR}
                ssh ${ADDR_BOARD} 'sudo modprobe -r ili9486'
                sleep 1
                ssh ${ADDR_BOARD} 'sudo modprobe ili9486'
                ;;
        esac
        shift
done

echo "Done!"

