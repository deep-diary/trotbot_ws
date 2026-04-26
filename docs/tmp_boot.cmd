# This is a boot script for U-Boot
#
# Recompile with:
# mkimage -A arm64 -O linux -T script -C none -n "Boot Script" -d boot.cmd boot.scr

setenv load_addr "0x7000000"
setenv overlay_error "false"
setenv overlay_attempted "false"

echo "Boot script loaded from ${devtype} ${devnum}"

if test -e ${devtype} ${devnum}:${distro_bootpart} /ubuntuEnv.txt; then
	load ${devtype} ${devnum}:${distro_bootpart} ${load_addr} /ubuntuEnv.txt
	env import -t ${load_addr} ${filesize}
fi

echo "U-Boot debug: fdtfile=${fdtfile}"
echo "U-Boot debug: overlays=${overlays}"
echo "U-Boot debug: overlay_prefix=${overlay_prefix}"

if test -z "${fdt_addr_r}"; then
	setenv fdt_addr_r 0x08300000
fi
if test -z "${fdtoverlay_addr_r}"; then
	setenv fdtoverlay_addr_r 0x08200000
fi

setenv bootargs "${bootargs} root=/dev/mmcblk${devnum}p2"
printenv bootargs

load ${devtype} ${devnum}:${distro_bootpart} ${fdt_addr_r} /dtbs/rockchip/${fdtfile}
fdt addr ${fdt_addr_r} && fdt resize 0x20000

if test -n "${overlays}"; then
	setenv overlay_attempted "true"
	for overlay_file in ${overlays}; do
		setenv overlay_found 0
		echo "U-Boot debug: trying overlay_file=${overlay_file}"
		for file in ${overlay_prefix}-${overlay_file}.dtbo ${overlay_prefix}-${overlay_file} ${overlay_file}.dtbo ${overlay_file}; do
			if test -e ${devtype} ${devnum}:${distro_bootpart} /dtbs/rockchip/overlay/${file}; then
				if load ${devtype} ${devnum}:${distro_bootpart} ${fdtoverlay_addr_r} /dtbs/rockchip/overlay/${file}; then
					echo "Applying device tree overlay: /dtbs/rockchip/overlay/${file}"
					if fdt apply ${fdtoverlay_addr_r}; then
						echo "U-Boot debug: apply success ${file}"
						setenv overlay_found 1
					else
						echo "U-Boot debug: apply failed ${file}"
						setenv overlay_error "true"
					fi
				else
					echo "U-Boot debug: load failed ${file}"
					setenv overlay_error "true"
				fi
			fi
		done
		if test "${overlay_found}" = "0"; then
			echo "Error: overlay not found for ${overlay_file}"
			setenv overlay_error "true"
		fi
	done
fi
if test "${overlay_error}" = "true"; then
	echo "Error applying device tree overlays, restoring original device tree"
	load ${devtype} ${devnum}:${distro_bootpart} ${fdt_addr_r} /dtbs/rockchip/${fdtfile}
fi

setenv bootargs "${bootargs} uboot_overlay_attempted=${overlay_attempted} uboot_overlay_error=${overlay_error}"
printenv bootargs

load ${devtype} ${devnum}:${distro_bootpart} ${kernel_addr_r} /vmlinuz
load ${devtype} ${devnum}:${distro_bootpart} ${ramdisk_addr_r} /initrd.img

booti ${kernel_addr_r} ${ramdisk_addr_r}:${filesize} ${fdt_addr_r}
