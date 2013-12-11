#
# Makefile for misc devices that really don't fit anywhere else.
#

obj-$(CONFIG_IBM_ASM)		+= ibmasm/
obj-$(CONFIG_AD525X_DPOT)	+= ad525x_dpot.o
obj-$(CONFIG_AD525X_DPOT_I2C)	+= ad525x_dpot-i2c.o
obj-$(CONFIG_AD525X_DPOT_SPI)	+= ad525x_dpot-spi.o
obj-$(CONFIG_INTEL_MID_PTI)	+= pti.o
obj-$(CONFIG_ATMEL_PWM)		+= atmel_pwm.o
obj-$(CONFIG_ATMEL_SSC)		+= atmel-ssc.o
obj-$(CONFIG_ATMEL_TCLIB)	+= atmel_tclib.o
obj-$(CONFIG_BMP085)		+= bmp085.o
obj-$(CONFIG_ICS932S401)	+= ics932s401.o
obj-$(CONFIG_LKDTM)		+= lkdtm.o
obj-$(CONFIG_TIFM_CORE)       	+= tifm_core.o
obj-$(CONFIG_TIFM_7XX1)       	+= tifm_7xx1.o
obj-$(CONFIG_PHANTOM)		+= phantom.o
obj-$(CONFIG_SENSORS_BH1780)	+= bh1780gli.o
obj-$(CONFIG_SENSORS_BH1770)	+= bh1770glc.o
obj-$(CONFIG_SENSORS_APDS990X)	+= apds990x.o
obj-$(CONFIG_ANDROID_PMEM)	+= pmem.o
obj-$(CONFIG_SGI_IOC4)		+= ioc4.o
obj-$(CONFIG_ENCLOSURE_SERVICES) += enclosure.o
obj-$(CONFIG_KGDB_TESTS)	+= kgdbts.o
obj-$(CONFIG_SGI_XP)		+= sgi-xp/
obj-$(CONFIG_SGI_GRU)		+= sgi-gru/
obj-$(CONFIG_CS5535_MFGPT)	+= cs5535-mfgpt.o
obj-$(CONFIG_HP_ILO)		+= hpilo.o
obj-$(CONFIG_APDS9802ALS)	+= apds9802als.o
obj-$(CONFIG_ISL29003)		+= isl29003.o
obj-$(CONFIG_ISL29020)		+= isl29020.o
obj-$(CONFIG_USB_HSIC_SMSC_HUB) += smsc_hub.o
obj-$(CONFIG_SENSORS_TSL2550)	+= tsl2550.o
obj-$(CONFIG_EP93XX_PWM)	+= ep93xx_pwm.o
obj-$(CONFIG_DS1682)		+= ds1682.o
obj-$(CONFIG_TI_DAC7512)	+= ti_dac7512.o
obj-$(CONFIG_UID_STAT)		+= uid_stat.o
obj-$(CONFIG_C2PORT)		+= c2port/
obj-$(CONFIG_IWMC3200TOP)      += iwmc3200top/
obj-$(CONFIG_HMC6352)		+= hmc6352.o
obj-y				+= eeprom/
obj-y				+= cb710/
obj-$(CONFIG_SPEAR13XX_PCIE_GADGET)	+= spear13xx_pcie_gadget.o
obj-$(CONFIG_VMWARE_BALLOON)	+= vmw_balloon.o
obj-$(CONFIG_ARM_CHARLCD)	+= arm-charlcd.o
obj-$(CONFIG_PCH_PHUB)		+= pch_phub.o
obj-y				+= ti-st/
obj-$(CONFIG_AB8500_PWM)	+= ab8500-pwm.o
obj-y				+= lis3lv02d/
obj-y				+= carma/
obj-$(CONFIG_USB_SWITCH_FSA9480) += fsa9480.o
obj-$(CONFIG_ALTERA_STAPL)	+=altera-stapl/
obj-$(CONFIG_MAX8997_MUIC)	+= max8997-muic.o
obj-$(CONFIG_WL127X_RFKILL)	+= wl127x-rfkill.o
obj-$(CONFIG_SENSORS_AK8975)	+= akm8975.o
obj-$(CONFIG_WL127X_RFKILL)	+= wl127x-rfkill.o
obj-$(CONFIG_APANIC)		+= apanic.o
obj-$(CONFIG_SENSORS_AK8975)	+= akm8975.o
obj-$(CONFIG_TSIF) += msm_tsif.o
msm_tsif-objs := tsif.o
obj-$(CONFIG_TSIF_CHRDEV) += tsif_chrdev.o
obj-$(CONFIG_TSPP) += tspp.o
obj-$(CONFIG_HAPTIC_ISA1200)		+= isa1200.o
obj-$(CONFIG_PMIC8058_PWM) += pmic8058-pwm.o
obj-$(CONFIG_PMIC8XXX_VIBRATOR) += pm8xxx-vibrator.o
obj-$(CONFIG_PMIC8XXX_NFC) += pm8xxx-nfc.o
obj-$(CONFIG_PMIC8XXX_UPL) += pm8xxx-upl.o
obj-$(CONFIG_MSM_MEMORY_LOW_POWER_MODE_SUSPEND_DEEP_POWER_DOWN) \
	+= msm_migrate_pages.o
obj-$(CONFIG_PMIC8058_XOADC) += pmic8058-xoadc.o
obj-$(CONFIG_QSEECOM) += qseecom.o
obj-$(CONFIG_QFP_FUSE) += qfp_fuse.o

# WH Lee+
EXTRA_CFLAGS += -DBMM_USE_BASIC_I2C_FUNC

# Don Lin+
obj-$(CONFIG_SENSORS_APDS9930)	+= apds9930.o
obj-$(CONFIG_SENSORS_BMA2X2)    += bma2x2_driver.o
# Don Lin-
obj-$(CONFIG_SENSORS_BMA250)    += bma250_driver.o
obj-$(CONFIG_SENSORS_BMM050)    += bmm050_driver.o bmm050.o
obj-$(CONFIG_SENSORS_BMG160)    += bmg160_driver.o bmg160.o
obj-$(CONFIG_SENSORS_L3GD20)    += l3gd20_gyr.o
obj-$(CONFIG_SENSORS_TSL2772)	+= tsl277x.o
# WH Lee-

obj-$(CONFIG_FTD_MISC_ITEM) += ftd_misc.o
# Bright Lee, 20120517, save last log when panic { 
obj-$(CONFIG_PANIC_LASTLOG) += lastlog.o
$(obj)/lastlog.o: KBUILD_CFLAGS += -DOEM_BUILD_VERSION=\"$(OEM_MAIN_VERSION)-$(BUILD_NUMBER_OEM)\"
# } Bright Lee, 20120517
# RK Chen, 20121220, reset module to reset the chip, {
# obj-$(CONFIG_RESET_MODULE) += reset/
# } RK Chen, 20121220
# okpa, 20111227, save aboot log {
obj-$(CONFIG_ABOOT_LOG) += aboot_log.o
# } okpa, 20111227
