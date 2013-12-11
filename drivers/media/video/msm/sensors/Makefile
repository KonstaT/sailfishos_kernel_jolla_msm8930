GCC_VERSION      := $(shell $(CONFIG_SHELL) $(PWD)/scripts/gcc-version.sh $(CROSS_COMPILE)gcc)
EXTRA_CFLAGS += -Idrivers/media/video/msm
EXTRA_CFLAGS += -Idrivers/media/video/msm/io
EXTRA_CFLAGS += -Idrivers/media/video/msm/eeprom
EXTRA_CFLAGS += -Idrivers/media/video/msm/csi

# Eric Liu+, remove sensor_bayer (we don't use it to run imx091 driver)
# obj-$(CONFIG_MSM_CAMERA_SENSOR) += msm_sensor_common.o msm_sensor.o msm_sensor_bayer.o msm_sensor_init.o
obj-$(CONFIG_MSM_CAMERA_SENSOR) += msm_sensor_common.o msm_sensor.o
# Eric Liu-

obj-$(CONFIG_OV5647) += ov5647_v4l2.o
obj-$(CONFIG_OV8825) += ov8825_v4l2.o

# Eric Liu, add imx091 driver
obj-$(CONFIG_IMX091) += imx091_v4l2.o
# Eric Liu-

obj-$(CONFIG_IMX074) += imx074_v4l2.o
obj-$(CONFIG_S5K3L1YX) += s5k3l1yx.o
obj-$(CONFIG_OV8830) += ov8830_v4l2.o
obj-$(CONFIG_OV2720) += ov2720.o
obj-$(CONFIG_MT9M114) += mt9m114_v4l2.o

# Eric Liu+, add ov2675 driver, must after mt9m114 (Casper)
obj-$(CONFIG_OV2675) += ov2675_v4l2.o
# Eric Liu-

obj-$(CONFIG_S5K4E1) += s5k4e1_v4l2.o
obj-$(CONFIG_MT9E013) += mt9e013_v4l2.o
obj-$(CONFIG_WEBCAM_OV9726) += ov9726_v4l2.o
obj-$(CONFIG_OV7692) += ov7692_v4l2.o
obj-$(CONFIG_VX6953) += vx6953.o
