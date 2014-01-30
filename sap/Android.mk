LOCAL_PATH:= $(call my-dir)

# SAP plugin
include $(CLEAR_VARS)

ifdef BLUETOOTH_ENABLE_FEATURE_SAP_BACKEND
LOCAL_SRC_FILES:= \
	manager.c \
	server.c \
	main.c \

ifeq ($(strip $(BLUETOOTH_ENABLE_FEATURE_SAP_BACKEND)),MAL)
LOCAL_SRC_FILES += sap-u8500.c
else
LOCAL_SRC_FILES += sap-caif.c
endif

LOCAL_CFLAGS:= \
	-DVERSION=\"4.93\" \
	-DSTORAGEDIR=\"/data/misc/bluetoothd\" \
	-DCONFIGDIR=\"/etc/bluetooth\" \
	-DDEBUG_SECTION=\"_sap\" \
	-Wno-missing-field-initializers

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../btio \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../common \
	$(LOCAL_PATH)/../gdbus \
	$(LOCAL_PATH)/../src \
	$(call include-path-for, glib) \
	$(call include-path-for, dbus)

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd \
	libbluetooth \
	libbtio \
	libcutils \
	libdbus \
	libglib


LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/bluez-plugin
LOCAL_UNSTRIPPED_PATH := $(TARGET_OUT_SHARED_LIBRARIES_UNSTRIPPED)/bluez-plugin
LOCAL_MODULE := sap
LOCAL_MODULE_TAGS := optional
LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)

# SAP PTS plugin. Enable only for not user variant.
ifneq ($(TARGET_BUILD_VARIANT),user)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
        manager.c \
        server.c \
        main.c \
        sap-dummy.c

LOCAL_CFLAGS:= \
	-DVERSION=\"4.93\" \
	-DSTORAGEDIR=\"/data/misc/bluetoothd\" \
	-DCONFIGDIR=\"/etc/bluetooth\" \
	-DDEBUG_SECTION=\"_sap\" \
	-Wno-missing-field-initializers

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../btio \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../common \
	$(LOCAL_PATH)/../gdbus \
	$(LOCAL_PATH)/../src \
	$(call include-path-for, glib) \
	$(call include-path-for, dbus)

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd \
	libbluetooth \
	libbtio \
	libcutils \
	libdbus \
	libglib

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/bluez-plugin
LOCAL_UNSTRIPPED_PATH := $(TARGET_OUT_SHARED_LIBRARIES_UNSTRIPPED)/bluez-plugin
LOCAL_MODULE := libsap_pts
LOCAL_MODULE_TAGS := optional
LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)
endif #!user
endif #BLUETOOTH_ENABLE_FEATURE_SAP_BACKEND
