# config.mk
#
# Product-specific compile-time definitions.
#
TARGET_CPU_ABI := armeabi-v7a
TARGET_CPU_ABI2 := armeabi
TARGET_CPU_VARIANT := cortex-a9
TARGET_CPU_SMP := true

# eMMC support
ifeq ($(MTK_EMMC_SUPPORT),yes)
TARGET_USERIMAGES_USE_EXT4:=true
TARGET_USERIMAGES_SPARSE_EXT_DISABLED := false
endif

USE_CAMERA_STUB := true

TARGET_NO_FACTORYIMAGE := true

# for migrate build system
# temporarily open this two options
HAVE_HTC_AUDIO_DRIVER := true
#BOARD_USES_GENERIC_AUDIO := true
 
BOARD_USES_MTK_AUDIO := true

BOARD_EGL_CFG := $(BOARD_CONFIG_DIR)/egl.cfg

BOARD_MTK_LIBSENSORS_NAME :=
BOARD_MTK_LIB_SENSOR :=

# MTK, Baochu Wang, 20101130, Add A-GPS {
ifeq ($(MTK_AGPS_APP), yes)
   BOARD_AGPS_SUPL_LIBRARIES := true
else
   BOARD_AGPS_SUPL_LIBRARIES := false
endif
# MTK, Baochu Wang, 20101130, Add A-GPS }

ifeq ($(MTK_GPS_SUPPORT), yes)
  BOARD_GPS_LIBRARIES := true
else
  BOARD_GPS_LIBRARIES := false
endif

# MTK, Infinity, 20090720, Add WiFi {
ifeq ($(MTK_WLAN_SUPPORT), yes)
BOARD_CONNECTIVITY_VENDOR := MediaTek
BOARD_CONNECTIVITY_MODULE := mt66xx

WPA_SUPPLICANT_VERSION := VER_0_8_KK_LGE
BOARD_HOSTAPD_DRIVER := NL80211
BOARD_HOSTAPD_PRIVATE_LIB := lib_driver_cmd_mt66xx
BOARD_WPA_SUPPLICANT_DRIVER := NL80211
BOARD_WPA_SUPPLICANT_PRIVATE_LIB := lib_driver_cmd_mt66xx
WIFI_DRIVER_FW_PATH_PARAM:="/dev/wmtWifi"
WIFI_DRIVER_FW_PATH_STA:=STA
WIFI_DRIVER_FW_PATH_AP:=AP
WIFI_DRIVER_FW_PATH_P2P:=P2P
#HAVE_CUSTOM_WIFI_DRIVER_2 := true
#HAVE_INTERNAL_WPA_SUPPLICANT_CONF := true
#HAVE_CUSTOM_WIFI_HAL := mediatek
endif
# MTK, Infinity, 20090720, Add WiFi }

TARGET_KMODULES := true

TARGET_ARCH_VARIANT := armv7-a-neon

ifeq ($(strip $(MTK_NAND_PAGE_SIZE)), 4K)
  BOARD_NAND_PAGE_SIZE := 4096 -s 128
else
  BOARD_NAND_PAGE_SIZE := 2048 -s 64   # default 2K
endif

# include all config files
include $(BOARD_CONFIG_DIR)/configs/*.mk

#SELinux Policy File Configuration
BOARD_SEPOLICY_DIRS := \
        mediatek/custom/common/sepolicy

#SELinux: MTK modified
BOARD_SEPOLICY_REPLACE := \
    keys.conf 

#SELinux: MTK added
BOARD_SEPOLICY_UNION := \
    app.te \
    device.te \
    domain.te \
    file.te \
    file_contexts \
    fs_use \
    installd.te \
    isolated_app.te \
    net.te \
    netd.te \
    te_macros \
    vold.te \
    untrusted_app.te \
    zygote.te \
    aal.te \
    abcc.te \
    adb.te \
    add_property_tag.te \
    aee_aed.te \
    aee_core_forwarder.te \
    aee_dumpstate.te \
    aee.te \
    agpscacertinit.te \
    akmd09911.te \
    akmd8963.te \
    akmd8975.te \
    ami304d.te \
    am.te \
    applypatch_static.te \
    applypatch.te \
    asan_app_process.te \
    asanwrapper.te \
    atcid.te \
    atci_service.te \
    atrace.te \
    audiocmdservice_atci.te \
    audiocommand.te \
    audioloop.te \
    audioregsetting.te \
    AudioSetParam.te \
    autofm.te \
    autosanity.te \
    badblocks.te \
    batterywarning.te \
    bdt.te \
    BGW.te \
    bmgr.te \
    bmm050d.te \
    bootanimation.te \
    boot_logo_updater.te \
    btconfig.te \
    btif_tester.te \
    btlogmask.te \
    btool.te \
    bugreport.te \
    bu.te \
    campipetest.te \
    camshottest.te \
    ccci_fsd.te \
    ccci_mdinit.te \
    check_lost_found.te \
    check_prereq.te \
    cjpeg.te \
    codec.te \
    content.te \
    corrupt_gdt_free_blocks.te \
    cpueater.te \
    cpustats.te \
    cputime.te \
    daemonize.te \
    dalvikvm.te \
    decoder.te \
    dexdump.te \
    dexopt.te \
    dhcp6c.te \
    dhcp6ctl.te \
    dhcp6s.te \
    directiotest.te \
    djpeg.te \
    dm_agent_binder.te \
    dualmdlogger.te \
    dumpstate.te \
    dumpsys.te \
    e2fsck.te \
    emcsmdlogger.te \
    em_svr.te \
    ext4_resize.te \
    factory.te \
    fbconfig.te \
    featured.te \
    flash_image.te \
    fsck_msdos_mtk.te \
    fsck_msdos.te \
    gatord.te \
    gdbjithelper.te \
    gdbserver.te \
    geomagneticd.te \
    GoogleOtaBinder.te \
    gsm0710muxdmd2.te \
    gsm0710muxd.te \
    gzip.te \
    hald.te \
    hdc.te \
    ime.te \
    InputDispatcher_test.te \
    InputReader_test.te \
    input.te \
    ip6tables.te \
    ipod.te \
    ipohctl.te \
    iptables.te \
    ip.te \
    keystore_cli.te \
    kfmapp.te \
    latencytop.te \
    lcdc_screen_cap.te \
    libmnlp_mt6582.te \
    libperfservice_test.te \
    librank.te \
    linker.te \
    logcat.te \
    logwrapper.te \
    lsm303md.te \
    lua.te \
    magd.te \
    make_ext4fs.te \
    malloc_debug_test.te \
    matv.te \
    mc6420d.te \
    mdlogger.te \
    md_minilog_util.te \
    mdnsd.te \
    media.te \
    memorydumper.te \
    memsicd3416x.te \
    memsicd.te \
    meta_tst.te \
    met_cmd.te \
    mfv_ut.te \
    micro_bench_static.te \
    micro_bench.te \
    mke2fs.te \
    mlcmd.te \
    mmp.te \
    mnld.te \
    mobile_log_d.te \
    monkey.te \
    msensord.te \
    msr3110_dev_test.te \
    mtk_6620_launcher.te \
    mtk_6620_wmt_concurrency.te \
    mtk_6620_wmt_lpbk.te \
    mtk_agpsd.te \
    mtkbt.te \
    mtop.te \
    muxer.te \
    muxreport.te \
    nc.te \
    ndc.te \
    netcfg.te \
    netdiag.te \
    netperf.te \
    netserver.te \
    nfcstackp.te \
    nvram_agent_binder.te \
    nvram_daemon.te \
    omx_ut.te \
    opcontrol.te \
    oprofiled.te \
    orientationd.te \
    perf.te \
    permission_check.te \
    ping6.te \
    pm.te \
    pngtest.te \
    poad.te \
    pppd_dt.te \
    pq.te \
    procmem.te \
    procrank.te \
    qmc5983d.te \
    radiooptions.te \
    radvd.te \
    rawbu.te \
    record.te \
    recordvideo.te \
    recovery.te \
    requestsync.te \
    resize2fs.te \
    resmon.te \
    resmon_test.te \
    rildmd2.te \
    rtt.te \
    s62xd.te \
    sane_schedstat.te \
    schedtest.te \
    screencap.te \
    screenshot.te \
    sdiotool.te \
    selinux_network.te \
    SEMTK_policy_check.py \
    sensorservice.te \
    server_open_nfc.te \
    service.te \
    set_ext4_err_bit.te \
    settings.te \
    sf2.te \
    showmap.te \
    showslab.te \
    shutdown.te \
    skia_test.te \
    sn.te \
    sqlite3.te \
    stagefright.te \
    stb.te \
    strace.te \
    stream.te \
    stressTestBench.te \
    superumount.te \
    svc.te \
    sysctld.te \
    sysctl.te \
    tcpdump.te \
    tc.te \
    terservice.te \
    tertestclient.te \
    testid3.te \
    test_rild_porting.te \
    thermald.te \
    thermal_manager.te \
    thermal.te \
    timeinfo.te \
    tiny_mkswap.te \
    tiny_swapoff.te \
    tiny_swapon.te \
    tiny_switch.te \
    toolbox.te \
    tune2fs.te \
    uiautomator.te \
    uim_sysfs.te \
    updater.te \
    vdc.te \
    vtservice.te \
    wfd.te \
    wlan_loader.te \
    wm.te \
    wmt_loader.te \
    wpa_cli.te \
    xlog.te

