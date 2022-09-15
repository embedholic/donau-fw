#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/ccs1040/ccs/ccs_base;C:/ti/bios_6_35_06_56/packages;C:/Users/isjeon/workspace/MY_WORK/destine/fw/stableen_mcu_jcnet/.config
override XDCROOT = C:/ti/xdctools_3_25_06_96
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/ccs1040/ccs/ccs_base;C:/ti/bios_6_35_06_56/packages;C:/Users/isjeon/workspace/MY_WORK/destine/fw/stableen_mcu_jcnet/.config;C:/ti/xdctools_3_25_06_96/packages;..
HOSTOS = Windows
endif
