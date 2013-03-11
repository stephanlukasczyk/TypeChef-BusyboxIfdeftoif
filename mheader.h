#define BB_VER "1.8.5"
#define BB_BT ""
#define CONFIG_DEFAULT_DEPMOD_FILE ""

#define CONFIG_SUBST_WCHAR 63
#define CONFIG_LAST_SUPPORTED_WCHAR 767
#define CONFIG_BUSYBOX_EXEC_PATH "/proc/self/exe"
#define CONFIG_CROSS_COMPILER_PREFIX ""
#define CONFIG_EXTRA_CFLAGS ""
#define CONFIG_PREFIX "./_install"
#define CONFIG_PASSWORD_MINLEN 6
#define CONFIG_MD5_SIZE_VS_SPEED 2
#define CONFIG_FEATURE_EDITING_MAX_LEN 1024
#define CONFIG_FEATURE_EDITING_HISTORY 255
#define CONFIG_FEATURE_COPYBUF_KB 4
#define CONFIG_DEFAULT_SETFONT_DIR ""
#define CONFIG_FEATURE_VI_MAX_LEN 4096
#define CONFIG_TELINIT_PATH ""
#define CONFIG_FEATURE_KILL_DELAY 0
#define CONFIG_INIT_TERMINAL_TYPE "linux"
#define CONFIG_FIRST_SYSTEM_ID 100
#define CONFIG_LAST_SYSTEM_ID 999
#define CONFIG_DEFAULT_MODULES_DIR "/lib/modules"
#define CONFIG_DEFAULT_DEPMOD_FILE "modules.dep"
#define CONFIG_FEATURE_BEEP_FREQ 4000
#define CONFIG_FEATURE_BEEP_LENGTH_MS 30
#define CONFIG_FEATURE_CROND_DIR "/var/spool/cron"
#define CONFIG_FEATURE_LESS_MAXLINES 9999999
#define CONFIG_IFUPDOWN_IFSTATE_PATH "/var/run/ifstate"
#define CONFIG_DHCPD_LEASES_FILE "/var/lib/misc/udhcpd.leases"
#define CONFIG_UDHCP_DEBUG 9
#define CONFIG_UDHCPC_DEFAULT_SCRIPT "/usr/share/udhcpc/default.script"
#define CONFIG_UDHCPC_SLACK_FOR_BUGGY_SERVERS 80
#define CONFIG_IFUPDOWN_UDHCPC_CMD_OPTIONS "-R -n"
#define CONFIG_FEATURE_MIME_CHARSET "us-ascii"
#define CONFIG_SV_DEFAULT_SERVICE_DIR "/var/service"
#define CONFIG_FEATURE_SYSLOGD_READ_BUFFER_SIZE 256
#define CONFIG_FEATURE_IPC_SYSLOG_BUFFER_SIZE 16
#define CONFIG_FEATURE_DEFAULT_PASSWD_ALGO "des"
#define __USE_GNU
#undef CONFIG_TC

#ifdef CONFIG_FEATURE_PIDFILE
  #define CONFIG_PID_FILE_PATH "/var/run"
#endif

#ifdef CONFIG_SHA3_SMALL_TR
  #define CONFIG_SHA3_SMALL 0
#else
  #define CONFIG_SHA3_SMALL 1
#endif