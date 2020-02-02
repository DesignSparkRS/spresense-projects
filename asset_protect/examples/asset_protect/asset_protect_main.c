/****************************************************************************
 * asset_protect/asset_protect_main.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *             2020 RS Components
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <sys/ioctl.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <assert.h>
#include <arch/chip/gnss.h>

#ifdef CONFIG_CXD56_SCU
#include <arch/chip/cxd56_scu.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ACC_DEVPATH      "/dev/accel0"

#define EVENT_SIGNAL              17
#define MY_GNSS_SIG0              19

#define FILE_NAME_LEN             256
#define MAX_LOG_COUNT             10

/****************************************************************************
 * Private Types
 ****************************************************************************/

 struct sample_s
 {
   int16_t x;
   int16_t y;
 };

struct cxd56_gnss_dms_s
{
  int8_t   sign;
  uint8_t  degree;
  uint8_t  minute;
  uint32_t frac;
};

struct asset_protect_s {
  struct cxd56_gnss_date_s         date;
  struct cxd56_gnss_time_s         time;
  double                           latitude;
  double                           longitude;
  uint32_t                         ts;         /* [in] sample timestamp */
  uint32_t                         type;       /* Event type (SCU_EV_RISE or SCU_EV_FALL) */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char *g_data;
static struct scuev_arg_s g_evarg;

static struct cxd56_gnss_positiondata_s posdat;
static struct asset_protect_s           aplogdat;
static const struct asset_protect_s     emptyLog;

/*
 * Equal multiplication setting for each IIR filters.
 */

static const struct math_filter_s g_filter =
{
  .pos = FILTER_POS_AA,
  .filter[0] = {
    .ishift = 0,
    .oshift = 0,
    .coeff[0].h = 0x20000000,
    .coeff[0].l = 0,
    .coeff[1].h = 0,
    .coeff[1].l = 0,
    .coeff[2].h = 0,
    .coeff[2].l = 0,
    .coeff[3].h = 0,
    .coeff[3].l = 0,
    .coeff[4].h = 0,
    .coeff[4].l = 0,
  },
  .filter[1] = {
    .ishift = 0,
    .oshift = 0,
    .coeff[0].h = 0x20000000,
    .coeff[0].l = 0,
    .coeff[1].h = 0,
    .coeff[1].l = 0,
    .coeff[2].h = 0,
    .coeff[2].l = 0,
    .coeff[3].h = 0,
    .coeff[3].l = 0,
    .coeff[4].h = 0,
    .coeff[4].l = 0,
  },
};

/*
 * SCU event notifier control for tilt detection
 *
 * Both of rise event and fall event are enabled, always output sensing data
 * to SCU FIFO.
 *
 */

static const struct scuev_notify_s g_notify =
{
  .signo = EVENT_SIGNAL,

  /* If you want to see about actual data, please change SCU_EV_NOTOUT to
   * SCU_EV_OUTALWAYS */

  .ctrl = SCU_EV_RISE_EN | SCU_EV_NOTOUT,
  .rise = {
    .threshold = 10000,
    .count0 = 5,
    .count1 = 10,
    .delaysamples = 0,
  },
  .fall = {
    .threshold = 10000,
    .count0 = 5,
    .count1 = 10,
    .delaysamples = 0,
  },
  .arg = &g_evarg,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int set_event_detection(int fd)
{
  int ret;

  /*
   * Set IIR filter
   *
   * This operation is mandatory because event notifier takes filtered sampling
   * data. If you don't need to IIR filter, then set filter to be equal
   * multiplication (x1).
   */

  printf("- set SCU IIR filter \n");
  ret = ioctl(fd, SCUIOC_SETFILTER, (unsigned long)(uintptr_t)&g_filter);
  if (ret < 0)
    {
      fprintf(stderr, "-- set filter failed: %d\n", errno);
      return ret;
    }

  /* Use X and Y from sampling data */

  ioctl(fd, SCUIOC_SETELEMENTS, 2);

  /*
   * Set event notifier configuration
   *
   * See g_notify declaration for configuration details.
   */

  printf("- set SCU event notifier \n");
  ret = ioctl(fd, SCUIOC_SETNOTIFY, (unsigned long)(uintptr_t)&g_notify);
  if (ret < 0)
    {
      fprintf(stderr, "-- set notify failed: %d\n", errno);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: set_signal()
 *
 * Description:
 *   Call CXD56_GNSS_IOCTL_SIGNAL_SET.
 *
 ****************************************************************************/

static int set_signal(int fd, int signo, uint8_t gnsssig, int enable,
                      sigset_t * mask)
{
  int ret;
  struct cxd56_gnss_signal_setting_s setting;

  sigaddset(mask, signo);
  ret = sigprocmask(SIG_BLOCK, mask, NULL);
  if (ret != OK)
    {
      printf("sigprocmask failed. %d\n", ret);
      goto _err1;
    }

  setting.fd      = fd;
  setting.enable  = enable;
  setting.gnsssig = gnsssig;
  setting.signo   = signo;
  setting.data    = NULL;

  ret = ioctl(fd, CXD56_GNSS_IOCTL_SIGNAL_SET, (unsigned long)&setting);

_err1:
  return ret;
}

/****************************************************************************
 * Name: double_to_dmf()
 *
 * Description:
 *   Convert from double format to degree-minute-frac format.
 *
 ****************************************************************************/

static void double_to_dmf(double x, struct cxd56_gnss_dms_s * dmf)
{
  int    b;
  int    d;
  int    m;
  double f;
  double t;

  if (x < 0)
    {
      b = 1;
      x = -x;
    }
  else
    {
      b = 0;
    }
  d = (int)x; /* = floor(x), x is always positive */
  t = (x - d) * 60;
  m = (int)t; /* = floor(t), t is always positive */
  f = (t - m) * 10000;

  dmf->sign   = b;
  dmf->degree = d;
  dmf->minute = m;
  dmf->frac   = f;
}

/****************************************************************************
 * Name: get_pos()
 *
 * Description:
 *   Get position and time data.
 *
 ****************************************************************************/

static int get_pos(int fd, sigset_t * mask)
{
  int ret;
  int sig_id    = -1;

  sig_id = sigwaitinfo(mask, NULL);

  if (sig_id == MY_GNSS_SIG0)
    {
      /* Seek data */
      ret = lseek(fd, CXD56_GNSS_READ_OFFSET_LAST_GNSS, SEEK_SET);
      if (ret < 0)
        {
          ret = errno;
          printf("lseek error %d\n", ret);
          goto _err1;
        }

      /* Read data */
      ret = read(fd, &posdat, sizeof(posdat));
      if (ret < 0)
        {
          ret = errno;
          printf("read error %d\n", ret);
          goto _err1;
        }
      else if (ret < sizeof(posdat))
        {
          ret = ERROR;
          printf("read size error %d\n", ret);
          goto _err1;
        }
      else
        {
          ret = OK;
        }
    }

  _err1:
    return ret;
}

/****************************************************************************
 * Name: writefile()
 *
 * Description:
 *   Write asset protect log data.
 *
 ****************************************************************************/

static int writefile(uint32_t file_count)
{
  int fd_write;
  int ret = OK;
  char filename[FILE_NAME_LEN];

  /* Make file name */

  snprintf(filename, FILE_NAME_LEN, "%s%d.dat",
           CONFIG_EXAMPLES_ASSET_PROTECT_FILEPATH, file_count);

  /* Open file */

  fd_write = open(filename, O_WRONLY | O_CREAT | O_BINARY);
  if (fd_write < 0)
    {
      printf("%s open error:%d\n", filename, errno);
      ret = ERROR;
    }
  else
    {
      if (write(fd_write, &aplogdat, sizeof(struct asset_protect_s)) !=
          sizeof(struct asset_protect_s))
        {
          printf("%s write error:%d\n", filename, errno);
          ret = ERROR;
        }
      else
        {
          printf("%s write OK\n", filename);
        }

      close(fd_write);
    }
  fd_write = 0;

  return ret;
}

/****************************************************************************
 * Name: create_log()
 *
 * Description:
 *   Create a new log with event details.
 *
 ****************************************************************************/

static int create_log(uint32_t ts, uint32_t type)
{
  aplogdat = emptyLog;

  aplogdat.ts = ts;
  aplogdat.type = type;

  if (posdat.receiver.pos_dataexist &&
      (posdat.receiver.pos_fixmode != CXD56_GNSS_PVT_POSFIX_INVALID))
    {
      aplogdat.date = posdat.receiver.date;
      aplogdat.time = posdat.receiver.time;
      aplogdat.latitude = posdat.receiver.latitude;
      aplogdat.longitude = posdat.receiver.longitude;
    }

  return 0;
}

/****************************************************************************
 * Name: print_log()
 *
 * Description:
 *   Print position data.
 *
 ****************************************************************************/

static int print_log(struct asset_protect_s * log)
{
  struct cxd56_gnss_dms_s      dmf;

  /* Printf asset protect log file */

  printf("%d-%02d-%02d",
             log->date.year, log->date.month, log->date.day);

  printf("T%02d:%02d:%02dZ", log->time.hour,
             log->time.minute, log->time.sec);

  double_to_dmf(log->latitude, &dmf);
  printf(", Lat: %d:%d:%d",
         dmf.degree, dmf.minute, dmf.frac);

  double_to_dmf(log->longitude, &dmf);
  printf(", Long: %d:%d:%d",
        dmf.degree, dmf.minute, dmf.frac);

  printf(", TS: %d", log->ts);

  printf(", Type: %s\n",
          log->type == SCU_EV_RISE ? "Rise" : "Fall");

  return 0;
}

/****************************************************************************
 * Name: asset_protect_read()
 *
 * Description:
 *   Read the asset protect log.
 *
 ****************************************************************************/

static int asset_protect_read(int argc, char *argv[])
{
  int      ret = OK;
  int      fd_read;
  uint32_t file_count;
  char     filename[FILE_NAME_LEN];
  /* struct asset_protect_s *log; */

  /* Program start */

  printf("%s() in\n", __func__);

  for (file_count = 1; file_count <= MAX_LOG_COUNT; file_count++)
    {
      /* Make file name */

      snprintf(filename, FILE_NAME_LEN, "%s%d.dat",
               CONFIG_EXAMPLES_ASSET_PROTECT_FILEPATH, file_count);

      /* Open file */

      fd_read = open(filename, O_RDONLY | O_BINARY);
      if (fd_read < 0)
        {
          /* Continue */
        }
      else
        {
          /* Read file */

          ret = read(fd_read, &aplogdat, sizeof(aplogdat));
          if (ret < 0)
            {
              ret = errno;
              printf("%s read error:%d\n", filename, errno);
            }
          else if (ret != sizeof(aplogdat))
            {
              ret = ERROR;
              printf("%s read error:%d\n", filename, errno);
            }
          else
            {
              ret = OK;
              printf("%s read OK:\n", filename);
              print_log(&aplogdat);
            }
        }

      /* Close file */

      close(fd_read);
      fd_read = 0;
    }

  printf("%s() out %d\n", __func__, ret);

  return ret;
}

/****************************************************************************
 * Name: asset_protect_delete()
 *
 * Description:
 *   Delete asset protect log files.
 *
 ****************************************************************************/

static int asset_protect_delete(int argc, char *argv[])
{
  int ret = OK;
  uint32_t file_count;
  char filename[FILE_NAME_LEN];

  /* Program start */

  printf("%s() in\n", __func__);

  for (file_count = 1; file_count <= MAX_LOG_COUNT; file_count++)
    {
      /* Make file name */

      snprintf(filename, FILE_NAME_LEN, "%s%d.dat",
               CONFIG_EXAMPLES_ASSET_PROTECT_FILEPATH, file_count);

      /* Delete file */

      if (unlink(filename) == OK)
        {
          printf("%s delete ok\n", filename);
        }
    }

  printf("%s() out %d\n", __func__, ret);

  return ret;
}

/****************************************************************************
 * Name: asset_protect_start()
 *
 * Description:
 *   Start the asset protect main application.
 *
 ****************************************************************************/

static int asset_protect_start(int argc, char *argv[])
{
  int ret = OK;
  int sense_fd;
  int gnss_fd;
  sigset_t mask;
  uint32_t file_count = 1;

  sigset_t  set;
  siginfo_t info;

  /* Start GNSS */

  printf("Start GNSS:\n");

  /* Get file descriptor for GNSS. */

  printf("- opening GNSS \n");
  gnss_fd = open("/dev/gps", O_RDONLY);
  if (gnss_fd < 0)
    {
      printf("-- open error:%d,%d\n", gnss_fd, errno);
      return -ENODEV;
    }

  sigemptyset(&mask);

  /* Init positioning signal */
  printf("- set GNSS signal \n");
  ret = set_signal(gnss_fd, MY_GNSS_SIG0, CXD56_GNSS_SIG_GNSS, TRUE, &mask);
  if (ret < 0)
    {
      printf("-- GNSS signal set error\n");
      goto _err3;
    }

  /* Start positioning. */
  printf("- start positioning \n");
  ret = ioctl(gnss_fd, CXD56_GNSS_IOCTL_START, CXD56_GNSS_STMOD_HOT);
  if (ret < 0)
    {
      printf("-- start positoning ERROR %d\n", errno);
      goto _err0;
    }
  else
    {
      printf("Started GNSS OK.\n");
    }

  /* Start tilt detection */

  printf("Start tilt detection:\n");

  g_data = (char *)malloc(1536);
  if (!g_data)
    {
      printf("-- Memory allocation failure.\n");
      return -1;
     }
  memset(g_data, 0, 1536);

  /* Get file descriptor for accelerometer. */

  printf("- opening accelerometer \n");
  sense_fd = open(ACC_DEVPATH, O_RDONLY);
  if (sense_fd < 0)
    {
      printf("-- device %s open failure. %d\n", ACC_DEVPATH, sense_fd);
      /* return -1; */
    }

  /* Set FIFO size to 6 bytes * 128 Hz = 768 */

  printf("- set FIFO \n");
  ret = ioctl(sense_fd, SCUIOC_SETFIFO, sizeof(struct sample_s) * 128);
  if (ret < 0)
    {
      fprintf(stderr, "-- SETFIFO failed. %d\n", errno);
      return ret;
    }

  /* Set sequencer sampling rate 128 Hz
   * (if config CXD56_SCU_PREDIV = 64)
   * 32768 / 64 / (2 ^ 2) = 128
   */

  printf("- set sequencer sampling rate \n");
  ret = ioctl(sense_fd, SCUIOC_SETSAMPLE, 2);
  if (ret < 0)
    {
      fprintf(stderr, "-- SETSAMPLE failed. %d\n", errno);
      return ret;
    }

  set_event_detection(sense_fd);

  sigemptyset(&set);
  sigaddset(&set, EVENT_SIGNAL);
  sigprocmask(SIG_BLOCK, &set, NULL);

  /* Start sequencer */

  printf("- start sequencer \n");
  ret = ioctl(sense_fd, SCUIOC_START, 0);
  if (ret != 0)
    {
      fprintf(stderr, "-- sequencer start failed. %d\n", errno);
      return ret;
    }

  do
    {
      /* Wait event detector signal */

      printf("Waiting for event detector signal. \n");
      ret = sigwaitinfo(&set, &info);
      if (ret < 0)
        {
          int errcode = errno;
          if (errcode == EINTR)
            {
              continue;
            }
          fprintf(stderr, "-- sigwaitinfo() failed. %d\n", errcode);
          break;
        }
      else if (ret == EVENT_SIGNAL)
        {
          struct scuev_arg_s *arg = (struct scuev_arg_s *)info.si_value.sival_ptr;

          /* Get position and time */
          get_pos(gnss_fd, &mask);

          /* Format log */
          create_log(arg->ts.sec, arg->type);

          /* Print log and write to flash */
          print_log(&aplogdat);
          writefile(file_count);

          file_count++;
        }
    }
  while (file_count < MAX_LOG_COUNT);

  /* Handle GNSS errors */

  _err0:
    /* Stop GNSS. */

    ret = ioctl(gnss_fd, CXD56_GNSS_IOCTL_STOP, 0);
    if (ret < 0)
      {
        printf("-- stop GNSS ERROR\n");
      }
    else
      {
        printf("-- stop GNSS OK\n");
      }

  _err1:
    /* TBD */

    set_signal(gnss_fd, MY_GNSS_SIG0, CXD56_GNSS_SIG_GNSS, FALSE, &mask);

  _err3:
    /* Release GNSS file descriptor. */

    ret = close(gnss_fd);
    if (ret < 0)
      {
        printf("-- close GNSS error\n");
      }

  /* Stop sequencer */

  (void) ioctl(sense_fd, SCUIOC_STOP, 0);

  /* Close sensor */

  close(sense_fd);
  free(g_data);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * sensor_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int asset_protect_main(int argc, char *argv[])
#endif
{
  int ret = OK;
  char *argmode = "s";  /* Default: start */

  /* Program start */

  printf("-- Asset Protect --\n");

  /* Argument check */

  if (argc >= 2)
    {
      argmode = argv[1];
    }

  switch (argmode[0])
    {
    case 'r':
    case 'R':
      /* Read and dump asset protect log file */

      printf("Events logged: \n");
      ret = asset_protect_read(argc, argv);
      break;

    case 'd':
    case 'D':
      /* Delete asset protect file */

      printf("Deleting event log ...\n");
      ret = asset_protect_delete(argc, argv);
      break;

    case 's':
    case 'S':
    default:
      /* Start in normal run mode */

      printf("Event detection and logging starting ...\n");
      printf("Logging up to %d events.\n", MAX_LOG_COUNT);
      ret = asset_protect_start(argc, argv);
      break;
    }

  printf("++ Asset Protect Stopped: %d\n ++", ret);

  return ret;
}
