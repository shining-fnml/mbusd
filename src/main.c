/*
 * OpenMODBUS/TCP to RS-232/485 MODBUS RTU gateway
 *
 * main.c - main module
 *
 * Copyright (c) 2002-2003, 2013, Victor Antonovich (v.antonovich@gmail.com)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: main.c,v 1.6 2015/02/25 10:33:57 kapyar Exp $
 */

#include "globals.h"
#include "string.h"
#include "errno.h"
#include "cfg.h"
#include "tty.h"
#include "sock.h"
#include "conn.h"
#include "queue.h"
#include "sig.h"
#ifdef LOG
#  include "log.h"
#endif

extern char logfullname[];
int isdaemon = TRUE;

static void usage(char *exename);

/* Server socket */
int server_sd = -1;
/* TTY related data storage variable */
ttydata_t tty;
/* Per slaves TTY serial parameters */
ttyparam_t *ttyparam;
/* Connections queue descriptor */
queue_t queue;

#ifndef HAVE_DAEMON
#include <fcntl.h>
#include <unistd.h>
/*
 * System function daemon() replacement based on FreeBSD implementation.
 * Original source file CVS tag:
 * $FreeBSD: src/lib/libc/gen/daemon.c,v 1.3 2000/01/27 23:06:14 jasone Exp $
 */
static int
daemon(nochdir, noclose)
  int nochdir, noclose;
{
  int fd;

  switch (fork()) {
    case -1:
      return (-1);
    case 0:
      break;
    default:
      _exit(0);
  }

  if (setsid() == -1)
    return (-1);

  if (!nochdir)
    (void)chdir("/");

  if (!noclose && (fd = _open("/dev/null", O_RDWR, 0)) != -1) {
    (void)dup2(fd, STDIN_FILENO);
    (void)dup2(fd, STDOUT_FILENO);
    (void)dup2(fd, STDERR_FILENO);
    if (fd > 2)
      (void)_close(fd);
  }
  return (0);
}
#endif

static void
usage(char *exename)
{
  cfg_init();
  printf("%s-%s Copyright (C) 2002-2003, 2011, 2013-2016 Victor Antonovich <v.antonovich@gmail.com>, "
   "Andrew Denysenko <nitr0@seti.kr.ua>\n\n"
   "Usage: %s [-h] [-d] "
#ifdef TRXCTL
   "[-t] [-y sysfsfile] [-Y sysfsfile]\n"
#endif
   "[-v level] [-L logfile] [-p device] [-s speed] [-m mode] [-P port]\n"
   "             [-C maxconn] [-N retries] [-R pause] [-W wait] [-T timeout]\n\n"
   "Options:\n"
   "  -h         : this help\n"
   "  -d         : don't daemonize\n"
#ifdef TRXCTL
   "  -t         : enable RTS RS-485 data direction control using RTS\n"
   "  -y         : enable RTS RS-485 data direction control using sysfs file, active transmit\n"
   "  -Y         : enable RTS RS-485 data direction control using sysfs file, active receive\n"
#endif
#ifdef LOG
#ifdef DEBUG
   "  -v level   : set log level (0-9, default %d, 0 - errors only)\n"
#else
   "  -v level   : set log level (0-2, default %d, 0 - errors only)\n"
#endif
   "  -L logfile : set log file name (default %s%s, \n"
   "               '-' for logging to STDOUT only)\n"
#endif
   "  -p device  : set serial port device name (default %s)\n"
   "  -s speed   : set serial port speed (default %d)\n"
   "  -m mode    : set serial port mode (default %s)\n"
   "  -x file    : exclude some slaves from the command line settings\n"
   "  -P port    : set TCP server port number (default %d)\n"
   "  -C maxconn : set maximum number of simultaneous TCP connections\n"
   "               (1-128, default %d)\n"
   "  -N retries : set maximum number of request retries\n"
   "               (0-15, default %d, 0 - without retries)\n"
   "  -R pause   : set pause between requests in milliseconds\n"
   "               (1-10000, default %lu)\n"
   "  -W wait    : set response wait time in milliseconds\n"
   "               (1-10000, default %lu)\n"
   "  -T timeout : set connection timeout value in seconds\n"
   "               (0-1000, default %d, 0 - no timeout)"
   "\n", PACKAGE, VERSION, exename,
#ifdef LOG
      cfg.dbglvl, LOGPATH, LOGNAME,
#endif
      cfg.ttyport, cfg.ttyspeed, cfg.ttymode, cfg.serverport, cfg.maxconn,
      cfg.maxtry, cfg.rqstpause, cfg.respwait, cfg.conntimeout);
  exit(0);
}

int
main(int argc, char *argv[])
{
  int err = 0, rc;
  char *exename;

  sig_init();
  cfg_init();
  ttyparam = NULL;

  if ((exename = strrchr(argv[0], '/')) == NULL)
    exename = argv[0];
  else
    exename++;

  /* command line argument list parsing */
  while ((rc = getopt(argc, argv,
               "dh"
#ifdef TRXCTL
               "ty:Y:"
#endif
#ifdef LOG
               "v:L:"
#endif
               "p:s:m:x:P:C:N:R:W:T:")) != RC_ERR)
  {
    switch (rc)
    {
      case '?':
        exit(-1);
      case 'd':
        isdaemon = FALSE;
        break;
#ifdef TRXCTL
      case 't':
        cfg.trxcntl = TRX_RTS;
        break;
      case 'y':
	cfg.trxcntl = TRX_SYSFS_1;
        strncpy(cfg.trxcntl_file, optarg, INTBUFSIZE);
	break;
      case 'Y':
	cfg.trxcntl = TRX_SYSFS_0;
        strncpy(cfg.trxcntl_file, optarg, INTBUFSIZE);
	break;
#endif
#ifdef LOG
      case 'v':
        cfg.dbglvl = (char)strtol(optarg, NULL, 0);
#  ifdef DEBUG
        if (cfg.dbglvl < 0 || cfg.dbglvl > 9)
        { /* report about invalid log level */
          printf("%s: -v: invalid loglevel value"
                 " (%d, must be 0-9)\n", exename, cfg.dbglvl);
#  else
        if (cfg.dbglvl < 0 || cfg.dbglvl > 9)
        { /* report about invalid log level */
          printf("%s: -v: invalid loglevel value"
                 " (%d, must be 0-2)\n", exename, cfg.dbglvl);
#  endif
          exit(-1);
        }
        break;
      case 'L':
        if (*optarg != '/')
        {
          if (*optarg == '-')
          {
            /* logging to file disabled */
            *cfg.logname = '\0';
          }
          else
          { /* concatenate given log file name with default path */
            strncpy(cfg.logname, LOGPATH, INTBUFSIZE);
            strncat(cfg.logname, optarg,
                    INTBUFSIZE - strlen(cfg.logname));
          }
        }
        else strncpy(cfg.logname, optarg, INTBUFSIZE);
        break;
#endif
      case 'p':
        if (*optarg != '/')
        { /* concatenate given port name with default
             path to devices mountpoint */
          strncpy(cfg.ttyport, "/dev/", INTBUFSIZE);
          strncat(cfg.ttyport, optarg,
                  INTBUFSIZE - strlen(cfg.ttyport));
        }
        else strncpy(cfg.ttyport, optarg, INTBUFSIZE);
        break;
      case 's':
        cfg.ttyspeed = strtoul(optarg, NULL, 0);
        break;
      case 'm':
        cfg.ttymode = optarg;
        if (tty_mode_validate(exename, cfg.ttymode)!=RC_OK)
          return EXIT_FAILURE;
        break;
      case 'P':
        cfg.serverport = strtoul(optarg, NULL, 0);
        break;
      case 'C':
        cfg.maxconn = strtoul(optarg, NULL, 0);
        if (cfg.maxconn < 1 || cfg.maxconn > 128)
        { /* report about invalid max conn number */
          printf("%s: -C: invalid maxconn value"
                 " (%d, must be 1-128)\n", exename, cfg.maxconn);
          exit(-1);
        }
        break;
      case 'N':
        cfg.maxtry = strtoul(optarg, NULL, 0);
        if (cfg.maxtry > 15)
        { /* report about invalid max try number */
          printf("%s: -N: invalid maxtry value"
                 " (%d, must be 0-15)\n", exename, cfg.maxtry);
          exit(-1);
        }
        break;
      case 'R':
        cfg.rqstpause = strtoul(optarg, NULL, 0);
        if (cfg.rqstpause < 1 || cfg.rqstpause > 10000)
        { /* report about invalid rqst pause value */
          printf("%s: -R: invalid inter-request pause value"
                 " (%lu, must be 1-10000)\n", exename, cfg.rqstpause);
          exit(-1);
        }
        break;
      case 'W':
        cfg.respwait = strtoul(optarg, NULL, 0);
        if (cfg.respwait < 1 || cfg.respwait > 10000)
        { /* report about invalid resp wait value */
          printf("%s: -W: invalid response wait time value"
                 " (%lu, must be 1-10000)\n", exename, cfg.respwait);
          exit(-1);
        }
        break;
      case 'T':
        cfg.conntimeout = strtoul(optarg, NULL, 0);
        if (cfg.conntimeout > 1000)
        { /* report about invalid conn timeout value */
          printf("%s: -T: invalid conn timeout value"
                 " (%d, must be 1-1000)\n", exename, cfg.conntimeout);
          exit(-1);
        }
        break;
      case 'h':
        usage(exename);
        break;
      case 'x':
        if ((cfg.exclusion=fopen(optarg, "r")) == NULL)
        { /* report about missing on unreadable exclusion table */
          printf("%s: -x: cannot open %s for reading\n",
                 exename, optarg);
          return EXIT_FAILURE;
        }
        break;
    }
  }

#ifdef LOG
  if (log_init(cfg.logname) != RC_OK)
  {
    printf("%s: can't open logfile '%s' (%s), exiting...\n",
           exename,
           logfullname[0] ? logfullname : "no log name was given",
           strerror(errno));
    exit(-1);
  }
  logw(2, "%s-%s started...", PACKAGE, VERSION);
#endif

  if (conn_init(exename))
  {
#ifdef LOG
    err = errno;
    logw(2, "conn_init() failed, exiting...");
#endif
    exit(err);
  }

  /* go or not to daemon mode? */
  if (isdaemon && (rc = daemon(TRUE, FALSE)))
  {
#ifdef LOG
    logw(0, "Can't be daemonized (%s), exiting...", strerror(errno));
#endif
    exit(rc);
  }

  conn_loop(exename);
  err = errno;
#ifdef LOG
  logw(2, "%s-%s exited...", PACKAGE, VERSION);
#endif
  return (err);
}
