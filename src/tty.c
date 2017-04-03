/*
 * OpenMODBUS/TCP to RS-232/485 MODBUS RTU gateway
 *
 * tty.c - terminal I/O related procedures
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
 * $Id: tty.c,v 1.4 2015/02/25 10:33:58 kapyar Exp $
 */

#include "tty.h"

extern cfg_t cfg;
extern ttyparam_t *ttyparam;

static int tty_break;

/*
 * Flag signal SIG
 */
void
tty_sighup(void)
{
  tty_break = TRUE;
  return;
}

/*
 * Init serial link parameters MOD to PORT name, SPEED and TRXCNTL type
 */
void
#ifdef  TRXCTL
tty_init(ttydata_t *mod, char *port, int trxcntl)
#else
tty_init(ttydata_t *mod, char *port)
#endif
{
  mod->fd = -1;
  mod->port = port;
#ifdef  TRXCTL
  mod->trxcntl = trxcntl;
#endif
}

static void
tty_set_mode_speed(ttyparam_t *param, char *ttymode, int ttyspeed)
{
  speed_t tspeed = tty_transpeed(ttyspeed);

  param->speed = ttyspeed;
  strncpy(param->mode, ttymode, 4);

  switch (ttymode[1])
  {
    case 'E':
      param->tios.c_cflag |= PARENB;
      break;
    case 'N':
      param->tios.c_cflag &= ~PARENB;
      break;
    case 'O':
      param->tios.c_cflag |= (PARENB | PARODD);
      break;
  }
  if (ttymode[2] == '2')
      param->tios.c_cflag |= CSTOPB;
  else
      param->tios.c_cflag &= ~CSTOPB;
  param->tios.c_cc[VTIME] = 0;
  param->tios.c_cc[VMIN] = 1;
#ifdef HAVE_CFSETSPEED
  cfsetspeed(&param->tios, tspeed);
#else
  cfsetispeed(&param->tios, tspeed);
  cfsetospeed(&param->tios, tspeed);
#endif
}

int
tty_alt_config(char *exename, ttydata_t *mod)
{
  int iter, slave, speed;
  char mode[4];
  ttyparam_t *known;

  ttyparam = (ttyparam_t *) calloc(1, sizeof(ttyparam_t));
  memset(&mod->savedtios, 0, sizeof(struct termios));
  if (tcgetattr(mod->fd, &mod->savedtios)) {
#ifdef LOG
    logw(3, "%s(): can't get current device termios", __FUNCTION__);
#endif
    return RC_ERR;
  }
  memcpy(&ttyparam->tios, &mod->savedtios, sizeof(struct termios));
#ifdef HAVE_CFMAKERAW
  cfmakeraw(&ttyparam->tios);
#else
  ttyparam->tios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  ttyparam->tios.c_oflag &= ~OPOST;
  ttyparam->tios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  ttyparam->tios.c_cflag |= CREAD | CLOCAL ;
#endif
  ttyparam->tios.c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD | CRTSCTS);
  ttyparam->tios.c_cflag |= CS8;
  tty_set_mode_speed(ttyparam, cfg.ttymode, cfg.ttyspeed);

  for (iter=0; iter<256; iter++)
    mod->param[iter] = ttyparam;
  mod->curslave = &ttyparam->tios;
  if (cfg.exclusion==NULL) {
#ifdef LOG
    logw(3, "%s(): no exclusion file defined", __FUNCTION__);
#endif
    return RC_OK;
  }
  while ((iter=fscanf(cfg.exclusion,"%d %d %3s", &slave, &speed, mode)) == 3)
  {
#ifdef LOG
    logw(5, "%s(): %d fields converted", __FUNCTION__, iter);
#endif
    if (tty_mode_validate(exename, mode) != RC_OK)
      return RC_ERR;
    for (known=ttyparam; known!=NULL; known=known->next)
      if (speed == known->speed && !strcmp(mode,known->mode))
        break;
    if (known!=NULL) {
      mod->param[slave] = known;
      continue;
    }
    known = (ttyparam_t *) calloc(1, sizeof(ttyparam_t));
    memcpy(&known->tios, &ttyparam->tios, sizeof(struct termios));
    tty_set_mode_speed(known, mode, speed);
    known->next = ttyparam;
    ttyparam = known;
    mod->param[slave] = known;
#ifdef LOG
    logw(5, "%s(): alternative configuration allocated", __FUNCTION__, iter);
#endif
  }
  if (!feof(cfg.exclusion)) {
#ifdef LOG
      logw(3, "%s(): Error processing exclusion file. %d fields converted", __FUNCTION__, iter);
#endif
      return RC_ERR;
  }
  fclose(cfg.exclusion);
  return RC_OK;
}

#ifdef HAVE_LIBUTIL
static char *tty_get_name(char *ttyfullname);

static char *
tty_get_name(char *ttyfullname)
{
  static char ttynamebuf[INTBUFSIZE + 1];
  char *ttyname = ttynamebuf, *ttynameptr = ttyname;
  strncpy(ttynamebuf, ttyfullname, INTBUFSIZE);
  for (ttynameptr = strtok(ttynamebuf, "/");
       ttynameptr;
       ttynameptr = strtok(NULL, "/"))
  {
    ttyname = ttynameptr;
  }
  return ttyname;
}
#endif

int
tty_mode_validate(char *exename, char *ttymode)
{
  /* tty mode sanity checks */
  if (strlen(ttymode) != 3)
  {
    printf("%s: -m: invalid serial port mode ('%s')\n", exename, ttymode);
    return RC_ERR;
  }
  if (ttymode[0] != '8')
  {
    printf("%s: -m: invalid serial port character size "
        "(%c, must be 8)\n",
        exename, ttymode[0]);
    return RC_ERR;
  }
  ttymode[1] = toupper(ttymode[1]);
  if (ttymode[1] != 'N' && ttymode[1] != 'E' && ttymode[1] != 'O')
  {
    printf("%s: -m: invalid serial port parity "
        "(%c, must be N, E or O)\n", exename, ttymode[1]);
    return RC_ERR;
  }
  if (ttymode[2] != '1' && ttymode[2] != '2')
  {
    printf("%s: -m: invalid serial port stop bits "
        "(%c, must be 1 or 2)\n", exename, ttymode[2]);
    return RC_ERR;
  }
  return RC_OK;
}


/*
 * Opening serial link whith parameters in MOD
 */
int
tty_open(char *exename, ttydata_t *mod)
{
#ifdef HAVE_LIBUTIL
  int buferr, uuerr;
  char *ttyname = tty_get_name(mod->port);
#endif
  if (mod->fd > 0)
    return RC_AOPEN;        /* if already open... */
  tty_break = FALSE;
#ifdef HAVE_LIBUTIL
  if ((uuerr = uu_lock(ttyname)) != UU_LOCK_OK)
  {
    buferr = errno;
#ifdef LOG
    logw(0, "uu_lock(): can't lock tty device %s (%s)",
        ttyname, uu_lockerr(uuerr));
#endif
    errno = buferr;
    return RC_ERR;
  }
#endif
  mod->fd = open(mod->port, O_RDWR | O_NONBLOCK | O_NOCTTY);
  if (mod->fd < 0)
    return RC_ERR;          /* attempt failed */
  if (mod->curslave==NULL && tty_alt_config(exename, mod) != RC_OK)
  {
#ifdef LOG
    logw(0, "conn_init():"
           " can't configure device %s", cfg.ttyport);
#endif
    return RC_ERR;
  }
  return tty_set_attr(mod);
}

/*
 * Setting up tty device MOD attributes
 */
int
tty_set_attr(ttydata_t *mod)
{
  int flag;

  if (tcsetattr(mod->fd, TCSANOW, mod->curslave))
    return RC_ERR;
  tcflush(mod->fd, TCIOFLUSH);
#ifdef  TRXCTL
  tty_clr_rts(mod->fd);
#endif
  flag = fcntl(mod->fd, F_GETFL, 0);
  if (flag < 0)
    return RC_ERR;
  return fcntl(mod->fd, F_SETFL, flag | O_NONBLOCK);
}

/*
 * Translate integer SPEED value to speed_t constant
 */
speed_t
tty_transpeed(int speed)
{
  speed_t tspeed;
  switch (speed)
  {
  case 0:
    tspeed = B0;
    break;
#if defined(B50)
  case 50:
    tspeed = B50;
    break;
#endif
#if defined(B75)
  case 75:
    tspeed = B75;
    break;
#endif
#if defined(B110)
  case 110:
    tspeed = B110;
    break;
#endif
#if defined(B134)
  case 134:
    tspeed = B134;
    break;
#endif
#if defined(B150)
  case 150:
    tspeed = B150;
    break;
#endif
#if defined(B200)
  case 200:
    tspeed = B200;
    break;
#endif
#if defined(B300)
  case 300:
    tspeed = B300;
    break;
#endif
#if defined(B600)
  case 600:
    tspeed = B600;
    break;
#endif
#if defined(B1200)
  case 1200:
    tspeed = B1200;
    break;
#endif
#if defined(B1800)
  case 1800:
    tspeed = B1800;
    break;
#endif
#if defined(B2400)
  case 2400:
    tspeed = B2400;
    break;
#endif
#if defined(B4800)
  case 4800:
    tspeed = B4800;
    break;
#endif
#if defined(B7200)
  case 7200:
    tspeed = B7200;
    break;
#endif
#if defined(B9600)
  case 9600:
    tspeed = B9600;
    break;
#endif
#if defined(B12000)
  case 12000:
    tspeed = B12000;
    break;
#endif
#if defined(B14400)
  case 14400:
    tspeed = B14400;
    break;
#endif
#if defined(B19200)
  case 19200:
    tspeed = B19200;
    break;
#elif defined(EXTA)
  case 19200:
    tspeed = EXTA;
    break;
#endif
#if defined(B38400)
  case 38400:
    tspeed = B38400;
    break;
#elif defined(EXTB)
  case 38400:
    tspeed = EXTB;
    break;
#endif
#if defined(B57600)
  case 57600:
    tspeed = B57600;
    break;
#endif
#if defined(B115200)
  case 115200:
    tspeed = B115200;
    break;
#endif
  default:
    tspeed = DEFAULT_BSPEED;
    break;
  }
  return tspeed;
}

/*
 * Prepare tty device MOD to closing
 */
int
tty_cooked(ttydata_t *mod)
{
  signal(SIGHUP, SIG_IGN);
  signal(SIGPIPE, SIG_IGN);
  if (!isatty(mod->fd))
    return RC_ERR;
  if (tcsetattr(mod->fd, TCSAFLUSH, &mod->savedtios))
    return RC_ERR;
  return RC_OK;
}

/*
 * Closing tty device MOD
 */
int
tty_close(ttydata_t *mod)
{
#ifdef HAVE_LIBUTIL
  int buferr;
  char *ttyname = tty_get_name(mod->port);
#endif
  if (mod->fd < 0)
    return RC_ACLOSE;       /* already closed */
  if (tty_cooked(mod))
    return RC_ERR;
#ifdef HAVE_LIBUTIL
  if (close(mod->fd))
    return RC_ERR;
  if (uu_unlock(ttyname))
  {
    buferr = errno;
#ifdef LOG
    logw(0, "uu_lock(): can't unlock tty device %s",
        ttyname);
#endif
    errno = buferr;
    return RC_ERR;
  }
  return RC_OK;
#else
  return close(mod->fd);
#endif
}

#ifdef  TRXCTL
void sysfs_gpio_set(char *filename, char *value) {
	int fd;

	fd = open(filename, O_WRONLY);
	/* write first byte. Should only be 0 or 1 */
	write(fd, value, 1);
	close(fd);

	logw(9, "tty: sysfs_gpio_set(%s,%s)\n",filename,value);

}

/* Set RTS line to active state */
void
tty_set_rts(int fd)
{
	if ( TRX_RTS == cfg.trxcntl ) {
		int mstat = TIOCM_RTS;
		ioctl(fd, TIOCMBIS, &mstat);
	} else if ( TRX_SYSFS_1 == cfg.trxcntl) {
		sysfs_gpio_set(cfg.trxcntl_file,"1");
	} else if ( TRX_SYSFS_0 == cfg.trxcntl) {
		sysfs_gpio_set(cfg.trxcntl_file,"0");
	}
}

/* Set RTS line to passive state */
void
tty_clr_rts(int fd)
{
	if ( TRX_RTS == cfg.trxcntl ) {
		int mstat = TIOCM_RTS;
		ioctl(fd, TIOCMBIC, &mstat);
	} else if ( TRX_SYSFS_1 == cfg.trxcntl) {
		sysfs_gpio_set(cfg.trxcntl_file,"0");
	} else if ( TRX_SYSFS_0 == cfg.trxcntl) {
		sysfs_gpio_set(cfg.trxcntl_file,"1");
	}
}
#endif

/*
 * Delay for USEC microsecs
 */
void
tty_delay(int usec)
{
  struct timeval tv, ttv;
  long ts;
  gettimeofday(&tv, NULL);
  do
  {
    (void)gettimeofday(&ttv, NULL);
    ts = 1000000 * (ttv.tv_sec - tv.tv_sec) + (ttv.tv_usec - tv.tv_usec);
  } while (ts < usec && !tty_break);
}

