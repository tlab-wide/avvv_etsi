/* ringbuffer.c
 * Routines for packet capture windows
 *
 * Wireshark - Network traffic analyzer
 * By Gerald Combs <gerald@wireshark.org>
 * Copyright 1998 Gerald Combs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/*
 * <laurent.deniel@free.fr>
 *
 * Almost completely rewritten in order to:
 *
 * - be able to use a unlimited number of ringbuffer files
 * - close the current file and open (truncating) the next file at switch
 * - set the final file name once open (or reopen)
 * - avoid the deletion of files that could not be truncated (can't arise now)
 *   and do not erase empty files
 *
 * The idea behind that is to remove the limitation of the maximum # of
 * ringbuffer files being less than the maximum # of open fd per process
 * and to be able to reduce the amount of virtual memory usage (having only
 * one file open at most) or the amount of file system usage (by truncating
 * the files at switch and not the capture stop, and by closing them which
 * makes possible their move or deletion after a switch).
 *
 */

#include <config.h>

#ifdef HAVE_LIBPCAP

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <stdlib.h>
#include <glib.h>

#include "wspcap.h"

#include <glib.h>

#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif

#ifdef _WIN32
#include <wsutil/win32-utils.h>
#endif

#include "ringbuffer.h"
#include <wsutil/file_util.h>

#ifdef HAVE_ZLIB
#include <zlib.h>
#endif

/* Ringbuffer file structure */
typedef struct _rb_file {
  gchar         *name;
} rb_file;

#define MAX_FILENAME_QUEUE  100

/** Ringbuffer data structure */
typedef struct _ringbuf_data {
  rb_file      *files;
  guint         num_files;           /**< Number of ringbuffer files (1 to ...) */
  guint         curr_file_num;       /**< Number of the current file (ever increasing) */
  gchar        *fprefix;             /**< Filename prefix */
  gchar        *fsuffix;             /**< Filename suffix */
  gboolean      nametimenum;         /**< ...num_time... or ...time_num...   */
  gboolean      unlimited;           /**< TRUE if unlimited number of files */

  int           fd;                  /**< Current ringbuffer file descriptor */
  FILE         *pdh;
  char         *io_buffer;              /**< The IO buffer used to write to the file */
  gboolean      group_read_access;   /**< TRUE if files need to be opened with group read access */
  FILE         *name_h;              /**< write names of completed files to this handle */
  gchar        *compress_type;       /**< compress type */

  GMutex        mutex;               /**< mutex for oldnames */
  gchar        *oldnames[MAX_FILENAME_QUEUE];       /**< filename list of pending to be deleted */
} ringbuf_data;

static ringbuf_data rb_data;

/*
 * delete pending uncompressed pcap files.
 */
static void CleanupOldCap(gchar* name)
{
  ws_statb64 statb;
  size_t i;

  g_mutex_lock(&rb_data.mutex);

  /* Delete pending delete file */
  for (i = 0; i < sizeof(rb_data.oldnames) / sizeof(rb_data.oldnames[0]); i++) {
    if (rb_data.oldnames[i] != NULL) {
      ws_unlink(rb_data.oldnames[i]);
      if (ws_stat64(rb_data.oldnames[i], &statb) != 0) {
        g_free(rb_data.oldnames[i]);
        rb_data.oldnames[i] = NULL;
      }
    }
  }

  if (name) {
    /* push the current file to pending list if it failed to delete */
    if (ws_stat64(name, &statb) == 0) {
      for (i = 0; i < sizeof(rb_data.oldnames) / sizeof(rb_data.oldnames[0]); i++) {
        if (rb_data.oldnames[i] == NULL) {
          rb_data.oldnames[i] = g_strdup(name);
          break;
        }
      }
    }
  }

  g_mutex_unlock(&rb_data.mutex);
}

/*
 * compress capture file
 */
static int ringbuf_exec_compress(gchar* name)
{
  guint8  *buffer = NULL;
  gchar* outgz = NULL;
  int  fd = -1;
  ssize_t nread;
  gboolean delete_org_file = TRUE;
  gzFile fi = NULL;

  fd = ws_open(name, O_RDONLY | O_BINARY, 0000);
  if (fd < 0) {
    return -1;
  }

  outgz = g_strdup_printf("%s.gz", name);
  fi = gzopen(outgz, "wb");
  g_free(outgz);
  if (fi == NULL) {
    ws_close(fd);
    return -1;
  }

#define FS_READ_SIZE 65536
  buffer = (guint8*)g_malloc(FS_READ_SIZE);
  if (buffer == NULL) {
    ws_close(fd);
    gzclose(fi);
    return -1;
  }

  while ((nread = ws_read(fd, buffer, FS_READ_SIZE)) > 0) {
    int n = gzwrite(fi, buffer, (unsigned int)nread);
    if (n <= 0) {
      /* mark compression as failed */
      delete_org_file = FALSE;
      break;
    }
  }
  if (nread < 0) {
    /* mark compression as failed */
    delete_org_file = FALSE;
  }
  ws_close(fd);
  gzclose(fi);
  g_free(buffer);

  /* delete the original file only if compression succeeds */
  if (delete_org_file) {
    ws_unlink(name);
    CleanupOldCap(name);
  }
  g_free(name);
  return 0;
}

/*
 * thread to compress capture file
 */
static void* exec_compress_thread(void* arg)
{
  ringbuf_exec_compress((gchar*)arg);
  return NULL;
}

/*
 * start a thread to compress capture file
 */
static int ringbuf_start_compress_file(rb_file* rfile)
{
  gchar* name = g_strdup(rfile->name);
  g_thread_new("exec_compress", &exec_compress_thread, name);
  return 0;
}

/*
 * create the next filename and open a new binary file with that name
 */
static int ringbuf_open_file(rb_file *rfile, int *err)
{
  char    filenum[5+1];
  char    timestr[14+1];
  time_t  current_time;
  struct tm *tm;

  if (rfile->name != NULL) {
    if (rb_data.unlimited == FALSE) {
      /* remove old file (if any, so ignore error) */
      ws_unlink(rfile->name);
    }
    else if (rb_data.compress_type != NULL && strcmp(rb_data.compress_type, "gzip") == 0) {
      ringbuf_start_compress_file(rfile);
    }
    g_free(rfile->name);
  }

#ifdef _WIN32
  _tzset();
#endif
  current_time = time(NULL);

  g_snprintf(filenum, sizeof(filenum), "%05u", (rb_data.curr_file_num + 1) % RINGBUFFER_MAX_NUM_FILES);
  tm = localtime(&current_time);
  if (tm != NULL)
    strftime(timestr, sizeof(timestr), "%Y%m%d%H%M%S", tm);
  else
    (void) g_strlcpy(timestr, "196912312359", sizeof(timestr)); /* second before the Epoch */
  if (rb_data.nametimenum) {
    rfile->name = g_strconcat(rb_data.fprefix, "_", timestr, "_", filenum, rb_data.fsuffix, NULL);
  } else {
    rfile->name = g_strconcat(rb_data.fprefix, "_", filenum, "_", timestr, rb_data.fsuffix, NULL);
  }

  if (rfile->name == NULL) {
    if (err != NULL)
      *err = ENOMEM;
    return -1;
  }

  rb_data.fd = ws_open(rfile->name, O_RDWR|O_BINARY|O_TRUNC|O_CREAT,
                            rb_data.group_read_access ? 0640 : 0600);

  if (rb_data.fd == -1 && err != NULL) {
    *err = errno;
  }

  return rb_data.fd;
}

/*
 * Initialize the ringbuffer data structures
 */
int
ringbuf_init(const char *capfile_name, guint num_files, gboolean group_read_access,
             gchar *compress_type, gboolean has_nametimenum)
{
  unsigned int i;
  char        *pfx, *last_pathsep;
  gchar       *save_file;

  rb_data.files = NULL;
  rb_data.curr_file_num = 0;
  rb_data.fprefix = NULL;
  rb_data.fsuffix = NULL;
  rb_data.nametimenum = has_nametimenum;
  rb_data.unlimited = FALSE;
  rb_data.fd = -1;
  rb_data.pdh = NULL;
  rb_data.io_buffer = NULL;
  rb_data.group_read_access = group_read_access;
  rb_data.name_h = NULL;
  rb_data.compress_type = compress_type;
  g_mutex_init(&rb_data.mutex);

  /* just to be sure ... */
  if (num_files <= RINGBUFFER_MAX_NUM_FILES) {
    rb_data.num_files = num_files;
  } else {
    rb_data.num_files = RINGBUFFER_MAX_NUM_FILES;
  }

  /* Check file name */
  if (capfile_name == NULL) {
    /* ringbuffer does not work with temporary files! */
    return -1;
  }

  /* set file name prefix/suffix */

  save_file = g_strdup(capfile_name);
  last_pathsep = strrchr(save_file, G_DIR_SEPARATOR);
  pfx = strrchr(save_file,'.');
  if (pfx != NULL && (last_pathsep == NULL || pfx > last_pathsep)) {
    /* The pathname has a "." in it, and it's in the last component
       of the pathname (because there is either only one component,
       i.e. last_pathsep is null as there are no path separators,
       or the "." is after the path separator before the last
       component.

       Treat it as a separator between the rest of the file name and
       the file name suffix, and arrange that the names given to the
       ring buffer files have the specified suffix, i.e. put the
       changing part of the name *before* the suffix. */
    pfx[0] = '\0';
    rb_data.fprefix = g_strdup(save_file);
    pfx[0] = '.'; /* restore capfile_name */
    rb_data.fsuffix = g_strdup(pfx);
  } else {
    /* Either there's no "." in the pathname, or it's in a directory
       component, so the last component has no suffix. */
    rb_data.fprefix = g_strdup(save_file);
    rb_data.fsuffix = NULL;
  }
  g_free(save_file);
  save_file = NULL;

  /* allocate rb_file structures (only one if unlimited since there is no
     need to save all file names in that case) */

  if (num_files == RINGBUFFER_UNLIMITED_FILES) {
    rb_data.unlimited = TRUE;
    rb_data.num_files = 1;
  }

  rb_data.files = g_new(rb_file, rb_data.num_files);
  if (rb_data.files == NULL) {
    return -1;
  }

  for (i=0; i < rb_data.num_files; i++) {
    rb_data.files[i].name = NULL;
  }

  /* create the first file */
  if (ringbuf_open_file(&rb_data.files[0], NULL) == -1) {
    ringbuf_error_cleanup();
    return -1;
  }

  return rb_data.fd;
}

/*
 * Set name of file to which to print ringbuffer file names.
 */
gboolean
ringbuf_set_print_name(gchar *name, int *err)
{
  if (rb_data.name_h != NULL) {
    if (EOF == fclose(rb_data.name_h)) {
      if (err != NULL) {
        *err = errno;
      }
      return FALSE;
    }
  }
  if (!strcmp(name, "-") || !strcmp(name, "stdout")) {
    rb_data.name_h = stdout;
  } else if (!strcmp(name, "stderr")) {
    rb_data.name_h = stderr;
  } else {
    if (NULL == (rb_data.name_h = ws_fopen(name, "wt"))) {
      if (err != NULL) {
        *err = errno;
      }
      return FALSE;
    }
  }
  return TRUE;
}

/*
 * Whether the ringbuf filenames are ready.
 * (Whether ringbuf_init is called and ringbuf_free is not called.)
 */
gboolean ringbuf_is_initialized(void)
{
  return rb_data.files != NULL;
}

const gchar *ringbuf_current_filename(void)
{
  return rb_data.files[rb_data.curr_file_num % rb_data.num_files].name;
}

/*
 * Calls ws_fdopen() for the current ringbuffer file
 */
FILE *
ringbuf_init_libpcap_fdopen(int *err)
{
  rb_data.pdh = ws_fdopen(rb_data.fd, "wb");
  if (rb_data.pdh == NULL) {
    if (err != NULL) {
      *err = errno;
    }
  } else {
    size_t buffsize = IO_BUF_SIZE;
#ifdef HAVE_STRUCT_STAT_ST_BLKSIZE
    ws_statb64 statb;

    if (ws_fstat64(rb_data.fd, &statb) == 0) {
      if (statb.st_blksize > IO_BUF_SIZE) {
        buffsize = statb.st_blksize;
      }
    }
#endif
    /* Increase the size of the IO buffer */
    rb_data.io_buffer = (char *)g_realloc(rb_data.io_buffer, buffsize);
    setvbuf(rb_data.pdh, rb_data.io_buffer, _IOFBF, buffsize);
  }

  return rb_data.pdh;
}

/*
 * Switches to the next ringbuffer file
 */
gboolean
ringbuf_switch_file(FILE **pdh, gchar **save_file, int *save_file_fd, int *err)
{
  int     next_file_index;
  rb_file *next_rfile = NULL;

  /* close current file */

  if (fclose(rb_data.pdh) == EOF) {
    if (err != NULL) {
      *err = errno;
    }
    ws_close(rb_data.fd);  /* XXX - the above should have closed this already */
    rb_data.pdh = NULL;    /* it's still closed, we just got an error while closing */
    rb_data.fd = -1;
    g_free(rb_data.io_buffer);
    rb_data.io_buffer = NULL;
    return FALSE;
  }

  rb_data.pdh = NULL;
  rb_data.fd  = -1;

  if (rb_data.name_h != NULL) {
    fprintf(rb_data.name_h, "%s\n", ringbuf_current_filename());
    fflush(rb_data.name_h);
  }

  /* get the next file number and open it */

  rb_data.curr_file_num++ /* = next_file_num*/;
  next_file_index = (rb_data.curr_file_num) % rb_data.num_files;
  next_rfile = &rb_data.files[next_file_index];

  if (ringbuf_open_file(next_rfile, err) == -1) {
    return FALSE;
  }

  if (ringbuf_init_libpcap_fdopen(err) == NULL) {
    return FALSE;
  }

  /* switch to the new file */
  *save_file = next_rfile->name;
  *save_file_fd = rb_data.fd;
  (*pdh) = rb_data.pdh;

  return TRUE;
}

/*
 * Calls fclose() for the current ringbuffer file
 */
gboolean
ringbuf_libpcap_dump_close(gchar **save_file, int *err)
{
  gboolean  ret_val = TRUE;

  /* close current file, if it's open */
  if (rb_data.pdh != NULL) {
    if (fclose(rb_data.pdh) == EOF) {
      if (err != NULL) {
        *err = errno;
      }
      ws_close(rb_data.fd);
      ret_val = FALSE;
    }
    rb_data.pdh = NULL;
    rb_data.fd  = -1;
    g_free(rb_data.io_buffer);
    rb_data.io_buffer = NULL;

  }

  if (rb_data.name_h != NULL) {
    fprintf(rb_data.name_h, "%s\n", ringbuf_current_filename());
    fflush(rb_data.name_h);

    if (EOF == fclose(rb_data.name_h)) {
      /* Can't really do much about this, can we? */
    }
  }

  /* set the save file name to the current file */
  *save_file = rb_data.files[rb_data.curr_file_num % rb_data.num_files].name;
  return ret_val;
}

/*
 * Frees all memory allocated by the ringbuffer
 */
void
ringbuf_free(void)
{
  unsigned int i;

  if (rb_data.files != NULL) {
    for (i=0; i < rb_data.num_files; i++) {
      if (rb_data.files[i].name != NULL) {
        g_free(rb_data.files[i].name);
        rb_data.files[i].name = NULL;
      }
    }
    g_free(rb_data.files);
    rb_data.files = NULL;
  }
  if (rb_data.fprefix != NULL) {
    g_free(rb_data.fprefix);
    rb_data.fprefix = NULL;
  }
  if (rb_data.fsuffix != NULL) {
    g_free(rb_data.fsuffix);
    rb_data.fsuffix = NULL;
  }

  CleanupOldCap(NULL);
}

/*
 * Frees all memory allocated by the ringbuffer
 */
void
ringbuf_error_cleanup(void)
{
  unsigned int i;

  /* try to close via wtap */
  if (rb_data.pdh != NULL) {
    if (fclose(rb_data.pdh) == 0) {
      rb_data.fd = -1;
    }
    rb_data.pdh = NULL;
  }

  /* close directly if still open */
  if (rb_data.fd != -1) {
    ws_close(rb_data.fd);
    rb_data.fd = -1;
  }

  if (rb_data.files != NULL) {
    for (i=0; i < rb_data.num_files; i++) {
      if (rb_data.files[i].name != NULL) {
        ws_unlink(rb_data.files[i].name);
      }
    }
  }
  g_free(rb_data.io_buffer);
  rb_data.io_buffer = NULL;

  if (rb_data.name_h != NULL) {
    if (EOF == fclose(rb_data.name_h)) {
      /* Can't really do much about this, can we? */
    }
  }

  /* free the memory */
  ringbuf_free();
}

#endif /* HAVE_LIBPCAP */

/*
 * Editor modelines  -  https://www.wireshark.org/tools/modelines.html
 *
 * Local Variables:
 * c-basic-offset: 2
 * tab-width: 8
 * indent-tabs-mode: nil
 * End:
 *
 * ex: set shiftwidth=2 tabstop=8 expandtab:
 * :indentSize=2:tabSize=8:noTabs=true:
 */
